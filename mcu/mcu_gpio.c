/*
 * mcu_gpio.c
 *
 *  Created on: 27 февр. 2017 г.
 *      Author: frost
 */

//#include "peripherals.h"
#include "buildparams.h"
#include "libopencm3_headers.h"

#include "mcu_gpio.h"
#include "../driver_manager.h"

#include "FreeRTOS.h"
#include "queue.h"

void button_timer_callback(TimerHandle_t pxExpiredTimer);

//This is not an ISR, so API usage is allowed here
void button_timer_callback(TimerHandle_t pxExpiredTimer)
{
    gpio_button_driver_descriptor_t *button_dd;

    button_events_t button_event;

    //Get expired timer ID
    button_dd = (gpio_button_driver_descriptor_t*)pvTimerGetTimerID(pxExpiredTimer);

    switch(button_dd->last_state)
    {
        case button_just_pressed:   //'Front' deglitching period expired

            //If button is still pressed
            if( gpio_read(button_dd->gpio) == 0)
            {
                //Short press timeout expired
                button_dd->last_state = button_short_press;

                //Do not send anything to queue until button release or long press timeout

                //Now start 'Long press' timer (t long press - t deglitching)
                xTimerChangePeriod(button_dd->xButtonTimer, button_dd->long_press_ticks - button_dd->deglitching_time_ticks, portMAX_DELAY);

                exti_set_trigger(button_dd->gpio->pin, EXTI_TRIGGER_RISING);

            } else  //Button was released during deglitching period
            {
                //No 'Rear' deglitching timer in this case
                button_dd->last_state = button_not_pressed;

                //Stop timer
                xTimerStop(button_dd->xButtonTimer, portMAX_DELAY);

                //Send button release event to queue
                button_event = button_released;
                xQueueSend(button_dd->xButtonEventQueue, &button_event, portMAX_DELAY);

                exti_set_trigger(button_dd->gpio->pin, EXTI_TRIGGER_FALLING);
            }

            exti_reset_request(button_dd->gpio->pin);
            exti_enable_request(button_dd->gpio->pin);


            break;

        case button_short_press:    //'Long press' period expired

            button_dd->last_state = button_long_press;

            button_event = button_long_press;
            xQueueSend(button_dd->xButtonEventQueue, &button_event, portMAX_DELAY);

            break;

        case button_released:   //'Rear' deglitching timer expired

            //If button is physically released after rear deglitching timer has expired
            if( gpio_read(button_dd->gpio) )
            {
                button_dd->last_state = button_not_pressed;

                //Stop timer
                xTimerStop(button_dd->xButtonTimer, portMAX_DELAY);
            } else
            {
                button_dd->last_state = button_just_pressed;

                //Start 'front' deglitching timer
                xTimerChangePeriod(button_dd->xButtonTimer, button_dd->long_press_ticks - button_dd->deglitching_time_ticks, portMAX_DELAY);

                button_event = button_short_press;
                xQueueSend(button_dd->xButtonEventQueue, &button_event, portMAX_DELAY);

                exti_set_trigger(button_dd->gpio->pin, EXTI_TRIGGER_RISING);
                exti_reset_request(button_dd->gpio->pin);
            }


            break;

        default:

            break;
    };


}

void button_state_changed_callback(gpio_button_driver_descriptor_t *button_dd)
{
    button_events_t button_event;

    switch(button_dd->last_state)
    {
        case button_not_pressed:

            //Disable EXTI interrupts on this GPIO until deglitching timer expires
            exti_disable_request(button_dd->gpio->pin);

            button_dd->last_state = button_just_pressed;

            //Send event to queue
            button_event = button_short_press;
            xQueueSendFromISR(button_dd->xButtonEventQueue, &button_event, button_dd->pxHigherPriorityTaskWoken);

            xTimerStopFromISR(button_dd->xButtonTimer, button_dd->pxHigherPriorityTaskWoken);

            //Start 'front' deglitching timer
            xTimerChangePeriodFromISR(button_dd->xButtonTimer, button_dd->deglitching_time_ticks, button_dd->pxHigherPriorityTaskWoken);
    //        xTimerStartFromISR(button_dd->xButtonTimer, button_dd->pxHigherPriorityTaskWoken);

            break;

//        case button_just_pressed:
//
//            break;


        case button_short_press:    //Interrupt after 'front' deglitching timer expired

            button_dd->last_state = button_released;

            //Send button release event to queue
            button_event = button_released;
            xQueueSendFromISR(button_dd->xButtonEventQueue, &button_event, button_dd->pxHigherPriorityTaskWoken);

            exti_set_trigger(button_dd->gpio->pin, EXTI_TRIGGER_FALLING);
            exti_reset_request(button_dd->gpio->pin);

            break;

        case button_long_press:     //Interrupt after long press timer expired

            button_dd->last_state = button_released;

            //Send button release event to queue
            button_event = button_released;
            xQueueSendFromISR(button_dd->xButtonEventQueue, &button_event, button_dd->pxHigherPriorityTaskWoken);

            exti_set_trigger(button_dd->gpio->pin, EXTI_TRIGGER_FALLING);
            exti_reset_request(button_dd->gpio->pin);

            break;

        default:

            break;
    };

}

void button_register_driver(gpio_button_driver_descriptor_t *button_dd)
{
    gpio_mode_setup(button_dd->gpio->port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, button_dd->gpio->pin);

    if(exti_manager_register_handler(&button_dd, &button_state_changed_callback, log2s( button_dd->gpio->pin)) == driver_register_ok)
    {
        //Get NVIC vector number for given EXTI channel
        uint8_t nvic_vector = exti_manager_get_exti_nvic_vector( log2s( button_dd->gpio->pin ) );

        exti_select_source(button_dd->gpio->pin, button_dd->gpio->port);
        exti_set_trigger(button_dd->gpio->pin, EXTI_TRIGGER_FALLING);

        //Clear if IRQ is in pending state
        nvic_clear_pending_irq(nvic_vector);

        nvic_set_priority(nvic_vector, NVIC_PRIORITY_SYSCALL_EXTI_LP);

        //Enable interrupt in NVIC
        nvic_enable_irq(nvic_vector);

        exti_reset_request(button_dd->gpio->pin);
        exti_enable_request(button_dd->gpio->pin);
    };

    button_dd->xButtonTimer = xTimerCreate
        (
         "BTNTMR",
         button_dd->deglitching_time_ticks,
         pdFALSE, //Do not autoreload when timer expired
         ( void * ) (button_dd),    //Set button driver handle as timer ID
         &button_timer_callback
        );

    // Create a queue for 1 button event.
    button_dd->xButtonEventQueue = xQueueCreate( 4, sizeof( button_events_t* ) );

    //Register driver in Driver Manager
    driver_manager_register_driver(button_dd->nameStr, button_dd);
}
