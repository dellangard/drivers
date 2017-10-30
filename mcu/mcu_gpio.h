/*
 * mcu_gpio.h
 *
 *  Created on: 3 мар. 2017 г.
 *      Author: frost
 */

#ifndef DRIVERS_MCU_MCU_GPIO_H_
#define DRIVERS_MCU_MCU_GPIO_H_

#include <libopencm3/stm32/gpio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

typedef struct
{
    const uint32_t port;
    const uint16_t pin;
} gpio_t;


typedef enum button_events
{
    button_not_pressed = 0,
    button_just_pressed = 1,
    button_short_press = 2,
    button_long_press = 3,
    button_released = 4
} button_events_t;


typedef struct
{
    const char* nameStr;  //"BUTTON_Pxx"

    const gpio_t *gpio;

    const uint8_t priority;

    const TickType_t deglitching_time_ticks;
    const TickType_t long_press_ticks;  //'long_press_ticks' value must be greatr than 'deglitching_time_ticks' value

    const uint8_t event_queue_length;   //Length of button event queue

    volatile button_events_t last_state;

    //Timer for deglitching and short/long press event determining
    TimerHandle_t xButtonTimer;

    //Queue for storing button event(s)
    QueueHandle_t xButtonEventQueue;

} gpio_button_driver_descriptor_t;


__inline__ void __attribute__((always_inline)) gpio_atomic_low(gpio_t *gpio)
{
    GPIO_BSRR(gpio->port) = (gpio->pin) << 16;
}

__inline__ void __attribute__((always_inline)) gpio_atomic_hi(gpio_t *gpio)
{
    GPIO_BSRR(gpio->port) = gpio->pin;
}

__inline__ uint8_t __attribute__((always_inline)) gpio_read(gpio_t *gpio)
{
    return ( GPIO_IDR(gpio->port) & (gpio->pin) ) ? 1 : 0;
}


void button_register_driver(gpio_button_driver_descriptor_t *button_dd);




//From StdPeriphLib


//AF 0 selection
#define 	GPIO_AF_RTC_50Hz   ((uint8_t)0x00)
#define 	GPIO_AF_MCO   ((uint8_t)0x00)
#define 	GPIO_AF_TAMPER   ((uint8_t)0x00)
#define 	GPIO_AF_SWJ   ((uint8_t)0x00)
#define 	GPIO_AF_TRACE   ((uint8_t)0x00)

//AF 1 selection
#define 	GPIO_AF_TIM1   ((uint8_t)0x01)
#define 	GPIO_AF_TIM2   ((uint8_t)0x01)

//AF 2 selection
#define 	GPIO_AF_TIM3   ((uint8_t)0x02)
#define 	GPIO_AF_TIM4   ((uint8_t)0x02)
#define 	GPIO_AF_TIM5   ((uint8_t)0x02)

//AF 3 selection.
#define 	GPIO_AF_TIM8   ((uint8_t)0x03)
#define 	GPIO_AF_TIM9   ((uint8_t)0x03)
#define 	GPIO_AF_TIM10   ((uint8_t)0x03)
#define 	GPIO_AF_TIM11   ((uint8_t)0x03)

//AF 4 selection
#define 	GPIO_AF_I2C1   ((uint8_t)0x04)
#define 	GPIO_AF_I2C2   ((uint8_t)0x04)
#define 	GPIO_AF_I2C3   ((uint8_t)0x04)

//AF 5 selection
#define 	GPIO_AF_SPI1   ((uint8_t)0x05)
#define 	GPIO_AF_SPI2   ((uint8_t)0x05)
#define    GPIO_AF_SPI4   ((uint8_t)0x05)
#define    GPIO_AF_SPI5   ((uint8_t)0x05)
#define    GPIO_AF_SPI6   ((uint8_t)0x05)

//AF 6 selection
#define 	GPIO_AF_SPI3   ((uint8_t)0x06)

//AF 7 selection
#define 	GPIO_AF_USART1   ((uint8_t)0x07)
#define 	GPIO_AF_USART2   ((uint8_t)0x07)
#define 	GPIO_AF_USART3   ((uint8_t)0x07)
#define 	GPIO_AF_I2S3ext   ((uint8_t)0x07)

//AF 8 selection
#define 	GPIO_AF_UART4   ((uint8_t)0x08)
#define 	GPIO_AF_UART5   ((uint8_t)0x08)
#define 	GPIO_AF_USART6   ((uint8_t)0x08)
#define    GPIO_AF_UART7    ((uint8_t)0x08)
#define    GPIO_AF_UART8    ((uint8_t)0x08)

//AF 9 selection
#define 	GPIO_AF_CAN1   ((uint8_t)0x09)
#define 	GPIO_AF_CAN2   ((uint8_t)0x09)
#define 	GPIO_AF_TIM12   ((uint8_t)0x09)
#define 	GPIO_AF_TIM13   ((uint8_t)0x09)
#define 	GPIO_AF_TIM14   ((uint8_t)0x09)

//AF 10 selection
#define 	GPIO_AF_OTG_FS   ((uint8_t)0xA)
#define 	GPIO_AF_OTG_HS   ((uint8_t)0xA)

//AF 11 selection
#define 	GPIO_AF_ETH   ((uint8_t)0x0B)

//AF 12 selection
#define 	GPIO_AF_FSMC   ((uint8_t)0xC)
#define 	GPIO_AF_OTG_HS_FS   ((uint8_t)0xC)
#define 	GPIO_AF_SDIO   ((uint8_t)0xC)

//AF 13 selection
#define 	GPIO_AF_DCMI   ((uint8_t)0x0D)

//AF 15 selection
#define 	GPIO_AF_EVENTOUT   ((uint8_t)0x0F)


#define 	GPIO_AF_OTG1_FS   GPIO_AF_OTG_FS
#define 	GPIO_AF_OTG2_HS   GPIO_AF_OTG_HS
#define 	GPIO_AF_OTG2_FS   GPIO_AF_OTG_HS_FS

#endif /* DRIVERS_MCU_MCU_GPIO_H_ */
