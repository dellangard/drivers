/*
 * mcu_timer.c
 *
 *  Created on: 14 янв. 2017 г.
 *      Author: frost
 */

#include <stdint.h>
#include <stddef.h>


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "mcu_timers.h"

extern volatile uint32_t SystemCoreClock;

static const enum rcc_periph_clken timer_rcc_periph[] = {
        RCC_TIM1,
        RCC_TIM2,
        RCC_TIM3,
        RCC_TIM4,
        RCC_TIM5,
        RCC_TIM6,
        RCC_TIM7,
        RCC_TIM8,
        RCC_TIM9,
        RCC_TIM10,
        RCC_TIM11,
        RCC_TIM12,
        RCC_TIM13,
        RCC_TIM14
};



static uint8_t timer_get_num_by_tim_base(uint32_t tim_base);


static uint8_t timer_get_num_by_tim_base(uint32_t tim_base)
{
    switch(tim_base)
    {
        case TIM1:
            return 0;
            break;

        case TIM2:
            return 1;
            break;

        case TIM3:
            return 2;
            break;

        case TIM4:
            return 3;
            break;

        case TIM5:
            return 4;
            break;

        case TIM6:
            return 5;
            break;

        case TIM7:
            return 6;
            break;

        case TIM8:
            return 7;
            break;

        case TIM9:
            return 8;
            break;

        case TIM10:
            return 9;
            break;

        case TIM11:
            return 10;
            break;

        case TIM12:
            return 11;
            break;

        case TIM13:
            return 12;
            break;

        case TIM14:
            return 13;
            break;

        default:
            //Error, TIMx not found
            //Returning 0 is also incorrect
            return 0;
            break;

    };
}

static const uint8_t timer_gpio_af[] = {
        GPIO_AF_TIM1,
        GPIO_AF_TIM2,
        GPIO_AF_TIM3,
        GPIO_AF_TIM4,
        GPIO_AF_TIM5,
        0,
        0,
        GPIO_AF_TIM8,
        GPIO_AF_TIM9,
        GPIO_AF_TIM10,
        GPIO_AF_TIM11,
        GPIO_AF_TIM12,
        GPIO_AF_TIM13,
        GPIO_AF_TIM14
};


void timer_register_driver(timer_driver_descriptor_t *timer_dd)
{
    uint8_t timer_num = timer_get_num_by_tim_base(timer_dd->tim_base);

    //Enable TIM clock
    rcc_periph_clock_enable(timer_rcc_periph[timer_num]);

    //Reset timer
    timer_reset(timer_dd->tim_base);    //ToDo: replace with rcc_periph_reset_pulse(RST_TIMx)

    timer_set_mode(timer_dd->tim_base, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    //Set prescaler
    timer_set_prescaler(timer_dd->tim_base, timer_dd->prescaler);

    timer_set_repetition_counter(timer_dd->tim_base, 0);

    timer_enable_preload(timer_dd->tim_base);

    timer_continuous_mode(timer_dd->tim_base);

    timer_set_period(timer_dd->tim_base, timer_dd->period);


    //For PWM mode

    if((timer_dd->tim_base != TIM6) && (timer_dd->tim_base != TIM7))
    {

        if(timer_dd->tim_mode == timer_mode_pwm)
        {
            //Configure timer outputs
            if(timer_dd->gpio_ch1 != NULL)
            {
                gpio_mode_setup(timer_dd->gpio_ch1->port, GPIO_MODE_AF, GPIO_PUPD_NONE, timer_dd->gpio_ch1->pin);
                gpio_set_output_options(timer_dd->gpio_ch1->port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, timer_dd->gpio_ch1->pin);
                gpio_set_af(timer_dd->gpio_ch1->port, timer_gpio_af[timer_num], timer_dd->gpio_ch1->pin);


                timer_disable_oc_output(timer_dd->tim_base, TIM_OC1);
                timer_set_oc_mode(timer_dd->tim_base, TIM_OC1, TIM_OCM_PWM1);
                timer_set_oc_value(timer_dd->tim_base, TIM_OC1, 0);
                timer_enable_oc_output(timer_dd->tim_base, TIM_OC1);
            }

            if(timer_dd->gpio_ch2 != NULL)
            {
                gpio_mode_setup(timer_dd->gpio_ch2->port, GPIO_MODE_AF, GPIO_PUPD_NONE, timer_dd->gpio_ch2->pin);
                gpio_set_output_options(timer_dd->gpio_ch2->port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, timer_dd->gpio_ch2->pin);
                gpio_set_af(timer_dd->gpio_ch2->port, timer_gpio_af[timer_num], timer_dd->gpio_ch2->pin);


                timer_disable_oc_output(timer_dd->tim_base, TIM_OC2);
                timer_set_oc_mode(timer_dd->tim_base, TIM_OC2, TIM_OCM_PWM1);
                timer_set_oc_value(timer_dd->tim_base, TIM_OC2, 0);
                timer_enable_oc_output(timer_dd->tim_base, TIM_OC2);
            }

//            timer_disable_oc_output(timer_dd->tim_base, TIM_OC2);
//            timer_set_oc_mode(timer_dd->tim_base, TIM_OC2, TIM_OCM_PWM1);
//            timer_set_oc_value(timer_dd->tim_base, TIM_OC2, 0);
//            timer_enable_oc_output(timer_dd->tim_base, TIM_OC2);
//
//            timer_disable_oc_output(timer_dd->tim_base, TIM_OC3);
//            timer_set_oc_mode(timer_dd->tim_base, TIM_OC3, TIM_OCM_PWM1);
//            timer_set_oc_value(timer_dd->tim_base, TIM_OC3, 0);
//            timer_enable_oc_output(timer_dd->tim_base, TIM_OC3);
//
//            timer_disable_oc_output(timer_dd->tim_base, TIM_OC4);
//            timer_set_oc_mode(timer_dd->tim_base, TIM_OC4, TIM_OCM_PWM1);
//            timer_set_oc_value(timer_dd->tim_base, TIM_OC4, 0);
//            timer_enable_oc_output(timer_dd->tim_base, TIM_OC4);

            //Only for advanced timers
            if((timer_dd->tim_base == TIM1) || (timer_dd->tim_base == TIM8))
            {
                //Set MOE bit to enable PWM outputs
                timer_enable_break_main_output(timer_dd->tim_base);
            }

        }
    }

    //Register driver in Driver Manager
    driver_manager_register_driver(timer_dd->nameStr, timer_dd);

    //Enable timer
    timer_enable_counter(timer_dd->tim_base);
}


void timer_set_pwm_pulse_width_us(timer_driver_descriptor_t *timer_dd, enum tim_oc_id channel, uint16_t pulse_width_us)
{
    timer_set_oc_value(timer_dd->tim_base, channel, pulse_width_us);
}

