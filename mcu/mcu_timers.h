/*
 * mcu_timers.h
 *
 *  Created on: 14 янв. 2017 г.
 *      Author: frost
 */

#ifndef MCU_TIMERS_H_
#define MCU_TIMERS_H_

#include <stdint.h>

#include <libopencm3/stm32/timer.h>

#include "mcu_gpio.h"

enum timer_mode
{
    timer_mode_counter = 0,
    timer_mode_rev_counter = 1,
    timer_mode_input_capture = 2,
    timer_mode_oitput_compare = 3,
    timer_mode_pwm = 4
};

typedef struct
{
    const char* nameStr;  //"TIMx"

    const uint32_t tim_base;

    uint32_t prescaler;
    uint32_t period;

    enum timer_mode tim_mode;

    gpio_t *gpio_ch1;
    gpio_t *gpio_ch1n;
    gpio_t *gpio_ch2;
    gpio_t *gpio_ch2n;
    gpio_t *gpio_ch3;
    gpio_t *gpio_ch3n;
    gpio_t *gpio_ch4;

    //ToDo: Add ETR, BREAK, etc gpios

//    const timer_settings_t settings;

} timer_driver_descriptor_t;


void timer_set_pwm_pulse_width_us(timer_driver_descriptor_t *timer_dd, enum tim_oc_id channel, uint16_t pulse_width_us);

#endif /* MCU_TIMERS_H_ */
