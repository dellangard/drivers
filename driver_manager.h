/*
 * driver_manager.h
 *
 *  Created on: 6 окт. 2017 г.
 *      Author: frost
 */

#ifndef DRIVERS_DRIVER_MANAGER_H_
#define DRIVERS_DRIVER_MANAGER_H_

#include <libopencm3/stm32/exti.h>

#include "mcu/mcu_gpio.h"

//Driver linked list structure type definition
typedef struct
{
    const char* nameStr;
    void* driver_handle;
    struct driver_list_t *prev;
    struct driver_list_t *next;
} driver_list_t;

enum driver_register_result
{
    driver_register_ok = 0,
    driver_register_memory_allocation_failed = 1,
    driver_register_device_not_found = 2,
    driver_register_unknown_error = 3
};

typedef struct
{
    const uint8_t enabled;
    const uint8_t nvic_priority;
    const enum exti_trigger_type trigger_type;

    //Pointer to callback function
    //This pointer (if corresponding INTx line is enabled) must be passed to EXTI manager
    const void* callback;

} exti_interrupt_config_t;


void exti_manager_init(void);

uint8_t exti_manager_get_exti_nvic_vector(gpio_t *gpio);

enum driver_register_result exti_manager_register_handler(void *instance_handle, void *callback_ptr, gpio_t *gpio);

enum driver_register_result driver_manager_register_driver(const char* driver_name_string, const uint32_t* driver_handle);

void driver_manager_get_driver_list(char* driver_list_string);

#endif /* DRIVERS_DRIVER_MANAGER_H_ */
