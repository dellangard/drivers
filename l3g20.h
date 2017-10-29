/*
 * l3g20.h
 *
 *  Created on: 8 янв. 2017 г.
 *      Author: frost
 */

#ifndef L3G20_H_
#define L3G20_H_

#include <stdint.h>

#include "helpers/vector_math.h"

#include "driver_manager.h"

#include "FreeRTOS.h"

#include "mcu/mcu_gpio.h"
#include "mcu/mcu_spi.h"

typedef struct
{
    const char* nameStr;  //"L3GD20"

    //Pointer to SPI bus driver descriptor
    const spi_driver_descriptor_t *spi_dd;

    const gpio_t *gpio_cs;

    const gpio_t *gpio_int1;
    const gpio_t *gpio_int2;

    const exti_interrupt_config_t *int1_config;
    const exti_interrupt_config_t *int2_config;

} l3gd20_driver_descriptor_t;



enum driver_register_result l3gd20_register_driver(l3gd20_driver_descriptor_t *l3gd20_instance);

void l3gd20_drdy_callback(BaseType_t *pxHigherPriorityTaskWoken);

void l3gd20_init(const l3gd20_driver_descriptor_t *l3gd20_instance);

uint8_t l3gd20_read_reg8(const l3gd20_driver_descriptor_t *l3gd20_instance, const uint8_t reg_addr);

void l3gd20_write_reg8(const l3gd20_driver_descriptor_t *l3gd20_instance, const uint8_t reg_addr, const uint8_t data);

void l3gd20_get_gyro3d(const l3gd20_driver_descriptor_t *l3gd20_instance, int16vector_t *vector);

uint8_t l3gd20_get_gyro3d_with_status(const l3gd20_driver_descriptor_t *l3gd20_instance, int16vector_t *vector);


#endif /* L3G20_H_ */
