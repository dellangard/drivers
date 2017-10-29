/*
 * l3g20.c
 *
 *  Created on: 11 янв. 2017 г.
 *      Author: frost
 */

#include "l3g20.h"

#include "libopencm3_headers.h"

#include "mcu/mcu_spi.h"
#include "mcu/mcu_gpio.h"
#include "driver_manager.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"



#define L3G20_REG_WHO_AM_I       (uint8_t)0x0F

#define L3G20_CTRL_REG1          (uint8_t)0x20

#define L3G20_CTRL_REG1_XEN          (uint8_t)(1 << 0)
#define L3G20_CTRL_REG1_YEN          (uint8_t)(1 << 1)
#define L3G20_CTRL_REG1_ZEN          (uint8_t)(1 << 2)
#define L3G20_CTRL_REG1_PD           (uint8_t)(1 << 3)

#define L3G20_CTRL_REG1_DRBW             (uint8_t)(0xF << 4) //ODR and Bandwidth selection mask
#define L3G20_CTRL_REG1_DRBW_ODR95_CO12_5            (uint8_t)(0x0 << 4) //ODR 95 Hz, Cut-off 12.5
#define L3G20_CTRL_REG1_DRBW_ODR95_CO25          (uint8_t)(0x1 << 4) //ODR 95 Hz, Cut-off 25
//#define L3G20_CTRL_REG1_DRBW_ODR95_CO25            (uint8_t)(0x2 << 4) //ODR 95 Hz, Cut-off 25
//#define L3G20_CTRL_REG1_DRBW_ODR95_CO25            (uint8_t)(0x3 << 4) //ODR 95 Hz, Cut-off 25
#define L3G20_CTRL_REG1_DRBW_ODR190_CO12_5       (uint8_t)(0x4 << 4) //ODR 190 Hz, Cut-off 12.5
#define L3G20_CTRL_REG1_DRBW_ODR190_CO25             (uint8_t)(0x5 << 4) //ODR 190 Hz, Cut-off 25
#define L3G20_CTRL_REG1_DRBW_ODR190_CO50            (uint8_t)(0x6 << 4) //ODR 190 Hz, Cut-off 50
#define L3G20_CTRL_REG1_DRBW_ODR190_CO70            (uint8_t)(0x7 << 4) //ODR 190 Hz, Cut-off 70
#define L3G20_CTRL_REG1_DRBW_ODR380_CO20            (uint8_t)(0x8 << 4) //ODR 380 Hz, Cut-off 20
#define L3G20_CTRL_REG1_DRBW_ODR380_CO25            (uint8_t)(0x9 << 4) //ODR 380 Hz, Cut-off 25
#define L3G20_CTRL_REG1_DRBW_ODR380_CO50            (uint8_t)(0xA << 4) //ODR 380 Hz, Cut-off 50
#define L3G20_CTRL_REG1_DRBW_ODR380_CO100       (uint8_t)(0xB << 4) //ODR 380 Hz, Cut-off 100
#define L3G20_CTRL_REG1_DRBW_ODR760_CO30            (uint8_t)(0xC << 4) //ODR 380 Hz, Cut-off 30
#define L3G20_CTRL_REG1_DRBW_ODR760_CO35            (uint8_t)(0xD << 4) //ODR 380 Hz, Cut-off 35
#define L3G20_CTRL_REG1_DRBW_ODR760_CO50            (uint8_t)(0xE << 4) //ODR 380 Hz, Cut-off 50
#define L3G20_CTRL_REG1_DRBW_ODR760_CO100       (uint8_t)(0xF << 4) //ODR 380 Hz, Cut-off 100



#define L3G20_CTRL_REG2          (uint8_t)0x21

#define L3G20_CTRL_REG2_HPCF                    (uint8_t)(0xF << 0) //High-pass filter cutoff frequency selection mask
#define L3G20_CTRL_REG2_HPCF_ODR95_CO7_2        (uint8_t)(0x0 << 0) //ODR 95 Hz, Cut-off 7.2
#define L3G20_CTRL_REG2_HPCF_ODR95_CO3_5        (uint8_t)(0x1 << 0) //ODR 95 Hz, Cut-off 3.5
#define L3G20_CTRL_REG2_HPCF_ODR95_CO1_8        (uint8_t)(0x2 << 0) //ODR 95 Hz, Cut-off 1.8
#define L3G20_CTRL_REG2_HPCF_ODR95_CO0_9        (uint8_t)(0x3 << 0) //ODR 95 Hz, Cut-off 0.9
#define L3G20_CTRL_REG2_HPCF_ODR95_CO0_45   (uint8_t)(0x4 << 0) //ODR 95 Hz, Cut-off 0.45
#define L3G20_CTRL_REG2_HPCF_ODR95_CO0_18   (uint8_t)(0x5 << 0) //ODR 95 Hz, Cut-off 0.18
#define L3G20_CTRL_REG2_HPCF_ODR95_CO0_09   (uint8_t)(0x6 << 0) //ODR 95 Hz, Cut-off 0.09
#define L3G20_CTRL_REG2_HPCF_ODR95_CO0_045  (uint8_t)(0x7 << 0) //ODR 95 Hz, Cut-off 0.045


//..

#define L3G20_CTRL_REG2_HPM (uint8_t)(0x3 << 4) //High-pass filter mode configuration mask
#define L3G20_CTRL_REG2_HPM_NORMAL  (uint8_t)(0x0 << 4) //HPM nomal mode
#define L3G20_CTRL_REG2_HPM_REFERENCE   (uint8_t)(0x1 << 4) //Reference signal for filtering

#define L3G20_CTRL_REG2_HPM_AUTOSET (uint8_t)(0x3 << 4) //Autoset on interrupt event




#define L3G20_CTRL_REG3          (uint8_t)0x22

#define L3G20_CTRL_REG3_I2EMPTY          (uint8_t)(1 << 0)  //FIFO Empty interrupt on DRDY/INT2
#define L3G20_CTRL_REG3_I2ORUN           (uint8_t)(1 << 1)  //FIFO Overrun interrupt on DRDY/INT2
#define L3G20_CTRL_REG3_I2WTM               (uint8_t)(1 << 2)   //FIFO Watermark interrupt on DRDY/INT2
#define L3G20_CTRL_REG3_I2DRDY          (uint8_t)(1 << 3)   //Data ready interrupt on DRDY/INT2
#define L3G20_CTRL_REG3_PPOD                (uint8_t)(1 << 4)   //Push-Pull/Open-Drain mode INTx pin mode
#define L3G20_CTRL_REG3_HLACTIVE            (uint8_t)(1 << 5)   //Interrupt active configuration on INT1
#define L3G20_CTRL_REG3_I1BOOT          (uint8_t)(1 << 4)   //Boot status on INT1
#define L3G20_CTRL_REG3_I1INT1          (uint8_t)(1 << 4)   //Interrupt enable on INT1



#define L3G20_CTRL_REG4          (uint8_t)0x23

#define L3G20_CTRL_REG4_SIM          (uint8_t)(1 << 0) //SPI serial interface mode selection
//Bits 1..3 are not used
#define L3G20_CTRL_REG4_FS           (uint8_t)(0x3 << 4) //Full scale selection mask
#define L3G20_CTRL_REG4_FS_250           (uint8_t)(0x0 << 4)    //250 dps
#define L3G20_CTRL_REG4_FS_500           (uint8_t)(0x1 << 4)    //500 dps
#define L3G20_CTRL_REG4_FS_2000          (uint8_t)(0x2 << 4)    //2000 dps


#define L3G20_CTRL_REG4_BLE          (uint8_t)(1 << 6) //Big/little endian data selection
#define L3G20_CTRL_REG4_BDU          (uint8_t)(1 << 7) //Block data update


#define L3G20_CTRL_REG5          (uint8_t)0x24

#define L3G20_CTRL_REG5_OUT1SEL (uint8_t)(0x3 << 0) //Out selection configuration mask
//..
#define L3G20_CTRL_REG5_INT1SEL (uint8_t)(0x3 << 2) //INT1 selection configuration mask
//..
#define L3G20_CTRL_REG5_HPEN        (uint8_t)(0x4 << 1) //High-pass filter enable
//Bit 5 not used
#define L3G20_CTRL_REG5_FIFOEN  (uint8_t)(0x6 << 1) //FIFO enable
#define L3G20_CTRL_REG5_BOOT        (uint8_t)(0x7 << 1) //Reboot memory content


#define L3G20_REFERENCE          (uint8_t)0x25
#define L3G20_OUT_TEMP           (uint8_t)0x26
#define L3G20_STATUS_REG             (uint8_t)0x27

#define L3G20_OUT_X_L                (uint8_t)0x28
#define L3G20_OUT_X_H                (uint8_t)0x29
#define L3G20_OUT_Y_L                (uint8_t)0x2A
#define L3G20_OUT_Y_H                (uint8_t)0x2B
#define L3G20_OUT_Z_L                (uint8_t)0x2C
#define L3G20_OUT_Z_H                (uint8_t)0x2D

#define L3G20_FIFO_CTRL_REG      (uint8_t)0x2E
#define L3G20_FIFO_SRC_REG       (uint8_t)0x2F

#define L3G20_INT1_CFG           (uint8_t)0x30
#define L3G20_INT1_SRC           (uint8_t)0x31
#define L3G20_INT1_TSH_XH            (uint8_t)0x32
#define L3G20_INT1_TSH_XL            (uint8_t)0x33
#define L3G20_INT1_TSH_YH            (uint8_t)0x34
#define L3G20_INT1_TSH_YL            (uint8_t)0x35
#define L3G20_INT1_TSH_ZH            (uint8_t)0x36
#define L3G20_INT1_TSH_Z             (uint8_t)0x37
#define L3G20_INT1_DURATION      (uint8_t)0x38



enum driver_register_result l3gd20_register_driver(l3gd20_driver_descriptor_t *l3gd20_instance)
{
    uint8_t nvic_vector = 0;

    gpio_mode_setup(l3gd20_instance->gpio_cs->port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, l3gd20_instance->gpio_cs->pin);
    gpio_set_output_options(l3gd20_instance->gpio_cs->port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, l3gd20_instance->gpio_cs->pin);

    //Set CS HIGH
    gpio_atomic_hi( l3gd20_instance->gpio_cs );

    if(l3gd20_instance->gpio_int1 != NULL)
    {
        //Set GPIO as input
        gpio_mode_setup(l3gd20_instance->gpio_int1->port, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, l3gd20_instance->gpio_int1->pin);

        //Try to request resources and register EXTI interrupt handlers via EXTI manager
        if(l3gd20_instance->int1_config != NULL)
        {
            if(exti_manager_register_handler(l3gd20_instance, l3gd20_instance->int1_config->callback, log2s( l3gd20_instance->gpio_int1->pin) ) != driver_register_ok)
            {
                //Error registering EXTI handler
                return driver_register_unknown_error;
            }

            nvic_vector = exti_manager_get_exti_nvic_vector( log2s( l3gd20_instance->gpio_int1->pin ) );

            exti_select_source(l3gd20_instance->gpio_int1->pin, l3gd20_instance->gpio_int1->port);
            exti_set_trigger(l3gd20_instance->gpio_int1->pin, l3gd20_instance->int1_config->trigger_type);

            //Get NVIC IRQ number for specified EXTI pin, clear if pending and enable EXTI in NVIC
            nvic_clear_pending_irq(nvic_vector);

            nvic_set_priority(nvic_vector, l3gd20_instance->int1_config->nvic_priority);

            if(l3gd20_instance->int1_config->enabled == true)
            {
                nvic_enable_irq(nvic_vector);
            }
        }
    }

    if(l3gd20_instance->gpio_int2 != NULL)
    {
        gpio_mode_setup(l3gd20_instance->gpio_int2->port, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, l3gd20_instance->gpio_int2->pin);

        if(l3gd20_instance->int2_config != true)
        {


            if(l3gd20_instance->int2_config->enabled == true)
            {
                if(exti_manager_register_handler(l3gd20_instance, l3gd20_instance->int2_config->callback,  log2s( l3gd20_instance->gpio_int2->pin) ) == 0)
                {
                    //Error registering EXTI handler
                    return driver_register_unknown_error;
                }

                nvic_vector = exti_manager_get_exti_nvic_vector( log2s( l3gd20_instance->gpio_int2->pin ) );

                exti_select_source(l3gd20_instance->gpio_int2->pin, l3gd20_instance->gpio_int2->port);
                exti_set_trigger(l3gd20_instance->gpio_int2->pin, l3gd20_instance->int2_config->trigger_type);

                //Get NVIC IRQ number for specified EXTI pin, clear if pending and enable EXTI in NVIC
                nvic_clear_pending_irq(nvic_vector);

                nvic_set_priority(nvic_vector, l3gd20_instance->int2_config->nvic_priority);

                if(l3gd20_instance->int1_config->enabled == true)
                {
                    nvic_enable_irq(nvic_vector);
                }
            }
        }
    }

    //Register driver in Driver Manager
    driver_manager_register_driver(l3gd20_instance->nameStr, l3gd20_instance);

    return driver_register_ok;
}


void l3gd20_init(const l3gd20_driver_descriptor_t *l3gd20_instance)
{
	//Temporary buffer for first sensor data reading
	uint8_t sensor_data[6];

	spi_write_single_reg8(
	        l3gd20_instance->spi_dd,
	        l3gd20_instance->gpio_cs,
	        L3G20_CTRL_REG4, L3G20_CTRL_REG4_BDU | L3G20_CTRL_REG4_FS_500);

	spi_write_single_reg8(
	            l3gd20_instance->spi_dd,
	            l3gd20_instance->gpio_cs,
	            L3G20_CTRL_REG3, L3G20_CTRL_REG3_I2DRDY);

	spi_write_single_reg8(
	            l3gd20_instance->spi_dd,
	            l3gd20_instance->gpio_cs,
	            L3G20_CTRL_REG1, L3G20_CTRL_REG1_XEN | L3G20_CTRL_REG1_YEN | L3G20_CTRL_REG1_ZEN |
			L3G20_CTRL_REG1_PD | L3G20_CTRL_REG1_DRBW_ODR95_CO25
	);

	//Enable DRDY interrupt from L3GD20 in EXTI controller
	exti_enable_request(l3gd20_instance->gpio_int2->pin);

	//First data reading
	spi_read_multi_reg8(
	        l3gd20_instance->spi_dd,
	        l3gd20_instance->gpio_cs,
	        L3G20_OUT_X_L, 6, sensor_data);
}


void l3gd20_get_gyro3d(const l3gd20_driver_descriptor_t *l3gd20_instance, int16vector_t *vector)
{
	uint8_t sensor_data[6];

	spi_read_multi_reg8(
	            l3gd20_instance->spi_dd,
	            l3gd20_instance->gpio_cs,
	            L3G20_OUT_X_L, 6, sensor_data);

	vector->x = (int16_t)(sensor_data[0] | (sensor_data[1] << 8));
	vector->y = (int16_t)(sensor_data[2] | (sensor_data[3] << 8));
	vector->z = (int16_t)(sensor_data[4] | (sensor_data[5] << 8));
}

uint8_t l3gd20_get_gyro3d_with_status(const l3gd20_driver_descriptor_t *l3gd20_instance, int16vector_t *vector)
{
	uint8_t sensor_data[7];

	spi_read_multi_reg8(
                l3gd20_instance->spi_dd,
                l3gd20_instance->gpio_cs,
                L3G20_STATUS_REG, 7, sensor_data);

	vector->x = (int16_t)(sensor_data[1] | (sensor_data[2] << 8));
	vector->y = (int16_t)(sensor_data[3] | (sensor_data[4] << 8));
	vector->z = (int16_t)(sensor_data[5] | (sensor_data[6] << 8));

	return sensor_data[0];
}

uint8_t l3gd20_get_temperature(const l3gd20_driver_descriptor_t *l3gd20_instance)
{
	return spi_read_single_reg8(
            l3gd20_instance->spi_dd,
            l3gd20_instance->gpio_cs,
            L3G20_OUT_TEMP);
}

uint8_t l3gd20_get_status(const l3gd20_driver_descriptor_t *l3gd20_instance)
{
	return spi_read_single_reg8(
            l3gd20_instance->spi_dd,
            l3gd20_instance->gpio_cs,
            L3G20_STATUS_REG);
}

