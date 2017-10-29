/*
 * bmp085.c
 *
 *  Created on: 31 дек. 2016 г.
 *      Author: frost
 */

#include <math.h>

#include "libopencm3_headers.h"
#include "bmp085.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "helpers/movingaverage.h"


static void bmp085_get_calibration_data(bmp085_driver_descriptor_t *bmp085_dd);

static int32_t bmp085_read_raw_temperature(const bmp085_driver_descriptor_t *bmp085_dd);
static int32_t bmp085_normalize_temperature(const bmp085_driver_descriptor_t *bmp085_dd, const int32_t temperature_raw_value);

static uint32_t bmp085_read_raw_pressure(const bmp085_driver_descriptor_t *bmp085_dd);


uint8_t bmp085_register_driver(bmp085_driver_descriptor_t *bmp085_dd)
{
    uint8_t nvic_vector = 0;

    //I2C bus driver must be initialized first
//    if(bmp085_dd->i2c_bus == NULL)
//    {
//        //Exit if I2C bus driver is not initialized
//        return 1;
//    }

    //If EOC output is used
    if(bmp085_dd->gpio_eoc != NULL)
    {
        //Set GPIO as input
        gpio_mode_setup(bmp085_dd->gpio_eoc->port, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, bmp085_dd->gpio_eoc->pin);

        //If interrupt on EOC is used
        if(bmp085_dd->eoc_callback != NULL)
        {
            //Register interrupt handler
            if(exti_manager_register_handler(bmp085_dd, bmp085_dd->eoc_callback, log2s( bmp085_dd->gpio_eoc->pin) ) == 0)
            {
                //Error registering EXTI handler
                return 1;
            }

            nvic_vector = exti_manager_get_exti_nvic_vector( log2s( bmp085_dd->gpio_eoc->pin ) );

            exti_select_source(bmp085_dd->gpio_eoc->pin, bmp085_dd->gpio_eoc->port);
            exti_set_trigger(bmp085_dd->gpio_eoc->pin, EXTI_TRIGGER_RISING);

            //Get NVIC IRQ number for specified EXTI pin, clear if pending and enable EXTI in NVIC
            nvic_clear_pending_irq(nvic_vector);

            nvic_set_priority(nvic_vector, bmp085_dd->settings->interrupt_priority);

            nvic_enable_irq(nvic_vector);

            exti_reset_request(bmp085_dd->gpio_eoc->pin);
            exti_enable_request(bmp085_dd->gpio_eoc->pin);
        }
    }

    //Read factory calibration
    bmp085_get_calibration_data(bmp085_dd);

    bmp085_dd->calibration->last_measurement_type = measure_unknown;

    //If moving average algorithm is enabled
    if(bmp085_dd->settings->averaging > 1)
    {
        //EOC interrupt must be disabled

//        //Allocate memory for moving average control structure
//        bmp085_dd->averaging = (movingaverage_t*)pvPortMalloc(sizeof(movingaverage_t));
//
//        //Exit with error if memory allocation failed
//        if(bmp085_dd->averaging == NULL)
//            return 1;
//
//        //Allocate memory for moving average buffer
//        int32_t* averaging_buffer = (int32_t*)pvPortMalloc( bmp085_dd->settings.averaging*sizeof(int32_t) );
//
//        //Exit with error if memory allocation failed
//        if(averaging_buffer == NULL)
//            return 1;


        //Start temperature measurement and conversion
        bmp085_start_measurement(bmp085_dd, measure_temperature);

        //Wait sufficient time, e.g. 10 ms

        //Read temperature data and normalize
        int32_t raw_temperature = bmp085_read_raw_temperature(bmp085_dd);
        int32_t normalized_temperature = bmp085_normalize_temperature(bmp085_dd, raw_temperature);

        //Start pressure measurement and conversion
        bmp085_start_measurement(bmp085_dd, measure_pressure);

        //Wait sufficient time, e.g. 20 ms

        //Read pressure data and normalize
        int32_t raw_pressure = bmp085_read_raw_pressure(bmp085_dd);
        int32_t normalized_pressure = bmp085_convert_raw_pressure_to_normalized(bmp085_dd, raw_pressure);

        bmp085_set_averaging(bmp085_dd, bmp085_dd->settings->averaging, normalized_pressure);

        //Initialize moving average algorithm with normalized pressure value
        //ma_init(bmp085_dd->averaging, averaging_buffer, 8, normalized_pressure);

        //Reset and enable EOC interrupt if required
    }

    //Register driver in Driver Manager
    driver_manager_register_driver(bmp085_dd->nameStr, bmp085_dd);

}


static void bmp085_get_calibration_data(bmp085_driver_descriptor_t *bmp085_dd)
{
    uint8_t temporary_storage[22];

    //Pointer to calibration structure
    bmp085_t *calibration = bmp085_dd->calibration;

    //Read 22 bytes from BMP085 starting from BMP085_CAL_AC1 register to temporary_storage[]
    i2c_read_multi(bmp085_dd->i2c_dd, bmp085_dd->i2c_address, BMP085_CAL_AC1, 22, temporary_storage);

    //Update calibration settings
    calibration->AC1 = (temporary_storage[0] << 8) | temporary_storage[1];
    calibration->AC2 = (temporary_storage[2] << 8) | temporary_storage[3];
    calibration->AC3 = (temporary_storage[4] << 8) | temporary_storage[5];
    calibration->AC4 = (temporary_storage[6] << 8) | temporary_storage[7];
    calibration->AC5 = (temporary_storage[8] << 8) | temporary_storage[9];
    calibration->AC6 = (temporary_storage[10] << 8) | temporary_storage[11];
    calibration->B1 = (temporary_storage[12] << 8) | temporary_storage[13];
    calibration->B2 = (temporary_storage[14] << 8) | temporary_storage[15];
    calibration->MB = (temporary_storage[16] << 8) | temporary_storage[17];
    calibration->MC = (temporary_storage[18] << 8) | temporary_storage[19];
    calibration->MD = (temporary_storage[20] << 8) | temporary_storage[21];
}

static int32_t bmp085_read_raw_temperature(const bmp085_driver_descriptor_t *bmp085_dd)
{
    uint8_t temporary_storage[2];

    int32_t raw_temperature;

    i2c_read_multi(bmp085_dd->i2c_dd, bmp085_dd->i2c_address, BMP085_TEMPDATA, 2, temporary_storage);

    raw_temperature = (uint32_t)((uint32_t)(temporary_storage[0]) << 8 | temporary_storage[1]);

    bmp085_dd->calibration->last_raw_value = raw_temperature;
    //bmp085_dd->calibration->last_measurement_type = measure_temperature;

    return raw_temperature;
}

static int32_t bmp085_normalize_temperature(const bmp085_driver_descriptor_t *bmp085_dd, const int32_t temperature_raw_value)
{
     int32_t x1, x2;

     //Pointer to calibration structure
     bmp085_t *calibration = bmp085_dd->calibration;

     x1 = ((temperature_raw_value - calibration->AC6) * calibration->AC5) >> 15;
     x2 = (calibration->MC << 11) / (x1 + calibration->MD);

     //Update temperature coefficient
     calibration->B6 = x1 + x2 - 4000;

     return ((x1+x2+8) >> 4);
}




static uint32_t bmp085_read_raw_pressure(const bmp085_driver_descriptor_t *bmp085_dd)
{
    uint8_t temporary_storage[3];

    uint32_t raw_pressure;

    i2c_read_multi(bmp085_dd->i2c_dd, bmp085_dd->i2c_address, BMP085_TEMPDATA, 3, temporary_storage);

    raw_pressure = (uint32_t)( (uint32_t)(temporary_storage[0]) << 16 |
                (uint32_t)(temporary_storage[1]) << 8 |
                temporary_storage[2]) >> (8 - bmp085_dd->settings->oversampling );

    bmp085_dd->calibration->last_raw_value = raw_pressure;
    //bmp085_dd->calibration->last_measurement_type = measure_pressure;

    return raw_pressure;
}


int32_t bmp085_convert_raw_pressure_to_normalized(const bmp085_driver_descriptor_t *bmp085_dd, const uint32_t pressure_raw_value)
{
    int32_t x1, x2, x3, b3;
    uint32_t b4, b7;
    int32_t p;

    //Pointer to calibration structure
    bmp085_t *calibration = bmp085_dd->calibration;

    //Calculate pressure value
    x1 = (calibration->B6 * calibration->B6) >> 12;
    x1 *= calibration->B2;
    x1 >>= 11;

    x2 = calibration->AC2 * calibration->B6;
    x2 >>= 11;

    x3 = x1 + x2;

    b3 = ((((/*(s32)*/calibration->AC1) * 4 + x3) << (bmp085_dd->settings->oversampling) ) + 2);
    b3 >>= 2;

    x1 = (calibration->AC3 * calibration->B6) >> 13;
    x2 = (calibration->B1 * ((calibration->B6 * calibration->B6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (calibration->AC4 * (uint32_t)(x3 + 32768)) >> 15;

    b7 = (uint32_t)((uint32_t)pressure_raw_value - b3) *(50000 >> (bmp085_dd->settings->oversampling) );

    if(b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) * 2;
    }

    x1 = p >> 8;
    x1 *= x1;
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return p;
}



void bmp085_start_measurement(bmp085_driver_descriptor_t *bmp085_dd, const bmp085_measerement_t meas_type)
{
    if(meas_type == measure_pressure)
    {
        //Start pressure measurement
        i2c_write_reg8(
                bmp085_dd->i2c_dd,
                bmp085_dd->i2c_address,
                BMP085_CONTROL,
                (BMP085_READPRESSURECMD + (bmp085_dd->settings->oversampling << 6))
                );

        bmp085_dd->calibration->last_measurement_type = measure_pressure;
    } else
    if(meas_type == measure_temperature)
    {
        //Start temperature measurement
        i2c_write_reg8(
                bmp085_dd->i2c_dd,
                bmp085_dd->i2c_address,
                BMP085_CONTROL,
                BMP085_READTEMPCMD
                );

        bmp085_dd->calibration->last_measurement_type = measure_temperature;
    } else
        bmp085_dd->calibration->last_measurement_type = measure_unknown;

    //Now wait for EOC interrupt
}

//ToDo: Move to moving average related code
void bmp085_set_averaging(bmp085_driver_descriptor_t *bmp085_dd, const uint8_t averaging_count, int32_t initial_value)
{
    //ToDo: Add critical section here


    ma_deinit_dynamic(bmp085_dd->averaging);

    bmp085_dd->averaging = ma_init_dynamic(averaging_count, initial_value);

    if(bmp085_dd->averaging == NULL)
    {
        //Error
        return;
    }

//    //Free buffer memory
//    vPortFree(bmp085_dd->averaging->buf);
//
//    if(averaging_count > 1)
//    {
//        //Enable averaging
//
//        if(averaging_count != bmp085_dd->settings->averaging)
//        {
//            //Allocate memory for moving average buffer of different size
//            int32_t* averaging_buffer = (int32_t*)pvPortMalloc( averaging_count*sizeof(int32_t) );
//
//            ma_init(bmp085_dd->averaging, averaging_buffer, averaging_count, initial_value);
//
//
//        }
//
//    } else
//    {
//        //disable averaging
//
//    }

    bmp085_dd->settings->averaging = averaging_count;

}







//void bmp085_init(bmp085_t *bmp085, const bmp085_oversampling_t oversampling)
//{
//	exti_select_source(EXTI_BMP085_INT, GPIO_PORT_BMP085_INT);
//	exti_set_trigger(EXTI_BMP085_INT, EXTI_TRIGGER_RISING);
//
//	bmp085->oversampling = (uint8_t)oversampling;
//	bmp085->last_measurement_type = measure_unknown;
//
//	exti_reset_request(EXTI_BMP085_INT);
//	//Enable EOC interrupt from BMP085 in EXTI controller
//	exti_enable_request(EXTI_BMP085_INT);
//
//	bmp085_get_calibration_data(bmp085);
//}

//void bmp085_averaging_init(const uint8_t averaging_count, const int32_t initial_value)
//{
//	ma_init(&ma_bmp085, bmp085_averaging_array, averaging_count, initial_value);
//}


//uint32_t bmp085_read_temperature_raw(void)
//{
//	uint8_t tmp[2];
//
//	i2c1_read_multi(I2C_BMP085_ADDRESS, BMP085_TEMPDATA, 2, tmp);
//
//	return (uint32_t)((uint32_t)(tmp[0]) << 8 | tmp[1]);
//}
//
//uint32_t bmp085_read_pressure_raw(const bmp085_t *bmp085)
//{
//	uint8_t tmp[3];
//
//	i2c1_read_multi(I2C_BMP085_ADDRESS, BMP085_TEMPDATA, 3, tmp);
//
//	return (uint32_t)( (uint32_t)(tmp[0]) << 16 | (uint32_t)(tmp[1]) << 8 | tmp[2]) >> (8 - bmp085->oversampling);
//}



bmp085_measerement_t bmp085_get_last_measurement_type(bmp085_driver_descriptor_t *bmp085_dd)
{
	return bmp085_dd->calibration->last_measurement_type;
}

//int32_t bmp085_get_normalized_temperature(bmp085_t *bmp085, const int32_t temperature_raw_value)
//{
//     int32_t x1, x2;
//
//     x1 = ((temperature_raw_value - bmp085->AC6) * bmp085->AC5) >> 15;
//     x2 = (bmp085->MC << 11) / (x1 + bmp085->MD);
//
//     //Update temperature coefficient
//     bmp085->B6 = x1 + x2 - 4000;
//
//     return ((x1+x2+8) >> 4);
//}





int32_t bmp085_pressure2altitude_cm(const int32_t pressure_hpa)
{
	const float bmp085_sealevel_pressure = 101325.0;

    return (int32_t)(4433000 * (1 - pow((pressure_hpa / bmp085_sealevel_pressure), 0.1903)) + 0);	//cm_offset;
}

//void bmp085_update_measurement(bmp085_t *bmp085, bmp085_result_t *bmp085_data)
//{
////	bmp085_result_t bmp085_data;
//
//	if(bmp085->last_measurement_type == measure_temperature)
//	{
//		//Read raw temperature value and update thermal calibration coefficient (B6), then convert raw value to normalized
//		bmp085_data->temperature = bmp085_get_normalized_temperature(bmp085, bmp085_read_temperature_raw());
//
//		//Start pressure measurement
//		bmp085_start_measurement(bmp085, measure_pressure);
//	} else
//	if(bmp085->last_measurement_type == measure_pressure)
//	{
//		//Read raw pressure data and calculate normalized pressure in hPa
//		bmp085_data->pressure = bmp085_convert_raw_pressure_to_normalized(bmp085, bmp085_read_pressure_raw(bmp085));
//
//		bmp085_data->average_pressure = ma_add_new_value(&ma_bmp085, bmp085_data->pressure);
//
//		//Start temperature measurement
//		bmp085_start_measurement(bmp085, measure_temperature);
//	}
////	else
////	{
////		//First measurement
////	}
//
//	//If zero, do temperature measurement and re-calibration
////	if(bmp085_temperature_coefficient_expired == 0)
////	{
////		//Wait for task notification from ISR
////		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
////
////		//Start temperature measurement
////		bmp085_start_measurement(&bmp085, measure_temperature);
////
////		//Wait for task notification from ISR when temperature measurement will be finished
////		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
////
////		//Read raw temperature value and update thermal calibration coefficient (B6), then convert raw value to normalized
////		bmp085_data.temperature = bmp085_get_normalized_temperature(&bmp085, bmp085_read_temperature_raw());
////
////		//Temperature correction coefficient is OK again
////		bmp085_temperature_coefficient_expired = BMP085_TEMPERATURE_COEFFICIENT_EXPIRATION;
////
////		//Start pressure measurement
////		bmp085_start_measurement(&bmp085, measure_pressure);
////
////	} else
////	{
////		bmp085_temperature_coefficient_expired--;
////	}
////
////	//Wait for task notification from ISR
////	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
////
////	//Add measured raw pressure value to moving average array
////	bmp085_moving_average_add_new_value( bmp085_read_pressure_raw(&bmp085) );
////
////	//Start next measurement
////	bmp085_start_measurement(&bmp085, measure_pressure);
////
////	//Calculate normalized pressure in hPa
////	bmp085_data.pressure = bmp085_get_normalized_pressure(&bmp085, bmp085_get_moving_average_value());
////
////	bmp085_altitude = bmp085_pressure2altitude_cm(bmp085_data.pressure);
//}

//void bmp085_moving_average_add_new_value(const int32_t new_value)
//{
//	//Substract oldest array element value from sum
//	bmp085_averaging_array_sum -= bmp085_averaging_array[bmp085_averaging_index];
//
//	//Add new element to array at current index
//	bmp085_averaging_array[bmp085_averaging_index] = new_value;
//
//	//Add most recent element's value to sum
//	bmp085_averaging_array_sum += new_value;
//
//	//Increment index
//	bmp085_averaging_index++;
//
//	if(bmp085_averaging_index == BMP085_AVERAGING_ARRAY_SIZE)
//	{
//		//Move index to start of the array
//		bmp085_averaging_index = 0;
//	}
//}
//
//int32_t bmp085_get_moving_average_value(void)
//{
//	//Calculate and return average value
//	return (bmp085_averaging_array_sum / BMP085_AVERAGING_ARRAY_SIZE);
//}
//
//void bmp085_init_moving_average_array(const int32_t initial_value)
//{
//	uint8_t i;
//
//	//Initial fill of averaging array and sum calculation
//	bmp085_averaging_array_sum = 0;
//
//	i = 0;
//
//	while(i < BMP085_AVERAGING_ARRAY_SIZE)
//	{
//		bmp085_averaging_array[i] = initial_value;
//		bmp085_averaging_array_sum += initial_value;
//		i++;
//	}
//}


