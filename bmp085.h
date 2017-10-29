/*
 * bmp085.h
 *
 *  Created on: 31 дек. 2016 г.
 *      Author: frost
 */

#ifndef BMP085_H_
#define BMP085_H_

#include <stdint.h>

#include "mcu/mcu_i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#include "helpers/movingaverage.h"

extern volatile TaskHandle_t BMP085_Task_Handle;

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD          0x2E
#define BMP085_READPRESSURECMD            0x34

// Bosch BMP085 chip ID
#define BMP085_CHIP_ID   		85    // Bosch BMP085 chip identifier - not mentioned in datasheet but found in example source code

// register addresses
#define BMP085_REG_ID			0xD0  // chip ID register
#define BMP085_REG_VER			0xD1 // chip version register

#define BMP085_AVERAGING_ARRAY_SIZE 	8

//We need to re-read temperature and re-calibrate every .. pressure readings
#define BMP085_TEMPERATURE_COEFFICIENT_EXPIRATION 	8

typedef enum
{
	bmp085_oversampling_disabled = 0,
	bmp085_oversampling_1 = 1,
	bmp085_oversampling_2 = 2,
	bmp085_oversampling_3 = 3
} bmp085_oversampling_t;

typedef enum
{
	measure_unknown = 0,
	measure_temperature = 1,
	measure_pressure = 2
} bmp085_measerement_t;

typedef struct
{
	//Calibration data to be read from each device during initialization
	int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2;
    int16_t MB, MC, MD;

    //Temperature correction coefficient calculated by software, updated on each temperature measurement
    int32_t B6;

    //Last measurement type - pressure or temperature
    bmp085_measerement_t last_measurement_type;

    //Last measurement raw value
    uint32_t last_raw_value;

    //Oversampling setting
//    uint8_t oversampling;

} bmp085_t;

typedef struct
{
    //Last measurement type - pressure or temperature
//    bmp085_measerement_t last_measurement_type;

    //Interrupt handler priority in NVIC format
    const uint8_t interrupt_priority;

    //Last measurement raw value
//    uint32_t last_raw_value;

    //Oversampling count
    uint8_t oversampling;

    //Averaging count, disabled, if set to less than 2
    uint8_t averaging;

} bmp085_settings_t;

typedef struct
{
	int16_t temperature;
	int32_t pressure;
	int32_t average_pressure;
} bmp085_result_t;



typedef struct
{
    const char* nameStr;  //"BMP085"

    const uint8_t i2c_address;

    //Pointer to driver descriptor of I2C bus
    //current instance is connected to
    const i2c_driver_descriptor_t *i2c_dd;

    //GPIOs used by interface
    const gpio_t *gpio_eoc;     //End of conversion output

    //Pointer to END OF CONVERSION callback function
    //Would be automatically registered in EXTI manager if not NULL
    const void* eoc_callback;

//    //Interrupt handler priority in NVIC format
//    const uint8_t priority;

    //Pointer to calibration and settings structure
    volatile bmp085_t* calibration;

    //Settings structure
    volatile bmp085_settings_t *settings;

    //Pointer to moving average structure
    //Averaging disabled, if NULL
    volatile movingaverage_t *averaging;

} bmp085_driver_descriptor_t;


//End of conversion
__inline__ void __attribute__((always_inline)) bmp085_eoc_callback(BaseType_t *pxHigherPriorityTaskWoken)
{
	vTaskNotifyGiveFromISR( BMP085_Task_Handle, pxHigherPriorityTaskWoken );
}

uint8_t bmp085_register_driver(bmp085_driver_descriptor_t *bmp085_dd);

int32_t bmp085_convert_raw_pressure_to_normalized(const bmp085_driver_descriptor_t *bmp085_dd, const uint32_t pressure_raw_value);

void bmp085_start_measurement(bmp085_driver_descriptor_t *bmp085_dd, const bmp085_measerement_t meas_type);

void bmp085_set_averaging(bmp085_driver_descriptor_t *bmp085_dd, const uint8_t averaging_count, int32_t initial_value);

bmp085_measerement_t bmp085_get_last_measurement_type(bmp085_driver_descriptor_t *bmp085_dd);

int32_t bmp085_pressure2altitude_cm(const int32_t pressure_hpa);


//void bmp085_init(bmp085_t *bmp085, const bmp085_oversampling_t oversampling);
//void bmp085_averaging_init(const uint8_t averaging_count, const int32_t initial_value);

//uint32_t bmp085_read_pressure_raw(const bmp085_t *bmp085);
//uint32_t bmp085_read_temperature_raw(void);

//int32_t bmp085_get_normalized_temperature(bmp085_t *bmp085, const int32_t temperature_raw_value);


#endif /* BMP085_H_ */
