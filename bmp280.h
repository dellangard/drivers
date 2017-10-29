/*
 * bmp280.h
 *
 *  Created on: 3 окт. 2017 г.
 *      Author: frost
 */

#ifndef DRIVERS_BMP280_H_
#define DRIVERS_BMP280_H_

#include "mcu/mcu_i2c.h"

#include "FreeRTOS.h"
#include "task.h"


enum bmp280_oversampling
{
    bmp280_oversampling_0 = 0,
    bmp280_oversampling_1 = 1,
    bmp280_oversampling_2 = 2,
    bmp280_oversampling_4 = 3,
    bmp280_oversampling_8 = 4,
    bmp280_oversampling_16 = 5
};

enum bmp280_stanby_time_ms
{
    bmp280_sb_0ms5 = 0,
    bmp280_sb_62ms5 = 1,
    bmp280_sb_125ms = 2,
    bmp280_sb_250ms = 3,
    bmp280_sb_500ms = 4,
    bmp280_sb_1000ms = 5,
    bmp280_sb_2000ms = 6,
    bmp280_sb_4000ms = 7
};

enum bmp280_iir_filter_coef
{
    bmp280_filter_off = 0,
    bmp280_filter_coeff_2 = 1,
    bmp280_filter_coeff_4 = 2,
    bmp280_filter_coeff_8 = 3,
    bmp280_filter_coeff_16 = 4
};

typedef struct
{
    uint16_t dig_T1;    //0x88
    int16_t dig_T2;     //0x8A
    int16_t dig_T3;     //0x8C
    uint16_t dig_P1;    //0x8E
    int16_t dig_P2;     //0x90
    int16_t dig_P3;     //0x92
    int16_t dig_P4;     //0x94
    int16_t dig_P5;     //0x96
    int16_t dig_P6;     //0x98
    int16_t dig_P7;     //0x9A
    int16_t dig_P8;     //0x9C
    int16_t dig_P9;     //0x9E

} bmp280_calibration_t;


typedef struct
{
    const char* nameStr;  //"BMP280"

    const uint8_t i2c_address;

    //Pointer to driver descriptor of I2C bus
    const i2c_driver_descriptor_t *i2c_dd;

    //Pointer to structure storing factory calibration values
    volatile bmp280_calibration_t* calibration;

    //Variable used to store intermediate temperature calculation result for further pressure calculations
    volatile double t_fine;

    //Various BMP280 settings
    volatile enum bmp280_oversampling oversampling_temp;
    volatile enum bmp280_oversampling oversampling_press;
    volatile enum bmp280_stanby_time_ms standby_time;
    volatile enum bmp280_iir_filter_coef iir_filter_coef;

} bmp280_driver_descriptor_t;


enum driver_register_result bmp280_register_driver(bmp280_driver_descriptor_t *bmp280_dd);

void bmp280_set_oversampling_temp(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_oversampling oversampling);
void bmp280_set_oversampling_press(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_oversampling oversampling);
void bmp280_set_iir_filter_coef(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_iir_filter_coef filter_coef);
void bmp280_set_standby_time_ms(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_iir_filter_coef standby_time_ms);

void bmp280_read_raw_values(bmp280_driver_descriptor_t *bmp280_dd, double *uP, double *uT);

double bmp280_calculate_temperature(bmp280_driver_descriptor_t *bmp280_dd, double raw_temp);
double bmp280_calculate_pressure(bmp280_driver_descriptor_t *bmp280_dd, double raw_press);
double bmp280_calculate_altitude(double pressure_measured, double presssure_baseline);
double bmp280_calculate_sealevel_pressure(bmp280_driver_descriptor_t *bmp280_dd, double pressure_measured, double altitude);


#endif /* DRIVERS_BMP280_H_ */
