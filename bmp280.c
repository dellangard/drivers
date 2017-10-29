/*
 * bmp280.c
 *
 *  Created on: 3 окт. 2017 г.
 *      Author: frost
 */

#include <math.h>

#include "mcu/mcu_i2c.h"
#include "driver_manager.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "bmp280.h"

// BMP280 registers

// Chip status register
#define BMP280_REG_STATUS       (uint8_t)( 0xF3 )

// Chip ID register
#define BMP280_REG_ID           (uint8_t)( 0xD0 )
//Value to be read out from BMP280_REG_ID register
#define BMP280_CHIP_ID                  (uint8_t)( 0x58 )    // Bosch BMP280 chip identifier


// Chip reset register - write 0xB6 here to reset device
#define BMP280_REG_RESET        (uint8_t)( 0xE0 )


//Temperature conversion result registers
#define BMP280_REG_TEMP_XLSB    (uint8_t)( 0xFC )
#define BMP280_REG_TEMP_LSB     (uint8_t)( 0xFB )
#define BMP280_REG_TEMP_MSB     (uint8_t)( 0xFA )

//Pressure conversion result registers
#define BMP280_REG_PRESS_XLSB   (uint8_t)( 0xF9 )
#define BMP280_REG_PRESS_LSB    (uint8_t)( 0xF8 )
#define BMP280_REG_PRESS_MSB    (uint8_t)( 0xF7 )


//Calibration table registers
#define BMP280_CALIB_T1           (uint8_t)( 0x88 )  // R   Calibration data (16 bits)
#define BMP280_CALIB_T2           (uint8_t)( 0x8A )  // R   Calibration data (16 bits)
#define BMP280_CALIB_T3           (uint8_t)( 0x8C )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P1           (uint8_t)( 0x8E )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P2           (uint8_t)( 0x90 )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P3           (uint8_t)( 0x92 )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P4           (uint8_t)( 0x94 )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P5           (uint8_t)( 0x96 )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P6           (uint8_t)( 0x98 )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P7           (uint8_t)( 0x9A )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P8           (uint8_t)( 0x9C )  // R   Calibration data (16 bits)
#define BMP280_CALIB_P9           (uint8_t)( 0x9E )  // R   Calibration data (16 bits)


//Control register
#define BMP280_REG_CTRL_MEAS    (uint8_t)( 0xF4 )

#define BMP280_REG_CTRL_MEAS_MODE          (uint8_t)(3 << 0)
#define BMP280_REG_CTRL_MEAS_MODE_SLEEP    (uint8_t)(0 << 0)
#define BMP280_REG_CTRL_MEAS_MODE_NORMAL    (uint8_t)(3 << 0)
#define BMP280_REG_CTRL_MEAS_MODE_FORCED    (uint8_t)(1 << 0)

#define BMP280_REG_CTRL_MEAS_OSRS_P_MASK          (uint8_t)(7)
#define BMP280_REG_CTRL_MEAS_OSRS_P_OFFSET          (uint8_t)(2)

#define BMP280_REG_CTRL_MEAS_OSRS_P0          (uint8_t)(0 << 2)
#define BMP280_REG_CTRL_MEAS_OSRS_P1          (uint8_t)(1 << 2)
#define BMP280_REG_CTRL_MEAS_OSRS_P2          (uint8_t)(2 << 2)
#define BMP280_REG_CTRL_MEAS_OSRS_P4          (uint8_t)(3 << 2)
#define BMP280_REG_CTRL_MEAS_OSRS_P8          (uint8_t)(4 << 2)
#define BMP280_REG_CTRL_MEAS_OSRS_P16          (uint8_t)(5 << 2)


#define BMP280_REG_CTRL_MEAS_OSRS_T_MASK          (uint8_t)(7)
#define BMP280_REG_CTRL_MEAS_OSRS_T_OFFSET          (uint8_t)(5)

#define BMP280_REG_CTRL_MEAS_OSRS_T0          (uint8_t)(0 << 5)
#define BMP280_REG_CTRL_MEAS_OSRS_T1          (uint8_t)(1 << 5)
#define BMP280_REG_CTRL_MEAS_OSRS_T2          (uint8_t)(2 << 5)
#define BMP280_REG_CTRL_MEAS_OSRS_T4          (uint8_t)(3 << 5)
#define BMP280_REG_CTRL_MEAS_OSRS_T8          (uint8_t)(4 << 5)
#define BMP280_REG_CTRL_MEAS_OSRS_T16          (uint8_t)(5 << 5)


//Configuration register
#define BMP280_REG_CONFIG       (uint8_t)( 0xF5 )

#define BMP280_REG_CONFIG_3WIRE_SPI      (uint8_t)(1 << 0)

#define BMP280_REG_CONFIG_IIR_FILT_MASK      (uint8_t)(7)
#define BMP280_REG_CONFIG_IIR_FILT_OFFSET      (uint8_t)(2)

#define BMP280_REG_CONFIG_IIR_FILT_OFF  (uint8_t)(0 << 2)   //Full bandwith
#define BMP280_REG_CONFIG_IIR_FILT_2    (uint8_t)(1 << 2)   //0.223 x ODR
#define BMP280_REG_CONFIG_IIR_FILT_4    (uint8_t)(2 << 2)   //0.092 x ODR
#define BMP280_REG_CONFIG_IIR_FILT_8    (uint8_t)(3 << 2)   //0.042 x ODR
#define BMP280_REG_CONFIG_IIR_FILT_16   (uint8_t)(4 << 2)   //0.021 x ODR


#define BMP280_REG_CONFIG_T_SB_MASK     (uint8_t)(7)
#define BMP280_REG_CONFIG_T_SB_OFFSET   (uint8_t)(5)

#define BMP280_REG_CONFIG_T_SB0_5       (uint8_t)(0 << 5)
#define BMP280_REG_CONFIG_T_SB62_5      (uint8_t)(1 << 5)
#define BMP280_REG_CONFIG_T_SB125       (uint8_t)(2 << 5)
#define BMP280_REG_CONFIG_T_SB250       (uint8_t)(3 << 5)
#define BMP280_REG_CONFIG_T_SB500       (uint8_t)(4 << 5)
#define BMP280_REG_CONFIG_T_SB1000      (uint8_t)(5 << 5)
#define BMP280_REG_CONFIG_T_SB2000      (uint8_t)(6 << 5)
#define BMP280_REG_CONFIG_T_SB4000      (uint8_t)(7 << 5)


static enum driver_register_result bmp280_get_calibration_data(bmp280_driver_descriptor_t *bmp280_dd)
{
    uint8_t data[24];

    //Allocate memory fot calibration structure
    bmp280_calibration_t *calibration = (bmp280_calibration_t*)pvPortMalloc(sizeof(bmp280_calibration_t));

    //Memory allocation failed
    if(calibration == NULL)
        return driver_register_memory_allocation_failed;

    bmp280_dd->calibration = calibration;

    //Read 24 bytes from BMP280 starting from BMP280_CALIB_T1 register to data[]
    i2c_read_multi(bmp280_dd->i2c_dd, bmp280_dd->i2c_address, BMP280_CALIB_T1, 24, data);

    //Update calibration settings
    calibration->dig_T1 = (data[1] << 8) | data[0];
    calibration->dig_T2 = (data[3] << 8) | data[2];
    calibration->dig_T3 = (data[5] << 8) | data[4];
    calibration->dig_P1 = (data[7] << 8) | data[6];
    calibration->dig_P2 = (data[9] << 8) | data[8];
    calibration->dig_P3 = (data[11] << 8) | data[10];
    calibration->dig_P4 = (data[13] << 8) | data[12];
    calibration->dig_P5 = (data[15] << 8) | data[14];
    calibration->dig_P6 = (data[17] << 8) | data[16];
    calibration->dig_P7 = (data[19] << 8) | data[18];
    calibration->dig_P8 = (data[21] << 8) | data[20];
    calibration->dig_P9 = (data[23] << 8) | data[22];

    return driver_register_ok;
}

enum driver_register_result bmp280_register_driver(bmp280_driver_descriptor_t *bmp280_dd)
{
    //Check if BMP280 present on the I2C bus
    if(i2c_read_reg8(bmp280_dd->i2c_dd, bmp280_dd->i2c_address, BMP280_REG_ID) == BMP280_CHIP_ID)
    {
        vTaskDelay(30);

        //Read calibration data from device
        bmp280_get_calibration_data(bmp280_dd);

        //Set normal mode
        i2c_write_reg8(bmp280_dd->i2c_dd, bmp280_dd->i2c_address,
            BMP280_REG_CTRL_MEAS,
            BMP280_REG_CTRL_MEAS_MODE_NORMAL |
            (bmp280_dd->oversampling_press << BMP280_REG_CTRL_MEAS_OSRS_P_OFFSET) |
            (bmp280_dd->oversampling_temp << BMP280_REG_CTRL_MEAS_OSRS_T_OFFSET)
            );

        //Set t_SB period
        i2c_write_reg8(bmp280_dd->i2c_dd, bmp280_dd->i2c_address,
            BMP280_REG_CONFIG,
            bmp280_dd->standby_time << BMP280_REG_CONFIG_T_SB_OFFSET |
            bmp280_dd->iir_filter_coef << BMP280_REG_CONFIG_IIR_FILT_OFFSET
            );

        //Register driver in Driver Manager
        driver_manager_register_driver(bmp280_dd->nameStr, bmp280_dd);

    } else
        return driver_register_device_not_found;

    return driver_register_ok;
}


void bmp280_set_oversampling_temp(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_oversampling oversampling)
{
    bmp280_dd->oversampling_temp = oversampling;

    i2c_write_reg8(bmp280_dd->i2c_dd, bmp280_dd->i2c_address,
            BMP280_REG_CTRL_MEAS,
            BMP280_REG_CTRL_MEAS_MODE_NORMAL |
            (bmp280_dd->oversampling_press << BMP280_REG_CTRL_MEAS_OSRS_P_OFFSET) |
            (bmp280_dd->oversampling_temp << BMP280_REG_CTRL_MEAS_OSRS_T_OFFSET)
            );
}

void bmp280_set_oversampling_press(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_oversampling oversampling)
{
    bmp280_dd->oversampling_press = oversampling;

    i2c_write_reg8(bmp280_dd->i2c_dd, bmp280_dd->i2c_address,
            BMP280_REG_CTRL_MEAS,
            BMP280_REG_CTRL_MEAS_MODE_NORMAL |
            (bmp280_dd->oversampling_press << BMP280_REG_CTRL_MEAS_OSRS_P_OFFSET) |
            (bmp280_dd->oversampling_temp << BMP280_REG_CTRL_MEAS_OSRS_T_OFFSET)
            );
}

void bmp280_set_iir_filter_coef(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_iir_filter_coef filter_coef)
{
    bmp280_dd->iir_filter_coef = filter_coef;

    i2c_write_reg8(bmp280_dd->i2c_dd, bmp280_dd->i2c_address,
            BMP280_REG_CONFIG,
            bmp280_dd->standby_time << BMP280_REG_CONFIG_T_SB_OFFSET |
            bmp280_dd->iir_filter_coef << BMP280_REG_CONFIG_IIR_FILT_OFFSET
            );
}

void bmp280_set_standby_time_ms(bmp280_driver_descriptor_t *bmp280_dd, enum bmp280_iir_filter_coef standby_time_ms)
{
    bmp280_dd->standby_time = standby_time_ms;

    i2c_write_reg8(bmp280_dd->i2c_dd, bmp280_dd->i2c_address,
            BMP280_REG_CONFIG,
            bmp280_dd->standby_time << BMP280_REG_CONFIG_T_SB_OFFSET |
            bmp280_dd->iir_filter_coef << BMP280_REG_CONFIG_IIR_FILT_OFFSET
            );
}


void bmp280_read_raw_values(bmp280_driver_descriptor_t *bmp280_dd, double *uP, double *uT)
{
    uint8_t data[6];

    i2c_read_multi(bmp280_dd->i2c_dd, bmp280_dd->i2c_address, BMP280_REG_PRESS_MSB, 6, data);

    *uP = (double)(data[0]*4096 + data[1]*16 + data[2]/16) ; //20bit uncompensated pressure value

    *uT = (double)(data[3]*4096 + data[4]*16 + data[5]/16) ; //20bit uncompensated temperature value
}

double bmp280_calculate_temperature(bmp280_driver_descriptor_t *bmp280_dd, double raw_temp)
{
    double var1 = ((double)raw_temp/16384.0 - (double)(bmp280_dd->calibration->dig_T1)/1024.0)*(double)(bmp280_dd->calibration->dig_T2);
    double var2 = (((double)raw_temp/131072.0 - (double)(bmp280_dd->calibration->dig_T1)/8192.0)*( (double)raw_temp/131072.0 - (double)(bmp280_dd->calibration->dig_T1)/8192.0) )*(double)(bmp280_dd->calibration->dig_T3);

    bmp280_dd->t_fine = var1+var2;

    return (bmp280_dd->t_fine/5120.0);
}

double bmp280_calculate_pressure(bmp280_driver_descriptor_t *bmp280_dd, double raw_press)
{
    double P;
    double var1;
    double var2;

    var1 = (bmp280_dd->t_fine/2.0) - 64000.0;
    var2 = var1 * (var1 * bmp280_dd->calibration->dig_P6/32768.0); //not overflow
    var2 = var2 + (var1 * bmp280_dd->calibration->dig_P5 * 2.0); //overflow
    var2 = (var2 / 4.0)+((bmp280_dd->calibration->dig_P4)*65536.0);
    var1 = (bmp280_dd->calibration->dig_P3 * var1 * var1 / 524288.0 + bmp280_dd->calibration->dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * bmp280_dd->calibration->dig_P1;

    P = 1048576.0 - raw_press;
    P = ( P - (var2 / 4096.0) )*6250.0/var1 ; //overflow
    var1 = bmp280_dd->calibration->dig_P9*P*P/2147483648.0; //overflow
    var2 = P*bmp280_dd->calibration->dig_P8/32768.0;
    P = P + (var1+var2+bmp280_dd->calibration->dig_P7)/16.0;
    P = P/100.0 ;

    return P;
}


double bmp280_calculate_altitude(double pressure_measured, double pressure_baseline)
{
    //ToDo: Replace pow() to reentrant version
    return (double)( 44330.0*(1 - pow(pressure_measured/pressure_baseline, 1.0/5.255) ) );
}

double bmp280_calculate_sealevel_pressure(bmp280_driver_descriptor_t *bmp280_dd, double pressure_measured, double altitude)
{
    //ToDo: Replace pow() to reentrant version
    return (double)( pressure_measured/pow(1 - (altitude/44330.0), 5.255) );
}


