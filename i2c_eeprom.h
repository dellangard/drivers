/*
 * i2c_eeprom.h
 *
 *  Created on: 17 окт. 2017 г.
 *      Author: frost
 */

#ifndef DRIVERS_I2C_EEPROM_H_
#define DRIVERS_I2C_EEPROM_H_

typedef struct
{
    const char* nameStr;  //"AT24C02"

    const uint8_t i2c_address;

    //Pointer to driver descriptor of I2C bus
    const i2c_driver_descriptor_t *i2c_dd;

    //Write protect control GPIO - set to NULL if not used
    const gpio_t *gpio_wp;

    const uint16_t eeprom_size;

    const uint8_t eeprom_page_size;
    const uint8_t eeprom_page_count;

} at24xx_driver_descriptor_t;

#endif /* DRIVERS_I2C_EEPROM_H_ */
