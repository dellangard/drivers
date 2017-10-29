/*
 * mcu_i2c.h
 *
 *  Created on: 5 янв. 2017 г.
 *      Author: frost
 */

#ifndef MCU_I2C_H_
#define MCU_I2C_H_

#include "mcu_gpio.h"

#include "FreeRTOS.h"
#include "semphr.h"

/* I2C FLAG mask */
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)

/**
  * @brief  Checks whether the last I2Cx Event is equal to the one passed
  *   as parameter.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_EVENT: specifies the event to be checked.
  *   This parameter can be one of the following values:
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED           : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED              : EV1
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED     : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED        : EV1
  *     @arg I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED            : EV1
  *     @arg I2C_EVENT_SLAVE_BYTE_RECEIVED                         : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF)      : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL)    : EV2
  *     @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED                      : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF)   : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) : EV3
  *     @arg I2C_EVENT_SLAVE_ACK_FAILURE                           : EV3_2
  *     @arg I2C_EVENT_SLAVE_STOP_DETECTED                         : EV4
  *     @arg I2C_EVENT_MASTER_MODE_SELECT                          : EV5
  *     @arg I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6
  *     @arg I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
  *     @arg I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
  *     @arg I2C_EVENT_MASTER_MODE_ADDRESS10                       : EV9
  * @retval An ErrorStatus enumuration value:
  * - SUCCESS: Last event is equal to the I2C_EVENT
  * - ERROR: Last event is different from the I2C_EVENT
  */

/*========================================

                     I2C Master Events (Events grouped in order of communication)
                                                        ==========================================*/
/**
  * @brief  Communication start
  *
  * After sending the START condition (I2C_GenerateSTART() function) the master
  * has to wait for this event. It means that the Start condition has been correctly
  * released on the I2C bus (the bus is free, no other devices is communicating).
  *
  */
/* --EV5 */
#define  I2C_EVENT_MASTER_MODE_SELECT                      ((uint32_t)0x00030001)  /* BUSY, MSL and SB flag */

/**
  * @brief  Address Acknowledge
  *
  * After checking on EV5 (start condition correctly released on the bus), the
  * master sends the address of the slave(s) with which it will communicate
  * (I2C_Send7bitAddress() function, it also determines the direction of the communication:
  * Master transmitter or Receiver). Then the master has to wait that a slave acknowledges
  * his address. If an acknowledge is sent on the bus, one of the following events will
  * be set:
  *
  *  1) In case of Master Receiver (7-bit addressing): the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED
  *     event is set.
  *
  *  2) In case of Master Transmitter (7-bit addressing): the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED
  *     is set
  *
  *  3) In case of 10-Bit addressing mode, the master (just after generating the START
  *  and checking on EV5) has to send the header of 10-bit addressing mode (I2C_SendData()
  *  function). Then master should wait on EV9. It means that the 10-bit addressing
  *  header has been correctly sent on the bus. Then master should send the second part of
  *  the 10-bit address (LSB) using the function I2C_Send7bitAddress(). Then master
  *  should wait for event EV6.
  *
  */

/* --EV6 */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED           ((uint32_t)0x00030002)  /* BUSY, MSL and ADDR flags */
/* --EV9 */
#define  I2C_EVENT_MASTER_MODE_ADDRESS10                   ((uint32_t)0x00030008)  /* BUSY, MSL and ADD10 flags */

/**
  * @brief Communication events
  *
  * If a communication is established (START condition generated and slave address
  * acknowledged) then the master has to check on one of the following events for
  * communication procedures:
  *
  * 1) Master Receiver mode: The master has to wait on the event EV7 then to read
  *    the data received from the slave (I2C_ReceiveData() function).
  *
  * 2) Master Transmitter mode: The master has to send data (I2C_SendData()
  *    function) then to wait on event EV8 or EV8_2.
  *    These two events are similar:
  *     - EV8 means that the data has been written in the data register and is
  *       being shifted out.
  *     - EV8_2 means that the data has been physically shifted out and output
  *       on the bus.
  *     In most cases, using EV8 is sufficient for the application.
  *     Using EV8_2 leads to a slower communication but ensure more reliable test.
  *     EV8_2 is also more suitable than EV8 for testing on the last data transmission
  *     (before Stop condition generation).
  *
  *  @note In case the  user software does not guarantee that this event EV7 is
  *  managed before the current byte end of transfer, then user may check on EV7
  *  and BTF flag at the same time (ie. (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)).
  *  In this case the communication may be slower.
  *
  */

/* Master RECEIVER mode -----------------------------*/
/* --EV7 */
#define  I2C_EVENT_MASTER_BYTE_RECEIVED                    ((uint32_t)0x00030040)  /* BUSY, MSL and RXNE flags */

/* Master TRANSMITTER mode --------------------------*/
/* --EV8 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                 ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */
/* --EV8_2 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTED ((uint32_t)0x00070084) /* TRA, BUSY, MSL, TXE and BTF flags */


typedef enum
{
    i2c_speed_100k = 100,
    i2c_speed_400k = 400

} i2c_speed_t;

//Settings structure
typedef struct
{
    //Interface speed - 100 or 400 kHz
    i2c_speed_t speed;

    //Interface timeout - for waiting semaphore and error handling
    uint32_t semaphore_timeout;
} i2c_settings_t;


typedef struct
{
    SemaphoreHandle_t xSemaphoreI2CReady;
    SemaphoreHandle_t xSemaphoreI2CReadyMutex;

    uint8_t slave_address;

    uint8_t *writemem;
    uint8_t *readmem;
    uint8_t bytes_to_write;
    uint8_t bytes_to_read;

    uint8_t is_busy;

    uint32_t prev_state;

} i2c_io_t;


typedef struct
{
    const char* nameStr;  //"I2Cx"

    const uint32_t i2c_base;

    //GPIOs used by interface
    const gpio_t *gpio_scl;
    const gpio_t *gpio_sda;

    //Pointer to settings structure
    const i2c_settings_t* settings;

    uint8_t i2c_num;

    //Pointer to interface IO structure
    i2c_io_t* io;

} i2c_driver_descriptor_t;


void i2c_register_driver(i2c_driver_descriptor_t *i2c_dd);
uint8_t i2c_read_reg8(const i2c_driver_descriptor_t *i2c_dd, const uint8_t slave_address, const uint8_t reg);
void i2c_write_reg8(const i2c_driver_descriptor_t *i2c_dd, const uint8_t slave_address, const uint8_t reg, const uint8_t data);
void i2c_read_multi(const i2c_driver_descriptor_t *i2c_dd, const uint8_t slave_address, const uint8_t reg_start, const uint8_t count, uint8_t *ptr_read_buffer);


#endif /* MCU_I2C_H_ */
