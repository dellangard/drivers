/*
 * mcu_spi.h
 *
 *  Created on: 5 янв. 2017 г.
 *      Author: frost
 */

#ifndef MCU_SPI_H_
#define MCU_SPI_H_

#include <libopencm3/stm32/spi.h>
#include "mcu_gpio.h"
#include "mcu_spi.h"
#include "../dma_manager.h"

#include "FreeRTOS.h"
#include "semphr.h"

typedef enum
{
    spi_dma_disabled = 0,
    spi_dma_tx = 1,
    spi_dma_rx = 2,
    spi_dma_rxtx = 3
} spi_dma_mode_t;

typedef enum
{
    spi_ff_msb = 0,
    spi_ff_lsb = 1
} spi_frame_format_t;

//Settings structure
typedef struct
{
//    spi_speed_t speed;
//    spi_mode_t spi_mode

    //Interface timeout - for waiting semaphore and error handling
    uint32_t semaphore_timeout;
} spi_settings_t;

typedef struct
{
    volatile SemaphoreHandle_t xSemaphoreSPIReady;
    volatile SemaphoreHandle_t xSemaphoreSPIReadyMutex;

    volatile uint8_t *writemem;
    volatile uint8_t *readmem;
    volatile uint8_t bytes_to_write;
    volatile uint8_t bytes_to_read;

    volatile spi_dma_mode_t dma_mode;

    volatile uint32_t dma_ctrl_base;

} spi_io_t;


typedef struct
{
    const char* nameStr;  //"SPIx"

    const uint32_t spi_base;

    //GPIOs used by interface
    const gpio_t *gpio_sck;
    const gpio_t *gpio_mosi;
    const gpio_t *gpio_miso;
    const gpio_t *gpio_nss;

    //Pointer to settings structure
    volatile spi_settings_t* settings;

    //Pointer to interface IO structure
    volatile spi_io_t* io;

    //Pointers to callback functions
    const void* callback_dma_tx;
//    const void* callback_dma_rx;

    const dma_config_t *dma_tx_config;
    const dma_config_t *dma_rx_config;

} spi_driver_descriptor_t;


void spi1_tx_dma_callback(void);
void spi2_tx_dma_callback(void);
void spi3_tx_dma_callback(void);
void spi4_tx_dma_callback(void);
void spi5_tx_dma_callback(void);
void spi6_tx_dma_callback(void);

void spi_set_frame_format(spi_driver_descriptor_t *spi_dd, const spi_frame_format_t frame_format);

void spi_register_driver(spi_driver_descriptor_t *spi_dd);

void spi_get_mutex(spi_driver_descriptor_t *spi_dd);

void spi_release_mutex(spi_driver_descriptor_t *spi_dd);

static int8_t spi_start_master_transaction(spi_driver_descriptor_t *spi_dd,
        const uint8_t wb_count, const uint8_t *write_array,
        const uint8_t rb_count, uint8_t *read_array);


uint8_t spi_read_single_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr);

void spi_read_multi_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr, const uint8_t count, uint8_t *ptr_read_buffer);

uint8_t spi_rw_single8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t data);

void spi_write_single_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr, const uint8_t data);

void spi_write_multi_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr, const uint8_t count, uint8_t *ptr_write_buffer);

void spi_dma_write_multi_bytes(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t count, uint8_t *ptr_write_buffer);


#endif /* MCU_SPI_H_ */
