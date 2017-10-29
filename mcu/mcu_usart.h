/*
 * mcu_usart.h
 *
 *  Created on: 5 янв. 2017 г.
 *      Author: frost
 */

#ifndef MCU_USART_H_
#define MCU_USART_H_

#include "mcu_gpio.h"
#include "../dma_manager.h"

#include "FreeRTOS.h"
#include "semphr.h"

//UART signals with modem control lines
//Assign NULL if line is not used
typedef struct
{
    gpio_t *tx;
    gpio_t *rx;
    gpio_t *cts;
    gpio_t *rts;
    gpio_t *dtr;
    gpio_t *dsr;
    gpio_t *ri;
} uart_hw_connection_t;


enum uart_mode
{
    uart_mode_normal_uart = 0,
    uart_mode_halfduplex_rs485 = 1,
    uart_mode_full_duplex_rs485 = 2,
    uart_mode_lin = 3
};

enum uart_lin_break_length
{
    uart_lin_break_length_10b = 0,
    uart_lin_break_length_11b = 1
};

typedef struct
{
    enum uart_lin_break_length lbd;


} uart_lin_settings_t;

typedef struct
{
    uint32_t baudrate;
    uint32_t databits;
    uint32_t parity;
    uint32_t stopbits;
    uint32_t mode;
    uint32_t flowcontrol;

    enum uart_mode bus_mode;

} uart_settings_t;

typedef struct
{
    const char* nameStr;  //"USARTx"

    const uint32_t uart_base;

    const uart_hw_connection_t hw;
    const uart_settings_t settings;

    const uart_lin_settings_t *lin;

    //Pointers to callback functions
    const void (*callback_rx)(void*);
    const void (*callback_tx)(void*);

    const void (*callback_dma_rx_transfer_finished)(void*);
    const void (*callback_dma_tx_transfer_finished)(void);


    const dma_config_t *dma_tx_config;
    const dma_config_t *dma_rx_config;

    /*volatile*/ BaseType_t *pxHigherPriorityTaskWoken;

    volatile SemaphoreHandle_t xSemaphoreTXFinished;

} uart_driver_descriptor_t;

void usart_txe_dma_transfer_finished_callback(uart_driver_descriptor_t *uart_dd);

void usart1_tx_dma_callback(void);
void usart2_tx_dma_callback(void);
void usart3_tx_dma_callback(void);
void uart4_tx_dma_callback(void);
void uart5_tx_dma_callback(void);
void usart6_tx_dma_callback(void);

void uart_register_driver(uart_driver_descriptor_t *uart_dd);


uint8_t usart_writestr_blocking(const uart_driver_descriptor_t *uart_dd, const char *str);

#endif /* MCU_USART_H_ */



