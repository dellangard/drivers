
#include <libopencmsis/dispatch/irqhandlers.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include "libopencm3_headers.h"

#include "mcu_usart.h"
#include "../dma_manager.h"

#include "buildparams.h"
#include <limits.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


static uart_driver_descriptor_t *usart_dd_pointers[] = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL
};


static void uart_default_handler(uart_driver_descriptor_t *uart_dd);


void usart_txe_dma_transfer_finished_callback(uart_driver_descriptor_t *uart_dd)
{
    //Disable USART TX interrupt request
    usart_disable_tx_interrupt(uart_dd->uart_base);

    //Give semaphore
    xSemaphoreGiveFromISR( uart_dd->xSemaphoreTXFinished, (uart_dd->pxHigherPriorityTaskWoken) );
}

void usart1_tx_dma_callback(void)
{
    //We come here after DMA transfer finished, and we must wait
    //for an interrupt after the last byte transmission finished.

    //Enable USART1 TX interrupt request
    usart_enable_tx_interrupt(USART1);
}

void usart2_tx_dma_callback(void)
{
    //We come here after DMA transfer finished, and we must wait
    //for an interrupt after the last byte transmission finished.

    //Enable USART2 TX interrupt request
    usart_enable_tx_interrupt(USART2);
}

void usart3_tx_dma_callback(void)
{
    //We come here after DMA transfer finished, and we must wait
    //for an interrupt after the last byte transmission finished.

    //Enable USART3 TX interrupt request
    usart_enable_tx_interrupt(USART3);
}

void uart4_tx_dma_callback(void)
{
    //We come here after DMA transfer finished, and we must wait
    //for an interrupt after the last byte transmission finished.

    //Enable UART4 TX interrupt request
    usart_enable_tx_interrupt(UART4);
}

void uart5_tx_dma_callback(void)
{
    //We come here after DMA transfer finished, and we must wait
    //for an interrupt after the last byte transmission finished.

    //Enable UART5 TX interrupt request
    usart_enable_tx_interrupt(UART5);
}

void usart6_tx_dma_callback(void)
{
    //We come here after DMA transfer finished, and we must wait
    //for an interrupt after the last byte transmission finished.

    //Enable USART6 TX interrupt request
    usart_enable_tx_interrupt(USART6);
}


static void uart_default_handler(uart_driver_descriptor_t *uart_dd)
{
    uint32_t uart_base = uart_dd->uart_base;
    uint16_t usart_sr;
    char usart_data;

    usart_sr = USART_SR(uart_base);
    usart_data = USART_DR(uart_base);

    //If overrun happened
    if(usart_sr & USART_SR_ORE)
    {
        //ToDo: Overrun condition handling must be here

        usart_data = USART_DR(uart_base);
    }

    //If RXNE interrupt is enabled
    if(USART_CR1(uart_base) & USART_CR1_RXNEIE)
    {
        //Check RXNE flag status (character received)
        if(usart_sr & USART_SR_RXNE)
        {
            if( (uart_dd->callback_rx) != NULL)
                uart_dd->callback_rx(&usart_data);
        }
    }

    //If TXE interrupt is enabled
    if(USART_CR1(uart_base) & USART_CR1_TXEIE)
    {
        //Check TXE flag status (transmit register empty)
        if(usart_sr & USART_SR_TXE)
        {
            if( (uart_dd->callback_tx) != NULL)
                uart_dd->callback_tx(uart_dd);
        }
    }

    //If LBDIE interrupt is enabled
    if(USART_CR2(uart_base) & USART_CR2_LBDIE)
    {
        //Check LBD flag status (LIN break detected)
        if(usart_sr & USART_SR_LBD)
        {
            //Give semaphore
//            xSemaphoreGiveFromISR( uart_dd->xSemaphoreTXFinished, (uart_dd->pxHigherPriorityTaskWoken) );
        }
    }

}


void USART1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[0] != NULL)
    {
        usart_dd_pointers[0]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;    //ToDo: WTF?
        uart_default_handler(usart_dd_pointers[0]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void USART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[1] != NULL)
    {
        usart_dd_pointers[1]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;
        uart_default_handler(usart_dd_pointers[1]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void USART3_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[2] != NULL)
    {
        usart_dd_pointers[2]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;
        uart_default_handler(usart_dd_pointers[2]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void UART4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[3] != NULL)
    {
        usart_dd_pointers[3]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;
        uart_default_handler(usart_dd_pointers[3]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void UART5_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[4] != NULL)
    {
        usart_dd_pointers[4]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;
        uart_default_handler(usart_dd_pointers[4]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void USART6_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[5] != NULL)
    {
        usart_dd_pointers[5]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;
        uart_default_handler(usart_dd_pointers[5]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void UART7_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[6] != NULL)
    {
        usart_dd_pointers[6]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;
        uart_default_handler(usart_dd_pointers[6]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void UART8_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(usart_dd_pointers[7] != NULL)
    {
        usart_dd_pointers[7]->pxHigherPriorityTaskWoken = &xHigherPriorityTaskWoken;
        uart_default_handler(usart_dd_pointers[7]);
    }

    //Switch context if necessary after exit from ISR
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


static const uint8_t uart_gpio_af[] = {
        GPIO_AF_USART1,
        GPIO_AF_USART2,
        GPIO_AF_USART3,
        GPIO_AF_UART4,
        GPIO_AF_UART5,
        GPIO_AF_USART6,
        GPIO_AF_UART7,
        GPIO_AF_UART8
};


static const enum rcc_periph_clken uart_rcc_periph[] = {
        RCC_USART1,
        RCC_USART2,
        RCC_USART3,
        RCC_UART4,
        RCC_UART5,
        RCC_USART6,
        RCC_UART7,
        RCC_UART8
};

static const uint32_t uart_base_addr[] = {
        USART1,
        USART2,
        USART3,
        UART4,
        UART5,
        USART6,
        UART7,
        UART8
};

static const uint8_t uart_nvic_vectors[] = {
        NVIC_USART1_IRQ,
        NVIC_USART2_IRQ,
        NVIC_USART3_IRQ,
        NVIC_UART4_IRQ,
        NVIC_UART5_IRQ,
        NVIC_USART6_IRQ,
        NVIC_UART7_IRQ,
        NVIC_UART8_IRQ
};

uint32_t uart_get_dma_controller_number_base_addr(uint32_t uart_base)
{
    switch(uart_base)
    {
        case USART1:

        case USART6:
            return DMA2;
            break;

        case USART2:

        case USART3:

        case UART4:

        case UART5:

        case UART7:

        case UART8:
            return DMA1;
            break;

        default:
            return 0;
            break;
    };
}


uint8_t uart_get_num_by_uart_base(uint32_t uart_base)
{
    switch(uart_base)
    {
        case USART1:
            return 0;
            break;

        case USART2:
            return 1;
            break;

        case USART3:
            return 2;
            break;

        case UART4:
            return 3;
            break;

        case UART5:
            return 4;
            break;

        case USART6:
            return 5;
            break;

        case UART7:
            return 6;
            break;

        case UART8:
            return 7;
            break;

    };
}


void uart_register_driver(uart_driver_descriptor_t *uart_dd)
{
    uint8_t uart_num = uart_get_num_by_uart_base(uart_dd->uart_base);

    uint32_t dma_controller_base = 0;

    dma_config_t *dma_tx_config = (uart_dd->dma_tx_config);

    uart_dd->pxHigherPriorityTaskWoken = NULL;

    //Initialize GPIOs required by U(S)ART controller
    gpio_mode_setup(uart_dd->hw.rx->port, GPIO_MODE_AF, GPIO_PUPD_NONE, uart_dd->hw.rx->pin);
    gpio_mode_setup(uart_dd->hw.tx->port, GPIO_MODE_AF, GPIO_PUPD_NONE, uart_dd->hw.tx->pin);

    gpio_set_output_options(uart_dd->hw.tx->port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, uart_dd->hw.tx->pin);

    gpio_set_af(uart_dd->hw.rx->port, uart_gpio_af[uart_num], uart_dd->hw.rx->pin);
    gpio_set_af(uart_dd->hw.tx->port, uart_gpio_af[uart_num], uart_dd->hw.tx->pin);

    if(uart_dd->settings.bus_mode == uart_mode_normal_uart)
    {
        //Initialize other GPIOs if required (RTS, CTS, etc)
        //..
    }

    //Enable peripheral clock
    rcc_periph_clock_enable( uart_rcc_periph[uart_num] );

    nvic_disable_irq(uart_nvic_vectors[uart_num]);

//    usart_disable_rx_interrupt(uart_dd->uart_base);
//    usart_disable_tx_interrupt(uart_dd->uart_base);
//
//    usart_disable_rx_dma(uart_dd->uart_base);
//    usart_disable_tx_dma(uart_dd->uart_base);

    usart_set_baudrate(uart_dd->uart_base, uart_dd->settings.baudrate);

    USART_CR2(uart_dd->uart_base) &=  ~USART_CR2_LBDIE;

    if(uart_dd->settings.bus_mode == uart_mode_lin)
    {
        if(uart_dd->lin != NULL)

            //If LIN is enabled, only 8 databits mode is allowed
            usart_set_databits(uart_dd->uart_base, 8);

            //If LIN is enabled, only 1 stopbit is allowed
            usart_set_stopbits(uart_dd->uart_base, USART_CR2_STOPBITS_1);

            //Enable LIN mode
            USART_CR2(uart_dd->uart_base) |= USART_CR2_LINEN;

            if(uart_dd->lin->lbd == uart_lin_break_length_11b)
            {
                //Set break detection length
                USART_CR2(uart_dd->uart_base) |=  USART_CR2_LBDL;

                //Set break detection interrupt enable
//                USART_CR2(uart_dd->uart_base) |=  USART_CR2_LBDIE;
            }
    } else
    {
        usart_set_databits(uart_dd->uart_base, uart_dd->settings.databits);
        usart_set_stopbits(uart_dd->uart_base, uart_dd->settings.stopbits);
    }

    usart_set_parity(uart_dd->uart_base, uart_dd->settings.parity);


    //Transmitter and receiver enable
    usart_set_mode(uart_dd->uart_base, uart_dd->settings.mode);

    usart_set_flow_control(uart_dd->uart_base, uart_dd->settings.flowcontrol);


    usart_enable(uart_dd->uart_base);


    //NVIC configuration

    //Clear pending IRQ request in case it presents
    nvic_clear_pending_irq( uart_nvic_vectors[uart_num] );

    //Enable callback
    usart_dd_pointers[uart_num] = uart_dd;

    //Set USART interrupt priority less than configLIBRARY_SYSCALL_INTERRUPT_PRIORITY
    nvic_set_priority(uart_nvic_vectors[uart_num], NVIC_PRIORITY_SYSCALL_USART1);

    //Enable USART interrupt handling in NVIC
    nvic_enable_irq(uart_nvic_vectors[uart_num]);



    //Determine required DMA controller base address
    dma_controller_base = uart_get_dma_controller_number_base_addr(uart_dd->uart_base);


    //If RX is enabled
    if( (uart_dd->settings.mode == USART_MODE_RX) || (uart_dd->settings.mode == USART_MODE_TX_RX))
    {
        //If RX DMA is required
        if(uart_dd->dma_rx_config != NULL)
        {
            //Configure RX DMA

            //Register USART RX DMA callback in DMA Manager
//            dma_manager_register_handler(dma_controller_base, dma_rx_config->stream, uart_dd, &usart1_rx_dma_callback);

        } else //RX DMA is not used
        {
            //Using interrupt mode for RX

            //Enable USART RX interrupt request
            usart_enable_rx_interrupt(uart_dd->uart_base);
        }
    }


//        //Query DMA manager if DMA streams required for U(S)ART controller are free
//        if(dma_manager_get_dma_handler_instance(dma_controller_base, dma_config->stream) != NULL)
//            return; //Error, DMA stream is busy

        //If TX is enabled
        if( (uart_dd->settings.mode == USART_MODE_TX) || (uart_dd->settings.mode == USART_MODE_TX_RX))
        {
            //Initialize TX semaphore
            uart_dd->xSemaphoreTXFinished = xSemaphoreCreateBinary();

            //If TX DMA is required
            if(uart_dd->dma_tx_config != NULL)
            {
                //Configure TX DMA

                //Register USART TX DMA callback in DMA Manager
                dma_manager_register_handler(dma_controller_base, dma_tx_config->stream, uart_dd, uart_dd->callback_dma_tx_transfer_finished);

                //Enable USART TX DMA
                usart_enable_tx_dma(uart_dd->uart_base);


                dma_stream_reset(dma_controller_base, dma_tx_config->stream);

                dma_set_priority(dma_controller_base, dma_tx_config->stream, dma_tx_config->priority /*DMA_SxCR_PL_MEDIUM*/);

                dma_set_memory_size(dma_controller_base, dma_tx_config->stream, DMA_SxCR_MSIZE_8BIT);

                dma_set_peripheral_size(dma_controller_base, dma_tx_config->stream, DMA_SxCR_PSIZE_8BIT);

                //Increment memory address after each transfer
                dma_enable_memory_increment_mode(dma_controller_base, dma_tx_config->stream);

                //Do not increment peripheral address
                dma_disable_peripheral_increment_mode(dma_controller_base, dma_tx_config->stream);

                dma_set_transfer_mode(dma_controller_base, dma_tx_config->stream, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

                //Address of buffer must be defined later!
                dma_set_memory_address(dma_controller_base, dma_tx_config->stream, NULL);

                dma_set_peripheral_address(dma_controller_base, dma_tx_config->stream, (uint32_t)&( USART_DR(uart_dd->uart_base) ) );

                dma_set_number_of_data(dma_controller_base, dma_tx_config->stream, 1);

                //Set USART TX DMA interrupt request priority
                nvic_set_priority( dma_manager_get_dma_nvic_vector(dma_controller_base, dma_tx_config->stream), NVIC_PRIORITY_SYSCALL_USART1_DMA );

                //Enable DMA IRQ in NVIC
                nvic_enable_irq( dma_manager_get_dma_nvic_vector(dma_controller_base, dma_tx_config->stream) );


                //Enable interrupt on DMA transfer complete
                dma_enable_transfer_complete_interrupt(dma_controller_base, dma_tx_config->stream);

                dma_channel_select(dma_controller_base, dma_tx_config->stream, dma_tx_config->channel);

            } else  //TX DMA is not used
            {
                //Using interrupt mode for TX

                //Enable USART TX interrupt request
                usart_enable_tx_interrupt(uart_dd->uart_base);
            }

            xSemaphoreGive(uart_dd->xSemaphoreTXFinished);

        }

        //Register driver in Driver Manager
        driver_manager_register_driver(uart_dd->nameStr, uart_dd);
}



void usart_start_dma_transfer(const uart_driver_descriptor_t *uart_dd, const char *str, const uint8_t chars_to_send)
{
    //Determine required DMA controller base address
    uint32_t dma_controller_base = uart_get_dma_controller_number_base_addr(uart_dd->uart_base);

    //Start DMA transfer
    dma_set_memory_address(dma_controller_base, uart_dd->dma_tx_config->stream, (uint32_t)str);
    dma_set_number_of_data(dma_controller_base, uart_dd->dma_tx_config->stream, chars_to_send);
    dma_enable_stream(dma_controller_base, uart_dd->dma_tx_config->stream);
}

uint8_t usart_writestr_blocking(const uart_driver_descriptor_t *uart_dd, const char *str)
{
    uint8_t bytes_written;

    bytes_written = strlen(str);

    if(bytes_written)
    {
        xSemaphoreTake( uart_dd->xSemaphoreTXFinished,  portMAX_DELAY);
        usart_start_dma_transfer(uart_dd, str, bytes_written);

        xSemaphoreTake( uart_dd->xSemaphoreTXFinished,  portMAX_DELAY);
        xSemaphoreGive( uart_dd->xSemaphoreTXFinished );
    }

    return bytes_written;
}

void usart_lin_send_break(const uart_driver_descriptor_t *uart_dd)
{
//    xSemaphoreTake( uart_dd->xSemaphoreTXFinished,  portMAX_DELAY);

    //Send break character
    USART_CR1(uart_dd->uart_base) |= USART_CR1_SBK;

//    Wait until transfer completes
    while(USART_CR1(uart_dd->uart_base) & USART_CR1_SBK)
    {
        vTaskDelay(1);
    }

//    while(USART_SR(uart_dd->uart_base) & USART_SR_TC)
//    {
//        vTaskDelay(1);
//    }


//    xSemaphoreGive( uart_dd->xSemaphoreTXFinished );
}


