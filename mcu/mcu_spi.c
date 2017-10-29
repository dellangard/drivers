/*
 * mcu_spi.c
 *
 *  Created on: 5 янв. 2017 г.
 *      Author: frost
 */

#include <stddef.h>
#include <libopencmsis/dispatch/irqhandlers.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>

#include "mcu_gpio.h"
#include "mcu_spi.h"

#include "FreeRTOS.h"
#include "semphr.h"

static spi_driver_descriptor_t *spi_dd_pointers[] = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL
};

static const enum rcc_periph_clken spi_rcc_periph[] = {
        RCC_SPI1,
        RCC_SPI2,
        RCC_SPI3,
        RCC_SPI4,
        RCC_SPI5,
        RCC_SPI6
};

static const uint8_t spi_nvic_vectors[] = {
        NVIC_SPI1_IRQ,
        NVIC_SPI2_IRQ,
        NVIC_SPI3_IRQ,
        NVIC_SPI4_IRQ,
        NVIC_SPI5_IRQ,
        NVIC_SPI6_IRQ
};

static const uint8_t spi_gpio_af[] = {
        GPIO_AF_SPI1,
        GPIO_AF_SPI2,
        GPIO_AF_SPI3,
        GPIO_AF_SPI4,
        GPIO_AF_SPI5,
        GPIO_AF_SPI6
};


static uint8_t spi_get_num_by_spi_base(uint32_t spi_base);
void spi_dma_transmit(spi_driver_descriptor_t *spi_dd, const uint8_t count, uint8_t *ptr_write_buffer);
void spi_dma_receive(spi_driver_descriptor_t *spi_dd, const uint8_t count, uint8_t *ptr_read_buffer);
static uint32_t spi_get_dma_controller_number_base_addr(uint32_t spi_base);


static void spi_rx_dma_callback(spi_driver_descriptor_t *spi_dd)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( spi_dd->io->xSemaphoreSPIReady, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



void spi1_tx_dma_callback(void)
{
    //Read SPI data register to clear flags
    volatile uint16_t tmp = SPI_DR(SPI1);

    //Enable SPI1 RXNE interrupt request
    SPI_CR2(SPI1) |= SPI_CR2_RXNEIE;
}

void spi2_tx_dma_callback(void)
{
    //Read SPI data register to clear flags
    volatile uint16_t tmp = SPI_DR(SPI2);

    //Enable SPI2 RXNE interrupt request
    SPI_CR2(SPI2) |= SPI_CR2_RXNEIE;
}

void spi3_tx_dma_callback(void)
{
    //Read SPI data register to clear flags
    volatile uint16_t tmp = SPI_DR(SPI3);

    //Enable SPI3 RXNE interrupt request
    SPI_CR2(SPI3) |= SPI_CR2_RXNEIE;
}

void spi4_tx_dma_callback(void)
{
    //Read SPI data register to clear flags
    volatile uint16_t tmp = SPI_DR(SPI4);

    //Enable SPI4 RXNE interrupt request
    SPI_CR2(SPI4) |= SPI_CR2_RXNEIE;
}

void spi5_tx_dma_callback(void)
{
    //Read SPI data register to clear flags
    volatile uint16_t tmp = SPI_DR(SPI5);

    //Enable SPI5 RXNE interrupt request
    SPI_CR2(SPI5) |= SPI_CR2_RXNEIE;
}

void spi6_tx_dma_callback(void)
{
    //Read SPI data register to clear flags
    volatile uint16_t tmp = SPI_DR(SPI6);

    //Enable SPI6 RXNE interrupt request
    SPI_CR2(SPI6) |= SPI_CR2_RXNEIE;
}

static void spi_isr_handler_callback(spi_driver_descriptor_t *spi_dd, BaseType_t *pxHigherPriorityTaskWoken)
{
    spi_io_t *spi_io = (spi_dd->io);

    uint16_t spi_dr;

    //RX not empty
    if(SPI_SR(spi_dd->spi_base) & SPI_SR_RXNE)
    {
        //'Dummy' read received data
        //(void)(SPI1_DR);
        spi_dr = SPI_DR(spi_dd->spi_base);

        //Interrupt mode
        if(spi_io->dma_mode == spi_dma_disabled)
        {

            if(spi_io->bytes_to_write)
            {
                if(spi_io->bytes_to_write == 1)
                {
                    //If we have to read something
                    if(spi_io->bytes_to_read)
                    {
                        //Send 'dummy' byte
                        spi_write(spi_dd->spi_base, 0x00);
                    } else
                    {
                        xSemaphoreGiveFromISR( spi_io->xSemaphoreSPIReady, pxHigherPriorityTaskWoken );
                    }

                } else
                {
                    (spi_io->writemem)++;
                    spi_write(spi_dd->spi_base, *(spi_io->writemem));
                }

                (spi_io->bytes_to_write)--;

            } else
            {
                //If nothing to write, but something to read
                if(spi_io->bytes_to_read)
                {
                    *(spi_io->readmem) = SPI_DR(spi_dd->spi_base);
                    (spi_io->readmem)++;

                    (spi_io->bytes_to_read)--;

                    //If we have some succeeding reads
                    if(spi_io->bytes_to_read)
                    {
                        //Send 'dummy' byte
                        spi_write(spi_dd->spi_base, 0x00);
                    } else
                    {
                        xSemaphoreGiveFromISR( spi_io->xSemaphoreSPIReady, pxHigherPriorityTaskWoken );
                    }
                }
            }

        } else //DMA mode
        {
            //Disable SPI RXNE interrupt
            SPI_CR2(spi_dd->spi_base) &= ~SPI_CR2_RXNEIE;
//            spi_disable_rx_buffer_not_empty_interrupt(spi_dd->spi_base);

            if(spi_io->dma_mode == spi_dma_tx)
            {
                xSemaphoreGiveFromISR( spi_io->xSemaphoreSPIReady, pxHigherPriorityTaskWoken );
            }
        }


    }

//    //TX empty
//    if(SPI_SR(spi_dd->spi_base) & SPI_SR_TXE)
//    {
//        (void)SPI_DR(spi_dd->spi_base);
//        (void)SPI_SR(spi_dd->spi_base);
//    }

    //RX overrun
//    if(SPI_SR(spi_dd->spi_base) & SPI_SR_OVR)
//    {
//        (void)SPI_DR(spi_dd->spi_base);
//        (void)SPI_SR(spi_dd->spi_base);
//    }
//
//    //MODE Fault
//    if(SPI_SR(spi_dd->spi_base) & SPI_SR_MODF)
//    {
//        (void)SPI_DR(spi_dd->spi_base);
//        (void)SPI_SR(spi_dd->spi_base);
//    }
}


void SPI1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(spi_dd_pointers[0] != NULL)
	    spi_isr_handler_callback(spi_dd_pointers[0], &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void SPI2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(spi_dd_pointers[1] != NULL)
	        spi_isr_handler_callback(spi_dd_pointers[1], &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void SPI3_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(spi_dd_pointers[2] != NULL)
            spi_isr_handler_callback(spi_dd_pointers[2], &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void SPI4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(spi_dd_pointers[3] != NULL)
            spi_isr_handler_callback(spi_dd_pointers[3], &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void SPI5_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(spi_dd_pointers[4] != NULL)
            spi_isr_handler_callback(spi_dd_pointers[4], &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void SPI6_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(spi_dd_pointers[5] != NULL)
            spi_isr_handler_callback(spi_dd_pointers[5], &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


static uint8_t spi_get_num_by_spi_base(uint32_t spi_base)
{
    switch(spi_base)
    {
        case SPI1:
            return 0;
            break;

        case SPI2:
            return 1;
            break;

        case SPI3:
            return 2;
            break;

        case SPI4:
            return 3;
            break;

        case SPI5:
            return 4;
            break;

        case SPI6:
            return 5;
            break;
    };
}

static uint32_t spi_get_dma_controller_number_base_addr(uint32_t spi_base)
{
    switch(spi_base)
    {
        case SPI2:

        case SPI3:
            return DMA1;
            break;

        case SPI1:

        case SPI4:

        case SPI5:

        case SPI6:
            return DMA2;
            break;
    };
}

//Set Frame fromat to MSB or LSB first
void spi_set_frame_format(spi_driver_descriptor_t *spi_dd, const spi_frame_format_t frame_format)
{
    //Obtain mutex
//    xSemaphoreTake( spi_dd->io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);

//    xSemaphoreTake( spi_dd->io->xSemaphoreSPIReady,  portMAX_DELAY);

    spi_disable(spi_dd->spi_base);

    if(frame_format == spi_ff_lsb)
        spi_send_lsb_first(spi_dd->spi_base);
    else
        spi_send_msb_first(spi_dd->spi_base);

//    spi_enable(spi_dd->spi_base);
    SPI_CR1(spi_dd->spi_base) |= SPI_CR1_SPE;

//    xSemaphoreGive( spi_dd->io->xSemaphoreSPIReady );

    //Release mutex
//    xSemaphoreGive( spi_dd->io->xSemaphoreSPIReadyMutex );
}


void spi_register_driver(spi_driver_descriptor_t *spi_dd)
{
    uint32_t dma_controller_base = 0;

    uint8_t spi_bus_num = spi_get_num_by_spi_base(spi_dd->spi_base);

    spi_dd->io = (spi_io_t*)pvPortMalloc(sizeof(spi_io_t));

    spi_io_t *spi_io = spi_dd->io;

    dma_config_t *dma_tx_config = (spi_dd->dma_tx_config);
    dma_config_t *dma_rx_config = (spi_dd->dma_rx_config);

    //Init GPIOs required for SPI controller
    gpio_mode_setup(spi_dd->gpio_miso->port, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_dd->gpio_miso->pin);
    gpio_mode_setup(spi_dd->gpio_mosi->port, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_dd->gpio_mosi->pin);
    gpio_mode_setup(spi_dd->gpio_sck->port, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_dd->gpio_sck->pin);

    gpio_set_output_options(spi_dd->gpio_mosi->port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_dd->gpio_mosi->pin);
    gpio_set_output_options(spi_dd->gpio_sck->port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_dd->gpio_sck->pin);

    //Set up alternative function mode for SPI GPIOs
    gpio_set_af(spi_dd->gpio_miso->port, spi_gpio_af[spi_bus_num], spi_dd->gpio_miso->pin);
    gpio_set_af(spi_dd->gpio_mosi->port, spi_gpio_af[spi_bus_num], spi_dd->gpio_mosi->pin);
    gpio_set_af(spi_dd->gpio_sck->port, spi_gpio_af[spi_bus_num], spi_dd->gpio_sck->pin);

    //Enable SPI peripheral clock
    rcc_periph_clock_enable( spi_rcc_periph[spi_bus_num] );

    spi_disable(spi_dd->spi_base);

    //Set SPI mode
    SPI_I2SCFGR(spi_dd->spi_base) = 0x00;

    //ToDo: Take clock divider value from settings
    spi_init_master(
            spi_dd->spi_base,
//            SPI_CR1_BAUDRATE_FPCLK_DIV_64,
            SPI_CR1_BAUDRATE_FPCLK_DIV_8,
//            SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
            SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
//            SPI_CR1_CPHA_CLK_TRANSITION_2,
            SPI_CR1_CPHA_CLK_TRANSITION_1,
            SPI_CR1_DFF_8BIT,
            SPI_CR1_MSBFIRST
            );

    //Do not use hardware NSS
    spi_disable_ss_output(spi_dd->spi_base);

    //Do not use hardware NSS
    spi_enable_software_slave_management(spi_dd->spi_base);

    spi_set_nss_high(spi_dd->spi_base);

    spi_enable(spi_dd->spi_base);


    //spi_enable_error_interrupt(spi_buses_array[spi_bus_num]->spi_base);

    //Disable SPI TX and RX interrupts
    spi_disable_tx_buffer_empty_interrupt(spi_dd->spi_base);
    spi_disable_rx_buffer_not_empty_interrupt(spi_dd->spi_base);


    //Determine required DMA controller base address
    dma_controller_base = spi_get_dma_controller_number_base_addr(spi_dd->spi_base);

    spi_io->dma_ctrl_base = dma_controller_base;

    //If TX DMA is required
    if(spi_dd->dma_tx_config != NULL)
    {
        //Configure SPI TX DMA

        //Register SPI TX DMA callback in DMA Manager
        dma_manager_register_handler(dma_controller_base, dma_tx_config->stream, spi_dd, spi_dd->callback_dma_tx);

        //Enable SPI TX DMA
        spi_enable_tx_dma(spi_dd->spi_base);

        dma_stream_reset(dma_controller_base, dma_tx_config->stream);

        dma_set_priority(dma_controller_base, dma_tx_config->stream, dma_tx_config->priority);

        dma_set_memory_size(dma_controller_base, dma_tx_config->stream, DMA_SxCR_MSIZE_8BIT);

        dma_set_peripheral_size(dma_controller_base, dma_tx_config->stream, DMA_SxCR_PSIZE_8BIT);

        //Increment memory address after each transfer
        dma_enable_memory_increment_mode(dma_controller_base, dma_tx_config->stream);

        //Do not increment peripheral address
        dma_disable_peripheral_increment_mode(dma_controller_base, dma_tx_config->stream);

        dma_set_transfer_mode(dma_controller_base, dma_tx_config->stream, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

        //Address of buffer must be defined later!
        dma_set_memory_address(dma_controller_base, dma_tx_config->stream, NULL);

        dma_set_peripheral_address(dma_controller_base, dma_tx_config->stream, (uint32_t)&( SPI_DR(spi_dd->spi_base) ) );

        dma_set_number_of_data(dma_controller_base, dma_tx_config->stream, 1);



        //Set SPI TX DMA interrupt request priority
        nvic_set_priority( dma_manager_get_dma_nvic_vector(dma_controller_base, dma_tx_config->stream), NVIC_PRIORITY_SYSCALL_SPI_HIGH );

        //Enable DMA IRQ in NVIC
        nvic_enable_irq( dma_manager_get_dma_nvic_vector(dma_controller_base, dma_tx_config->stream) );

        //Enable interrupt on DMA transfer complete
        dma_enable_transfer_complete_interrupt(dma_controller_base, dma_tx_config->stream);

        dma_channel_select(dma_controller_base, dma_tx_config->stream, dma_tx_config->channel);

    }

    //If RX DMA is required
    if(spi_dd->dma_rx_config != NULL)
    {
        //Configure SPI RX DMA

        //Register SPI RX DMA callback in DMA Manager
        dma_manager_register_handler(dma_controller_base, dma_rx_config->stream, spi_dd, &spi_rx_dma_callback /*spi_dd->callback_dma_rx*/);

        //Enable SPI RX DMA
        spi_enable_rx_dma(spi_dd->spi_base);

        dma_stream_reset(dma_controller_base, dma_rx_config->stream);

        dma_set_priority(dma_controller_base, dma_rx_config->stream, dma_rx_config->priority);

        dma_set_memory_size(dma_controller_base, dma_rx_config->stream, DMA_SxCR_MSIZE_8BIT);

        dma_set_peripheral_size(dma_controller_base, dma_rx_config->stream, DMA_SxCR_PSIZE_8BIT);

        //Increment memory address after each transfer
        dma_enable_memory_increment_mode(dma_controller_base, dma_rx_config->stream);

        //Do not increment peripheral address
        dma_disable_peripheral_increment_mode(dma_controller_base, dma_rx_config->stream);

        dma_set_transfer_mode(dma_controller_base, dma_rx_config->stream, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

        //Address of buffer must be defined later!
        dma_set_memory_address(dma_controller_base, dma_rx_config->stream, NULL);

        dma_set_peripheral_address(dma_controller_base, dma_rx_config->stream, (uint32_t)&( SPI_DR(spi_dd->spi_base) ) );

        dma_set_number_of_data(dma_controller_base, dma_rx_config->stream, 1);



        //Set SPI RX DMA interrupt request priority
        nvic_set_priority( dma_manager_get_dma_nvic_vector(dma_controller_base, dma_rx_config->stream), NVIC_PRIORITY_SYSCALL_SPI );

        //Enable DMA IRQ in NVIC
        nvic_enable_irq( dma_manager_get_dma_nvic_vector(dma_controller_base, dma_rx_config->stream) );

        //Enable interrupt on DMA transfer complete
        dma_enable_transfer_complete_interrupt(dma_controller_base, dma_rx_config->stream);

        dma_channel_select(dma_controller_base, dma_rx_config->stream, dma_rx_config->channel);
    }

    nvic_set_priority( spi_nvic_vectors[spi_bus_num], NVIC_PRIORITY_SYSCALL_SPI);   //Priority less than configLIBRARY_SYSCALL_INTERRUPT_PRIORITY

    (void)(SPI_SR(spi_dd->spi_base));

    //Reset IO structure members
    spi_io->xSemaphoreSPIReady = NULL;
    spi_io->xSemaphoreSPIReadyMutex = NULL;
    spi_io->writemem = NULL;
    spi_io->readmem = NULL;
    spi_io->bytes_to_read = 0;
    spi_io->bytes_to_write = 0;
    spi_io->dma_mode = spi_dma_disabled;

    //Initialize semaphore and mutex
    spi_io->xSemaphoreSPIReady = xSemaphoreCreateBinary();
    xSemaphoreGive( spi_io->xSemaphoreSPIReady );

    spi_io->xSemaphoreSPIReadyMutex = xSemaphoreCreateMutex();
    xSemaphoreGive( spi_io->xSemaphoreSPIReadyMutex );

    nvic_clear_pending_irq( spi_nvic_vectors[spi_bus_num] );

    //Register callback for SPI ISR
    spi_dd_pointers[spi_bus_num] = spi_dd;

    //Enable SPI interrupt in NVIC
    nvic_enable_irq( spi_nvic_vectors[spi_bus_num] );

    //Register driver in Driver Manager
    driver_manager_register_driver(spi_dd->nameStr, spi_dd);

}

void spi_get_mutex(spi_driver_descriptor_t *spi_dd)
{
    //Obtain mutex
    xSemaphoreTake( spi_dd->io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);
}

void spi_release_mutex(spi_driver_descriptor_t *spi_dd)
{
    //Release mutex
    xSemaphoreGive( spi_dd->io->xSemaphoreSPIReadyMutex );
}

void spi_get_semaphore(spi_driver_descriptor_t *spi_dd)
{
    //Wait for semaphore
    xSemaphoreTake( spi_dd->io->xSemaphoreSPIReady,  portMAX_DELAY);
}

void spi_release_semaphore(spi_driver_descriptor_t *spi_dd)
{
    //Release semaphore
    xSemaphoreGive( spi_dd->io->xSemaphoreSPIReady );
}



//Start SPI transaction in interrupt mode
static int8_t spi_start_master_transaction(spi_driver_descriptor_t *spi_dd, const uint8_t wb_count, const uint8_t *write_array, const uint8_t rb_count, uint8_t *read_array)
{
    spi_io_t *spi_io = (spi_dd->io);

    (void)(SPI_SR(spi_dd->spi_base));

    spi_io->dma_mode = spi_dma_disabled;

    //Enable RX not empty interrupt
    spi_enable_rx_buffer_not_empty_interrupt(spi_dd->spi_base);

    spi_io->writemem = write_array;

    if(rb_count == 0)
        spi_io->readmem = NULL;
    else
        spi_io->readmem = read_array;

    spi_io->bytes_to_write = wb_count;
    spi_io->bytes_to_read = rb_count;

    SPI_DR(spi_dd->spi_base) = *(spi_io->writemem);

    return 0;
}


//**************** SPI peripheral device single/multi register read/write functions ****************

//Read single 8-bit register, defined in reg_addr
uint8_t spi_read_single_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr)
{
    spi_io_t *spi_io = (spi_dd->io);

    uint8_t spi_wb[1];
    uint8_t spi_rb[1];

    //Obtain mutex
    xSemaphoreTake( spi_io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);

    spi_wb[0] = reg_addr | 0x80;
    spi_start_master_transaction(spi_dd, 1, spi_wb, 1, spi_rb);
    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

    xSemaphoreGive( spi_io->xSemaphoreSPIReady );

    //Release mutex
    xSemaphoreGive( spi_io->xSemaphoreSPIReadyMutex );

    return spi_rb[0];
}



//Read multiple 8-bit registers, starting address defined in 'reg_addr', register count defined in 'count'
void spi_read_multi_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr, const uint8_t count, uint8_t *ptr_read_buffer)
{
    spi_io_t *spi_io = (spi_dd->io);

    uint8_t spi_wb[1];

    //Obtain mutex
    xSemaphoreTake( spi_io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);

    spi_wb[0] = reg_addr | 0x40 | 0x80;
    spi_start_master_transaction(spi_dd, 1, spi_wb, count, ptr_read_buffer);
    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

    xSemaphoreGive( spi_io->xSemaphoreSPIReady );

    //Release mutex
    xSemaphoreGive( spi_io->xSemaphoreSPIReadyMutex );
}

//Write one byte to SPI and read one byte from SPI at the same time
uint8_t spi_rw_single8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t data)
{
    spi_io_t *spi_io = (spi_dd->io);

    uint8_t spi_wb[1];
    uint8_t spi_rb[1];

    //Obtain mutex
    xSemaphoreTake( spi_io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);

    spi_wb[0] = data;

    spi_start_master_transaction(spi_dd, 0, spi_wb, 1, spi_rb);
    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

    xSemaphoreGive( spi_io->xSemaphoreSPIReady );

    //Release mutex
    xSemaphoreGive( spi_io->xSemaphoreSPIReadyMutex );

    return spi_rb[0];
}

//Write single 8-bit register at address 'reg_addr'
void spi_write_single_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr, const uint8_t data)
{
    spi_io_t *spi_io = (spi_dd->io);

    uint8_t spi_wb[2];

    //Obtain mutex
    xSemaphoreTake( spi_io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);

    spi_wb[0] = reg_addr;
    spi_wb[1] = data;

    spi_start_master_transaction(spi_dd, 2, spi_wb, 0, NULL);
    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

    xSemaphoreGive( spi_io->xSemaphoreSPIReady );

    //Release mutex
    xSemaphoreGive( spi_io->xSemaphoreSPIReadyMutex );
}


//Write multiple 8-bit registers, beginning from address 'reg_addr'
void spi_write_multi_reg8(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t reg_addr, const uint8_t count, uint8_t *ptr_write_buffer)
{
    spi_io_t *spi_io = (spi_dd->io);

    uint8_t spi_wb[1];

    //Obtain mutex
    xSemaphoreTake( spi_io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);

    spi_wb[0] = reg_addr;

    spi_start_master_transaction(spi_dd, 1, spi_wb, 0, NULL);
    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    spi_dma_transmit(spi_dd, count, ptr_write_buffer);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

    xSemaphoreGive( spi_io->xSemaphoreSPIReady );

    //Release mutex
    xSemaphoreGive( spi_io->xSemaphoreSPIReadyMutex );
}


void spi_dma_transmit(spi_driver_descriptor_t *spi_dd, const uint8_t count, uint8_t *ptr_write_buffer)
{
    spi_dd->io->dma_mode = spi_dma_tx;

    //Start SPI DMA transfer
    dma_set_memory_address(spi_dd->io->dma_ctrl_base, spi_dd->dma_tx_config->stream, (uint32_t)ptr_write_buffer);
    dma_set_number_of_data(spi_dd->io->dma_ctrl_base, spi_dd->dma_tx_config->stream, count);
    dma_enable_stream(spi_dd->io->dma_ctrl_base, spi_dd->dma_tx_config->stream);
}

void spi_dma_receive(spi_driver_descriptor_t *spi_dd, const uint8_t count, uint8_t *ptr_read_buffer)
{
    spi_dd->io->dma_mode = spi_dma_rx;

    //Dummy array of 8 zero elements
    static uint8_t wb[8];// = {0, 0, 0, 0, 0, 0, 0, 0};

    //Prepare for SPI RX DMA transfer

    //Configure SPI TX DMA controller to send 8 dummy bytes
    dma_set_memory_address(spi_dd->io->dma_ctrl_base, spi_dd->dma_tx_config->stream, wb);
    dma_set_number_of_data(spi_dd->io->dma_ctrl_base, spi_dd->dma_tx_config->stream, count);


    //Start SPI DMA transfer
    dma_set_memory_address(spi_dd->io->dma_ctrl_base, spi_dd->dma_rx_config->stream, (uint32_t)ptr_read_buffer);
    dma_set_number_of_data(spi_dd->io->dma_ctrl_base, spi_dd->dma_rx_config->stream, count);

//    (void)(SPI_SR(spi_dd->spi_base));
//    (void)(SPI_DR(spi_dd->spi_base));

    dma_enable_stream(spi_dd->io->dma_ctrl_base, spi_dd->dma_rx_config->stream);

    dma_enable_stream(spi_dd->io->dma_ctrl_base, spi_dd->dma_tx_config->stream);
}


void spi_dma_write_multi_bytes(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t count, uint8_t *ptr_write_buffer)
{
    spi_io_t *spi_io = (spi_dd->io);

    //Obtain mutex
    xSemaphoreTake( spi_io->xSemaphoreSPIReadyMutex,  portMAX_DELAY);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);

    spi_dma_transmit(spi_dd, count, ptr_write_buffer);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

    xSemaphoreGive( spi_io->xSemaphoreSPIReady );

    //Release mutex
    xSemaphoreGive( spi_io->xSemaphoreSPIReadyMutex );
}


void spi_dma_write_multi_bytes_no_mutex(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t count, uint8_t *ptr_write_buffer)
{
    spi_io_t *spi_io = (spi_dd->io);

    //Disable SPI RXNE interrupt because we want DMA mode
    spi_disable_rx_buffer_not_empty_interrupt(spi_dd->spi_base);

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);


    spi_dma_transmit(spi_dd, count, ptr_write_buffer);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

}

void spi_dma_read_multi_bytes_no_mutex(spi_driver_descriptor_t *spi_dd, const gpio_t *cs, const uint8_t count, uint8_t *ptr_read_buffer)
{
    spi_io_t *spi_io = (spi_dd->io);

    //Disable SPI RXNE interrupt because we want DMA mode
    spi_disable_rx_buffer_not_empty_interrupt(spi_dd->spi_base);

//    (void)(SPI_SR(spi_dd->spi_base));
//    (void)(SPI_DR(spi_dd->spi_base));

    //Set ChipSelect GPIO to logic low
    gpio_atomic_low(cs);

    spi_dma_receive(spi_dd, count, ptr_read_buffer);

    xSemaphoreTake( spi_io->xSemaphoreSPIReady,  portMAX_DELAY);

    //Set ChipSelect GPIO to logic hi
    gpio_atomic_hi(cs);

}


//************************************************


