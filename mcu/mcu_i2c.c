/*
 * mcu_i2c.c
 *
 *  Created on: 2 янв. 2017 г.
 *      Author: frost
 */

#include <stddef.h>

#include <libopencmsis/dispatch/irqhandlers.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "mcu_i2c.h"


#include "FreeRTOS.h"
#include "semphr.h"

static i2c_driver_descriptor_t *i2c_dd_pointers[] = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL
};


static const enum rcc_periph_clken i2c_rcc_periph[] = {
        RCC_I2C1,
        RCC_I2C2,
        RCC_I2C3,
};

//I2C Event interrupt vectors
static const uint8_t i2c_ev_nvic_vectors[] = {
        NVIC_I2C1_EV_IRQ,
        NVIC_I2C2_EV_IRQ,
        NVIC_I2C3_EV_IRQ
};

//I2C Error interrupt vectors
static const uint8_t i2c_er_nvic_vectors[] = {
        NVIC_I2C1_ER_IRQ,
        NVIC_I2C2_ER_IRQ,
        NVIC_I2C3_ER_IRQ
};

static const uint8_t i2c_gpio_af[] = {
        GPIO_AF_I2C1,
        GPIO_AF_I2C2,
        GPIO_AF_I2C3
};


//Declarations

void i2c_event_callback(i2c_driver_descriptor_t *i2c_dd, BaseType_t *pxHigherPriorityTaskWoken);

static uint8_t i2c_get_num_by_i2c_base(uint32_t i2c_base);

__inline__ uint32_t __attribute__((always_inline)) I2C_GetLastEvent(uint32_t i2c);

static int8_t i2c_master_transaction_8b_start(
        const i2c_driver_descriptor_t *i2c_dd,
        const uint8_t slave_address,
        const uint8_t wb_count, const uint8_t *write_array,
        const uint8_t rb_count, uint8_t *read_array
        );

static uint8_t i2c_get_num_by_i2c_base(uint32_t i2c_base)
{
    switch(i2c_base)
    {
        case I2C1:
            return 0;
            break;

        case I2C2:
            return 1;
            break;

        case I2C3:
            return 2;
            break;

    };
}



void i2c_event_callback(i2c_driver_descriptor_t *i2c_dd, BaseType_t *pxHigherPriorityTaskWoken)
{
    volatile uint16_t i2c_sr1;
    volatile uint16_t i2c_sr2;
    volatile uint16_t i2c_dr;

    volatile uint32_t lastevent;

    volatile uint8_t i2c_num;

    i2c_num = i2c_dd->i2c_num;

    lastevent = I2C_GetLastEvent(i2c_dd->i2c_base);

    switch (lastevent)
    {
        //EV5
        case I2C_EVENT_MASTER_MODE_SELECT:

            //Send slave address
            //If we have to write some bytes, we must write them first
            if(i2c_dd->io->bytes_to_write)
            {
                //Write mode
                //Send slave address with READ bit NOT set
                i2c_send_7bit_address(i2c_dd->i2c_base, i2c_dd->io->slave_address, 0);

            } else
            {
                //We have to read some bytes

                //Send slave address with READ bit set
                i2c_send_7bit_address(i2c_dd->i2c_base, i2c_dd->io->slave_address, I2C_READ);

                if(i2c_dd->io->bytes_to_read == 2)
                {
                    //We have two bytes to read

                    //Send NACK after receiving next byte
                    i2c_nack_next(i2c_dd->i2c_base);    //Set POS
                    i2c_enable_ack(i2c_dd->i2c_base);
                }

                if(i2c_dd->io->bytes_to_read > 2)
                {
                    i2c_enable_ack(i2c_dd->i2c_base);
                }

                //Now waiting for ADDR flag to be set (I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)

            }

            break;

        //EV6
        case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:

            /* Clear ADDR Register */
            i2c_sr1 = (I2C_SR1(i2c_dd->i2c_base));
            i2c_sr2 = (I2C_SR2(i2c_dd->i2c_base));

            //TEST
            if(i2c_dd->io->bytes_to_write)
            {
                //Send data and increment pointer
                i2c_send_data(i2c_dd->i2c_base, *(i2c_dd->io->writemem)++);

                //Decrement counter
                i2c_dd->io->bytes_to_write--;

                //Enable interrupt ITBUFEN (To catch TXE)
                //i2c_enable_interrupt(I2C1, I2C_CR2_ITBUFEN);
            }

            break;

        //EV8
        case I2C_EVENT_MASTER_BYTE_TRANSMITTING:

            /* Clear ADDR Register */
            i2c_sr1 = (I2C_SR1(i2c_dd->i2c_base));
            //i2c_sr2 = (I2C1_SR2);

//          if(i2c_bytes_to_write)
//          {
//              //Send data and increment pointer
//              i2c_send_data(I2C1, *ptr_i2c_writemem++);
//
//              //Decrement counter
//              i2c_bytes_to_write--;
//              //Now wait for BTF
//
//              //ToDo: Decrease interrupt priority _here_ to match FreeRTOS API requirements
//              //Give semaphore after EV8_2
//          }
//
//          //Decrease interrupt priority
//          if((i2c_bytes_to_write == 0) && (i2c_bytes_to_read == 0))
//          {
//              //Change interrupt priority - event interrupt
//              nvic_set_priority(NVIC_I2C1_EV_IRQ, NVIC_PRIORITY_SYSCALL_I2C1);    //Priority equal to configLIBRARY_SYSCALL_INTERRUPT_PRIORITY
//          }

            break;

        //EV8_2
        case I2C_EVENT_MASTER_BYTE_TRANSMITTED:

            if(i2c_dd->io->bytes_to_write)
            {
                //Decrease interrupt priority
                if((i2c_dd->io->bytes_to_write == 1) && (i2c_dd->io->bytes_to_read == 0))
                {
                    //Change interrupt priority - event interrupt
                    nvic_set_priority( i2c_ev_nvic_vectors[i2c_num], NVIC_PRIORITY_SYSCALL_I2C1);    //Priority equal to configLIBRARY_SYSCALL_INTERRUPT_PRIORITY
                }

                //Send data and increment pointer
                i2c_send_data(i2c_dd->i2c_base, *(i2c_dd->io->writemem)++);

                //Decrement counter
                i2c_dd->io->bytes_to_write--;

                //Now wait for next BTF

            } else
            {
                //If we have something to read from I2C slave
                if(i2c_dd->io->bytes_to_read)
                {
                    //Increase interrupt priority - event interrupt
                    nvic_set_priority( i2c_ev_nvic_vectors[i2c_num], NVIC_PRIORITY_SYSCALL_I2C1_HIGH);   //Priority greater than configLIBRARY_SYSCALL_INTERRUPT_PRIORITY

                    //Send repeated start
                    i2c_send_start(i2c_dd->i2c_base);
                } else
                {
                    //Writing finished, nothing to read

                    //ToDo: Remove is_busy usage
                    if(i2c_dd->io->is_busy) //WTF
                    {
                        //Send STOP and complete transaction
                        i2c_send_stop(i2c_dd->i2c_base);

                        //Disable interrupt BUF
                        i2c_disable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITBUFEN);

                        i2c_dd->io->is_busy = 0;

                        xSemaphoreGiveFromISR( i2c_dd->io->xSemaphoreI2CReady, pxHigherPriorityTaskWoken );
                    }
                }
            }

            break;

        //EV6
        case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:   //ADDR bit is set

            //Enable interrupt ITBUFEN (To catch RXNE)
            i2c_enable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITBUFEN);

            //If only one byte to read
            if(i2c_dd->io->bytes_to_read == 1)
            {
                //Do not send ACK after receiving this byte, because it is last
                i2c_nack_current(i2c_dd->i2c_base); //POS = 0
                i2c_disable_ack(i2c_dd->i2c_base);

                /* Clear ADDR Register */
                i2c_sr1 = (I2C_SR1(i2c_dd->i2c_base));
                i2c_sr2 = (I2C_SR2(i2c_dd->i2c_base));

                i2c_send_stop(i2c_dd->i2c_base);

                //Change interrupt priority
                nvic_set_priority( i2c_ev_nvic_vectors[i2c_num], NVIC_PRIORITY_SYSCALL_I2C1);    //Priority greater equal to configLIBRARY_SYSCALL_INTERRUPT_PRIORITY

            } else
            if(i2c_dd->io->bytes_to_read == 2)
            {
                //We have two bytes to read

                /* Clear ADDR Register */
                i2c_sr1 = (I2C_SR1(i2c_dd->i2c_base));
                i2c_sr2 = (I2C_SR2(i2c_dd->i2c_base));

                //Clear ACK
                i2c_disable_ack(i2c_dd->i2c_base);

                //Change interrupt priority
                nvic_set_priority( i2c_ev_nvic_vectors[i2c_num], NVIC_PRIORITY_SYSCALL_I2C1);    //Priority greater equal to configLIBRARY_SYSCALL_INTERRUPT_PRIORITY

                //ToDo: Decrease interrupt priority _here_ to match FreeRTOS API requirements
                //Give semaphore after BTF

                //Now wait for BTF is set
            }
//          else
//          {
//              //Regular receiving
//
//              /* Clear ADDR Register */
//              i2c_sr1 = (I2C1_SR1);
//              i2c_sr2 = (I2C1_SR2);
//          }

            break;

        //EV7_3
        case I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_SR1_BTF:

            if(i2c_dd->io->bytes_to_read == 2)
            {
                //Send STOP
                i2c_send_stop(i2c_dd->i2c_base);

                //Read I2C_DR twice
                i2c_dr = i2c_get_data(i2c_dd->i2c_base);
                *(i2c_dd->io->readmem) = i2c_dr;
                (i2c_dd->io->readmem)++;

                i2c_dr = i2c_get_data(i2c_dd->i2c_base);
                *(i2c_dd->io->readmem) = i2c_dr;
                (i2c_dd->io->readmem)++;

                /* Clear ADDR Register */
//              i2c_sr1 = (I2C1_SR1);
//              i2c_sr2 = (I2C1_SR2);

                //Disable interrupt BUF
                i2c_disable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITBUFEN);

                //All readings finished
                i2c_dd->io->bytes_to_read = 0;

                i2c_dd->io->is_busy = 0;

                xSemaphoreGiveFromISR( i2c_dd->io->xSemaphoreI2CReady, pxHigherPriorityTaskWoken );

            } else
            {
                if(i2c_dd->io->bytes_to_read == 3)
                {
                    //Clear ACK
                    i2c_disable_ack(i2c_dd->i2c_base);

                    //Read DataN-2
                    i2c_dr = i2c_get_data(i2c_dd->i2c_base);
                    *(i2c_dd->io->readmem) = i2c_dr;
                    (i2c_dd->io->readmem)++;

                    //Send STOP
                    i2c_send_stop(i2c_dd->i2c_base);

                    //Read DataN-1
                    i2c_dr = i2c_get_data(i2c_dd->i2c_base);
                    *(i2c_dd->io->readmem) = i2c_dr;
                    (i2c_dd->io->readmem)++;

                    //Enable BUF interrupt to receive RXNE
                    i2c_enable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITBUFEN);

                    //Last byte left
                    i2c_dd->io->bytes_to_read = 1;

                    //Decrease interrupt priority for next interrupt to match FreeRTOS API requirements
                    nvic_set_priority( i2c_ev_nvic_vectors[i2c_num], NVIC_PRIORITY_SYSCALL_I2C1);    //Priority equal to configLIBRARY_SYSCALL_INTERRUPT_PRIORITY

                    //Wait for RXNE and give semaphore after RXNE

                }

            }

            break;

        //EV7
        case I2C_EVENT_MASTER_BYTE_RECEIVED:

            //The only (or last) byte to read
            if(i2c_dd->io->bytes_to_read == 1)
            {
                //Read I2C_DR once
                i2c_dr = i2c_get_data(i2c_dd->i2c_base);
                *(i2c_dd->io->readmem) = i2c_dr;
                (i2c_dd->io->readmem)++;

                //Disable interrupt BUF
                i2c_disable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITBUFEN);

                //Disable interrupt EVENT
                i2c_disable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITEVTEN);

                //All readings finished
                i2c_dd->io->bytes_to_read = 0;

                i2c_dd->io->is_busy = 0;

                xSemaphoreGiveFromISR( i2c_dd->io->xSemaphoreI2CReady, pxHigherPriorityTaskWoken );

            } else
            {
                //Normal receive
                if(i2c_dd->io->bytes_to_read > 3)
                {
                    i2c_dr = i2c_get_data(i2c_dd->i2c_base);
                    *(i2c_dd->io->readmem) = i2c_dr;
                    (i2c_dd->io->readmem)++;

                    //Decrement counter
                    i2c_dd->io->bytes_to_read--;

                    //3 bytes left
                    if(i2c_dd->io->bytes_to_read == 3)
                    {
                        //Disable interrupt BUF
                        i2c_disable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITBUFEN);
                    }
                }
            }

            break;


        default:

            if((lastevent == I2C_SR1_RxNE) && (i2c_dd->io->prev_state == (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_SR1_BTF)))
            {
                i2c_dr = i2c_get_data(i2c_dd->i2c_base);
                *(i2c_dd->io->readmem) = i2c_dr;
                (i2c_dd->io->readmem)++;

                i2c_dd->io->is_busy = 0;

                //Disable interrupt BUF
                i2c_disable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITBUFEN);

                //Disable interrupt EVENT
                i2c_disable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITEVTEN);

                //All readings finished
                i2c_dd->io->bytes_to_read = 0;

                xSemaphoreGiveFromISR( i2c_dd->io->xSemaphoreI2CReady, pxHigherPriorityTaskWoken );
            }

//          if(prev_state == I2C_EVENT_MASTER_MODE_SELECT)
//          {
//              gpio_set(GPIO_PORT_LEDS, GPIO_PIN_LED_RED);
//          }

            /* Clear ADDR Register */
//          i2c_sr1 = (I2C1_SR1);
//          i2c_sr2 = (I2C1_SR2);

            break;
    };


    i2c_dd->io->prev_state = lastevent;
}


void i2c_register_driver(i2c_driver_descriptor_t *i2c_dd)
{
    uint8_t i2c_num;

    i2c_num = i2c_get_num_by_i2c_base(i2c_dd->i2c_base);

    i2c_dd->i2c_num = i2c_num;

    i2c_dd->io = (i2c_io_t*)pvPortMalloc(sizeof(i2c_io_t));

    //Init GPIOs required for I2C controller
    gpio_mode_setup(i2c_dd->gpio_scl->port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, i2c_dd->gpio_scl->pin);
    gpio_mode_setup(i2c_dd->gpio_sda->port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, i2c_dd->gpio_sda->pin);

    gpio_set_output_options(i2c_dd->gpio_scl->port, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, i2c_dd->gpio_scl->pin);
    gpio_set_output_options(i2c_dd->gpio_sda->port, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, i2c_dd->gpio_sda->pin);

    gpio_set_af(i2c_dd->gpio_scl->port, i2c_gpio_af[i2c_num], i2c_dd->gpio_scl->pin);
    gpio_set_af(i2c_dd->gpio_sda->port, i2c_gpio_af[i2c_num], i2c_dd->gpio_sda->pin);

    //Enable I2C peripheral clock in RCC
    rcc_periph_clock_enable( i2c_rcc_periph[i2c_num] );

    //Disable and reset peripheral
    i2c_peripheral_disable(i2c_dd->i2c_base);
    i2c_reset(i2c_dd->i2c_base);

    i2c_set_clock_frequency(i2c_dd->i2c_base, I2C_CR2_FREQ_42MHZ);

    if(i2c_dd->settings->speed == i2c_speed_400k)
    {
        //400 KHz
        i2c_set_fast_mode(i2c_dd->i2c_base);

        i2c_set_ccr(i2c_dd->i2c_base, 35);

        i2c_set_trise(i2c_dd->i2c_base, 43);
    } else
    {
        //100 KHz
        i2c_set_standard_mode(i2c_dd->i2c_base);

        i2c_set_ccr(i2c_dd->i2c_base, 360);

        i2c_set_trise(i2c_dd->i2c_base, 36);
    }


    //Enable peripheral
    i2c_peripheral_enable(i2c_dd->i2c_base);

    //Set interrupt priorities
    nvic_set_priority( i2c_er_nvic_vectors[i2c_num], NVIC_PRIORITY_SYSCALL_I2C1);        //Priority equal configLIBRARY_SYSCALL_INTERRUPT_PRIORITY
    nvic_set_priority( i2c_ev_nvic_vectors[i2c_num], NVIC_PRIORITY_SYSCALL_I2C1_HIGH);   //Priority greater than configLIBRARY_SYSCALL_INTERRUPT_PRIORITY

    //Clear pending IRQs
    nvic_clear_pending_irq( i2c_ev_nvic_vectors[i2c_num] );
    nvic_clear_pending_irq( i2c_er_nvic_vectors[i2c_num] );

    //Enable interrupts in NVIC
    nvic_enable_irq( i2c_ev_nvic_vectors[i2c_num] );
    nvic_enable_irq( i2c_er_nvic_vectors[i2c_num] );

    //Initialize semaphore
    i2c_dd->io->xSemaphoreI2CReady = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_dd->io->xSemaphoreI2CReady);

    //Initialize mutex
    i2c_dd->io->xSemaphoreI2CReadyMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(i2c_dd->io->xSemaphoreI2CReadyMutex);

    //Register callback for I2C ISR
    i2c_dd_pointers[i2c_num] = i2c_dd;

    //Enable I2C Error interrupt
    i2c_enable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITERREN);

    //Register driver in Driver Manager
    driver_manager_register_driver(i2c_dd->nameStr, i2c_dd);
}




//I2C_GetLastEvent implementation
__inline__ uint32_t __attribute__((always_inline)) I2C_GetLastEvent(uint32_t i2c)
{
	uint32_t lastevent = 0;
	uint32_t flag1 = 0, flag2 = 0;

	/* Read the I2Cx status register */
	flag1 = I2C_SR1(i2c);
	flag2 = I2C_SR2(i2c);
	flag2 = flag2 << 16;

	/* Get the last event value from I2C status register */
	lastevent = (flag1 | flag2) & FLAG_Mask;

	/* Return status */
	return lastevent;
}




//I2C1 Event IRQ Handler
void I2C1_EV_IRQHandler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(i2c_dd_pointers[0] != NULL)
	            i2c_event_callback(i2c_dd_pointers[0], &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//I2C1 Error IRQ Handler
void I2C1_ER_IRQHandler(void)
{
	uint16_t i2c_sr1 = I2C_SR1(I2C1);
	uint16_t i2c_sr2 = I2C_SR2(I2C1);

	uint16_t i2c_dr = I2C_DR(I2C1);

	//Acknowledge failure
	if(i2c_sr1 & I2C_SR1_AF)
	{
		//Clear AF bit
		I2C_SR1(I2C1) &= ~I2C_SR1_AF;
	}

	//Increment error counter
}


void I2C2_EV_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(i2c_dd_pointers[1] != NULL)
            i2c_event_callback(i2c_dd_pointers[1], &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void I2C2_ER_IRQHandler(void)
{
    uint16_t i2c_sr1 = I2C_SR1(I2C2);
    uint16_t i2c_sr2 = I2C_SR2(I2C2);

    uint16_t i2c_dr = I2C_DR(I2C2);

    //Acknowledge failure
    if(i2c_sr1 & I2C_SR1_AF)
    {
        //Clear AF bit
        I2C_SR1(I2C2) &= ~I2C_SR1_AF;
    }
}



void I2C3_EV_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(i2c_dd_pointers[2] != NULL)
            i2c_event_callback(i2c_dd_pointers[2], &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void I2C3_ER_IRQHandler(void)
{
    uint16_t i2c_sr1 = I2C_SR1(I2C3);
    uint16_t i2c_sr2 = I2C_SR2(I2C3);

    uint16_t i2c_dr = I2C_DR(I2C3);

    //Acknowledge failure
    if(i2c_sr1 & I2C_SR1_AF)
    {
        //Clear AF bit
        I2C_SR1(I2C3) &= ~I2C_SR1_AF;
    }
}

//Send to I2C device wb_count of bytes from *write_array, then read from device rb_count of bytes to *read_array.
//wb_count >= 1, rb_count >= 0.
//If rb_count is 0 or read_array is NULL, no read operation is performed.
static int8_t i2c_master_transaction_8b_start(const i2c_driver_descriptor_t *i2c_dd, const uint8_t slave_address, const uint8_t wb_count, const uint8_t *write_array, const uint8_t rb_count, uint8_t *read_array)
{
    int8_t res = 0;

    uint8_t i2c_num = i2c_get_num_by_i2c_base(i2c_dd->i2c_base);

    if(i2c_dd->io->is_busy == 0)
    {
        i2c_dd->io->is_busy = 1;
    } else
        return 1;   //Error, I2C1 is busy

    if(wb_count == 0)
        return 2;   //Error, wb_count must be > 0

    i2c_dd->io->slave_address = slave_address & 0x7F;

    i2c_dd->io->writemem = write_array;

    if(rb_count == 0)
        i2c_dd->io->readmem = NULL;
    else
        i2c_dd->io->readmem = read_array;

    i2c_dd->io->bytes_to_write = wb_count;
    i2c_dd->io->bytes_to_read = rb_count;

    nvic_set_priority( i2c_ev_nvic_vectors[i2c_num] , NVIC_PRIORITY_SYSCALL_I2C1_HIGH);   //Priority greater than configLIBRARY_SYSCALL_INTERRUPT_PRIORITY

    i2c_enable_interrupt(i2c_dd->i2c_base, I2C_CR2_ITEVTEN /*| I2C_CR2_ITBUFEN*/) ;

    i2c_nack_current(i2c_dd->i2c_base); //POS = 0

    //After sending START we will get into interrupt handler
    i2c_send_start(i2c_dd->i2c_base);

    return res;
}


uint8_t i2c_read_reg8(const i2c_driver_descriptor_t *i2c_dd, const uint8_t slave_address, const uint8_t reg)
{
    uint8_t res = 0;

    xSemaphoreTake( i2c_dd->io->xSemaphoreI2CReady, portMAX_DELAY);
    i2c_master_transaction_8b_start(i2c_dd, slave_address, 1, &reg, 1, &res);

    if(xSemaphoreTake( i2c_dd->io->xSemaphoreI2CReady, ( TickType_t ) 20) == pdFALSE)
    {
        nvic_set_priority( i2c_ev_nvic_vectors[ i2c_get_num_by_i2c_base(i2c_dd->i2c_base) ] , NVIC_PRIORITY_SYSCALL_I2C1);

        //Send STOP
        i2c_send_stop(i2c_dd->i2c_base);

        i2c_dd->io->is_busy = 0;
    }

    xSemaphoreGive( i2c_dd->io->xSemaphoreI2CReady );

    return res;
}


void i2c_write_reg8(const i2c_driver_descriptor_t *i2c_dd, const uint8_t slave_address, const uint8_t reg, const uint8_t data)
{
  uint8_t i2c_wb[2];

  i2c_wb[0] = reg;
  i2c_wb[1] = data;

  xSemaphoreTake( i2c_dd->io->xSemaphoreI2CReady, portMAX_DELAY);
  i2c_master_transaction_8b_start(i2c_dd, slave_address, 2, i2c_wb, 0, NULL);
  xSemaphoreTake( i2c_dd->io->xSemaphoreI2CReady, portMAX_DELAY);
  xSemaphoreGive( i2c_dd->io->xSemaphoreI2CReady );
}


void i2c_read_multi(const i2c_driver_descriptor_t *i2c_dd, const uint8_t slave_address, const uint8_t reg_start, const uint8_t count, uint8_t *ptr_read_buffer)
{
  uint8_t i2c_wb[1];

  i2c_wb[0] = reg_start;

  xSemaphoreTake( i2c_dd->io->xSemaphoreI2CReady,  portMAX_DELAY);
  i2c_master_transaction_8b_start(i2c_dd, slave_address, 1, i2c_wb, count, ptr_read_buffer);
  xSemaphoreTake( i2c_dd->io->xSemaphoreI2CReady,  portMAX_DELAY);
  xSemaphoreGive( i2c_dd->io->xSemaphoreI2CReady );
}


