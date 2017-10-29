/*
 * dma_manager.c
 *
 *  Created on: 2 сент. 2017 г.
 *      Author: frost
 */

#include <libopencmsis/dispatch/irqhandlers.h>

#include "buildparams.h"

#include "libopencm3_headers.h"

#include <libopencm3/stm32/dma.h>

#include "FreeRTOS.h"


#define DMA_STREAMS_COUNT       ((uint8_t)(8))

const uint8_t dma1_streams_array[] = {
        DMA_STREAM0,
        DMA_STREAM1,
        DMA_STREAM2,
        DMA_STREAM3,
        DMA_STREAM4,
        DMA_STREAM5,
        DMA_STREAM6,
        DMA_STREAM7
};

const uint8_t dma2_streams_array[] = {
        DMA_STREAM0,
        DMA_STREAM1,
        DMA_STREAM2,
        DMA_STREAM3,
        DMA_STREAM4,
        DMA_STREAM5,
        DMA_STREAM6,
        DMA_STREAM7
};

const uint8_t dma1_streams_nvic_vectors[] = {
        NVIC_DMA1_STREAM0_IRQ,
        NVIC_DMA1_STREAM1_IRQ,
        NVIC_DMA1_STREAM2_IRQ,
        NVIC_DMA1_STREAM3_IRQ,
        NVIC_DMA1_STREAM4_IRQ,
        NVIC_DMA1_STREAM5_IRQ,
        NVIC_DMA1_STREAM6_IRQ,
        NVIC_DMA1_STREAM7_IRQ
};

const uint8_t dma2_streams_nvic_vectors[] = {
        NVIC_DMA2_STREAM0_IRQ,
        NVIC_DMA2_STREAM1_IRQ,
        NVIC_DMA2_STREAM2_IRQ,
        NVIC_DMA2_STREAM3_IRQ,
        NVIC_DMA2_STREAM4_IRQ,
        NVIC_DMA2_STREAM5_IRQ,
        NVIC_DMA2_STREAM6_IRQ,
        NVIC_DMA2_STREAM7_IRQ
};



static volatile uint32_t *dma1_handlers[DMA_STREAMS_COUNT];
static volatile uint32_t *dma2_handlers[DMA_STREAMS_COUNT];


//Void callback for unhandled DMA interrupts
static void dma_void_callback(void*);

static void dma_void_callback(void* any_param)
{

}

//DMA callback declarations
void (*dma1_stream0_callback)(void*);
void (*dma1_stream1_callback)(void*);
void (*dma1_stream2_callback)(void*);
void (*dma1_stream3_callback)(void*);
void (*dma1_stream4_callback)(void*);
void (*dma1_stream5_callback)(void*);
void (*dma1_stream6_callback)(void*);
void (*dma1_stream7_callback)(void*);

void (*dma2_stream0_callback)(void*);
void (*dma2_stream1_callback)(void*);
void (*dma2_stream2_callback)(void*);
void (*dma2_stream3_callback)(void*);
void (*dma2_stream4_callback)(void*);
void (*dma2_stream5_callback)(void*);
void (*dma2_stream6_callback)(void*);
void (*dma2_stream7_callback)(void*);


//DMA ISRs

//ToDo: Add xHigherPriorityTaskWoken to every handler
void DMA1_STREAM0_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM0);

    dma1_stream0_callback( dma1_handlers[0] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM0, DMA_TCIF);
}

void DMA1_STREAM1_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM1);

    dma1_stream1_callback( dma1_handlers[1] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM1, DMA_TCIF);
}

void DMA1_STREAM2_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM2);

    dma1_stream2_callback( dma1_handlers[2] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM2, DMA_TCIF);
}

void DMA1_STREAM3_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM3);

    dma1_stream3_callback( dma1_handlers[3] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM3, DMA_TCIF);
}

void DMA1_STREAM4_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM4);

    dma1_stream4_callback( dma1_handlers[4] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM4, DMA_TCIF);
}

void DMA1_STREAM5_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM5);

    dma1_stream5_callback( dma1_handlers[5] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);
}

void DMA1_STREAM6_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM6);

    dma1_stream6_callback( dma1_handlers[6] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
}

void DMA1_STREAM7_IRQHandler(void)
{
    //Disable DMA stream
    dma_disable_stream(DMA1, DMA_STREAM7);

    dma1_stream7_callback( dma1_handlers[7] );

    dma_clear_interrupt_flags(DMA1, DMA_STREAM7, DMA_TCIF);
}

void DMA2_STREAM0_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM0) &= ~DMA_SxCR_EN;

    dma2_stream0_callback( dma2_handlers[0] );

    DMA_LIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM0));
}

void DMA2_STREAM1_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM1) &= ~DMA_SxCR_EN;

    dma2_stream1_callback( dma2_handlers[1] );

    DMA_LIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM1));
}

void DMA2_STREAM2_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM2) &= ~DMA_SxCR_EN;

    dma2_stream2_callback( dma2_handlers[2] );

    DMA_LIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM2));
}

void DMA2_STREAM3_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM3) &= ~DMA_SxCR_EN;

    dma2_stream3_callback( dma2_handlers[3] );

    DMA_LIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM3));
}

void DMA2_STREAM4_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM4) &= ~DMA_SxCR_EN;

    dma2_stream4_callback( dma2_handlers[4] );

    DMA_HIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM4));
}

void DMA2_STREAM5_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM5) &= ~DMA_SxCR_EN;

    dma2_stream5_callback( dma2_handlers[5] );

    DMA_HIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM5));
}

void DMA2_STREAM6_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM6) &= ~DMA_SxCR_EN;

    dma2_stream6_callback( dma2_handlers[6] );

    DMA_HIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM6));
}

void DMA2_STREAM7_IRQHandler(void)
{
    //Disable DMA stream
    DMA_SCR(DMA2, DMA_STREAM7) &= ~DMA_SxCR_EN;

    dma2_stream7_callback( dma2_handlers[7] );

    DMA_HIFCR(DMA2) = (DMA_TCIF << DMA_ISR_OFFSET(DMA_STREAM7));
}


// **************** DMA manager ****************

//Register callback function for specified DMA controller and stream
static void dma_manager_set_callback(const uint32_t dma_controller, const uint8_t dma_stream, void *callback_ptr)
{
    if(callback_ptr == NULL)
    {
        callback_ptr = &dma_void_callback;
    }

    if(dma_controller == DMA1)
    {
        switch(dma_stream)
        {
            case 0:
                dma1_stream0_callback = callback_ptr;
                break;

            case 1:
                dma1_stream1_callback = callback_ptr;
                break;

            case 2:
                dma1_stream2_callback = callback_ptr;
                break;

            case 3:
                dma1_stream3_callback = callback_ptr;
                break;

            case 4:
                dma1_stream4_callback = callback_ptr;
                break;

            case 5:
                dma1_stream5_callback = callback_ptr;
                break;

            case 6:
                dma1_stream6_callback = callback_ptr;
                break;

            case 7:
                dma1_stream7_callback = callback_ptr;
                break;

            default:
                break;

        };
    } else
    if(dma_controller == DMA2)
    {
        switch(dma_stream)
        {
            case 0:
                dma2_stream0_callback = callback_ptr;
                break;

            case 1:
                dma2_stream1_callback = callback_ptr;
                break;

            case 2:
                dma2_stream2_callback = callback_ptr;
                break;

            case 3:
                dma2_stream3_callback = callback_ptr;
                break;

            case 4:
                dma2_stream4_callback = callback_ptr;
                break;

            case 5:
                dma2_stream5_callback = callback_ptr;
                break;

            case 6:
                dma2_stream6_callback = callback_ptr;
                break;

            case 7:
                dma2_stream7_callback = callback_ptr;
                break;

            default:
                break;

        };
    }
}


//Initialize DMA manager
//Must be called before enabling DMA
void dma_manager_init(void)
{
    //Fill dmax_handlers arrays with zeroes (NULL)
    memset(dma1_handlers, NULL, DMA_STREAMS_COUNT);
    memset(dma2_handlers, NULL, DMA_STREAMS_COUNT);

    //Set void callbacks for all DMA streams
    for(uint8_t i = 0; i < DMA_STREAMS_COUNT; i++)
    {
        //Disable all DMA streams
        dma_disable_stream(DMA1, dma1_streams_array[i]);
        dma_disable_stream(DMA2, dma2_streams_array[i]);

        //Disable all DMA interrupts in NVIC
        nvic_disable_irq(dma1_streams_nvic_vectors[i]);
        nvic_disable_irq(dma2_streams_nvic_vectors[i]);

        //Register void callbacks for all DMA1 & DMA2 ISRs
        dma_manager_set_callback(DMA1, dma1_streams_array[i], &dma_void_callback);
        dma_manager_set_callback(DMA2, dma2_streams_array[i], &dma_void_callback);
    }
}

//Return pointer to handler of given dma_stream, or NULL, if specified dma_stream at dma_controller is currently unhandled
uint32_t *dma_manager_get_dma_handler_instance(const uint32_t dma_controller, const uint8_t dma_stream)
{
    if(dma_stream < DMA_STREAMS_COUNT)
    {
        switch(dma_controller)
        {
            case DMA1:
                return dma1_handlers[dma_stream];
                break;

            case DMA2:
                return dma2_handlers[dma_stream];
                break;

            default:
                return NULL;
                break;
        };
    }

    return NULL;
}


uint8_t dma_manager_register_handler(const uint32_t dma_controller, const uint8_t dma_stream, uint32_t *driver_handle, void *callback_ptr)
{
    //If specified dma_stream is unhandled
    if(dma_manager_get_dma_handler_instance(dma_controller, dma_stream) == NULL)
    {
        if(dma_controller == DMA1)
        {
            //Enable DMA1 controller
            rcc_periph_clock_enable(RCC_DMA1);

            //Add handler instance
            dma1_handlers[dma_stream] = driver_handle;
        } else
        if(dma_controller == DMA2)
        {
            //Enable DMA2 controller
            rcc_periph_clock_enable(RCC_DMA2);

            //Add handler instance
            dma2_handlers[dma_stream] = driver_handle;
        }

        //Register callback function for stream "dma_stream" at controller "dma_controller"
        dma_manager_set_callback(dma_controller, dma_stream, callback_ptr);

        return 1;
    } else
    {
        return 0;
    }
}

uint8_t dma_manager_get_dma_nvic_vector(const uint32_t dma_controller, uint8_t dma_stream)
{
    if(dma_stream < DMA_STREAMS_COUNT)
    {
        switch(dma_controller)
        {
            case DMA1:
                return dma1_streams_nvic_vectors[dma_stream];
                break;

            case DMA2:
                return dma2_streams_nvic_vectors[dma_stream];
                break;

        };
    }
}



