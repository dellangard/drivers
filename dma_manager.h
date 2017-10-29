/*
 * dma_manager.h
 *
 *  Created on: 2 сент. 2017 г.
 *      Author: frost
 */

#ifndef DMA_MANAGER_H_
#define DMA_MANAGER_H_

typedef struct
{
    uint32_t priority;
    uint32_t channel;
    uint8_t stream;
} dma_config_t;

typedef struct
{
    uint32_t channel;
    uint8_t stream;
} dma_channel_stream_t;


void dma_manager_init(void);

uint32_t *dma_manager_get_dma_handler_instance(const uint32_t dma_controller, const uint8_t dma_stream);

uint8_t dma_manager_register_handler(const uint32_t dma_controller, const uint8_t dma_stream, const uint32_t *driver_handle, void *callback_ptr);



#endif /* DMA_MANAGER_H_ */
