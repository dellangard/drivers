/*
 * driver_manager.c
 *
 *  Created on: 7 авг. 2017 г.
 *      Author: frost
 */


#include <libopencmsis/dispatch/irqhandlers.h>

#include "buildparams.h"

#include "libopencm3_headers.h"

#include "FreeRTOS.h"

#include "driver_manager.h"

//Global variables realated to driver manager
driver_list_t *driver_list = NULL;
driver_list_t *driver_list_start = NULL;


//EXTI manager

#define EXTI_MAX_COUNT  16

static volatile uint32_t *exti_handlers[EXTI_MAX_COUNT];

static const uint8_t exti_nvic_vectors[EXTI_MAX_COUNT] = {
        NVIC_EXTI0_IRQ,
        NVIC_EXTI1_IRQ,
        NVIC_EXTI2_IRQ,
        NVIC_EXTI3_IRQ,
        NVIC_EXTI4_IRQ,
        NVIC_EXTI9_5_IRQ,
        NVIC_EXTI9_5_IRQ,
        NVIC_EXTI9_5_IRQ,
        NVIC_EXTI9_5_IRQ,
        NVIC_EXTI9_5_IRQ,
        NVIC_EXTI15_10_IRQ,
        NVIC_EXTI15_10_IRQ,
        NVIC_EXTI15_10_IRQ,
        NVIC_EXTI15_10_IRQ,
        NVIC_EXTI15_10_IRQ,
        NVIC_EXTI15_10_IRQ
};


static void exti_manager_set_callback(uint8_t exti_number, void *callback_ptr);


//Void callback for unhandled EXTI interrupts
static void exti_void_callback(void*);

static void exti_void_callback(void* any_param)
{

}

//EXTI callback declarations
void (*exti_0_callback)(void*);
void (*exti_1_callback)(void*);
void (*exti_2_callback)(void*);
void (*exti_3_callback)(void*);
void (*exti_4_callback)(void*);
void (*exti_5_callback)(void*);
void (*exti_6_callback)(void*);
void (*exti_7_callback)(void*);
void (*exti_8_callback)(void*);
void (*exti_9_callback)(void*);
void (*exti_10_callback)(void*);
void (*exti_11_callback)(void*);
void (*exti_12_callback)(void*);
void (*exti_13_callback)(void*);
void (*exti_14_callback)(void*);
void (*exti_15_callback)(void*);

//void (*exti_16_callback)(void*);
//void (*exti_17_callback)(void*);
//void (*exti_18_callback)(void*);
//void (*exti_19_callback)(void*);
//void (*exti_20_callback)(void*);
//void (*exti_21_callback)(void*);
//void (*exti_22_callback)(void*);
//void (*exti_23_callback)(void*);



//EXTI ISRs

void EXTI0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    exti_0_callback(&xHigherPriorityTaskWoken);

    exti_reset_request(EXTI0);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EXTI1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    exti_1_callback(&xHigherPriorityTaskWoken);

    exti_reset_request(EXTI1);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EXTI2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    exti_2_callback(&xHigherPriorityTaskWoken);

    exti_reset_request(EXTI2);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EXTI3_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    exti_3_callback(&xHigherPriorityTaskWoken);

    exti_reset_request(EXTI3);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EXTI4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    exti_4_callback(&xHigherPriorityTaskWoken);

    exti_reset_request(EXTI4);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EXTI9_5_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//    uint16_t timer_us10 = tim7_get_value();

    //EXTI 5
    if(exti_get_flag_status(EXTI5))
    {
        exti_5_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI5);
    }

    //EXTI 6
    if(exti_get_flag_status(EXTI6))
    {
        exti_6_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI6);
    }

    //EXTI 7
    if(exti_get_flag_status(EXTI7))
    {
        exti_7_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI7);
    }

    //EXTI 8
    if(exti_get_flag_status(EXTI8))
    {
        exti_8_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI8);
    }

    //Switch FreeRTOS context after leaving ISR if high priority task is pending
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EXTI15_10_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//    uint16_t timer_us10 = tim7_get_value();

    //EXTI 10
    if(exti_get_flag_status(EXTI10))
    {
        exti_10_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI10);
    }

    //EXTI 11
    if(exti_get_flag_status(EXTI11))
    {
        exti_11_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI11);
    }

    //EXTI 12
    if(exti_get_flag_status(EXTI12))
    {
        exti_12_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI12);
    }

    //EXTI 13
    if(exti_get_flag_status(EXTI13))
    {
        exti_13_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI13);
    }

    //EXTI 14
    if(exti_get_flag_status(EXTI14))
    {
        exti_14_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI14);
    }

    //EXTI 15
    if(exti_get_flag_status(EXTI15))
    {
        exti_15_callback(&xHigherPriorityTaskWoken);
        exti_reset_request(EXTI15);
    }

    //Switch FreeRTOS context after leaving ISR if high priority task is pending
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



// **************** EXTI manager ****************


//Initialize EXTI manager
//Must be called before enabling EXTI
void exti_manager_init(void)
{
    //Disable all interrupts from GPIO
    exti_disable_request(0xFFFF);

    //Clear all pending EXTI interrupt requests
    exti_reset_request(0xFFFF);

    //Disable all EXTI interrupts in NVIC
    nvic_disable_irq(NVIC_EXTI0_IRQ);
    nvic_disable_irq(NVIC_EXTI1_IRQ);
    nvic_disable_irq(NVIC_EXTI2_IRQ);
    nvic_disable_irq(NVIC_EXTI3_IRQ);
    nvic_disable_irq(NVIC_EXTI4_IRQ);
    nvic_disable_irq(NVIC_EXTI9_5_IRQ);
    nvic_disable_irq(NVIC_EXTI15_10_IRQ);

    //Fill exti_handlers array with zeroes (NULL)
    memset(exti_handlers, NULL, EXTI_MAX_COUNT);

    //Set void callbacks for all EXTI lines
    for(uint8_t i = 0; i < EXTI_MAX_COUNT; i++)
    {
       exti_manager_set_callback(i, &exti_void_callback);
    }
}



//Return pointer to handler of given exti_number, or NULL, if given exti is currently unhandled
uint32_t *exti_manager_get_exti_handler_instance(uint8_t exti_number)
{
    if(exti_number < EXTI_MAX_COUNT)
    {
        return exti_handlers[exti_number];
    }

    return NULL;
}

//Register callback function for exti_number EXTI line
static void exti_manager_set_callback(uint8_t exti_number, void *callback_ptr)
{
    if(callback_ptr == NULL)
    {
        callback_ptr = &exti_void_callback;
    }

    switch(exti_number)
    {
        case 0:
            exti_0_callback = callback_ptr;
            break;

        case 1:
            exti_1_callback = callback_ptr;
            break;

        case 2:
            exti_2_callback = callback_ptr;
            break;

        case 3:
            exti_3_callback = callback_ptr;
            break;

        case 4:
            exti_4_callback = callback_ptr;
            break;

        case 5:
            exti_5_callback = callback_ptr;
            break;

        case 6:
            exti_6_callback = callback_ptr;
            break;

        case 7:
            exti_7_callback = callback_ptr;
            break;

        case 8:
            exti_8_callback = callback_ptr;
            break;

        case 9:
            exti_9_callback = callback_ptr;
            break;

        case 10:
            exti_10_callback = callback_ptr;
            break;

        case 11:
            exti_11_callback = callback_ptr;
            break;

        case 12:
            exti_12_callback = callback_ptr;
            break;

        case 13:
            exti_13_callback = callback_ptr;
            break;

        case 14:
            exti_14_callback = callback_ptr;
            break;

        case 15:
            exti_15_callback = callback_ptr;
            break;

        default:
            break;

    };
}


uint8_t exti_manager_get_exti_nvic_vector(uint8_t exti_number)
{
    //ToDo: Add range check

    return exti_nvic_vectors[exti_number];
}


enum driver_register_result exti_manager_register_handler(uint32_t *instance_handle, void *callback_ptr, uint8_t exti_number)
{
    //If exti_number is unhandled
    if(exti_manager_get_exti_handler_instance(exti_number) == NULL)
    {
        exti_disable_request((uint32_t)(1 << exti_number));

        exti_reset_request((uint32_t)(1 << exti_number));

        //Add handler instance
        exti_handlers[exti_number] = instance_handle;

        //Register callback function for exti_number EXTI line
        exti_manager_set_callback(exti_number, callback_ptr);

        return driver_register_ok;
    } else
    {
        return driver_register_unknown_error;
    }
}



void exti_manager_deregister_handler(uint8_t exti_number)
{
    exti_disable_request((uint32_t)(1 << exti_number));

    exti_reset_request((uint32_t)(1 << exti_number));

    //Register void callback function for exti_number EXTI line
    exti_manager_set_callback(exti_number, &exti_void_callback);

    //Remove handler instance
    exti_handlers[exti_number] = NULL;
}


uint8_t log2s(uint32_t power_of_two)
{
    uint8_t i = 0;

    while(!(power_of_two & 0x01))
    {
        power_of_two >>= 1;
        i++;
    }

    return i;
}


// ********************************



//Register driver in system by adding it's name and handle to driver_list linked list
enum driver_register_result driver_manager_register_driver(const char* driver_name_string, const uint32_t* driver_handle)
{
    driver_list_t *driver_list_prev = NULL;

    if(driver_list == NULL)
    {
        //Allocate memory for driver_list_t struct
        driver_list = pvPortMalloc(sizeof(driver_list_t));

        if(driver_list == NULL)
            return driver_register_memory_allocation_failed;

        driver_list->prev = NULL;

        //Pointer to first element
        driver_list_start = driver_list;

    } else
    {
        //Allocate memory for driver_list_t struct
        driver_list->next = pvPortMalloc(sizeof(driver_list_t));

        if(driver_list->next == NULL)
            return driver_register_memory_allocation_failed;

        //Move to ->next
        driver_list_prev = driver_list;
        driver_list = driver_list->next;
        driver_list->prev = driver_list_prev;
    }

    driver_list->next = NULL;
    driver_list->nameStr = driver_name_string;  //Pointer to const string in FLASH memory
    driver_list->driver_handle = driver_handle;

    return driver_register_ok;
}

void driver_manager_get_driver_list(char* driver_list_string)
{
    uint8_t driver_index = 0;
    uint16_t i = 0;

    //Local pointer to drive_list
    driver_list_t *driver_list_local = driver_list;

    while(driver_list_local != NULL)
    {
        strcpy(&(driver_list_string[i]), driver_list_local->nameStr);
        i += (strlen(driver_list_local->nameStr));

        driver_list_string[i] = '\n';

        i++;

        driver_index++;

        driver_list_local = driver_list_local->prev;
    }

    driver_list_string[i] = '\0';
}




