/*
 * cli.h
 *
 *  Created on: 14 февр. 2017 г.
 *      Author: frost
 */

#ifndef CLI_H_
#define CLI_H_

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../helpers/fifo.h"

#include "../mcu/mcu_usart.h"

#define CLI_COMMAND_PROMPT  "cli>"
#define CLI_LINE_END_SYMBOL		(char)('\n')

//Allowed parameter value types
typedef enum
{
	value_type_none = 0,	//parameter has no value
	value_type_bool = 1,	//Binary 0, 1
	value_type_int = 2,		//signed int
	value_type_uint = 3,	//unsigned int
	value_type_string = 4	//string
} cmd_parameter_value_type_t;

//Descritor for each parameter
typedef struct
{
	const char *parameter_name;
	const cmd_parameter_value_type_t value_type;
	const uint8_t values_count;
} cli_parameter_descriptor_t;

//Main command descriptor
typedef struct
{
	const char *cmdname;	//Command name string
	const void (*callback_ptr)(void*, char*);	//Pointer to handler function
//	const uint8_t minimal_parameter_count;
	const cli_parameter_descriptor_t **parameters;	//Pointer to parameter descriptor array (may be located in FLASH ROM)
	const void **values;	//Pointer to values array (must be located in RAM, rw access required)
} cli_cmd_descriptor_t;

//Command/parameter parsing result - only result_ok means correct parsing
typedef enum
{
	result_ok = 0,
	result_unknown_cmd = 1,
	result_unknown_parameter = 2,
	result_no_value = 3,
	result_incorrect_value_type = 4,
	result_incorrect_parameter_count = 5,
	result_no_memory = 6

} cli_parse_result_t;


typedef struct
{
    const char* nameStr;  //"CLIx"

    //U(S)ART driver used by interface
    const uart_driver_descriptor_t* uart_dd;

    //Pointer to array of command descriptors
    cli_cmd_descriptor_t **cmd_descriptors_array;

    //Size of command descriptors array
    uint16_t cmd_descriptors_array_size;

    //Pointer to settings structure
//    const cli_settings_t* settings;

    //Pointer to interface FIFO
    volatile fifo_t* fifo;

    volatile uint8_t fifo_overflow_flag;

    //Last completely received line
    volatile int32_t last_received_cmd;

    //Buffer for single command read out from FIFO
    volatile char* current_cmd;

} cli_driver_descriptor_t;


void cli_register_driver(cli_driver_descriptor_t *cli_dd, const cli_cmd_descriptor_t **cmd_descriptors_array, const uint16_t cmd_descriptors_array_size /*, TaskHandle_t Task_Handle*/);

uint8_t cli_fifo_write_char_callback(cli_driver_descriptor_t *cli_dd, char c);

void cli_parse_new_message(cli_driver_descriptor_t *cli_dd, uint32_t message_count);


#endif /* CLI_H_ */
