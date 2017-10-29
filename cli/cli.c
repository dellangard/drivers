/*
 * cli.c
 *
 *  Created on: 10 февр. 2017 г.
 *      Author: frost
 */

#include "buildparams.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "cli.h"

#include "libopencm3_headers.h"

#include "../mcu/mcu_usart.h"
#include "../helpers/fifo.h"


void cmd_handler_help(cli_driver_descriptor_t *cli_dd, char *cmd_params);


//HELP cmd descriptor
const cli_cmd_descriptor_t cmd_descriptor_help = {
		.cmdname = "help",
		.callback_ptr = &cmd_handler_help,
		.parameters = NULL,
		.values = NULL
};




uint8_t cli_get_parameter_index(cli_parameter_descriptor_t **list, const uint8_t list_length, char *str)
{
	uint8_t i = 0;

	while(i < list_length)
	{
		if(strncmp(str, list[i]->parameter_name, strlen(list[i]->parameter_name)) == 0)
			return i;

		i++;
	}

	return i;
}


void cli_parameter_values_cleanup(const cli_cmd_descriptor_t *cmd_descriptor, const uint8_t array_size)
{
	uint8_t i, j;
	char **c;	//Pointer to pointer to char

	for(i=0; i < array_size; i++)
	{
		if(cmd_descriptor->values[i] != NULL)
		{
			if(cmd_descriptor->parameters[i]->value_type != value_type_none)
			{

				if(cmd_descriptor->parameters[i]->value_type == value_type_string)
				{
					//Each array element is pointer to void
					//Dereferencing it to pointer to char
					c = cmd_descriptor->values[i];

					for(j=0; j < cmd_descriptor->parameters[i]->values_count; j++)
					{
						//Freeing memory block beginning at address (*c)
						vPortFree(*c);

						*c++;
					}

				}

				vPortFree(cmd_descriptor->values[i]);
			}

			cmd_descriptor->values[i] = NULL;
		}
	}
}


cli_parse_result_t cli_parse_command_parameters(const cli_cmd_descriptor_t *cmd_descriptor, const uint8_t parameter_count, const char *parameter_string)
{
	char *current_token_string = NULL;
	char *tail = parameter_string;

	uint8_t parameter_index = 0;
	uint8_t value_index = 0;

	//These variables share the same place in memory and only one is used at any time
	union
	{
		uint8_t *uint8_tmp;
		int32_t *int32_tmp;
		uint32_t *uint32_tmp;
		char** str_tmp;
	} sharedmemvariables;

	//ToDo: Move to sharedmem
	char **char_ptr_ptr;

	//Pointer for dynamically allocated memory
	void* mem_ptr;

	//For any cmd that has parameters array of pointers to values must be declared
	if(cmd_descriptor->values != NULL)
	{
		//Fill array of pointers to parameter values with NULL
		memset(cmd_descriptor->values, NULL, parameter_count);
	}

	//Get first parameter name
	current_token_string = strtok_r(NULL, " :.,\n", &tail);

	while(current_token_string != NULL)
	{
		//Get currently parsed parameter index in cmd_descriptor->parameters array
		parameter_index = cli_get_parameter_index(cmd_descriptor->parameters, parameter_count, current_token_string);

		//End of array reached, parameter not found
		if(parameter_index == parameter_count)
		{
			return result_unknown_parameter;
		}

		//If parameter must have any value
		if(cmd_descriptor->parameters[parameter_index]->value_type != value_type_none)
		{
			if(cmd_descriptor->parameters[parameter_index]->value_type != value_type_string)
			{
				//Get next token - parameter value
				current_token_string = strtok_r(NULL, " :.,\n", &tail);
			}

			if(current_token_string == NULL)
			{
				return result_no_value;
			}

			//Value pointers array must be filled with NULL
			switch(cmd_descriptor->parameters[parameter_index]->value_type)
			{
				case value_type_bool:

					//Allocate memory for <values_count> values (byte size cells)
					mem_ptr = pvPortMalloc(cmd_descriptor->parameters[parameter_index]->values_count);

					if(mem_ptr != NULL)
					{
						//Now cmd_descriptor->values[parameter_index] points to beginning of allocated memory region
						cmd_descriptor->values[parameter_index] = mem_ptr;

						//Parameter may have many values of the same type, so iterate trough all of them
						value_index = 0;

						sharedmemvariables.uint8_tmp = cmd_descriptor->values[parameter_index];

						while(value_index < cmd_descriptor->parameters[parameter_index]->values_count)
						{
							//Place converted value to memory via dereferenced pointer
							*sharedmemvariables.uint8_tmp = strtol(current_token_string, NULL, 2);

							//Read next value - get next token
							current_token_string = strtok_r(NULL, " :.,\n", &tail);

							//Increment memory pointer
							sharedmemvariables.uint8_tmp++;

							value_index++;
						}

					} else
					{
						//Could not allocate memory - exit with error after cleaning
						return result_no_memory;
					}

					break;

				case value_type_int:

					//Allocate memory for <values_count> values (int32_t size cells)
					mem_ptr = pvPortMalloc(sizeof(int32_t) * (cmd_descriptor->parameters[parameter_index]->values_count));

					if(mem_ptr != NULL)
					{
						//Now cmd_descriptor->values[parameter_index] points to beginning of allocated memory region
						cmd_descriptor->values[parameter_index] = mem_ptr;

						//Parameter may have many values of the same type, so iterate trough all of them
						value_index = 0;

						sharedmemvariables.int32_tmp = cmd_descriptor->values[parameter_index];

						while(value_index < cmd_descriptor->parameters[parameter_index]->values_count)
						{
							//Place converted value to memory via dereferenced pointer
							*sharedmemvariables.int32_tmp = strtol(current_token_string, NULL, 10);

							//Read next value - get next token
							current_token_string = strtok_r(NULL, " :.,\n", &tail);

							//Increment memory pointer
							sharedmemvariables.int32_tmp++;

							value_index++;
						}

					} else
					{
						//Could not allocate memory - exit with error after cleaning
						return result_no_memory;
					}


					break;

				case value_type_uint:

					//Allocate memory for <values_count> values (uint32_t size cells)
					mem_ptr = pvPortMalloc(sizeof(uint32_t) * (cmd_descriptor->parameters[parameter_index]->values_count));

					if(mem_ptr != NULL)
					{
						//Now cmd_descriptor->values[parameter_index] points to beginning of allocated memory region
						cmd_descriptor->values[parameter_index] = mem_ptr;

						//Parameter may have many values of the same type, so iterate trough all of them
						value_index = 0;

						sharedmemvariables.uint32_tmp = cmd_descriptor->values[parameter_index];

						while(value_index < cmd_descriptor->parameters[parameter_index]->values_count)
						{
							//Place converted value to memory via dereferenced pointer
							*sharedmemvariables.uint32_tmp = strtol(current_token_string, NULL, 10);

							//Read next value - get next token
							current_token_string = strtok_r(NULL, " :.,\n", &tail);

							//Increment memory pointer
							sharedmemvariables.uint32_tmp++;

							value_index++;
						}

					} else
					{
						//Could not allocate memory - exit with error after cleaning
						return result_no_memory;
					}

					break;

				case value_type_string:

					//Allocate memory for <values_count> pointers to strings (char* size cells)
					mem_ptr = pvPortMalloc(sizeof(char*) * (cmd_descriptor->parameters[parameter_index]->values_count));



					if(mem_ptr != NULL)
					{
						//Now cmd_descriptor->values[parameter_index] points to beginning of allocated memory region
						cmd_descriptor->values[parameter_index] = mem_ptr;

						//Parameter may have many values of the same type, so iterate trough all of them
						value_index = 0;

						char_ptr_ptr = cmd_descriptor->values[parameter_index];

						//sharedmemvariables.str_tmp = cmd_descriptor->values[parameter_index];

						while(value_index < cmd_descriptor->parameters[parameter_index]->values_count)
						{

							while(((*tail) != '\0') && ((*tail) != '\n') && ((*tail) != '"'))
							{
								*tail = "\0";
								tail++;
							}

							if((*tail) == '"')
							{
								current_token_string = tail;
							}
							else
								return result_no_value;

							//Read next value - get next token
							current_token_string = strtok_r(NULL, "\"\n", &tail);

							mem_ptr = pvPortMalloc(sizeof(char*) * strlen(current_token_string));

							*char_ptr_ptr = mem_ptr;

							//sharedmemvariables.str_tmp = mem_ptr;

							strcpy(*char_ptr_ptr, current_token_string);

							//Increment memory pointer
							//sharedmemvariables.str_tmp++;

							*char_ptr_ptr++;

							value_index++;
						}

					} else
					{
						//Could not allocate memory - exit with error after cleaning
						return result_no_memory;
					}

//					sharedmemvariables.str_tmp = pointers_to_value[parameter_index];
//					*sharedmemvariables.str_tmp = current_token_string;
					break;

				default:
					return result_incorrect_value_type;
					break;	//Not necessary
			}

		} else	//Current parameter has no value
		{
			//Dirty method
			//Assign 0x04 instead of NULL
			cmd_descriptor->values[parameter_index] =  0x04;
		}

		//Get next token
		current_token_string = strtok_r(NULL, " :.,\n", &tail);

	}


	return result_ok;
}



void cli_parsing_error_handler(cli_driver_descriptor_t *cli_dd, cli_parse_result_t parsing_result)
{
	switch(parsing_result)
	{
		case result_ok:

			break;

		case result_unknown_cmd:
		    usart_writestr_blocking(cli_dd->uart_dd, "Unknown command\n");
			break;

		case result_unknown_parameter:
		    usart_writestr_blocking(cli_dd->uart_dd, "\nUnknown parameter\n");
			break;

		case result_no_value:
		    usart_writestr_blocking(cli_dd->uart_dd, "\nNo parameter value\n");
			break;

		case result_incorrect_value_type:
		    usart_writestr_blocking(cli_dd->uart_dd, "\nIncorrect parameter type\n");
			break;

		case result_incorrect_parameter_count:
		    usart_writestr_blocking(cli_dd->uart_dd, "\nIncorrect parameter count");
			break;

		case result_no_memory:
		    usart_writestr_blocking(cli_dd->uart_dd, "\nNot enough RAM");
			break;

		default:
		    usart_writestr_blocking(cli_dd->uart_dd, "\nUnknown error\n");
			break;
	};
}


void cmd_handler_help(cli_driver_descriptor_t *cli_dd, char *cmd_params)
{
	uint16_t i=0;

	char *cmd_output_buffer;

	cmd_output_buffer = pvPortMalloc(128);

	if(cmd_output_buffer != NULL)
	{
		sprintf(cmd_output_buffer, "Cmd list knows %d commands:\r\n", cli_dd->cmd_descriptors_array_size);
		usart_writestr_blocking(cli_dd->uart_dd, cmd_output_buffer);

		for(i = 0; i < cli_dd->cmd_descriptors_array_size; i++)
		{
		    usart_writestr_blocking(cli_dd->uart_dd, cli_dd->cmd_descriptors_array[i]->cmdname);
		    usart_writestr_blocking(cli_dd->uart_dd, "\n");
		}

		vPortFree(cmd_output_buffer);
	}
}


void cli_register_driver(cli_driver_descriptor_t *cli_dd, const cli_cmd_descriptor_t **cmd_descriptors_array, const uint16_t cmd_descriptors_array_size /*, TaskHandle_t Task_Handle*/)
{
    cli_dd->cmd_descriptors_array = cmd_descriptors_array;

    cli_dd->cmd_descriptors_array_size = cmd_descriptors_array_size;

	//Initialize software FIFO for CLI data receiving
    cli_dd->fifo = fifo_create_dynamic(CLI_RX_BUFFER_SIZE);

    //Allocate memory for temporary buffer used for holding currend command
    cli_dd->current_cmd = (char*)pvPortMalloc(CLI_RX_BUFFER_SIZE * sizeof(char));

    //Initialize flags
    cli_dd->fifo_overflow_flag = 0;
    cli_dd->last_received_cmd = 0;

//    cli_dd->CLI_Task_Handle = Task_Handle;

    //Register driver in Driver Manager
    driver_manager_register_driver(cli_dd->nameStr, cli_dd);
}

//Called indirectly from UART ISR
uint8_t cli_fifo_write_char_callback(cli_driver_descriptor_t *cli_dd, char c)
{
	//Write byte to FIFO, if no byte written, set FIFO overflow flag
	if(fifo_write_byte(cli_dd->fifo, c) == 0)
	{
	    cli_dd->fifo_overflow_flag = 1;
	}

	//New line detected ('Enter' pressed)
	if(c == CLI_LINE_END_SYMBOL)
	{
		//Write '\0' after of '\n' to FIFO
		if(fifo_write_byte(cli_dd->fifo, '\0') != 0)
		{
			//In case of overflow condition while receiving, cmd must be skipped.
			if(cli_dd->fifo_overflow_flag)
			{
				//Move FIFO head back to last cmd start
				fifo_set_head(cli_dd->fifo, cli_dd->last_received_cmd);

				//Reset FIFO overflow flag
				cli_dd->fifo_overflow_flag = 0;
			} else
			{
				//Remember current position as new cmd start
				cli_dd->last_received_cmd = fifo_get_head(cli_dd->fifo);

				//Notify CLI task with incrementing notification value
				//              vTaskNotifyGiveFromISR( cli_dd->CLI_Task_Handle, cli_dd->uart_dd->pxHigherPriorityTaskWoken );

				//Return 1 if CLI task notification required
				return 1;
			}
		}

	}

	//Return 0 if CLI task notification not required (no valid message in FIFO)
	return 0;
}

cli_parse_result_t cli_parse_command(cli_driver_descriptor_t *cli_dd /*, char *buffer*/)
{
	uint16_t i;

	char *tmp_ptr;
	char *rest = cli_dd->current_cmd; //buffer;

	tmp_ptr = strtok_r(rest, " :.,\n", &rest);

	i = 0;
	while(i < cli_dd->cmd_descriptors_array_size)
	{
		if(strncmp(tmp_ptr, cli_dd->cmd_descriptors_array[i]->cmdname, strlen(cli_dd->cmd_descriptors_array[i]->cmdname)) == 0)
		{
		    cli_dd->cmd_descriptors_array[i]->callback_ptr(cli_dd, rest);

			break;
		}

		i++;
	}

	if(i == cli_dd->cmd_descriptors_array_size)
		return result_unknown_cmd;

	return result_ok;
}


//Called from RTOS task after getting semaphore/notification
void cli_parse_new_message(cli_driver_descriptor_t *cli_dd, uint32_t message_count)
{
	while(message_count > 0)
	{
		if(fifo_read_str(cli_dd->fifo, cli_dd->current_cmd) != 0)
		{
			message_count--;

			cli_parsing_error_handler(cli_dd, cli_parse_command(cli_dd) );

			usart_writestr_blocking(cli_dd->uart_dd, CLI_COMMAND_PROMPT);
		}

	}
}

