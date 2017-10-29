/*
 * gnss.c
 *
 *  Created on: 6 янв. 2017 г.
 *      Author: frost
 */


/*
 *
 * NMEA 0183 Message Format
 *

	$IDMSG,D1,D2,D3,D4,.......,Dn*CS[CR][LF]
	“$”		The “$” signifies the start of a message.

	ID		The identification is a two letter mnemonic which describes the
	source of the navigation information. The GP identification signifies
	a GPS source.

	MSG		The message identification is a three letter mnemonic which
	describes the message content and the number and order of the data
	fields.

	“,”		Commas serve as delimiters for the data fields.

	Dn		Each message contains multiple data fields (Dn) which are delimited
	by commas. The length of the fields can be variable.


	“*”		The asterisk serves as a checksum delimiter.

	CS		The checksum field contains two ASCII characters which indicate
	the hexadecimal value of the checksum.

	[CR][LF]	The carriage return [CR] and line feed [LF] combination terminate
	the message.


	NMEA 0183 standard messages vary in length, but each message is limited to 79
	characters or less. This length limitation excludes the “$” and the [CR][LF]. The
	standard message data field block, including delimiters, is limited to 74 characters or
	less.

	Note – Trimble proprietary messages can exceed 79 characters and the data field
	block of these messages can exceed 74 characters.

*/


#include "gnss.h"

#include <stdint.h>
//#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "driver_manager.h"

#include "helpers/fifo.h"
#include "helpers/nmea.h"

#define GNSS_NMEA_BUFFER_SIZE                 4*96    //Must fit several complete NMEA messages
#define GNSS_MAX_NMEA_MESSAGE_LENGTH         79


void gnss_register_driver(gnss_driver_descriptor_t *gnss_dd, TaskHandle_t Task_Handle)
{
    //Initialize software FIFO for GPS data receiving
    gnss_dd->fifo = fifo_create_dynamic(GNSS_NMEA_BUFFER_SIZE);

    //Allocate memory for temporary buffer used for holding currend message
    gnss_dd->current_message = (char*)pvPortMalloc(GNSS_MAX_NMEA_MESSAGE_LENGTH * sizeof(char));

    //Initialize flags
    gnss_dd->fifo_overflow_flag = 0;
    gnss_dd->last_received_msg = 0;

    //Register driver in Driver Manager
    driver_manager_register_driver(gnss_dd->nameStr, gnss_dd);
}


//GPS NMEA char receive callback (to be called from U(S)ART ISR)
uint8_t gnss_nmea_fifo_write_char_callback(gnss_driver_descriptor_t *gnss_dd, const char c)
{
    //Do not write to FIFO if overflow flag is set
    if(gnss_dd->fifo_overflow_flag == 0)
    {
        //Write byte to FIFO, if no byte written, set FIFO overflow flag
        if(fifo_write_byte(gnss_dd->fifo, c) == 0)
        {
            gnss_dd->fifo_overflow_flag = 1;
        }
    }

	//New line detected (end of NMEA sentence)
	if(c == '\n')
	{
		//Write '\0' after of '\n' to FIFO
		if(fifo_write_byte(gnss_dd->fifo, '\0') != 0)
		{
			//In case of overflow condition while receive, NMEA sentence must be skipped.
			if(gnss_dd->fifo_overflow_flag)
			{
				//Move FIFO head back to previous NMEA sentence end
				fifo_set_head(gnss_dd->fifo, gnss_dd->last_received_msg);

				//Reset FIFO overflow flag
				gnss_dd->fifo_overflow_flag = 0;
			} else //No overflow condition, message was received successfuly
			{
			    //Remember current position as new NMEA sentence start
			    gnss_dd->last_received_msg = fifo_get_head(gnss_dd->fifo);

			    //Return 1 beacause task notification required (valid message written to FIFO)
			    return 1;
			}
		}

	}

	//Return 0 because task notification not required (no valid message in FIFO)
	return 0;
}


uint8_t gnss_read_nmea_message_from_fifo(fifo_t *gnss_fifo, char *message)
{
	//Read FIFO contents to 'message' (Only one message, until first '\0')
	if(fifo_read_str(&gnss_fifo, message) > 1)
	{
	    if(nmea_test_checksum(message) == nmea_field_parsing_ok)
	    {
	        return 1;
	    }

	}

    //Nothing received from FIFO (FIFO is empty) or checksum was incorrect;
    return 0;
}


