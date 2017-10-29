/*
 * gnss.h
 *
 *  Created on: 20 янв. 2017 г.
 *      Author: frost
 */

#ifndef GPS_H_
#define GPS_H_

//#include <stdint.h>

#include "mcu/mcu_usart.h"

#include "helpers/fifo.h"

typedef struct
{
    const char* nameStr;  //"GPSx"

    //U(S)ART driver used by GPS
    const uart_driver_descriptor_t* uart_dd;

    //Pointer to settings structure
//    const cli_settings_t* settings;

    //Pointer to interface FIFO
    volatile struct fifo_t* fifo;

    volatile uint8_t fifo_overflow_flag;

    //Last completely received line
    volatile int32_t last_received_msg;

    //Buffer for single NMEA message read out from FIFO
    volatile char* current_message;

    //GPS task handle (used for notification)
//    volatile TaskHandle_t GPS_Task_Handle;

} gnss_driver_descriptor_t;



uint8_t gnss_nmea_fifo_write_char_callback(gnss_driver_descriptor_t *gnss_dd, const char c);

uint8_t gnss_read_nmea_message_from_fifo(fifo_t *gps_fifo, char *message);

//nmea_message_type_t gps_new_message_received(gps_driver_descriptor_t *gps_dd);
//
//nmea_message_type_t nmea_get_message_type(const char *message);
//uint8_t nmea_parse_gga(const char *message, nmea_gpgga_t *data);
//char* nmea_calculate_checksum(const char *message, uint8_t *checksum);

#endif /* GPS_H_ */

