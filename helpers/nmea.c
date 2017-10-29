/*
 * nmea.c
 *
 *  Created on: 18 окт. 2017 г.
 *      Author: frost
 */

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include "nmea.h"

#define NMEA_HEADER_SENDER_ID_LENGTH		(uint8_t)(2)
#define NMEA_HEADER_MSG_TYPE_LENGTH		(uint8_t)(3)

#define NMEA_BUFFER_SIZE                	(uint8_t)(2*96)    //Must fit complete NMEA message


enum nmea_field_parsing_result nmea_test_checksum(char **message)
{
	int32_t i = 0;
	uint8_t calculated_checksum = 0;
	uint8_t received_checksum;
	char *nmea_message_body = *message;

	//If message does not begin from '$', exit with error
	if(*nmea_message_body == '$')
	{
		nmea_message_body++;
	}
	else
	{
		return nmea_field_parsing_incorrect_symbols;
	}

	while(*nmea_message_body != '*')
	{
		calculated_checksum ^= *nmea_message_body;

		nmea_message_body++;

		i++;

		//If end of string or end of buffer reached, and '*' not found
		if((*nmea_message_body == '\0') || (i == NMEA_BUFFER_SIZE))
		{
			return nmea_field_parsing_incorrect_symbols;
		}
	}

	nmea_message_body++;

	//Convert received checksum value
	received_checksum = (uint8_t)strtoul(nmea_message_body, NULL, 16);

	//Compare checksums
	if(received_checksum != calculated_checksum)
		return nmea_parsing_invalid_checksum;

	//Assign zero characters to calculated_checksum field (needed for further parsing)
	*nmea_message_body = '\0';

	nmea_message_body++;
	*nmea_message_body = '\0';


	return nmea_field_parsing_ok;
}

enum nmea_field_parsing_result nmea_get_id(const char *header_str, struct nmea_message_id_t *header)
{
	char *tail = header_str + 1;	//Skip leading '$'

	header->sender_id = nmea_sender_unknown;
	header->message_id = nmea_message_id_unknown;

	//Get NMEA sender ID
	if(strncmp("GP", tail, NMEA_HEADER_SENDER_ID_LENGTH) == 0)
	{
		header->sender_id = nmea_sender_gps;
	} else
	if(strncmp("GL", tail, NMEA_HEADER_SENDER_ID_LENGTH) == 0)
	{
		header->sender_id = nmea_sender_glonass;
	} else
	if(strncmp("GA", tail, NMEA_HEADER_SENDER_ID_LENGTH) == 0)
	{
		header->sender_id = nmea_sender_galileo;
	} else
	if(strncmp("GN", tail, NMEA_HEADER_SENDER_ID_LENGTH) == 0)
	{
		header->sender_id = nmea_sender_combined;
	} else
		return nmea_field_parsing_incorrect_symbols;	//Sender ID unknown

	//Move pointer forward
	tail = tail + NMEA_HEADER_SENDER_ID_LENGTH;


	//Get NMEA message ID
	if(strncmp("GGA", tail, NMEA_HEADER_MSG_TYPE_LENGTH) == 0)
	{
		header->message_id = nmea_gga;
	} else
	if(strncmp("VTG", tail, NMEA_HEADER_MSG_TYPE_LENGTH) == 0)
	{
		header->message_id = nmea_vtg;
	} else
	if(strncmp("GSV", tail, NMEA_HEADER_MSG_TYPE_LENGTH) == 0)
	{
		header->message_id = nmea_gsv;
	} else
	if(strncmp("ZDA", tail, NMEA_HEADER_MSG_TYPE_LENGTH) == 0)
	{
		header->message_id = nmea_zda;
	} else
	if(strncmp("GLL", tail, NMEA_HEADER_MSG_TYPE_LENGTH) == 0)
	{
		header->message_id = nmea_gll;
	} else
	if(strncmp("RMC", tail, NMEA_HEADER_MSG_TYPE_LENGTH) == 0)
	{
		header->message_id = nmea_rmc;
	} else
		return nmea_field_parsing_incorrect_symbols;	//Message ID unknown

	//Normal exit
	return nmea_field_parsing_ok;
}


enum nmea_field_parsing_result nmea_get_field_value(char **tail, enum nmea_field_type field_type, void *value)
{
	char *current_token_string = NULL;
	char *tail_after_conversion = NULL;

	switch(*tail[0])
	{
		case '*':
			return nmea_field_parsing_empty;
			break;

		case ',':
			(*tail)++;
			return nmea_field_parsing_empty;
			break;

		case '\0':
			return nmea_field_parsing_eol;
			break;

		default:

			current_token_string = strtok_r(NULL, ",*", tail);

			if(current_token_string == NULL)
				return nmea_field_parsing_eol;

			tail_after_conversion = current_token_string;

			switch(field_type)
			{
				case nmea_field_integer:

					*((int32_t*)value) = strtol(current_token_string, &tail_after_conversion, 10);

					break;

				case nmea_field_unsigned_integer:

					*((uint32_t*)value) = strtoul(current_token_string, &tail_after_conversion, 10);

					break;

				case nmea_field_fractional:

					//Integer part
					(*((struct fixed_point_t*)value)).integer = strtoul(current_token_string, &tail_after_conversion, 10);

					//If pointer didn't moved after invocing strtoul()
					if(current_token_string == tail_after_conversion)
						return nmea_field_parsing_incorrect_symbols;
					else
						return nmea_field_parsing_ok;

					current_token_string = ++tail_after_conversion;

					//Fractional part
					(*((struct fixed_point_t*)value)).integer = strtoul(current_token_string, &tail_after_conversion, 10);

					break;

				case nmea_field_float:

					*((float*)value) = strtof(current_token_string, &tail_after_conversion);

					break;

				case nmea_field_char:

					if(isalnum(current_token_string[0]))
					{
						*((char*)value) = current_token_string[0];
//						tail_after_conversion++;
						return nmea_field_parsing_ok;
					}
					else
						return nmea_field_parsing_incorrect_symbols;

					break;

				default:
					return nmea_field_parsing_incorrect_symbols;
					break;
			};

			//If pointer didn't moved after invocing strto**()
			if(current_token_string == tail_after_conversion)
				return nmea_field_parsing_incorrect_symbols;
			else
				return nmea_field_parsing_ok;



			break;
	};

	//Should never reach here
	return nmea_field_parsing_incorrect_symbols;
}

//Parse NMEA *GGA message, fill gga struct, and return parsing result
uint32_t nmea_parse_gga(const char *message, struct nmea_gga_t *gga)
{
	char *tail = message;

	uint32_t available_fields = 0;

	struct fixed_point_t fract;
	char tmp_c;
	uint32_t tmp_u32;
	float tmp_f;


	if(message == NULL)
		return 0;

	tail = tail + NMEA_HEADER_SENDER_ID_LENGTH + NMEA_HEADER_MSG_TYPE_LENGTH + 2;	//Move pointer forward to skip header

	if(nmea_get_field_value(&tail, nmea_field_fractional, &fract) == nmea_field_parsing_ok)
	{
		//Fill the time struct
		gga->time.hours = fract.integer / 10000;
		gga->time.minutes = (fract.integer % 10000) / 100;
		gga->time.seconds = fract.integer % 100;
		gga->time.subseconds = fract.fractional;

		available_fields |= NMEA_GGA_FIELD_TIME;
	}

	//Latitude
	if(nmea_get_field_value(&tail, nmea_field_fractional, &fract) == nmea_field_parsing_ok)
	{
		gga->latitude.degrees = fract.integer / 100;
		gga->latitude.minutes = fract.integer % 100;
		gga->latitude.subminutes = fract.fractional;

		available_fields |= NMEA_GGA_FIELD_LATITUDE;
	}

	// N/S
	gga->latitude.direction = latitude_unknown;

	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		switch(tmp_c)
		{
			case 'N':
				gga->latitude.direction = latitude_north;
				break;

			case 'S':
				gga->latitude.direction = latitude_south;
				break;
		};

		available_fields |= NMEA_GGA_FIELD_NS;
	}

	//Longitude
	if(nmea_get_field_value(&tail, nmea_field_fractional, &fract) == nmea_field_parsing_ok)
	{
		gga->longitude.degrees = fract.integer / 100;
		gga->longitude.minutes = fract.integer % 100;
		gga->longitude.subminutes = fract.fractional;

		available_fields |= NMEA_GGA_FIELD_LONGITUDE;
	}

	// E/W
	gga->longitude.direction = longitude_unknown;

	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		switch(tmp_c)
		{
			case 'E':
				gga->longitude.direction = longitude_east;
				break;

			case 'W':
				gga->longitude.direction = longitude_west;
				break;
		};

		available_fields |= NMEA_GGA_FIELD_EW;
	}

	//Quality indicator
	gga->quality = nmea_quality_fix_not_available;

	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		switch(tmp_c)
		{
			case '0':
				gga->quality = nmea_quality_fix_not_available;
				break;

			case '1':
				gga->quality = nmea_quality_sps_mode;
				break;

			case '2':
				gga->quality = nmea_quality_dgps_mode;
				break;

			case '3':
				gga->quality = nmea_quality_pps_mode;
				break;

			case '4':
				gga->quality = nmea_quality_rtk_mode;
				break;

			case '5':
				gga->quality = nmea_quality_float_rtk_mode;
				break;

			case '6':
				gga->quality = nmea_quality_estimated_mode;
				break;

			case '7':
				gga->quality = nmea_quality_manual_input;
				break;

			case '8':
				gga->quality = nmea_quality_simulator_mode;
				break;

		};

		available_fields |= NMEA_GGA_FIELD_QUALITY;
	}

	//Number of satellites in use
	if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
	{
		//Must fit in uint8_t
		gga->satellites = tmp_u32;

		available_fields |= NMEA_GGA_FIELD_SATELLITES;
	}

	//HDOP
	if(nmea_get_field_value(&tail, nmea_field_float, &tmp_f) == nmea_field_parsing_ok)
	{
		gga->hdop = tmp_f;

		available_fields |= NMEA_GGA_FIELD_HDOP;
	}

	//Altitude
	if(nmea_get_field_value(&tail, nmea_field_float, &tmp_f) == nmea_field_parsing_ok)
	{
		gga->altitude = tmp_f;

		available_fields |= NMEA_GGA_FIELD_ALTITUDE;
	}

	//Altitude units - M
	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		available_fields |= NMEA_GGA_FIELD_ALTITUDE_UNITS;
	}

	//Height of geoid above WGS84 ellipsoid
	if(nmea_get_field_value(&tail, nmea_field_float, &tmp_f) == nmea_field_parsing_ok)
	{
		gga->hog = tmp_f;

		available_fields |= NMEA_GGA_FIELD_HOG;
	}

	//Height units - M
	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		available_fields |= NMEA_GGA_FIELD_HOG_UNITS;
	}

	//Time since last update (Float?)
	if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
	{
		gga->dgps_age = tmp_u32;

		available_fields |= NMEA_GGA_FIELD_DGPS_AGE;
	}

	//DGPS station ID
	if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
	{
		gga->station_id = tmp_u32;

		available_fields |= NMEA_GGA_FIELD_STATION;
	}

	return available_fields | NMEA_MESSAGE_PARSING_OK;
}

//Parse NMEA *RMC message, fill rmc struct, and return parsing result
uint32_t nmea_parse_rmc(const char *message, struct nmea_rmc_t *rmc)
{
	char *tail = message;

	uint32_t available_fields = 0;

	struct fixed_point_t fract;
	char tmp_c;
	uint32_t tmp_u32;
	float tmp_f;


	if(message == NULL)
		return 0;

	tail = tail + NMEA_HEADER_SENDER_ID_LENGTH + NMEA_HEADER_MSG_TYPE_LENGTH + 2;	//Move pointer forward to skip header

	if(nmea_get_field_value(&tail, nmea_field_fractional, &fract) == nmea_field_parsing_ok)
	{
		//Fill the time part of the datetime struct
		rmc->datetime.hours = fract.integer / 10000;
		rmc->datetime.minutes = (fract.integer % 10000) / 100;
		rmc->datetime.seconds = fract.integer % 100;
		rmc->datetime.subseconds = fract.fractional;

		available_fields |= NMEA_RMC_FIELD_TIME;
	}

	//Receiver warning indicator
	rmc->status = nmea_data_invalid;

	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		switch(tmp_c)
		{
			case 'A':
				rmc->status = nmea_data_valid;
				break;

			case 'V':
				rmc->status = nmea_data_invalid;
				break;
		};

		available_fields |= NMEA_RMC_FIELD_STATUS;
	}

	//Latitude
	if(nmea_get_field_value(&tail, nmea_field_fractional, &fract) == nmea_field_parsing_ok)
	{
		rmc->latitude.degrees = fract.integer / 100;
		rmc->latitude.minutes = fract.integer % 100;
		rmc->latitude.subminutes = fract.fractional;

		available_fields |= NMEA_RMC_FIELD_LATITUDE;
	}

	// N/S
	rmc->latitude.direction = latitude_unknown;

	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		switch(tmp_c)
		{
			case 'N':
				rmc->latitude.direction = latitude_north;
				break;

			case 'S':
				rmc->latitude.direction = latitude_south;
				break;
		};

		available_fields |= NMEA_RMC_FIELD_NS;
	}

	//Longitude
	if(nmea_get_field_value(&tail, nmea_field_fractional, &fract) == nmea_field_parsing_ok)
	{
		rmc->longitude.degrees = fract.integer / 100;
		rmc->longitude.minutes = fract.integer % 100;
		rmc->longitude.subminutes = fract.fractional;

		available_fields |= NMEA_RMC_FIELD_LONGITUDE;
	}

	// E/W
	rmc->longitude.direction = longitude_unknown;

	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		switch(tmp_c)
		{
			case 'E':
				rmc->longitude.direction = longitude_east;
				break;

			case 'W':
				rmc->longitude.direction = longitude_west;
				break;
		};

		available_fields |= NMEA_RMC_FIELD_EW;
	}

	//Speed over ground (in knots)
	if(nmea_get_field_value(&tail, nmea_field_float, &tmp_f) == nmea_field_parsing_ok)
	{
		rmc->speed = tmp_f;

		available_fields |= NMEA_RMC_FIELD_SPEED;
	}

	//Course over ground (in degrees)
	if(nmea_get_field_value(&tail, nmea_field_float, &tmp_f) == nmea_field_parsing_ok)
	{
		rmc->course = tmp_f;

		available_fields |= NMEA_RMC_FIELD_COURSE;
	}

	//Date part of datetime struct
	if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
	{
		rmc->datetime.day = tmp_u32 / 10000;
		rmc->datetime.month = (tmp_u32 % 10000) / 100;
		rmc->datetime.year = tmp_u32 % 100;

		available_fields |= NMEA_GGA_FIELD_SATELLITES;
	}

	//Magnetic variation (in degrees)
	if(nmea_get_field_value(&tail, nmea_field_float, &tmp_f) == nmea_field_parsing_ok)
	{
		rmc->variation = tmp_f;

		available_fields |= NMEA_RMC_FIELD_VARIATION;
	}

	//Magnetic variation direction (E/W)
	rmc->variation_dir = magnetic_variation_unknown;

	if(nmea_get_field_value(&tail, nmea_field_char, &tmp_c) == nmea_field_parsing_ok)
	{
		switch(tmp_c)
		{
			case 'E':
				rmc->variation_dir = magnetic_variation_east;
				break;

			case 'W':
				rmc->variation_dir = magnetic_variation_west;
				break;
		};

		available_fields |= NMEA_RMC_FIELD_VARIATION_DIR;
	}


	return available_fields | NMEA_MESSAGE_PARSING_OK;
}

uint32_t nmea_parse_gsv(const char *message, struct nmea_gsv_t *gsv)
{
	char *tail = message;

	uint32_t available_fields = 0;

	uint32_t tmp_u32;

	uint8_t i = 0;


	if(message == NULL)
		return 0;

	tail = tail + NMEA_HEADER_SENDER_ID_LENGTH + NMEA_HEADER_MSG_TYPE_LENGTH + 2;	//Move pointer forward to skip header

	//Total message count
	if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
	{
		gsv->message_count = tmp_u32;

		available_fields |= NMEA_GSV_FIELD_MSG_TOTAL_COUNT;
	}

	//Current message number
	if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
	{
		gsv->message_num = tmp_u32;

		available_fields |= NMEA_GSV_FIELD_MSG_NUMBER;
	}

	//Satellites in view
	if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
	{
		gsv->satellites = tmp_u32;

		available_fields |= NMEA_GSV_FIELD_SATELLITES;
	}

	do
	{
		switch(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32))
		{
			case nmea_field_parsing_ok:
				gsv->sv[i].prn = tmp_u32;
				available_fields |= (NMEA_GSV_FIELD_SAT1_PRN << ( i * 4 ));
				break;

			case nmea_field_parsing_eol:	//End of line reached
				return available_fields | NMEA_MESSAGE_PARSING_OK;
				break;

			default:
				break;
		};

		if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
		{
			gsv->sv[i].elevation = tmp_u32;

			available_fields |= (NMEA_GSV_FIELD_SAT1_ELEVATION << ( i * 4 ));
		}

		if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
		{
			gsv->sv[i].azimuth = tmp_u32;

			available_fields |= (NMEA_GSV_FIELD_SAT1_AZIMUTH << ( i * 4 ));
		}

		if(nmea_get_field_value(&tail, nmea_field_unsigned_integer, &tmp_u32) == nmea_field_parsing_ok)
		{
			gsv->sv[i].snr = tmp_u32;

			available_fields |= (NMEA_GSV_FIELD_SAT1_SNR << ( i * 4 ));
		}

		i++;

	} while (i < 4);


	return available_fields | NMEA_MESSAGE_PARSING_OK;
}

