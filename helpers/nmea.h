/*
 * nmea.h
 *
 *  Created on: 18 окт. 2017 г.
 *      Author: frost
 */

#ifndef NMEA_H_
#define NMEA_H_

#define NMEA_MESSAGE_PARSING_OK	 			(uint32_t)(1 << 31)

//Available fields for NMEA GGA sentence
#define NMEA_GGA_FIELD_TIME	 			(uint32_t)(1 << 0)
#define NMEA_GGA_FIELD_LATITUDE	 		(uint32_t)(1 << 1)
#define NMEA_GGA_FIELD_NS		 			(uint32_t)(1 << 2)
#define NMEA_GGA_FIELD_LONGITUDE 		(uint32_t)(1 << 3)
#define NMEA_GGA_FIELD_EW			 		(uint32_t)(1 << 4)
#define NMEA_GGA_FIELD_QUALITY 			(uint32_t)(1 << 5)
#define NMEA_GGA_FIELD_SATELLITES 		(uint32_t)(1 << 6)
#define NMEA_GGA_FIELD_HDOP 				(uint32_t)(1 << 7)
#define NMEA_GGA_FIELD_ALTITUDE 			(uint32_t)(1 << 8)
#define NMEA_GGA_FIELD_ALTITUDE_UNITS	(uint32_t)(1 << 9)
#define NMEA_GGA_FIELD_HOG 				(uint32_t)(1 << 10)
#define NMEA_GGA_FIELD_HOG_UNITS 		(uint32_t)(1 << 11)
#define NMEA_GGA_FIELD_DGPS_AGE 			(uint32_t)(1 << 12)
#define NMEA_GGA_FIELD_STATION 			(uint32_t)(1 << 13)


//Available fields for NMEA RMC sentence
#define NMEA_RMC_FIELD_TIME	 			(uint32_t)(1 << 0)
#define NMEA_RMC_FIELD_STATUS	 			(uint32_t)(1 << 1)
#define NMEA_RMC_FIELD_LATITUDE	 		(uint32_t)(1 << 2)
#define NMEA_RMC_FIELD_NS		 			(uint32_t)(1 << 3)
#define NMEA_RMC_FIELD_LONGITUDE 		(uint32_t)(1 << 4)
#define NMEA_RMC_FIELD_EW			 		(uint32_t)(1 << 5)
#define NMEA_RMC_FIELD_SPEED			 	(uint32_t)(1 << 6)
#define NMEA_RMC_FIELD_COURSE			 	(uint32_t)(1 << 7)
#define NMEA_RMC_FIELD_DATE		 		(uint32_t)(1 << 8)
//#define NMEA_RMC_FIELD_MODE			 	(uint32_t)(1 << 9)
#define NMEA_RMC_FIELD_VARIATION			(uint32_t)(1 << 9)
#define NMEA_RMC_FIELD_VARIATION_DIR		(uint32_t)(1 << 10)


//Available fields for NMEA GSV sentence
#define NMEA_GSV_FIELD_MSG_TOTAL_COUNT	(uint32_t)(1 << 0)
#define NMEA_GSV_FIELD_MSG_NUMBER		(uint32_t)(1 << 1)
#define NMEA_GSV_FIELD_SATELLITES		(uint32_t)(1 << 2)

#define NMEA_GSV_FIELD_SAT1_PRN			(uint32_t)(1 << 3)
#define NMEA_GSV_FIELD_SAT1_ELEVATION	(uint32_t)(1 << 4)
#define NMEA_GSV_FIELD_SAT1_AZIMUTH		(uint32_t)(1 << 5)
#define NMEA_GSV_FIELD_SAT1_SNR			(uint32_t)(1 << 6)

#define NMEA_GSV_FIELD_SAT2_PRN			(uint32_t)(1 << 7)
#define NMEA_GSV_FIELD_SAT2_ELEVATION	(uint32_t)(1 << 8)
#define NMEA_GSV_FIELD_SAT2_AZIMUTH		(uint32_t)(1 << 9)
#define NMEA_GSV_FIELD_SAT2_SNR			(uint32_t)(1 << 10)

#define NMEA_GSV_FIELD_SAT3_PRN			(uint32_t)(1 << 11)
#define NMEA_GSV_FIELD_SAT3_ELEVATION	(uint32_t)(1 << 12)
#define NMEA_GSV_FIELD_SAT3_AZIMUTH		(uint32_t)(1 << 13)
#define NMEA_GSV_FIELD_SAT3_SNR			(uint32_t)(1 << 14)

#define NMEA_GSV_FIELD_SAT4_PRN			(uint32_t)(1 << 15)
#define NMEA_GSV_FIELD_SAT4_ELEVATION	(uint32_t)(1 << 16)
#define NMEA_GSV_FIELD_SAT4_AZIMUTH		(uint32_t)(1 << 17)
#define NMEA_GSV_FIELD_SAT4_SNR			(uint32_t)(1 << 18)




typedef struct fixed_point_t
{
	uint32_t integer;
	uint32_t fractional;
};

typedef struct fixed_point_signed_t
{
	int32_t integer;
	uint32_t fractional;
};

enum nmea_field_parsing_result
{
	nmea_field_parsing_ok = 0,
	nmea_field_parsing_empty = 1,
	nmea_field_parsing_incorrect_symbols = 2,
	nmea_field_parsing_eol = 3,
	nmea_parsing_invalid_checksum = 4
};

//Possible NMEA sentence field data types
enum nmea_field_type
{
	nmea_field_empty = 0,
	nmea_field_integer = 1,
	nmea_field_unsigned_integer = 2,
	nmea_field_fractional = 3,	//For latitude, longitude, time, etc. fields use fractional struct instead of float
	nmea_field_float = 4,
	nmea_field_char = 5,
	nmea_field_string = 6
};

enum nmea_sender_id
{
	nmea_sender_unknown = 0,
	nmea_sender_gps = 1,		//$GP - GPS source
	nmea_sender_glonass = 2,	//$GN - GLONASS source
	nmea_sender_galileo = 3,	//$GA - GALILEO source
	nmea_sender_combined = 4	//$GN - combination of more than one source (e.g. GPS + GLONASS)
};

enum nmea_message_id
{
	nmea_message_id_unknown = 0,
	nmea_gga = 1,
	nmea_gll = 2,
	nmea_gsa = 3,
	nmea_gsv = 4,
	nmea_vtg = 5,
	nmea_rmc = 6,
	nmea_zda = 7
};

struct nmea_message_id_t
{
	enum nmea_sender_id sender_id;
	enum nmea_message_id message_id;
};

struct nmea_time_t
{
	uint8_t hours;		//0..23
	uint8_t minutes;	//0..59
	uint8_t seconds;	//0..59
	uint16_t subseconds;	//0..999
};


struct datetime_t
{
	uint8_t hours;		//0..23
	uint8_t minutes;	//0..59
	uint8_t seconds;	//0..59
	uint16_t subseconds;	//0..999
	uint8_t day;	//1..31
	uint8_t month;	//1..12
	uint16_t year;
};


enum nmea_latitude_direction
{
	latitude_unknown = 0,
	latitude_north = 1,
	latitude_south = 2
};

enum nmea_longitude_direction
{
	longitude_unknown = 0,
	longitude_east = 1,
	longitude_west = 2
};

enum nmea_magnetic_variation_direction
{
	magnetic_variation_unknown = 0,
	magnetic_variation_east = 1,
	magnetic_variation_west = 2
};

enum nmea_quality
{
	nmea_quality_fix_not_available = 0,	//Fix not available or invalid
	nmea_quality_sps_mode = 1,	//GPS SPS Mode, fix valid
	nmea_quality_dgps_mode = 2,	//Differential GPS, SPS Mode, fix valid
	nmea_quality_pps_mode = 3,	//GPS PPS Mode, fix valid
	nmea_quality_rtk_mode = 4,	//Real Time Kinematic
	nmea_quality_float_rtk_mode = 5, //Float RTK
	nmea_quality_estimated_mode = 6, //Estimated (dead reckoning) Mode
	nmea_quality_manual_input = 7,	//Manual Input Mode
	nmea_quality_simulator_mode	= 8 //Simulator Mode
};

struct nmea_latitude_t
{
	uint32_t degrees;
	uint32_t minutes;
	uint32_t subminutes;
//	uint32_t seconds;
	enum nmea_latitude_direction direction;
};

struct nmea_longitude_t
{
	uint32_t degrees;
	uint32_t minutes;
	uint32_t subminutes;
//	uint32_t seconds;
	enum nmea_longitude_direction direction;
};


struct nmea_gga_t
{
	struct nmea_time_t time;				//1 = UTC time

	struct nmea_latitude_t latitude;		//2 = Latitude

	struct nmea_longitude_t longitude;		//4 = Longitude

	enum nmea_quality quality;			//6 = Quality indicator
	uint8_t satellites;	//7 = Satellites in view

	float hdop;					//8 = Horizontal dilution of precision coefficient

	float altitude;				//9 = Altitude above mean sea level
									//10 = Altitude above mean sea level measurement unints (M), already included in previous field
	float hog;						//11 = Geoid separation
									//12 = Geoid separation measurement units (M), already included in previous field
	uint32_t dgps_age;		//Float?		//13 = Time in seconds since last update, or null field if DGPS is not used.
	uint32_t station_id;				//14 = Reference station ID or null field if not used;
};

enum nmea_navigation_receiver_warning
{
	nmea_data_invalid = 0,
	nmea_data_valid = 1
};

//enum nmea_mode_indicator
//{
//	nmea_mode_data_not_valid = 0,
//	nmea_mode_autonomous = 1,
//	nmea_mode_differential = 2,
//	nmea_mode_estimated = 3
//};

struct nmea_rmc_t
{
	struct datetime_t datetime;

	enum nmea_navigation_receiver_warning status;

	struct nmea_latitude_t latitude;

	struct nmea_longitude_t longitude;

	float speed;

	float course;

	float variation;

	enum nmea_magnetic_variation_direction variation_dir;

//	enum nmea_mode_indicator mode;
};


struct nmea_satellite_info_t
{
	uint16_t prn;
	int16_t elevation;	//Set to -1 if field is empty
	int16_t azimuth;	//Set to -1 if field is empty
	int16_t snr;		//Set to -1 if field is empty
};

struct nmea_gsv_t
{
	uint8_t message_count;		//Total message count in current cycle

	uint8_t message_num;		//Message number in current cycle

	uint8_t satellites;			//Total satellites visible

	struct nmea_satellite_info_t sv[4];	//Array of 4 satellite information structures
};

enum nmea_field_parsing_result nmea_test_checksum(char **message);

enum nmea_message_id nmea_get_message_type(const char *message);
enum nmea_field_parsing_result nmea_get_id(const char *header_str, struct nmea_message_id_t *header);

enum nmea_field_parsing_result nmea_get_field_value(char **tail, enum nmea_field_type field_type, void *value);

uint32_t nmea_parse_gga(const char *message, struct nmea_gga_t *gga);
uint32_t nmea_parse_rmc(const char *message, struct nmea_rmc_t *rmc);
uint32_t nmea_parse_gsv(const char *message, struct nmea_gsv_t *gsv);


#endif /* NMEA_H_ */
