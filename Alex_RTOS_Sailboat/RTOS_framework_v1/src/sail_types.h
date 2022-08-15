/* sail_types.h
 * A header to maintain data structures shared between different modules of the
 * autonomous sailboat.
 * Created on August 4, 2016.
 * Created by Thomas Gwynne-Timothy.
 */

#ifndef SAIL_TYPES_H_ 
#define SAIL_TYPES_H_

/*
 *** CONTROL ****************************************************
 * Data structures and enumerations to maintain the state of the 
 * sailboat's control system.
 *
 * CTRL_Mode: List of operating modes.
 *     * AUTO		- autonomous navigation
 *     * REMOTE		- remote control
 *     * LOAD		- mission loading to EEPROM
 *
 * CTRL_State: List of operating states.
 *     * DEPLOY		- status messages are sent via satellite
 *     * TEST		- status messages are sent via radio
 *
 * CTRL_ResetCause: List of reset causes.
 *     * POWER		- power loss
 *     * WDT		- watchdog timer
 *     * SW			- software controlled
 *     * EXT		- external reset pin
 *     * OTHER		- other causes
 *
 **************************************************************
 */

typedef enum CTRL_Modes {
	CTRL_MODE_AUTO,
	CTRL_MODE_REMOTE,
	CTRL_MODE_LOAD,
	CTRL_NUM_MODES
} CTRL_Mode;

typedef enum CTRL_States {
	CTRL_STATE_DEPLOY,
	CTRL_STATE_TEST,
	CTRL_NUM_STATES
} CTRL_State;

typedef enum CTRL_ResetCauses {
	CTRL_RESET_POWER,
	CTRL_RESET_WDT,
	CTRL_RESET_SW,
	CTRL_RESET_EXT,
	CTRL_RESET_OTHER,
	CTRL_NUM_CAUSES
} CTRL_ResetCause;


/**** SENSORS ************************************************************
 * Data structures and enumerations to maintain readings from the sailboat's
 * various sensors.
 *************************************************************************/

typedef struct GPS_Reading {
	double lat;
	double lon;
} GPS_Reading;

typedef struct WIND_Reading{
	float angle;
	float speed;
} WIND_Reading;

typedef enum COMP_ReadingTypes {
	COMP_ACCEL,
	COMP_MAG,
	COMP_HEADING,
	COMP_TILT,
	COMP_NUM_TYPES
} COMP_ReadingType;

typedef struct COMP_AccelData {
	signed short a_x;
	signed short a_y;
	signed short a_z;
} COMP_AccelData;

typedef struct COMP_MagData {
	signed short m_x;
	signed short m_y;
	signed short m_z;
} COMP_MagData;

typedef struct COMP_HeadingData {
	uint16_t heading;
	uint16_t h_pitch;
	uint16_t h_roll;
} COMP_HeadingData;

typedef struct COMP_TiltData {
	signed short t_pitch;
	signed short t_roll;
	signed short temp;
} COMP_TiltData;

typedef struct COMP_Reading {
	COMP_ReadingType type;
	union {
		COMP_AccelData accel;
		COMP_MagData mag;
		COMP_HeadingData heading;
		COMP_TiltData tilt;
		uint16_t fields[3];
	} data;
} COMP_Reading;

typedef struct Compass_Data{
	COMP_Reading type[COMP_NUM_TYPES];
} Compass_Data_t;

/**** EEPROM ************************************************************
 * Data structures and enumerations for reading and writing to the EEPROM.
 *************************************************************************/
 
typedef struct EEPROM_WayPoint {
	GPS_Reading pos;
    double rad;     // tolerance radius of way point (meters)
} EEPROM_WayPoint;

#endif /* SAIL_TYPES_H_ */
