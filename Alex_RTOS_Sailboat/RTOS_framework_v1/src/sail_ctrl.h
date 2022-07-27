
#ifndef SAIL_CTRL_H_
#define SAIL_CTRL_H_
/*                           *******************
******************************* C Library FILE ******************************
**                           *******************                           **
**                                                                         **
** filename  : sail_actuator.h                                             **
** author    : Matthew Cockburn                                            **
** created   : 2022-07-23                                                  **
**                                                                         **
*****************************************************************************

This C Library file is contains the definitions of Nautono's Task Runner 
Functions 

/***************************************************************************/
/**                                                                       **/
/**                     MODULES USED                                      **/
/**                                                                       **/
/***************************************************************************/
/* Standard Headers */
#include <asf.h>
#include <status_codes.h>
/* Sail Headers */
#include "sail_radio.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
/***************************************************************************/
/**                                                                       **/
/**                     DEFINITIONS AND MACROS                            **/
/**                                                                       **/
/***************************************************************************/
#define LOG_DATA_SLEEP_PERIOD_MS 5000

#define UPDATECOURSE_ON_TIME_SEC 2
#define UPDATECOURSE_LOOP_LIM  UPDATECOURSE_ON_TIME_SEC * configTICK_RATE_HZ
#define UPDATE_COURSE_SLEEP_PERIOD_MS 60000

#define CONTROLRUDDER_ON_TIME_SEC 2
#define CONTROLRUDDER_LOOP_LIM CONTROLRUDDER_ON_TIME_SEC * configTICK_RATE_HZ
#define CONTROL_RUDDER_SLEEP_PERIOD_MS 1000

#define CONTROL_SAIL_ON_TIME_SEC 2
#define CONTROL_SAIL_LOOP_LIM CONTROL_SAIL_ON_TIME_SEC * configTICK_RATE_HZ
#define CONTROL_SAIL_SLEEP_PERIOD_MS 1000

#define READ_COMPASS_SLEEP_PERIOD_MS 500
/***************************************************************************/
/**                                                                       **/
/**                     TYPDEFS AND STRUCTURES                            **/
/**                                                                       **/
/***************************************************************************/
typedef enum Sensor_Types {
	SENSOR_GPS,
	SENSOR_WIND,
	SENSOR_COMP,
	SENSOR_COUNT
} Sensor_Type;



/***************************************************************************/
/**                                                                       **/
/**                     EXPORTED VARIABLES                                **/
/**                                                                       **/
/***************************************************************************/
extern CTRL_Mode mode;
extern CTRL_State state;

/***************************************************************************/
/**                                                                       **/
/**                     PROTOTYPES OF EXPORTED FUNCTIONS                  **/
/**                                                                       **/
/***************************************************************************/
/* CTRL_InitSystem
 * Initialize the sail boat controller.
 * Status:
 *		STATUS_OK - sail boat controller initialization was successful	
 * Will generate process information in DEBUG mode
 */
enum status_code CTRL_InitSystem(void);


/* CTRL_InitSensors
 * Initialize each sensor
 * Status:
 *		STATUS_OK - Sensors initialization was successful
 * Will generate process information in DEBUG mode
 */
enum status_code CTRL_InitSensors(void);


/* startup
 * Initialize WeatherStation
 * Status:
 *		STATUS_OK - WeatherStation initialization was successful
 * Will generate process information in DEBUG mode
 */ 
enum status_code startup(void);

/* init_tasks
 * Initialize all tasks
 * Status:
 *		STATUS_ERR_INSUFFICIENT_RTOS_HEAP - Contains error when system create task
 */ 
enum status_code init_tasks();

/* UpdateCourse
 * Update Navigation Parameters
 */ 
void UpdateCourse(void);


/* ControlRudder
 * Commented Function, set rubber degree
 */ 
void ControlRudder(void);

/* ControlSail
*  Used to set the sail actuator position
*/
void ControlSail(void);

/* LogData
 * Record all sensors data
 */ 
void LogData(void);


/* ReadCompass
 * Get reading from compass
 */ 
void ReadCompass(void);


/* check_waypoint_state
 * Check the difference between the location of the ship and the scheduled route
 */ 
void check_waypoint_state(void);


/* assign_weatherstation_readings
 * Assign GPS Wind and compass reading
 */ 
void assign_weatherstation_readings(void);

/* CTRL_Sleep
 * Set the sleep time of the control unit
 * Input:
 *	 time_sec - sleep time length
 */ 
static void CTRL_Sleep(unsigned time_sec);

enum status_code CTRL_InitSail(void);

#endif /* SAIL_CTRL_H_ */ 