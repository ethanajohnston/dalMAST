/*                           *******************
******************************* C SOURCE FILE *******************************
**                           *******************                           **
**                                                                         **
** filename  : sail_ctrl.c												   **
** author    : DalMAST Team                                                **
** created   : 2016-08-11                                                  **
**                                                                         **
*****************************************************************************

sail_ctrl.c contains all of Nautono initialization functions,
and all of the task's running functions 
 
 * Message handling functions:
 * HandleMessage()
 * - checks the message type and calls the appropriate function
 * ChangeMode()
 * - changes the operating mode of the controller
 * - starts and stops EEPROM loading process when required
 * ChangeState()
 * - change the state of the controller
 * ChangeLogPeriod()
 * - change the period at which log messages are sent
 * AddWayPoint()
 * - add a way point to the EEPROM
 * AdjustMotors()
 * - apply remotely controlled adjustments to the motors
  
/***************************************************************************/
/**                                                                       **/
/**                     MODULES USED                                      **/
/**                                                                       **/
/***************************************************************************/
/* Standard Headers */
#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
/* Sail Headers */
#include "sail_ctrl.h"
#include "sail_math.h"
#include "sail_debug.h"
#include "sail_radio.h"
#include "sail_wind.h"
#include "sail_eeprom.h"
#include "sail_types.h"
#include "sail_nav.h"
#include "sail_motor.h"
#include "sail_actuator.h"
#include "sail_comp.h"
#include "sail_tasksinit.h"
#include "sail_actuator.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/***************************************************************************/
/**                                                                       **/
/**                     GLOBAL VARIABLES                                  **/
/**                                                                       **/
/***************************************************************************/
/* RTC/Rest */
static uint64_t t_ms; // Time since last reset
static uint16_t dt_ms = 200; // Time step (resolution of RTC)
struct rtc_module rtc_timer; // RTC timer
/* Sensor status */
static bool sensor_statuses[SENSOR_COUNT] = {
	false,
	false,
	false
};
/* Sensor Data */
static GPS_Reading gps;
static WIND_Reading wind, avg_wind;
static COMP_Reading comp;
/* Radio Messages */
RADIO_GenericMsg tx_msg;
/* EEPROM Data */
EEPROM_Entry wp_entry;
uint16_t wp_count;
EEPROM_WayPoint wp; // Current way point
uint16_t wp_complete_count; // Counter to track # of times distance < radius
/* Data Items */
double wp_distance;
float course, bearing, sail_deg, rudder_deg;
float avg_heading_deg = 0.0;
/***************************************************************************/
/**                                                                       **/
/**                     PROTOTYPES OF LOCAL FUNCTIONS                     **/
/**                                                                       **/
/***************************************************************************/
/* Task functions */
void process_wind_readings(void);
static void EnableWeatherStation(void);
static void DisableWeatherStation(void);
void process_heading_readings(void);
/* Init functions */
static void InitTimer(void);
static void InitPins(void);
/***************************************************************************/
/**                                                                       **/
/**                     EXPORTED FUNCTIONS                                **/
/**                                                                       **/
/***************************************************************************/
enum status_code CTRL_InitSystem(void)
{
	// Initialize SAMD20
	system_init();
	
	// Initialize debug UART
	DEBUG_Init();
	
	// Initialize the radio
	if (RADIO_Init() != STATUS_OK) {
		DEBUG_Write_Unprotected("Radio not initialized!\r\n");
		} else {
		DEBUG_Write_Unprotected("Radio initialized!\r\n");
	}
	
	// Enable the radio receiver
	if (RADIO_Enable() != STATUS_OK) {
		DEBUG_Write_Unprotected("Radio not enabled!\r\n");
		} else {
		DEBUG_Write_Unprotected("Radio enabled!\r\n");
	}

	// Determine reset cause
	tx_msg.type = RADIO_RESET;
	switch (system_get_reset_cause()) {
		case  SYSTEM_RESET_CAUSE_WDT:
		DEBUG_Write_Unprotected("WDT reset detected!\r\n");
		tx_msg.fields.reset.cause = CTRL_RESET_WDT;
		break;
		case SYSTEM_RESET_CAUSE_POR:
		DEBUG_Write_Unprotected("Power on reset detected!\r\n");
		tx_msg.fields.reset.cause = CTRL_RESET_POWER;
		break;
		case SYSTEM_RESET_CAUSE_SOFTWARE:
		DEBUG_Write_Unprotected("Software reset detected!\r\n");
		tx_msg.fields.reset.cause = CTRL_RESET_SW;
		break;
		case SYSTEM_RESET_CAUSE_EXTERNAL_RESET:
		DEBUG_Write_Unprotected("External reset detected!\r\n");
		tx_msg.fields.reset.cause = CTRL_RESET_EXT;
		break;
		default:
		DEBUG_Write_Unprotected("Other type of reset detected!\r\n");
		tx_msg.fields.reset.cause = CTRL_RESET_OTHER;
		break;
	}
	RADIO_TxMsg_Unprotected(&tx_msg);

	
	// Initialize the EEPROM
	if (EEPROM_Init() != STATUS_OK) {
		DEBUG_Write_Unprotected("EEPROM not initialized!\r\n");
	}
	
	// Set default mode and state
	mode = CTRL_MODE_AUTO;
	state = CTRL_STATE_TEST;
	
	return STATUS_OK;
}
/***************************************************************************/
enum status_code CTRL_InitSensors(void)
{
	
	//todo: add initialization for AIS module
	//DEBUG_Write("Test 456");
	if (COMP_Init() != STATUS_OK) {
		DEBUG_Write_Unprotected("Compass not initialized...\r\n");
	}
	//DEBUG_Write("Test 123");
	if (WEATHERSTATION_Init() != STATUS_OK) {
		DEBUG_Write_Unprotected("WS not initialized...\r\n");
	}
	else{
		DEBUG_Write_Unprotected("WS initialized.\r\n");
		//DEBUG_Write("WS initialized.\r\n");
	}
	
	return STATUS_OK;
}
/***************************************************************************/
enum status_code CTRL_InitSail(void)
{
	// Initialize the control pins
	InitPins();
	// Initialize the ADC
	ADC_Init();
	// Initialize the PWM
	PWM_Init();
	// Turn on the timer
	InitTimer();
	return STATUS_OK;
	
}
/***************************************************************************/
enum status_code startup(void)
{
	// Enable wind vane
	if (WS_Enable() != STATUS_OK) {
		DEBUG_Write_Unprotected("WS not enabled...\r\n");
		} else {
		DEBUG_Write_Unprotected("WS enabled...\r\n");
	}
	
	// Get the current way point
	//EEPROM_GetCurrentWayPoint(&wp);
	
	// Reset the way point complete count
	//wp_complete_count = 0;
	
	//DEBUG_Write_Unprotected("way point: lat - %d lon - %d rad - %d\r\n", (int)(wp.pos.lat * 1000000.0), (int)(wp.pos.lon * 1000000.0), (int)(wp.rad));
	/*
	// Start the motor controller
	MOTOR_Init();
*/
	
	return STATUS_OK;
}

/***************************************************************************/
void LogData(void)
{
	// Event bits for holding the state of the event group
	EventBits_t event_bits;
	
	TickType_t log_data_delay = pdMS_TO_TICKS(LOG_DATA_SLEEP_PERIOD_MS);
	
	while(1) {
		
		event_bits = xEventGroupWaitBits(mode_event_group, /* Test the mode event group */
										 CTRL_MODE_AUTO_BIT | CTRL_MODE_REMOTE_BIT, /* Wait until the sailboat is in AUTO or REMOTE mode */
										 pdFALSE,/* Bits should not be cleared before returning. */
									     pdFALSE, /* Don't wait for both bits, either bit will do. */
									 	 portMAX_DELAY); /* Wait time does not expire */
										  
		taskENTER_CRITICAL();
		watchdog_counter |= 0x10;
		taskEXIT_CRITICAL();
										  
		DEBUG_Write("----------------------Do log data-------------------\r\n");
		
		RADIO_GenericMsg tx_msg_log;
	    running_task = eLogData;
		
		// Log the GPS coordinates
		tx_msg_log.type = RADIO_GPS;
		tx_msg_log.fields.gps.data = gps;
		RADIO_TxMsg(&tx_msg_log);
	
		// Log the wind speed and direction
		tx_msg_log.type = RADIO_WIND;
		tx_msg_log.fields.wind.data = wind;
		
		//not needed because wind is reported in relation to the vessel's center line
		/*
		// Correct wind angle with average heading
		tx_msg.fields.wind.data.angle += avg_heading_deg;
		*/
		
		RADIO_TxMsg(&tx_msg_log);
	
		// Log the compass data
		tx_msg_log.type = RADIO_COMP;
		tx_msg_log.fields.comp.data = comp;
		RADIO_TxMsg(&tx_msg_log);

		// Log the navigation data
		tx_msg.type = RADIO_NAV;
		tx_msg.fields.nav.wp = wp;
		tx_msg.fields.nav.distance = wp_distance;
		tx_msg.fields.nav.bearing = bearing;
		tx_msg.fields.nav.course = course;
		tx_msg.fields.nav.sail_angle = sail_deg;
		tx_msg.fields.nav.rudder_angle = rudder_deg;
		RADIO_TxMsg(&tx_msg);

		//put thread to sleep until a specific tick count is reached
		vTaskDelay(log_data_delay);
	}
}
/***************************************************************************/
void ControlRudder(void)
{
	
	// Event bits for holding the state of the event group
	EventBits_t event_bits;
	
	TickType_t control_rudder_delay = pdMS_TO_TICKS(CONTROL_RUDDER_SLEEP_PERIOD_MS);
	
	while(1) {
		
		
		event_bits = xEventGroupWaitBits(mode_event_group,    /* Test the mode event group */
										 CTRL_MODE_AUTO_BIT,  /* Wait until the sailboat is in AUTO mode */
										 pdFALSE,             /* Bits should not be cleared before returning. */
									     pdFALSE,             /* Don't wait for both bits, either bit will do. */
										 portMAX_DELAY);      /* Wait time does not expire */
										 
	    taskENTER_CRITICAL();
		watchdog_counter |= 0x04;
		taskEXIT_CRITICAL();
										 
	    DEBUG_Write("***************************Do rudder***********************\r\n");
		
		running_task = eControlRudder;
		/*NAV_CalculateRudderPosition(course, comp.data.heading.heading, &rudder_deg);
		DEBUG_Write("Setting rudder - %d\r\n", (int)rudder_deg);
		MOTOR_SetRudder(rudder_deg); */
		
		//put thread to sleep until a specific tick count is reached
		vTaskDelay(control_rudder_delay);
	}
}
/***************************************************************************/
void UpdateCourse(void)
{
	// Event bits for holding the state of the event group
	EventBits_t event_bits;
	
	TickType_t update_course_delay = pdMS_TO_TICKS(UPDATE_COURSE_SLEEP_PERIOD_MS);

	while(1) {
		
		event_bits = xEventGroupWaitBits(mode_event_group,    /* Test the mode event group */
		CTRL_MODE_AUTO_BIT,  /* Wait until the sailboat is in AUTO mode */
		pdFALSE,             /* Bits should not be cleared before returning. */
		pdFALSE,             /* Don't wait for both bits, either bit will do. */
		portMAX_DELAY);      /* Wait time does not expire */
		
		taskENTER_CRITICAL();
		watchdog_counter |= 0x02;
		taskEXIT_CRITICAL();
		
		DEBUG_Write("\n<<<<<<<<<<<<<<<<<<<<<<<Do update course>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
		
		running_task = eUpdateCourse;
		
		//update course
		NAV_UpdateCourse(wp.pos, gps, avg_wind, avg_heading_deg, &course, &sail_deg);
		
		DEBUG_Write("course: %6d  sail: %d\r\n", (int)(course*1000.0), (int)(sail_deg*1000.0));
		MOTOR_SetSail(sail_deg);
		
		vTaskDelay(update_course_delay);

	}
	
}
/***************************************************************************/
void ReadCompass(void)
{
	
	// Event bits for holding the state of the event group
	EventBits_t event_bits;
	
	TickType_t read_compass_delay = pdMS_TO_TICKS(READ_COMPASS_SLEEP_PERIOD_MS);

	while(1) {
		
		event_bits = xEventGroupWaitBits(mode_event_group, /* Test the mode event group */
		CTRL_MODE_AUTO_BIT | CTRL_MODE_REMOTE_BIT, /* Wait until the sailboat is in AUTO or REMOTE mode */
		pdFALSE,								   /* Bits should not be cleared before returning. */
		pdFALSE,                                   /* Don't wait for both bits, either bit will do. */
		portMAX_DELAY);                            /* Wait time does not expire */
		
		taskENTER_CRITICAL();
		watchdog_counter |= 0x20;
		taskEXIT_CRITICAL();
		
		DEBUG_Write("\n<<<<<<<<<<<<<<<<<<<<<<<Do read compass>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
		
		running_task = eReadCompass;
		

		// Get the compass reading
		if(COMP_GetReading(COMP_HEADING, &comp) !=  STATUS_OK){
			DEBUG_Write("\nERROR\r\n");
		}

		// Update the averaged heading
		avg_heading_deg = 0.9 * avg_heading_deg + 0.1 * comp.data.heading.heading;
		
		vTaskDelay(read_compass_delay);

	}


}
/***************************************************************************/
/***** DEPRECATED *****/
static void CTRL_Sleep(unsigned time_sec) {
	vTaskDelay(time_sec * configTICK_RATE_HZ);;
}
/***************************************************************************/
/**                                                                       **/
/**                     LOCAL FUNCTIONS                                   **/
/**                                                                       **/
/***************************************************************************/
void process_wind_readings(void)
{

	// Force the wind to range [0, 360)
	wind.angle = MATH_ForceAngleTo360(wind.angle);
	
	// Convert wind vector to rectangular coordinates
	float wind_x = wind.speed * cos(wind.angle * M_PI / 180.0);
	float wind_y = wind.speed * sin(wind.angle * M_PI / 180.0);
	float avg_x = avg_wind.speed * cos(avg_wind.angle * M_PI / 180.0);
	float avg_y = avg_wind.speed * sin(avg_wind.angle * M_PI / 180.0);
	
	// Update average wind
	avg_x = 0.667 * avg_x + 0.333 * wind_x;
	avg_y = 0.667 * avg_y + 0.333 * wind_y;
	avg_wind.speed = sqrt(avg_x * avg_x + avg_y * avg_y);
	//avg_wind.angle = 180.0 * atan2(avg_y, avg_x) / M_PI;
	
	DEBUG_Write("cur_wind.speed = %4d cm/s | cur_wind.angle = %4d deg\r\n", 
		(int)(wind.speed * 100), (int)wind.angle);
	DEBUG_Write("avg_wind.speed = %4d cm/s | avg_wind.angle = %4d deg\r\n", 
		(int)(avg_wind.speed * 100), (int)avg_wind.angle);
}
/***************************************************************************/
static void EnableWeatherStation(void)
{
	// Return if the controller is in LOAD mode
	if (mode == CTRL_MODE_LOAD) {
		return;
	}

	// Enable the wind vane
	WS_Enable();
}
/***************************************************************************/
static void DisableWeatherStation(void)
{
	// Return if the controller is in LOAD mode
	if (mode == CTRL_MODE_LOAD) {
		return;
	}
	// Disable the wind vane
	WS_Disable();
}
/***************************************************************************/
void process_heading_readings(void)
{
	// Update the averaged heading
	avg_heading_deg = 0.9 * avg_heading_deg + 0.1 * comp.data.heading.heading;
}
/***************************************************************************/
void assign_weatherstation_readings(void) {
	//assign gps weather sensor data to gps struct
	gps.lat = weathersensor_data.msg_array[eGPGGA].fields.gpgga.lat.lat;
	gps.lon = weathersensor_data.msg_array[eGPGGA].fields.gpgga.lon.lon;
	//assign wind readings
	wind.angle = weathersensor_data.msg_array[eWIMWV].fields.wimwv.wind_dir_rel;
	wind.speed = weathersensor_data.msg_array[eWIMWV].fields.wimwv.wind_speed_ms;
	//assign compass readings
	//comp.data.heading.heading = weathersensor_data.msg_array[eGPVTG].fields.gpvtg.course_over_ground;
	comp.data.heading.heading = weathersensor_data.msg_array[eHCHDT].fields.hchdt.bearing;
	comp.data.heading.pitch = weathersensor_data.msg_array[eYXXDR].fields.yxxdr.pitch_deg;
	comp.data.heading.roll = weathersensor_data.msg_array[eYXXDR].fields.yxxdr.roll_deg;
	
	//assign distance between boat and waypoint to wp_distance
	//NAV_GetDistance(wp.pos, gps, &wp_distance);
	//assign bearing
	//NAV_GetBearing(wp.pos, gps, &bearing);
	
}
/***************************************************************************/
void check_waypoint_state(void) {
	DEBUG_Write("wp_dist: %d, wp_rad: %d\r\n", (int)wp_distance, (int)wp.rad);
	// Check if the boat is in the zone
	if (wp_distance < wp.rad) {
		wp_complete_count++;
		DEBUG_Write("way point in range - #%d\r\n", (int)wp_complete_count);
		} else {
		// Reset the count
		wp_complete_count = 0;
	}
	
	// Check if we can move to the next way point
	if (wp_complete_count > 5) {
		DEBUG_Write("getting new way point!\r\n");
		EEPROM_CompleteCurrentWayPoint();
		EEPROM_GetCurrentWayPoint(&wp);
		wp_complete_count = 0;
	}
}
/***************************************************************************/
static void InitPins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;	
	// Set the power pin for the rudder
	port_pin_set_config(RUDDER_POWER_PIN, &config_port_pin);
	// Set the power pin for the sail
	port_pin_set_config(SAIL_POWER_PIN,&config_port_pin);
	// Set the rudder direction pin
	port_pin_set_config(RUDDER_DIR_PIN, &config_port_pin);
}
/***************************************************************************/
static void InitTimer(void)
{
	// Get the default configuration
	struct tc_config timer_config;
	tc_get_config_defaults(&timer_config);

	// Set the counter size
	timer_config.clock_source         = GCLK_GENERATOR_4;
	timer_config.counter_size         = TC_COUNTER_SIZE_8BIT;
	timer_config.clock_prescaler      = TC_CLOCK_PRESCALER_DIV1;
	timer_config.counter_8_bit.period = 10;
	timer_config.counter_8_bit.compare_capture_channel[0] = 2;
	timer_config.counter_8_bit.compare_capture_channel[1] = 7;

	tc_init(&timer, timer_hw, &timer_config);

	tc_enable(&timer);

	tc_register_callback(&timer, SailControlCallback, TC_CALLBACK_CC_CHANNEL0);
	tc_register_callback(&timer, ShaftControlCallback, TC_CALLBACK_CC_CHANNEL1);	
	
}
/***************************************************************************/
/**                                                                       **/
/**                               EOF                                     **/
/**                                                                       **/
/***************************************************************************/




















