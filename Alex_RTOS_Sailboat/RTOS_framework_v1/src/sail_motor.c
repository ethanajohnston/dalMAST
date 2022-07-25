/*                           *******************
******************************* C SOURCE FILE *******************************
**                           *******************                           **
**                                                                         **
** filename  : sail_motor.c                                                **
** author    : Thomas Gwynne-Timothy                                       **
** created   : 2016-08-16                                                  **
**                                                                         **
***************************************************************************** 
 
 Implementation of the motor controller for the autonomous sailboat project.
 
 /***************************************************************************/
 /**                                                                       **/
 /**                     MODULES USED                                      **/
 /**                                                                       **/
 /***************************************************************************/
/* Standard Headers */
#include <math.h>
#include <stdbool.h>
/* Sail Headers*/
#include "sail_motor.h"
#include "sail_adc.h"
#include "sail_pwm.h"
#include "sail_debug.h"
#include "sail_math.h"
/***************************************************************************/
/**                                                                       **/
/**                     DEFINITIONS AND MACROS                            **/
/**                                                                       **/
/***************************************************************************/
#define MOTOR_ON_STATE			true
#define shaft_min_deg			0.0;
#define shaft_max_deg			90.0;
#define shaft_neutral_deg		45.0;
#define shaft_threshold_deg		1.0;
#define shaft_motor_min_speed	100.0;
#define shaft_motor_max_speed	PWM_MAX_DUTY;
#define error_min_deg			5.0;
#define error_max_deg			20.0;
/***************************************************************************/
/**                                                                       **/
/**                     TYPDEFS AND STRUCTURES                            **/
/**                                                                       **/
/***************************************************************************/
typedef enum MOTOR_Directions {
	MOTOR_CW,
	MOTOR_CCW
} MOTOR_Direction;

typedef struct RudderMotor
{
	bool		on = false;
	double		current_shaft_deg = shaft_min_deg - 100.0;
	double		target_shaft_deg  = shaft_neutral_deg;
	
}RudderMotor_t;
/***************************************************************************/
/**                                                                       **/
/**                     GLOBAL VARIABLES                                  **/
/**                                                                       **/
/***************************************************************************/
// Define the angle range for each motor
RudderMotor_t motor;

static uint16_t shaft_threshold_count   =   0;
static const uint16_t required_count    =   2;

// Define a timer to control the feedback loop
static struct tc_module timer;
static Tc *const timer_hw = TC0;

/***************************************************************************/
/**                                                                       **/
/**                     PROTOTYPES OF LOCAL FUNCTIONS                     **/
/**                                                                       **/
/***************************************************************************/
// Function to map rudder angle to shaft angle
static double RudderMap(double rudder_deg);
// Function to map rudder angle (relative to boat) to shaft angle
static double ShaftMap(double shaft_volt);
// Function to compute motor speed from error
static uint8_t SpeedControl(double error_deg);
// Function to turn on the specified motor
static void RudderOn(void);
// Function to turn off the specified motor
static void RudderOff(void);
// Function to turn set the specified motor clockwise
static void SetDirection(MOTOR_Direction dir);

/***************************************************************************/
/**                                                                       **/
/**                     EXPORTED FUNCTIONS                                **/
/**                                                                       **/
/***************************************************************************/

/***************************************************************************/
enum status_code MOTOR_SetRudder(double rudder_deg)
{
	motor.target_shaft_deg = MATH_Clamp(RudderMap(rudder_deg), shaft_min_deg,
		 shaft_max_deg);

	// Return if the angle hasn't changed significantly
	if (fabs(motor.target_shaft_deg - motor.current_shaft_deg) 
		< shaft_threshold_deg) {
		motor.current_shaft_deg = motor.target_shaft_deg;
		return STATUS_NO_CHANGE;
	}

	// Enable the motor if it's currently off
	if (!motor.on) {
		// Enable callback
		tc_enable_callback(&timer, TC_CALLBACK_CC_CHANNEL1);
		// Turn on the motor
		RuddderOn();
	}

	motor.current_shaft_deg = motor.target_shaft_deg;
	
	return STATUS_OK;
}

/***************************************************************************/
static void ShaftControlCallback(struct tc_module *const module_inst)
{
	double shaft_volt;
	
	// Read the voltage at each potentiometer
	if (ADC_GetReading(ADC_RUDDER, &shaft_volt) != STATUS_OK) {
		DEBUG_Write("Error\r\n");
	}
	
	// Get the corresponding angles
	double shaft_deg = ShaftMap(shaft_volt);

	// Compare the angles
	double shaft_error_deg = motor.target_shaft_deg - shaft_deg;

	// Stop if the error is less than the threshold
	if (fabs(shaft_error_deg) < shaft_threshold_deg) {
		// Increment the counter
		shaft_threshold_count++;
		if (shaft_threshold_count >= required_count) {
			// Turn off PWM
			PWM_Disable(PWM_RUDDER);
			// Turn off the motor
			RudderOff();
			// Disable the callback
			tc_disable_callback(&timer, TC_CALLBACK_CC_CHANNEL1);
			return;
		}
	} else {
		// Reset the counter
		shaft_threshold_count = 0;
	}
	
	// Map the error to a speed
	double speed = MATH_Map(fabs(shaft_error_deg), error_min_deg, error_max_deg, shaft_motor_min_speed, shaft_motor_max_speed);
	speed = MATH_Clamp(speed, shaft_motor_min_speed, shaft_motor_max_speed);	
	
	// Update the motor
	PWM_SetDuty(PWM_RUDDER, speed);
	SetDirection(RUDDER_DIR_PIN, (shaft_error_deg >= 0.0 ? MOTOR_CCW : MOTOR_CW));
}
/***************************************************************************/
/**                                                                       **/
/**                     LOCAL FUNCTIONS                                   **/
/**                                                                       **/
/***************************************************************************/
// Function to map rudder angle to shaft angle
static double RudderMap(double rudder_deg)
{
	return rudder_deg + 45.0;
}
/***************************************************************************/
// Function to map shaft pot. voltage to shaft angle
static double ShaftMap(double shaft_volt)
{
	return MATH_Map(shaft_volt, 1.10, 2.25, shaft_min_deg, shaft_max_deg);
}
/***************************************************************************/
// Function to compute motor speed from error
static uint8_t SpeedControl(double error_deg)
{
	// Map the error to a speed
	double speed = MATH_Map(fabs(error_deg), error_min_deg, error_max_deg, shaft_motor_min_speed, shaft_motor_max_speed);

	// Clamp the motor speed 
	return MATH_Clamp(speed, shaft_motor_min_speed, shaft_motor_max_speed);
}
/***************************************************************************/
// Function to turn on the specified motor
static void RuddderOn(void)
{
	motor.on = true;
	// Set the pin
	port_pin_set_output_level(RUDDER_POWER_PIN, MOTOR_ON_STATE);
	return;
	
}
/***************************************************************************/
// Function to turn off the specified motor
static void RudderOff(void)
{
	motor.on = false;
	port_pin_set_output_level(RUDDER_POWER_PIN, !MOTOR_ON_STATE);
	return;
}
/***************************************************************************/
// Function to turn set the direction of the specified motor
static void SetDirection(MOTOR_Direction dir)
{
	port_pin_set_output_level(RUDDER_DIR_PIN, (dir == MOTOR_CW ?
		MOTOR_RUDDER_CW_STATE :!MOTOR_RUDDER_CW_STATE));
	
	return;
}
/***************************************************************************/
/**                                                                       **/
/**                               EOF                                     **/
/**                                                                       **/
/***************************************************************************/
