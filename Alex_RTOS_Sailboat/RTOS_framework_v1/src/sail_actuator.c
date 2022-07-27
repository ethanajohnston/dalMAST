/*                           *******************
******************************* C SOURCE FILE *******************************
**                           *******************                           **
**                                                                         **
** filename  : sail_actuator.c                                             **
** author    : Matthew Cockburn                                            **
** created   : 2022-07-23                                                  **
**                                                                         **
*****************************************************************************

This C Source file is the implementation of Nautono's Linear Actuator Driver
 
/***************************************************************************/
/**                                                                       **/
/**                     MODULES USED                                      **/
/**                                                                       **/
/***************************************************************************/

/* Standard Headers */
#include <math.h>
#include <stdbool.h>
/* Sail Headers */
#include "sail_adc.h"
#include "sail_pwm.h"
#include "sail_debug.h"
#include "sail_math.h"
#include "sail_actuator.h"

/***************************************************************************/
/**                                                                       **/
/**                     DEFINITIONS AND MACROS                            **/
/**                                                                       **/
/***************************************************************************/
#define SAIL_ON_STATE			true
#define ACTUATOR_MIN_POS		0.0
#define ACTUATOR_MAX_POS		180.0
#define ACTUATOR_THRESHOLD		1.0
#define POS_ERROR_MIN			5.0
#define POS_ERROR_MAX			20.0
#define ACTUATOR_PWM_MIN		0.0
#define ACTUATOR_PWM_MAX		255.0
#define ACTUATOR_REQUIRED_COUNT 5
#define ACTUATOR_POT_MIN		0
#define ACTUATOR_POT_MAX		5.0
/***************************************************************************/
/**                                                                       **/
/**                     TYPDEFS AND STRUCTURES                            **/
/**                                                                       **/
/***************************************************************************/

/***************************************************************************/
/**                                                                       **/
/**                     GLOBAL VARIABLES                                  **/
/**                                                                       **/
/***************************************************************************/
SailActuator_t actuator = {false,NEUTRAL_POS,DEFAULT_TARGET_POS};
uint8_t actuator_threshold_count = 0;

static struct tc_module timer;
static Tc *const timer_hw = TC0;
/***************************************************************************/
/**                                                                       **/
/**                     PROTOTYPES OF LOCAL FUNCTIONS                     **/
/**                                                                       **/
/***************************************************************************/
// Function to Map Sail Back Flap Angle to Actuator Position
static double SailMap(double sail_deg);
// Function to compute motor speed from error
static uint8_t SpeedControl(double error_deg);
// Function to turn on the specified motor
static void SailOn(void);
// Function to turn off the specified motor
static void SailOff(void);
// Map Actuator Position to PWM Duty Cycle
static double ActuatorMap(double position);
// Map the feedback from the Actuator
static double ActuatorFeedbackMap(double voltage);
/***************************************************************************/
/**                                                                       **/
/**                     EXPORTED FUNCTIONS                                **/
/**                                                                       **/
/***************************************************************************/
enum status_code SetActuatorPos(double position)
{
	actuator.target_pos = MATH_Clamp(SailMap(position),ACTUATOR_MIN_POS,
		ACTUATOR_MAX_POS);

	// Return if the angle hasn't changed significantly
	if (fabs(actuator.target_pos - actuator.current_pos) < ACTUATOR_THRESHOLD) {
		actuator.current_pos = actuator.target_pos;
		return STATUS_NO_CHANGE;
	}

	// Enable the actuator if it's currently off
	if (!actuator.on) {
		// Enable callback
		tc_enable_callback(&timer, TC_CALLBACK_CC_CHANNEL0);
		// Turn on the motor
		SailOn();
	}

	actuator.current_pos = actuator.target_pos;
	
	return STATUS_OK;	
	
};
/***************************************************************************/
 void SailControlCallback(struct tc_module *const module_inst)
{
	double actuator_volt;
	
	// Read the voltage at each potentiometer
	ADC_GetReading(ADC_SAIL, &actuator_volt);
	
	// Get the corresponding angles
	double actuator_pos = ActuatorFeedbackMap(actuator_volt);
	#ifdef ACTUATOR_DEBUG
	char temp[50];
	snprintf(temp, 50, ">>%lf<< V mapped to >>%lf<< \n\r",
		actuator_volt,actuator_pos);
	DEBUG_Write_Unprotected("%s",temp);
	#endif	
	// Compare the angles
	double actuator_error_pos = actuator.target_pos - actuator_pos;

	// Stop if the error is less than the threshold
	if (fabs(actuator_error_pos) < ACTUATOR_THRESHOLD) {
		// Increment the counter
		actuator_threshold_count++;
		// Stop if the counter limit has been reached
		if (actuator_threshold_count >= ACTUATOR_REQUIRED_COUNT) {
			// Turn off PWM
			PWM_Disable(PWM_SAIL);
			// Turn off the motor
			SailOff();
			// Disable the callback
			tc_disable_callback(&timer, TC_CALLBACK_CC_CHANNEL0);
			return;
		}
		} else {
		// Reset the counter
		actuator_threshold_count = 0;
	}
	
	// Update the actuator
	PWM_SetDuty(PWM_SAIL, ActuatorMap(actuator.target_pos));
}
/***************************************************************************/
/**                                                                       **/
/**                     LOCAL FUNCTIONS                                   **/
/**                                                                       **/
/***************************************************************************/
// Need a way to get the main sail position/orientation 
static double SailMap(double sail_deg) //TODO
{
	return sail_deg;
}
/***************************************************************************/
static void SailOn(void)
{
	
	actuator.on = true;
	port_pin_set_output_level(ACTUATOR_PWR_PIN, SAIL_ON_STATE);
	return;
}
/***************************************************************************/
static void SailOff(void)
{
	actuator.on = false;
	port_pin_set_output_level(ACTUATOR_PWR_PIN, SAIL_ON_STATE);
	return;	
}
/***************************************************************************/
//TODO we should be using more pointers to optimize heap usage
static double ActuatorMap(double position)
{
	return MATH_Map(position,ACTUATOR_MIN_POS,ACTUATOR_MAX_POS,
		ACTUATOR_PWM_MIN,ACTUATOR_PWM_MAX);
}
/***************************************************************************/
//TODO we should be using more pointers to optimize heap usage
static double ActuatorFeedbackMap(double voltage)
{
	return MATH_Map(voltage,ACTUATOR_POT_MIN,ACTUATOR_POT_MAX,
		ACTUATOR_MIN_POS,ACTUATOR_MAX_POS);
	
}
/***************************************************************************/
/**                                                                       **/
/**                               EOF                                     **/
/**                                                                       **/
/***************************************************************************/
