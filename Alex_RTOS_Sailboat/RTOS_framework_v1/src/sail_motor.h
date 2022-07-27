#ifndef SAIL_MOTOR_H_
#define SAIL_MOTOR_H_
/*                           *******************
******************************* C HEADER FILE *******************************
**                           *******************                           **
**                                                                         **
** filename  : sail_motor.h                                                **
** author    : Thomas Gwynne-Timothy                                       **
** created   : 2016-08-16                                                  **
**                                                                         **
*****************************************************************************
Header file for the motor controller for the autonomous sailboat project.


/***************************************************************************/
/**                                                                       **/
/**                     MODULES USED                                      **/
/**                                                                       **/
/***************************************************************************/

#include <asf.h>

/***************************************************************************/
/**                                                                       **/
/**                     DEFINITIONS AND MACROS                            **/
/**                                                                       **/
/***************************************************************************/

#define RUDDER_DIR_PIN			PIN_PB07
#define RUDDER_POWER_PIN		PIN_PB15
#define MOTOR_RUDDER_CW_STATE	true

/***************************************************************************/
/**                                                                       **/
/**                     TYPDEFS AND STRUCTURES                            **/
/**                                                                       **/
/***************************************************************************/

/***************************************************************************/
/**                                                                       **/
/**                     PROTOTYPES OF EXPORTED FUNCTIONS                  **/
/**                                                                       **/
/***************************************************************************/
/* MOTOR_Init
 * Initialize motor
 * Status:
 *   STATUS_OK - Motor initialization was successful
 */
enum status_code MOTOR_Init(void);

/* MOTOR_SetSail
 * Set sail degree
 * Input:
 *	 angle - original sail angle value
 * Status:
 *	 STATUS_OK - Successfully set sail angle
 */
enum status_code MOTOR_SetSail(double angle);

/* MOTOR_SetRudder
 * Set rudder degree
 * Input:
 *	 angle - original rudder angle value
 * Status:
 *	 STATUS_OK - Successfully set rudder angle
 */
enum status_code MOTOR_SetRudder(double angle);

extern void ShaftControlCallback(struct tc_module *const module_inst);

#endif // SAIL_MOTOR_H_
/***************************************************************************/
/**                                                                       **/
/**                               EOF                                     **/
/**                                                                       **/
/***************************************************************************/