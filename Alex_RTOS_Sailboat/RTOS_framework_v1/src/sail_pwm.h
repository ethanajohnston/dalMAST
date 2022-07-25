#ifndef SAIL_PWM_H
#define SAIL_PWM_H
/*                           *******************
******************************* C HEADER FILE *******************************
**                           *******************                           **
**                                                                         **
** filename  : sail_pwm.h                                                 **
** author    : Thomas Gwynne-Timothy                                       **
** created   : 2016-08-16                                                  **
**                                                                         **
*****************************************************************************

Header file for the PWM driver module for the autonomous sailboat project.

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
#define PWM_MAX_DUTY 250
/***************************************************************************/
/**                                                                       **/
/**                     TYPDEFS AND STRUCTURES                            **/
/**                                                                       **/
/***************************************************************************/
typedef enum PWM_ChannelIDs {
	PWM_SAIL,
	PWM_RUDDER,
	PWM_NUM_CHANNELS
} PWM_ChannelID;
/***************************************************************************/
/**                                                                       **/
/**                     PROTOTYPES OF EXPORTED FUNCTIONS                  **/
/**                                                                       **/
/***************************************************************************/
/* PWM_Init
 * Initialize PWM driver module
 * Status:
 *		STATUS_OK - PWM driver module initialization was successful
 */
enum status_code PWM_Init(void);


/* PWM_SetDuty
 * Set the sleep time of the control unit
 * Status:
 *		STATUS_OK - Successfully set duty
 */ 
enum status_code PWM_SetDuty(PWM_ChannelID id, uint8_t duty);


/* PWM_Disable
 * Set duty circle to 0%, equivalent to disable
 * Status:
 *		STATUS_OK - Successfully disable duty
 */ 
enum status_code PWM_Disable(PWM_ChannelID id);

#endif // SAIL_PWM_H
/***************************************************************************/
/**                                                                       **/
/**                               EOF                                     **/
/**                                                                       **/
/***************************************************************************/