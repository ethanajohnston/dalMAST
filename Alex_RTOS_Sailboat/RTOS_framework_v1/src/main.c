/* main.c
 * Main function for the sailboat.
 * Initializes the system and then starts the freeRTOS tasks
 * 
 * 
 */

#include "sail_ctrl.h"
#include "sail_debug.h"
#include "sail_tasksinit.h"

int main(void)
{
	CTRL_InitSystem(); // Init -> DEBUG UART, RADIO, EERPROM
	#if 0
	CTRL_InitSensors(); // Initialize the WeatherStation
	startup(); //Enable WS - Init Motors - Get the first waypoint
	#endif
	init_tasks();
}
