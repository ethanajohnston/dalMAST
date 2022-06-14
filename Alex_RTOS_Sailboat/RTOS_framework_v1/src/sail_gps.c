/*
 * sail_gps.c
 *
 * Created: 2022-05-19 8:48:28 PM
 * Author: mrcoc
 */ 
#include "Sail_WEATHERSTATION.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "sail_ctrl.h"
#include "sail_tasksinit.h"
#include "usart_interrupt.h"
#include "sail_gps.h"

#define GPS_SLEEP_PERIOD_MS 10000

static char msg[1024];
void ReadGps(void){
	
	DEBUG_Write("GPS Module \r\n");
	
	EventBits_t event_bits;
	
	TickType_t read_weather_sensor_delay = pdMS_TO_TICKS(GPS_SLEEP_PERIOD_MS);
	
	event_bits = xEventGroupWaitBits(mode_event_group,                        /* Test the mode event group */
	CTRL_MODE_AUTO_BIT | CTRL_MODE_REMOTE_BIT, /* Wait until the sailboat is in AUTO or REMOTE mode */
	pdFALSE,                                 /* Bits should not be cleared before returning. */
	pdFALSE,                                 /* Don't wait for both bits, either bit will do. */
	portMAX_DELAY);                          /* Wait time does not expire */


	running_task = ;
	
	while(1){
		
		if(UART_RxString(&msg) == STATUS_OK) {
			UART_TxString(UART_GPS, &msg);
		}
		vTaskDelay(read_weather_sensor_delay);
	}
	
}
