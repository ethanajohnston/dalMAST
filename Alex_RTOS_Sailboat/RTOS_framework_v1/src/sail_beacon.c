/*
 * sail_beacon.c
 *
 *	Created: 2022-06-03 6:44:48 PM
 *  Author: Matthew
 */ 

#include "sail_beacon.h"
#include "sail_uart.h"
#include "sail_tasksinit.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "sail_math.h"
#include "sail_debug.h"

uint16_t beacon_msg_length [beacon_msg_type_total] ={
	3,
	5,
};

#define BEACON_SLEEP_PERIOD_MS 10000
/* 
This function is the beacon task start point


*/
void DataLogTask(void){

	EventBits_t event_bits;
	TickType_t beacon_task_delay = pdMS_TO_TICKS(BEACON_SLEEP_PERIOD_MS);
	
	enum status_code rc;
	
	rc = UART_Init(UART_BEACON);
	
	switch(rc){
		case STATUS_OK:
			DEBUG_Write("INIT\n\r");
			break;
		default:
			DEBUG_Write("ERROR_INIT\n\r");
			break;		
	}
	
	rc = UART_Enable(UART_BEACON);
	
	switch(rc){
		case STATUS_OK:
			DEBUG_Write("ENABLE\n\r");
			break;
		default:
			DEBUG_Write("ERROR_ENABLE\n\r");
			break;
					
	}
	while(1){
		
		event_bits = xEventGroupWaitBits(mode_event_group,
										CTRL_MODE_AUTO_BIT | CTRL_MODE_REMOTE_BIT,
										pdFALSE,
										pdFALSE,
										portMAX_DELAY);
		
		taskENTER_CRITICAL();
		watchdog_counter = 0x02;
		taskEXIT_CRITICAL();
		
		DEBUG_Write("\n\r <<<<<<<<<<< Do Log Data >>>>>>>>>>>\n\r");
		
		running_task = eLogData;
		
		rc = beacon_status();
		
		//switch(rc){
		
		
			
			
		//}
		
		vTaskDelay(beacon_task_delay);
	}
	
	
}
/*
This function will check the status of the beacon and return a status code 
accordingly 
*/
enum status_code beacon_status(void){
	
	char msgtx[3] = "AT\n";
	static char msgrx[200];
	
	for ( int i = 0; i <200;++i){
		msgrx[i] = NULL;
	}
	enum status_code rc;
	DEBUG_Write_Unprotected("%s\n\r",msgtx);
	
	rc = UART_TxString(UART_BEACON,&msgtx);
	
	switch (rc){
		
		case STATUS_OK:
			DEBUG_Write_Unprotected("STATUS OK\n\r");
			break;
		default:
			DEBUG_Write_Unprotected("ERROR\n\r");	
			break;
	}
	rc = UART_RxString(UART_BEACON,( uint8_t *)msgrx, beacon_msg_length[beacon_AT_msg]);
	
	DEBUG_Write("%s\n\r",msgrx);
	DEBUG_Write("%d\n\r %d\n\r", rc, STATUS_ERR_NO_MEMORY);
	return STATUS_OK;
	
};
/*
This function will check the signal strength of the beacon to the iridium
network. A status code is return according to the signal strength
*/
enum status_code signal_strength(void){
	
	
	
};

enum status_code tx_sbd_msg(void){
	
	
};

enum status_code rx_sbd_msg(void){
	
	
	
};

enum status_code gather_data(void){
	
	
};

