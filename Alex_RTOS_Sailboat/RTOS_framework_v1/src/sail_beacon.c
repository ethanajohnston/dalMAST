/*
 * sail_beacon.c
 *
 *	Created: 2022-06-03 6:44:48 PM
 *  Author: Matthew
 */ 

#include "sail_beacon.h"
#include "sail_uart.h"
/* 
This function is the beacon task start point


*/
void DataLogTask(void){

	EventBits_t event_bits;
	TickType_t beacon_task_delay = pdMS_TO_TICKS(RADIO_SLEEP_PERIOD_MS);
	
	enum status_code rc;
	
	
	while(1){
		
		event_bits = xEventGroupWaitBits(mode_event_group,
										CTRL_MODE_AUTO_BIT,
										pdFALSE,
										pdFALSE,
										portMAX_DELAY);
		
		taskENTER_CRITICAL();
		watchdog_counter | = 0x02;
		taskEXIT_CRITICAL();
		
		DEBUG_Write("\n\r <<<<<<<<<<< Do Log Data >>>>>>>>>>>\n\r");
		
		running_task = eLogData;
		
		rc = BeaconStatus();
		
		switch(rc){
			
			
			
		}
		
		
	}
	
	
}
/*
This function will check the status of the beacon and return a status code 
accordingly 
*/
enum status_code beacon_status(void){
	
	uint16_t msg_length = 2;
	uint8_t *msgtx;  ;
	uint8_t *msgrx;
	
	UART_TxString(UART_BEACON,msgtx);
	
	UART_RxString(UART_BEACON,msgrx, msg_length);
	
	DEBUG_Write("%s\n\r",msgrx);
	
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

