/* sail_debug.c
 * Implementation of the debug module.
 * Created on June 27, 2016.
 * Created by Thomas Gwynne-Timothy.
 */
/*
Revision by Matthew,
Adding a improve version of the Debug function
LOG(), will log msg and data items to the proper devices depending on the 
sail boat configuration 
*/ 
#include "sail_debug.h"


#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef DEBUG
//static uint8_t debug_buffer[DEBUG_BUFFER_LENGTH];
static bool init_flag = false;
#endif


enum status_code DEBUG_Init(void) {
#ifdef DEBUG
	// Check if the module has already been initialized
	if (init_flag) {
		// Error, already initialized
		return STATUS_ERR_ALREADY_INITIALIZED;
	}
		// Return if the UART cannot be initialized
	if (UART_Init(UART_XEOS) != STATUS_OK) {
		return STATUS_ERR_DENIED;
	}
	
	// Set the initialization flag
	init_flag = true;
#endif	
	return STATUS_OK;
}

//static uint8_t debug_buffer[DEBUG_BUFFER_LENGTH];

enum status_code DEBUG_Write(const char *format, ...) {
#ifdef DEBUG
	uint8_t debug_buffer[DEBUG_BUFFER_LENGTH];
	// Check if the module has not been initialized
	if (!init_flag) {
		// Error, already initialized
		return STATUS_ERR_NOT_INITIALIZED;
	}
		
	va_list args;
	// Load the buffer
	va_start(args, format);
	vsnprintf((char *)debug_buffer, DEBUG_BUFFER_LENGTH, format, args);
	va_end(args);
	
	// Send the string
	// This prints to a serial terminal when the SAMD20 is powered by a laptop
	UART_TxString(UART_XEOS, debug_buffer); 
#endif	
	
	return STATUS_OK;
}

enum status_code DEBUG_Write_Unprotected(const char *format, ...) {
	#ifdef DEBUG
	uint8_t debug_buffer[DEBUG_BUFFER_LENGTH];
	// Check if the module has not been initialized
	if (!init_flag) {
		// Error, already initialized
		return STATUS_ERR_NOT_INITIALIZED;
	}
	
	va_list args;
	// Load the buffer
	va_start(args, format);
	vsnprintf((char *)debug_buffer, DEBUG_BUFFER_LENGTH, format, args);
	va_end(args);
	
	// Send the string
	// This prints to a serial terminal when the SAMD20 is powered by a laptop
	UART_TxString_Unprotected(UART_XEOS, debug_buffer);
	#endif
	return STATUS_OK;
}
