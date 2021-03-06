/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "GPSSensor.h"

int main (void)
{
	gps_gprmc gps_sensor;
//	gps_deci_signed *ds_main;
//	gps_deci_marge *marge_main;
	distance_deci_signed distance_main;
	
	gps_deci_signed ds_position,ds_poi;
//	gps_deci_marge marge_poi;
	
	system_init();
	init_gps_link();	

	/* Insert application code here, after the board has been initialized. */

	/* This skeleton code simply sets the LED to the state of the button. */
	while (1) {
		/* Is button pressed? */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, so turn LED on. */
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
		} else {
			/* No, so turn LED off. */
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
		}
		
		receive_usart();
		
		get_gprmc(&gps_sensor);
		        
		//aquisition des donn?es et calculs
		convert_gprmc_to_deci_signed(&gps_sensor, &ds_position);//Convert raw GPS data into our desired format
		calcul_distance(&ds_position, &ds_poi, &distance_main);//distance calculus

	}
}
