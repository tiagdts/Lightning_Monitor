/*
 * MAX17048.h
 *
 *  Created on: Sep 4, 2024
 *      Author: dschmidt
 */

#ifndef MAIN_MAX17048_H_
#define MAIN_MAX17048_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Error library
#include "esp_err.h"
// I2C driver
#include "driver/i2c_master.h"
#include "IO.h"

#define MAX17048 	0x36
#define MAX17048_CLOCK 	1000000
#define MAX17048_TIMEOUT 	30

// registers
#define VCELL 		0x02		// ADC measurement of VCELL.
#define	SOC			0x04		// Battery state of charge.
#define MODE		0x06		// Initiates quick-start, reports hibernate mode, and enables sleep mode.
#define VERSION		0x08		// IC production version.
#define HIBRT		0x0a		// Controls thresholds for entering and exiting hibernate mode.
#define CONFIG		0x0c		//Compensation to optimize performance, sleep mode, alert indicators, and configuration
#define VALRT		0x14		// Configures the VCELL range outside of which alerts are generated.
#define CRATE		0x16		// Approximate charge or discharge rate of the battery.
#define VRESET_ID	0x18		// Configures VCELL threshold below which the IC resets itself, ID is a one-time factoryprogrammable identifier.
#define STATUS		0x1a		// Indicates overvoltage, undervoltage, SOC change, SOC low, and reset alerts.
#define CMD			0xfe		// Sends POR command.

#define VCELL_RES		0.000078125 // VOLTS
#define SOC_RES			0.00390625	// Percent
#define CRATE_RES		0.208		// Percent/Hour
#define RCOMP0			0x97		// default RCOMP
#define TEMP_CO_UP		-0.5
#define TEMP_CO_DOWN	-5.0
#define SLEEP			0x80
#define ALSC			0x00
#define ALRT			0X00
#define ATHD			0x1c

esp_err_t MAX17048_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle );
esp_err_t MAX17048_read_VCELL_data(double *batt_volts);
esp_err_t MAX17048_read_SOC_data(double *batt_soc);
//esp_err_t MAX17048_read_CRATE_data( float *c_rate );
esp_err_t MAX17048_read_CRATE_data( uint8_t *data );
esp_err_t MAX17048_reset( void );
esp_err_t MAX17048_set_Rcomp( float T );

#endif /* MAIN_MAX17048_H_ */
