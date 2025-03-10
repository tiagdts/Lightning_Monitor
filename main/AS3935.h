/*
 * AS3935.h
 *
 *  Created on: Feb 11, 2025
 *      Author: tiagd
 */

#ifndef MAIN_AS3935_H_
#define MAIN_AS3935_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Error library
#include "esp_err.h"
// I2C driver
#include "driver/i2c_master.h"
// SPI driver
#include "driver/spi_master.h"

#include "io.h"
#include "Station_Data_Types.h"

// #define AS3935				0x01
#define AS3935_CLOCK 		1000000
#define AS3935_TIMEOUT 		30

#define ACK_CHECK_EN 0x01				/*!< I2C master will check ack from slave*/
#define NACK_VAL 0x01					/*!< I2C nack value */



// registers
#define REG_X00					0x00
#define 	PDW_BIT				0b00000001		// 1 Power down, 0 active
#define 	AFE_GB_BITS			0b00111110		// AFE Gain Boost, 5 bits
#define			INDOOR_AFE		0b00100100		// AFE indoor setting
#define			OUTDOOR_AFE		0b00011100		// AFE outdoor setting

#define REG_X01					0x01
#define		NF_LEV_BITS			0b01110000		// Noise Floor Level
#define		WDTH_BITS			0b00001111		// Watchdog threshold	

#define REG_X02					0x02
#define 	CL_STAT_BIT			0b01000000		//	Clear statistics
#define		MIN_NUM_LIGH_BITS	0b00110000		//  Minimum number of lightning
#define		SREJ_BITS			0b00001111		//  Spike rejection

/* --------------------------------------------------
Minimum Number of Lightning 	REG0x02[5] REG0x02[4]
-----------------------------------------------------
	1 								0 			0
	5 								0 			1
	9 								1 			0
	16 								1 			1
*/	

#define REG_X03					0x03
#define 	LCO_FDIV_BITS		0b11000000		//	Frequency division ration for antenna tuning
#define 	MASK_DIST_BIT		0b00100000		// 	Mask Disturber
#define 	INT_BITS			0b00001111		// 	Interrupt
#define 		INT_NH 			0b0001			//		Noise level too high
#define 		INT_D			0b0100			//		Disturber detected
#define 		INT_L			0b1000			//		Lightning interrupt	

/*
Interrupt Name 	REG0x03[3:0] 			Description
	INT_NH 			0001 			Noise level too high
	INT_D 			0100 			Disturber detected
	INT_L 			1000 			Lightning interrupt
*/
	
												

#define REG_X04					0x04			//	Energy of the Single Lightning LSBYTE

#define REG_X05					0x05			//	Energy of the Single Lightning MSBYTE


#define REG_X06					0x06			
#define 	S_LIG_MM			0b00011111		//	Energy of the Single Lightning MMSBYTE

#define REG_X07					0x07
#define 	DISTANCE			0b00111111		//	Distance estimation

#define REG_X08					0x08
#define 	DISP_LCO			0b10000000		//	Display LCO on IRQ pin
#define 	DISP_SRCO			0b01000000		//	Display SRCO on IRQ pin  RC oscillators 1.1 MHz
#define 	DISP_TRCO			0b00100000		//	Display TRCO on IRQ pin` RC oscillators 32.768 kHz
#define 	TUN_CAP				0b00001111		//	Internal Tuning Capacitors (from 0 to 120pF in steps of 8pF)
#define 	PIN_MASK			0b11100000		//  mask for setting display pin
#define		LCO_CALIBRATION		0b00001001		//	prototype tuning calibration value - verified with scope (2/17/2025)

#define REG_X3A					0x3a
#define 	TRCO_CALIB_DONE		0b10000000		// Calibration of TRCO done (1=successful)
#define 	TRCO_CALIB_NOK		0b01000000		// Calibration of TRCO unsuccessful (1=not successful)

#define REG_X3B					0x3b
#define 	SRCO_CALIB_DONE		0b10000000		// Calibration of SRCO done (1=successful)
#define 	SRCO_CALIB_NOK		0b01000000		// Calibration of SRCO unsuccessful (1=not successful)

#define PRESET_DEFAULT 			0x3c
#define CALIB_RCO				0x3d
#define DIRECT_COMMAND			0x96

/* SPI */
#define SPI_WRITE				0b00000000
#define SPI_READ				0b01000000
#define SPI_CMD_MASK			0b00111111

esp_err_t set_AS3935_irq_pin_function( uint8_t pin_setting, uint8_t tuning );
esp_err_t add_AS3935_to_SPI_bus( void );
esp_err_t get_AS3935_reg( uint8_t reg, uint8_t *value );
esp_err_t set_AS3935_reg( uint8_t reg, uint8_t value );
esp_err_t direct_command_AS3935( uint8_t reg );
esp_err_t calibrate_AS3935( uint16_t location );
void AS3935_createSemaphores(void);
void AS3935_isr_handler(void* arg);
esp_err_t get_AS3935_reg32( uint8_t reg, uint32_t *value );
void AS3935_Task(void *pvParameter);
esp_err_t get_lightning( uint32_t *energy, uint8_t *distance );
esp_err_t get_lightning_data( uint8_t *status, uint32_t *energy, uint8_t *distance );
#endif /* MAIN_AS3935_H_ */

