/*
 * HDC3020.h
 *
 *  Created on: Jun 13, 2021
 *      Author: tiagd
 */

#ifndef MAIN_HDC3020_H_
#define MAIN_HDC3020_H_

// I2C driver
#include "driver/i2c.h"

#include "freertos/task.h"
// Error library
#include "esp_err.h"
#include "io.h"

#define HDC3020	0x44


#define ACK_CHECK_EN 0x01				/*!< I2C master will check ack from slave*/
#define NACK_VAL 0x01					/*!< I2C nack value */

/*				 HDC3020 Registers	*/
#define HDC3020_CMD_AUTO1_MSB			0x21	// Auto Measurement 1 measurement per second
#define HDC3020_CMD_AUTO1_LSB			0x30

#define HDC3020_CMD_MEAS_TH_MSB			0xE0	// Auto Measurement Mode Readout Temp and Humidity
#define HDC3020_CMD_MEAS_TH_LSB			0x00

#define HDC3020_CMD_MANUF_ID_MSB		0x37	// Read Munufacture ID command MSB
#define HDC3020_CMD_MANUF_ID_LSB		0x81	//						    	LSB

#define TI_MANUFACTURE_ID				0x3000

#ifdef HDC2080
#define HDC3020_TEMP_LOW				0x00		// Temperature low byte
#define HDC3020_TEMP_HIGH				0x01		// Temperature high byte
#define HDC3020_HUMID_LOW				0x02		// Humidity low byte
#define HDC3020_HUMID_HIGH				0x03		// Humidity high byte
#define HDC3020_IRQ_STATUS_REG			0x04		// Interrupt and Data Ready Status Register
#define HDC3020_TEMP_MAX				0x05		// Maximum measured temperature
#define HDC3020_HUMID_MAX				0x06		// Maximum measured humidity
#define HDC3020_IRQ_ENA_REG				0x07		// Interrupt Enable Register
#define HDC3020_TEMP_OFFSET				0x08		// Temperature offset register
#define HDC3020_HUMID_OFFSET			0x09		// Humidity offset register
#define HDC3020_TEMP_THRES_L			0x0a		// Temperature Threshold low
#define HDC3020_TEMP_THRES_H			0x0b		// Temperature Threshold high
#define HDC3020_HUMID_THRES_L			0x0c		//	Humidity Threshold low
#define HDC3020_HUMID_THRES_H			0x0d		// 	Humidity Threshold high
#define HDC3020_IRQ_CONFIG				0x0e		//	Interrupt Configuration Register
#define HDC3020_MEAS_CONFIG				0x0f		//	Measurement Configuration Register
#define HDC3020_MANUF_ID_L				0xfc		//	Manufacture ID low byte
#define HDC3020_MANUF_ID_H				0xfd		//	Manufacture ID high byte
#define HDC3020_DEVICE_ID_L				0xfe		//	Device ID low byte
#define HDC3020_DEVICE_ID_H				0xff		//	Device ID high byte


/* 			Interrupt Status Bits- HDC3020_IRQ_STATUS_REG					*/
/* 				bits 2:0 reserved 	*/
#define HDC3020_DRDY_STATUS				0x80		// 0 = data not ready, 1 = data ready, cleared when read
#define HDC3020_TH_STATUS				0x40		// 0 = no interrupt, 1 = Threshold high temp interrupt, cleared when read
#define HDC3020_TL_STATUS				0x20		// 0 = no interrupt, 1 = Threshold low temp interrupt, cleared when read
#define HDC3020_HH_STATUS				0x10		// 0 = no interrupt, 1 = Threshold high humidity interrupt, cleared when read
#define HDC3020_HL_STATUS				0x08		// 0 = no interrupt, 1 = Threshold low humidity interrupt, cleared when read


/* 			Interrupt Enable Bits - HDC3020_IRQ_ENA_REG			*/
/* 				bits 2:0 reserved 					*/
#define HDC3020_DRDY_ENABLE				0x80		// 0 = interrupt disabled, 1 = interrupt on data ready, cleared when read
#define HDC3020_TH_ENABLE				0x40		// 0 = interrupt disabled, 1 = interrupt on Threshold high temp, cleared when read
#define HDC3020_TL_ENABLE				0x20		// 0 = interrupt disabled, 1 = interrupt on Threshold low temp interrupt, cleared when read
#define HDC3020_HH_ENABLE				0x10		// 0 = interrupt disabled, 1 = interrupt on Threshold high humidity interrupt, cleared when read
#define HDC3020_HL_ENABLE				0x08		// 0 = interrupt disabled, 1 = interrupt on Threshold low humidity interrupt, cleared when read

/* 			Interrupt Configuration Bits - HDC3020_IRQ_CONFIG			*/
/* 													*/
#define HDC3020_RESET					0x01		// 0 normal operation, 1 = reset chip
#define HDC3020_AUTO_MEAS_MODE			0x70		// Auto Measurement Mode setting bits
#define HDC3020_AUTO_DISABLED			0x00		// Auto Measurement off
#define HDC3020_AUTO_TWO_MIN			0x10		// Auto Measurement 1 sample per minute
#define HDC3020_AUTO_ONE_MIN			0x20		// Auto Measurement 2 samples per minute
#define HDC3020_AUTO_1_TENTH_HZ			0x30		// Auto Measurement 0.1 Hz
#define HDC3020_AUTO_2_TENTH_HZ			0x40		// Auto Measurement 0.2 Hz
#define HDC3020_AUTO_ONE_HZ				0x50		// Auto Measurement 1 Hz
#define HDC3020_AUTO_TWO_HZ				0x60		// Auto Measurement 2 Hz
#define HDC3020_AUTO_FIVE_HZ			0x70		// Auto Measurement 5 Hz
#define HDC3020_HEAT_ENABLE				0x08		// 0 = heater off, 1 = heater on
#define HDC3020_IRQ_ENABLE				0x04		// 0 =  irq pin disabled, 1 = pin output enabled
#define HDC3020_IRQ_POLARITY			0x02		// 0 =  active low, 1 = active high
#define HDC3020_IRQ_MODE				0x01		// 0 = level sensitive, 1 = comparator mode


/* 			Measurement+ Configuration Bits - HDC3020_MEAS_CONFIG			*/
/* 			Bit 3 reserved							*/

#define HDC3020_TEMP_14BIT				0x00		// 14 bit temperature measurements
#define HDC3020_TEMP_11BIT				0x80		// 11 bit temperature measurements
#define HDC3020_TEMP_9BIT				0x40		// 9 bit temperature measurements
#define HDC3020_HUMID_14BIT				0x00		// 14 bit temperature measurements
#define HDC3020_HUMID_11BIT				0x20		// 11 bit temperature measurements
#define HDC3020_HUMID_9BIT				0x10		// 9 bit temperature measurements
#define HDC3020_TMP_HUMID				0x00		// measure temperature and humidity
#define HDC3020_TEMP_ONLY				0x02		// measure temperature only
#define HDC3020_MEAS_TRIGGER			0x01		// 1 = start measurment, self clearing when measurement complete

#define HDC3020_TEMP_OFFSET_7			-20.60
#define HDC3020_TEMP_OFFSET_6			10.32
#define HDC3020_TEMP_OFFSET_5			5.16
#define HDC3020_TEMP_OFFSET_4			2.58
#define HDC3020_TEMP_OFFSET_3			1.28
#define HDC3020_TEMP_OFFSET_2			0.64
#define HDC3020_TEMP_OFFSET_1			0.32
#define HDC3020_TEMP_OFFSET_0			0.16

#define HDC3020_TEMP_CORRECTION			0xf2		// -2.24 degree correction

#endif

#ifdef DEGREES_C
#define HDC3020_TEMP_SLOPE				175.0	// Degrees C conversion values
#define	HDC3020_TEMP_INTERCEPT			-45.0

#else
#define HDC3020_TEMP_SLOPE				315.0	// Degrees F conversion values
#define	HDC3020_TEMP_INTERCEPT			-49.0
#endif

#define HDC3020_HUMID_SLOPE				100.0

#define HDC3020_DIVISOR					65535.0


esp_err_t HDC3020_read(i2c_port_t i2c_num, uint8_t* i2c_reg, uint8_t* data_rd, size_t size);
esp_err_t HDC3020_write(i2c_port_t i2c_num, uint8_t* i2c_reg, uint8_t* data_wr, size_t size);
esp_err_t HDC3020_start(void);
esp_err_t HDC3020_config(void);
esp_err_t HDC3020_read_humid( float *humidity );
esp_err_t HDC3020_read_temp( float *temperature );
esp_err_t HDC3020_read_both( float *temperature, float *humidity );
esp_err_t HDC3020_read_deviceID(uint8_t *deviceID);
esp_err_t HDC3020_read_ManufID(uint16_t *manufID);
esp_err_t HDC3020_read_config(uint8_t *config);
esp_err_t HDC3020_enable_irq(void);
esp_err_t print_temp_hum(void);
esp_err_t BMP_read_deviceID(uint8_t *deviceID);

#endif /* MAIN_HDC3020_H_ */
