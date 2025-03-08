/*
 * SHT45.h
 *
 *  Created on: Feb 18, 2023
 *      Author: tiagd
 */

#ifndef MAIN_SHT45_H_
#define MAIN_SHT45_H_


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Error library
#include "esp_err.h"
// I2C driver
#include "driver/i2c_master.h"
#include "io.h"

#define SHT45 			0x44
#define SHT45_CLOCK 	1000000
#define SHT45_TIMEOUT 	30

#define ACK_CHECK_EN 0x01				/*!< I2C master will check ack from slave*/
#define NACK_VAL 0x01					/*!< I2C nack value */

// COMMANDS
#define SHT45_TMP_RH_HIGH_RES			0xfd
#define SHT45_TMP_RH_MID_RES			0xf6
#define SHT45_TMP_RH_LOW_RES			0xe0
#define SHT45_SERIAL_NUM				0x89
#define SHT45_RESET						0x94

#define SHT45_TEMP_SLOPE				315.0	// Degrees F conversion values
#define	SHT45_TEMP_INTERCEPT			-49.0

#define SHT45_HUMID_SLOPE				125.0
#define	SHT45_HUMID_INTERCEPT			-6.0

#define SHT45_DIVISOR					65535.0

esp_err_t SHT45_read(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);
esp_err_t SHT45_write(i2c_port_t i2c_num, uint8_t i2c_reg);
esp_err_t get_SHT45_TMP_RH(float *temperature, float *humidity, float *tempC);
esp_err_t get_SHT45_serial_no(uint8_t *sn);
esp_err_t SHT45_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle );

#endif /* MAIN_SHT45_H_ */
