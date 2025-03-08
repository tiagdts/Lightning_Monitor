/*
 * IO.h
 *
 *  Created on: May 19, 2021
 *      Author: dschmidt
 */

#ifndef MAIN_IO_H_
#define MAIN_IO_H_

#include <ctype.h>
#include <stdint.h>
// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
// GPIO
#include "driver/gpio.h"

// I2C driver
#include "driver/i2c_master.h"

// SPI driver
#include "driver/spi_master.h"

// Error library
#include "esp_err.h"

#include "SHT45.h"
#include "BMP390.h"
#include "MAX17048.h"




//#define TASK_DATA_WAIT_TIME 100
#define TASK_WAIT_TIME		300
#define I2C_SCAN_WAIT_TIME 	15

// I2C pins
#define PIN_NUM_SCL			4
#define PIN_NUM_SDA 		3

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_FREQ_HZ 100000

// SPI Pins
#  define IC_SPI_HOST       SPI2_HOST
#  define PIN_NUM_MISO      7
#  define PIN_NUM_MOSI      8
#  define PIN_NUM_CLK       10
#  define PIN_NUM_CS        2

#define I2C_PORT I2C_NUM_0

#define BMP390_IRQ			5

#define EXPECTED_I2C_ADDRESSES 3

void init_GPIO( void );
uint8_t scan_i2c( i2c_master_bus_handle_t *bus_handle);
esp_err_t init_I2C(i2c_master_bus_handle_t *bus_handle);
void wifi_connect_LED_on( void );
void wifi_connect_LED_off( void );
//void display_ip_address( wifi_status_t *wifi_status );
bool get_connection_state( void );

#endif /* MAIN_IO_H_ */
