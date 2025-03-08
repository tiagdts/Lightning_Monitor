/*
 * SHT45.c
 *
 *  Created on: Feb 19, 2023
 *      Author: tiagd
 */

#include "SHT45.h"
//#include "driver/i2c_master.h"

extern SemaphoreHandle_t xSemaphore_I2C;

static i2c_master_dev_handle_t SHT45_handle;

#ifdef USE_OLD_I2C
/**
 *
 * ________________________________________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register msb + ack | register lsb + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|--------------------|--------------------|----------------------|--------------------|------|
 *
 */
esp_err_t SHT45_read(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{

	if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( SHT45 << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}



 /** __________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register msb + ack | register lsb + ack | write n bytes + ack  | stop |
 * --------|---------------------------|--------------------|--------------------|----------------------|------|
 *
 */
esp_err_t SHT45_write(i2c_port_t i2c_num, uint8_t i2c_reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( SHT45 << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // send register we want msb
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

#endif


esp_err_t get_SHT45_TMP_RH(float *temperature, float *humidity, float *tempC)
{
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};

	uint16_t tmp;
	esp_err_t ret;

	uint8_t config_data = SHT45_TMP_RH_HIGH_RES;

	// write command
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ESP_ERROR_CHECK(ret = i2c_master_transmit(SHT45_handle, &config_data, 1, -1));
		//ret = SHT45_write( I2C_NUM_0, config_data );
		xSemaphoreGive( xSemaphore_I2C );
		if( ret != ESP_OK )return ret;
	}
	else ret = ESP_ERR_TIMEOUT;

	// delay 20 ms
	vTaskDelay(20 / portTICK_PERIOD_MS);


	// read data
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ESP_ERROR_CHECK(ret = i2c_master_receive(SHT45_handle, data, 6, -1));
		//ret = SHT45_read( I2C_NUM_0, data, 6 );
		xSemaphoreGive( xSemaphore_I2C );
		if( ret != ESP_OK )return ret;
		// Temperture bytes
		tmp = ( data[0] << 8 ) + data[1];

		//*temperature = ( ( tmp * HDC3020_TEMP_SLOPE ) / HDC3020_DIVISOR) - HDC3020_TEMP_INTERCEPT;
		*temperature = ( (tmp / SHT45_DIVISOR) * SHT45_TEMP_SLOPE ) + SHT45_TEMP_INTERCEPT;
		// (32°F − 32) × 5/9 = 0°C
		*tempC = ( *temperature - 32.0 ) * 0.555555;

		// humidity bytes
		tmp = ( data[3] << 8 ) + data[4];
		*humidity = ((tmp / SHT45_DIVISOR) * SHT45_HUMID_SLOPE) + SHT45_HUMID_INTERCEPT;
	}
	else ret = ESP_ERR_TIMEOUT;

	return ret;
}


esp_err_t get_SHT45_serial_no(uint8_t *sn)
{
	//uint8_t data[6] = {0, 0, 0, 0, 0, 0};

	//uint16_t tmp;
	esp_err_t ret;

	uint8_t config_data = SHT45_SERIAL_NUM;

	// write command
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ESP_ERROR_CHECK(ret = i2c_master_transmit(SHT45_handle, &config_data, 1, -1));
		//ret = SHT45_write( I2C_NUM_0, config_data );
		xSemaphoreGive( xSemaphore_I2C );
		if( ret != ESP_OK )return ret;
	}
	else ret = ESP_ERR_TIMEOUT;

	// delay 50 ms
	vTaskDelay(50 / portTICK_PERIOD_MS);

	// read data
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ESP_ERROR_CHECK(ret = i2c_master_receive(SHT45_handle, sn, 6, -1));
		//ret = SHT45_read( I2C_NUM_0, sn, 6 );
		xSemaphoreGive( xSemaphore_I2C );
	}
	else ret = ESP_ERR_TIMEOUT;

	return ret;
}

esp_err_t SHT45_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle )
{
	esp_err_t ret;
	
	i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SHT45,
    .scl_speed_hz = SHT45_CLOCK,
	};

	ESP_ERROR_CHECK(ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &SHT45_handle));
	return ret;
}
