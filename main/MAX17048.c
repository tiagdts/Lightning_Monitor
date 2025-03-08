/*
 * MAX17048.c
 *
 *  Created on: Sep 4, 2024
 *      Author: dschmidt
 */

#include "MAX17048.h"

extern SemaphoreHandle_t xSemaphore_I2C;
static i2c_master_dev_handle_t MAX17048_handle;

esp_err_t MAX17048_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle )
{
	esp_err_t ret;
	
	i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MAX17048,
    .scl_speed_hz = MAX17048_CLOCK,
	};

	ESP_ERROR_CHECK(ret = i2c_master_bus_add_device(bus_handle,&dev_cfg,&MAX17048_handle));
	return ret;
}

esp_err_t MAX17048_read_VCELL_data( double *batt_volts )
{
	esp_err_t ret;
	uint8_t cmd = VCELL;
	uint8_t data[2];
	
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = i2c_master_transmit_receive( MAX17048_handle, &cmd, sizeof(cmd), data, 2, -1);
		xSemaphoreGive( xSemaphore_I2C );
		*batt_volts = ( (data[0] * 256) + data[1] ) * VCELL_RES;
	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}

esp_err_t MAX17048_read_SOC_data( double *batt_soc )
{
	esp_err_t ret;
	uint8_t cmd = SOC;
	uint8_t data[2];
	
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = i2c_master_transmit_receive( MAX17048_handle, &cmd, sizeof(cmd), data, 2, -1);
		xSemaphoreGive( xSemaphore_I2C );
		*batt_soc = ( ( data[0] * 256) + data[1] ) * SOC_RES;
	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}

esp_err_t MAX17048_read_CRATE_data( uint8_t *data )
{
	esp_err_t ret;
	uint8_t cmd = CRATE;
	//uint8_t data[2];
	
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = i2c_master_transmit_receive( MAX17048_handle, &cmd, sizeof(cmd), data, 2, -1);
		xSemaphoreGive( xSemaphore_I2C );
		//*c_rate = ( (data[0] * 256) + data[1] ) * CRATE_RES;
		
	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}

esp_err_t MAX17048_reset( void )
{
	esp_err_t ret;

	uint8_t data[3] = {MODE,0x40,0x00};
	
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = i2c_master_transmit( MAX17048_handle, data, 3, -1);
		xSemaphoreGive( xSemaphore_I2C );
		
	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}

esp_err_t MAX17048_set_Rcomp( float T )
{
	esp_err_t ret;

	uint8_t data[3] = {CONFIG,0x00,ATHD};
	float RCOMP;

	// T is battery temperature (degrees Celsius)
	if (T > 20)
	{
		RCOMP = RCOMP0 + ( (T - 20) * TEMP_CO_UP );
	}
	else
	{
		RCOMP = RCOMP0 + ( (T - 20) * TEMP_CO_DOWN );
	}
	// integer part of RCOMP
	data[1] = (uint8_t)(RCOMP);

	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = i2c_master_transmit( MAX17048_handle, data, 3, -1);
		xSemaphoreGive( xSemaphore_I2C );

	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}
