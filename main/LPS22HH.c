/*
 * LPS22HH.c
 *
 *  Created on: Mar 22, 2025
 *      Author: tiagdts
 */
#include "LPS22HH.h"

extern SemaphoreHandle_t xSemaphore_I2C;

static i2c_master_dev_handle_t LPS22HH_handle;

esp_err_t LPS22HH_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle )
{
	esp_err_t ret;
	
	i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = LPS22HH_0,
    .scl_speed_hz = LPS22HH_CLOCK,
	};

	ESP_ERROR_CHECK(ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &LPS22HH_handle));
	return ret;
}

esp_err_t LPS22HH_read_deviceID(uint8_t *deviceID)
{
	uint8_t cmd = WHO_AM_I;
	
	esp_err_t ret = i2c_master_transmit_receive( LPS22HH_handle, &cmd, sizeof(cmd), deviceID, 1, -1);

	return ret;
}

esp_err_t LPS22HH_configure( void )
{
	esp_err_t ret = ESP_OK;
	uint8_t cmd[2] = { BMP390_REG_CALIB_DATA, 0 };
	
	return ret;
	
}

// check status, read data if ready
esp_err_t  LPS22HH_get_pressure_temperature(uint8_t *data, bool *data_good )
{
	esp_err_t ret;
	uint8_t cmd = CTRL_REG3;

	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		if( (ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 1, -1) ) == ESP_OK )
		//if( (ret = BMP390_read( I2C_NUM_0, BMP390_REG_INT_STATUS, data, 1 ) ) == ESP_OK )
		{
			if( *data & DRDY )
			{
				cmd = PRESSURE_OUT_XL;
				// read data
				if( (ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 5, -1) ) == ESP_OK )
				{
					*data_good = true;
				}	
				else *data_good = false;	
			}
		}
		xSemaphoreGive( xSemaphore_I2C );
	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}
	
