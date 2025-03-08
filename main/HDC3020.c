/*
 * HDC3020.c
 *
 *  Created on: Jun 13, 2021
 *      Author: tiagd
 */

#include "HDC3020.h"

extern SemaphoreHandle_t xSemaphore_I2C;
// Measurement Resolution Configuration
// configure to measure both humidity and temperature at 14 bit resolution
// static uint8_t HDC3020_configuration = HDC3020_TEMP_14BIT | HDC3020_HUMID_14BIT | HDC3020_TMP_HUMID;

// Interrupt configuration
// configure to manually trigger measurement and enable tie interrupt pin output
// static uint8_t HDC3020_irq_config = HDC3020_AUTO_DISABLED | HDC3020_IRQ_ENABLE;


/**
 *
 * ________________________________________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register msb + ack | register lsb + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|--------------------|--------------------|----------------------|--------------------|------|
 *
 */
esp_err_t HDC3020_read(i2c_port_t i2c_num, uint8_t* i2c_reg, uint8_t* data_rd, size_t size)
{

	if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( HDC3020 << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // send register we want msb
    i2c_master_write_byte(cmd, i2c_reg[0], ACK_CHECK_EN);
    // send register we want lsb
    i2c_master_write_byte(cmd, i2c_reg[1], ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( HDC3020 << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}



 /** __________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register msb + ack | register lsb + ack | write n bytes + ack  | stop |
 * --------|---------------------------|--------------------|--------------------|----------------------|------|
 *
 */
esp_err_t HDC3020_write(i2c_port_t i2c_num, uint8_t* i2c_reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( HDC3020 << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // send register we want msb
    i2c_master_write_byte(cmd, i2c_reg[0], ACK_CHECK_EN);
    // send register we want lsb
    i2c_master_write_byte(cmd, i2c_reg[1], ACK_CHECK_EN);
    // write the data
    if( size != 0 )
    {
    	i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    }
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t HDC3020_config(void)
{

	uint8_t config_data[2] = { HDC3020_CMD_AUTO1_MSB, HDC3020_CMD_AUTO1_LSB };
	uint8_t tmp_data = 0 ;

	// set for auto-update at 1 second
	esp_err_t ret = HDC3020_write( I2C_NUM_0, config_data, &tmp_data, 0 );
	if( ret != ESP_OK )return ret;


	return ret;
}

#ifdef UPDATED
esp_err_t HDC3020_enable_irq(void)
{
	uint8_t tmp = HDC3020_DRDY_ENABLE;
	esp_err_t ret = HDC3020_write( I2C_NUM_0, HDC3020_MEAS_CONFIG, &tmp, 1 );
	return ret;
}

esp_err_t HDC3020_start(void)
{

	uint8_t tmp = HDC3020_configuration;
	// set the start conversion bit
	tmp = tmp | HDC3020_MEAS_TRIGGER;
	esp_err_t ret = HDC3020_write( I2C_NUM_0, HDC3020_MEAS_CONFIG, &tmp, 1 );
	return ret;
}

esp_err_t HDC3020_read_humid( float *humidity )
{
	uint8_t data[4] = {0, 0};

	uint16_t tmp;

	esp_err_t ret = HDC3020_read( I2C_NUM_0, HDC3020_HUMID_LOW, data, 2);

    if(ret != ESP_OK ) return ret;  // something is not correct

    // Humidity bytes
    tmp = ( data[0] << 8 ) + data[1];

    *humidity = (tmp / HDC3020_DIVISOR) * HDC3020_HUMID_SLOPE;

    return ret;

}

esp_err_t HDC3020_read_temp( float *temperature )
{
	uint8_t data[2] = {0, 0};

	uint16_t tmp;

	esp_err_t ret = HDC3020_read( I2C_NUM_0, HDC3020_TEMP_LOW, data, 2);

    if(ret != ESP_OK ) return ret;  // something is not correct

    // Temperture bytes
    tmp = ( data[0] << 8 ) + data[1];

    //*temperature = ( ( tmp * HDC3020_TEMP_SLOPE ) / HDC3020_DIVISOR) - HDC3020_TEMP_INTERCEPT;
    *temperature = ( (tmp / HDC3020_DIVISOR) * HDC3020_TEMP_SLOPE ) - HDC3020_TEMP_INTERCEPT;

    return ret;

}

esp_err_t HDC3020_read_deviceID(uint8_t *deviceID)
{
	esp_err_t ret = HDC3020_read( I2C_NUM_0, HDC3020_DEVICE_ID_L, deviceID, 2);

	return ret;
}

esp_err_t HDC3020_read_ManufID(uint16_t *manufID)
{
	esp_err_t ret = HDC3020_read( I2C_NUM_0, HDC3020_MANUF_ID_L, manufID, 2);

	return ret;
}
//HDC3020_ID1_REG
esp_err_t HDC3020_read_config(uint8_t *config)
{

	// read interrupt and measurement configuration registers
	esp_err_t ret = HDC3020_read( I2C_NUM_0, HDC3020_IRQ_CONFIG, config, 2);

	return ret;
}


#endif

esp_err_t HDC3020_read_both( float *temperature, float *humidity )
{
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};

	uint16_t tmp;
	esp_err_t ret;

	uint8_t config_data[2] = { HDC3020_CMD_MEAS_TH_MSB, HDC3020_CMD_MEAS_TH_LSB };

	// read data
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = HDC3020_read( I2C_NUM_0, config_data, data, 6 );
		xSemaphoreGive( xSemaphore_I2C );
		if( ret != ESP_OK )return ret;
		// Temperture bytes
		tmp = ( data[0] << 8 ) + data[1];

		//*temperature = ( ( tmp * HDC3020_TEMP_SLOPE ) / HDC3020_DIVISOR) - HDC3020_TEMP_INTERCEPT;
		*temperature = ( (tmp / HDC3020_DIVISOR) * HDC3020_TEMP_SLOPE ) + HDC3020_TEMP_INTERCEPT;

		// humidity bytes
		tmp = ( data[3] << 8 ) + data[4];
		*humidity = (tmp / HDC3020_DIVISOR) * HDC3020_HUMID_SLOPE;
	}
	else ret = ESP_ERR_TIMEOUT;

	return ret;
}

esp_err_t print_temp_hum(void)
{
	float temp = 0.0;
	float humidity = 0.0;
	esp_err_t ret;


	vTaskDelay(3000 / portTICK_PERIOD_MS);

	// read data
	if( (ret = HDC3020_read_both( &temp, &humidity ) ) == ESP_OK )
	{
		printf("Temperature,%3.1f,Humidity,%3.1f\n", temp, humidity);
	}
	else printf("HDC3020 Read Failed\n");

	return ret;
}

esp_err_t HDC3020_read_ManufID(uint16_t *manufID)
{
	uint8_t data[2];
	esp_err_t ret;
	uint8_t command[2] = { HDC3020_CMD_MANUF_ID_MSB, HDC3020_CMD_MANUF_ID_LSB };

	// read ID
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		if( (ret = HDC3020_read( I2C_NUM_0, command, data, 6 ) ) == ESP_OK )
		{
			*manufID = ( data[0] << 8 ) + data[1];
		}
		// release bus
		xSemaphoreGive( xSemaphore_I2C );

	}
	else ret = ESP_ERR_TIMEOUT;

	return ret;
}
