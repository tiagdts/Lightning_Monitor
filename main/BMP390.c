/*
 * BMP390.c
 *
 *  Created on: Sep 10, 2022
 *      Author: tiagd
 */


#include "BMP390.h"
#include "esp_err.h"

struct BMP390_reg_calib_data cal_data;

static i2c_master_dev_handle_t BMP390_handle;

// semaphore used by BMP390 conversion complete routine
SemaphoreHandle_t xSemaphore_BMP390;

extern SemaphoreHandle_t xSemaphore_I2C;

struct bmp3_dev BMP390_device;

static int8_t compensate_temperature(double *temperature,
                                     const struct bmp3_uncomp_data *uncomp_data,
                                     struct bmp3_calib_data *calib_data);
static int8_t compensate_pressure(double *pressure,
                                  const struct bmp3_uncomp_data *uncomp_data,
                                  const struct bmp3_calib_data *calib_data);
static void parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev);
static float pow_bmp3(double base, uint8_t power);

static float BMP_pressure = -100.0;
static float BMP_temperature = -100.0;

#ifdef USE_OLD_I2C
/**
 *
 * _______________________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | stop | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|------|----------------------|-------------|------|------|
 *
 */
esp_err_t BMP390_read(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( BMP390 << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( BMP390 << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}



/** ____________________________________________________________________________________
* | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
* --------|---------------------------|----------------|----------------------|------|
*
*/
esp_err_t BMP390_write(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( BMP390 << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

#endif

void getBMPpressure( float *pressure )
{
	if( BMP_pressure == -100.0 ) *pressure = 0.0;
	else *pressure = BMP_pressure;
}

esp_err_t BMP390_read_deviceID(uint8_t *deviceID)
{
	uint8_t cmd = BMP390_DEVICE_ID;
	
	esp_err_t ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), deviceID, 1, -1);
	//esp_err_t ret = BMP390_read( I2C_NUM_0, BMP390_DEVICE_ID, deviceID, 1);

	return ret;
}

esp_err_t BMP390_read_revID(uint8_t *revID)
{
	uint8_t cmd = BMP390_REV_ID;
	
	esp_err_t ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), revID, 1, -1);
	//esp_err_t ret = BMP390_read( I2C_NUM_0, BMP390_REV_ID, revID, 1);

	return ret;
}

esp_err_t BMP390_read_pressure(uint8_t *data)
{
	uint8_t cmd = BMP390_PRESSURE;
	esp_err_t ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 3, -1);
	//esp_err_t ret = BMP390_read( I2C_NUM_0, BMP390_PRESSURE, data, 3);
	return ret;
}

esp_err_t BMP390_start_forced_mode(uint8_t *data)
{
	esp_err_t ret;
	uint8_t tmp_data = 0;
	uint8_t cmd = BMP390_REG_PWR_CTRL;

	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		/* Set power control to enable pressure and temperature in forced mode - PWR_CNTRL reg 0x1b */
		tmp_data = BMP390_MODE_FORCED | BMP390_PRES_ENA | BMP390_TEMP_ENA;
		ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), &tmp_data, 1, -1);
		//ret  = BMP390_write(I2C_NUM_0, BMP390_REG_PWR_CTRL, &tmp_data, 1);
	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}


esp_err_t BMP390_read_pressure_temperature_data(uint8_t *data)
{
	esp_err_t ret;
	uint8_t cmd = BMP390_PRESSURE;
	
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 6, -1);
		//ret = BMP390_read( I2C_NUM_0, BMP390_PRESSURE, data, 6);
		xSemaphoreGive( xSemaphore_I2C );
	}
	else ret = ESP_ERR_TIMEOUT;
	return ret;
}


// check status, read data if ready
esp_err_t BMP390_get_pressure_temperature(uint8_t *data, bool *data_good )
{
	esp_err_t ret;
	uint8_t cmd = BMP390_REG_INT_STATUS;

	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		if( (ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 1, -1) ) == ESP_OK )
		//if( (ret = BMP390_read( I2C_NUM_0, BMP390_REG_INT_STATUS, data, 1 ) ) == ESP_OK )
		{
			if( *data & BMP390_DRDY )
			{
				cmd = BMP390_PRESSURE;
				// read data
				if( (ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 6, -1) ) == ESP_OK )
				//if( (ret = BMP390_read( I2C_NUM_0, BMP390_PRESSURE, data, 6) ) == ESP_OK )
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



esp_err_t BMP390_read_cal_data(struct BMP390_reg_calib_data *data)
{
	uint8_t cmd = BMP390_REG_CALIB_DATA;
	esp_err_t ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), (uint8_t *)(data), 20, -1);
	//esp_err_t ret = BMP390_read( I2C_NUM_0, BMP390_REG_CALIB_DATA, (uint8_t *)(data), 20 );
	return ret;
}

esp_err_t BMP390_configure( uint8_t *cal_data )
{
	//uint8_t tmp_data = 0;
	uint8_t cmd[2] = { BMP390_REG_CALIB_DATA, 0 };
	
	/* Read out coefficient data */
	esp_err_t ret = i2c_master_transmit_receive( BMP390_handle, cmd,1, cal_data, 21, -1);
	//esp_err_t ret = BMP390_read( I2C_NUM_0, BMP390_REG_CALIB_DATA, cal_data, 21);
	if(ret != ESP_OK ) return ret;
	
	/* Set Interrupt register to "data ready" - INT_CNTRL reg 0x19  */	
	
	//tmp_data = BMP390_INT_PIN_ACTIVE_HIGH | BMP390_INT_ON_DRDY | BMP390_INT_PIN_PUSH_PULL | BMP390_INT_PIN_NON_LATCH;
	cmd[0] = BMP390_REG_INT_CTRL;
//	cmd[1] = BMP390_INT_PIN_ACTIVE_LOW |  BMP390_INT_PIN_PUSH_PULL | BMP390_INT_PIN_NON_LATCH;
//	cmd[1] = BMP390_INT_PIN_ACTIVE_LOW | BMP390_INT_ON_DRDY | BMP390_INT_PIN_PUSH_PULL | BMP390_INT_PIN_LATCH;
	cmd[1] = BMP390_INT_PIN_ACTIVE_LOW | BMP390_INT_ON_DRDY | BMP390_INT_PIN_PUSH_PULL | BMP390_INT_PIN_NON_LATCH;
//	cmd[1] = BMP390_INT_PIN_ACTIVE_HIGH | BMP390_INT_PIN_PUSH_PULL | BMP390_INT_PIN_NON_LATCH;
	ret = i2c_master_transmit( BMP390_handle, cmd, sizeof(cmd), -1);
	//ret  = BMP390_write(I2C_NUM_0, BMP390_REG_INT_CTRL, &tmp_data, 1);
	if(ret != ESP_OK ) return ret;

	/* Set 2x over-sampling for temperature and x32 for pressure - OSR reg 0x1c */
	cmd[0] = BMP390_REG_OSR;
	cmd[1] = BMP390_PRES_OVERSAMPLING_2X | BMP390_NO_TEMP_OVERSAMPLING;
	ret = i2c_master_transmit( BMP390_handle, cmd, sizeof(cmd), -1);
	//ret  = BMP390_write(I2C_NUM_0, BMP390_REG_OSR, &tmp_data, 1);
	if(ret != ESP_OK ) return ret;
	
	/* Set output data rate to 1/10 Hz -  ODR reg 0x1d */
	cmd[0] = BMP390_REG_ODR;
	cmd[1] = BMP390_ODR_0_1_HZ;
	ret = i2c_master_transmit( BMP390_handle, cmd, sizeof(cmd), -1);
	//ret  = BMP390_write(I2C_NUM_0, BMP390_REG_ODR, &tmp_data, 1);
	if(ret != ESP_OK ) return ret;
	
	/* Set power control to enable pressure and temperature in normal mode - PWR_CNTRL reg 0x1b */
	cmd[0] = BMP390_REG_PWR_CTRL;
	cmd[1] = BMP390_MODE_NORMAL | BMP390_PRES_ENA | BMP390_TEMP_ENA;
	ret = i2c_master_transmit( BMP390_handle, cmd, sizeof(cmd), -1);
	//ret  = BMP390_write(I2C_NUM_0, BMP390_REG_PWR_CTRL, &tmp_data, 1);


	return ret;
}


esp_err_t BMP390_initialize( void )
{
	uint8_t cal_data[21];

	// zero BMP390_device structure
	memset(&BMP390_device,0,sizeof(BMP390_device));

	// configure registers and get calibration data from device
	esp_err_t ret = BMP390_configure( cal_data );

	if( ret == ESP_OK )
	{
		// store the calibration in the sturcture
		parse_calib_data(cal_data, &BMP390_device);
		printf("cal data parsed\n");
	}


	return ret;
}


// interrupt service routine, called when an interrupt is detected
//
void IRAM_ATTR pressure_isr_handler(void* arg)
{

    // notify the "pcnt_task" to zero counter
	xSemaphoreGiveFromISR(xSemaphore_BMP390, NULL);
}

/*!
 *  @brief This internal API is used to parse the pressure or temperature or
 *  both the data and store it in the bmp3_uncomp_data structure instance.
 */
static void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data)
{
    /* Temporary variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_xlsb = (uint32_t)reg_data[0];
    data_lsb = (uint32_t)reg_data[1] << 8;
    data_msb = (uint32_t)reg_data[2] << 16;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_xlsb = (uint32_t)reg_data[3];
    data_lsb = (uint32_t)reg_data[4] << 8;
    data_msb = (uint32_t)reg_data[5] << 16;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
}

static int8_t compensate_data(uint8_t sensor_comp,
                              const struct bmp3_uncomp_data *uncomp_data,
                              struct bmp3_data *comp_data,
                              struct bmp3_calib_data *calib_data)
{
    int8_t rslt = BMP3_OK;

    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL))
    {
        /* If pressure and temperature component is selected */
        if (sensor_comp == BMP3_PRESS_TEMP)
        {
            /*
             * NOTE : Temperature compensation must be done first.
             * Followed by pressure compensation
             * Compensated temperature updated in calib structure,
             * is needed for pressure calculation
             */

            /* Compensate pressure and temperature data */
            rslt = compensate_temperature(&comp_data->temperature, uncomp_data, calib_data);

            if (rslt == BMP3_OK)
            {
                rslt = compensate_pressure(&comp_data->pressure, uncomp_data, calib_data);
            }
        }
        else if (sensor_comp == BMP3_PRESS)
        {
            /*
             * NOTE : Temperature compensation must be done first.
             * Followed by pressure compensation
             * Compensated temperature updated in calib structure,
             * is needed for pressure calculation.
             * As only pressure is enabled in 'sensor_comp', after calculating
             * compensated temperature, assign it to zero.
             */
            (void)compensate_temperature(&comp_data->temperature, uncomp_data, calib_data);
            comp_data->temperature = 0;

            /* Compensate the pressure data */
            rslt = compensate_pressure(&comp_data->pressure, uncomp_data, calib_data);
        }
        else if (sensor_comp == BMP3_TEMP)
        {
            /* Compensate the temperature data */
            rslt = compensate_temperature(&comp_data->temperature, uncomp_data, calib_data);

            /*
             * As only temperature is enabled in 'sensor_comp'
             * make compensated pressure as zero
             */
            comp_data->pressure = 0;
        }
        else
        {
            comp_data->pressure = 0;
            comp_data->temperature = 0;
        }
    }
    else
    {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}


/*!
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure
 */
static void parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev)
{
    /* Temporary variable to store the aligned trim data */
    struct bmp3_reg_calib_data *reg_calib_data = &dev->calib_data.reg_calib_data;
    struct bmp3_quantized_calib_data *quantized_calib_data = &dev->calib_data.quantized_calib_data;

    /* Temporary variable */
    double temp_var;

    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    quantized_calib_data->par_t1 = ((double)reg_calib_data->par_t1 / temp_var);
    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
    temp_var = 1073741824.0f;
    quantized_calib_data->par_t2 = ((double)reg_calib_data->par_t2 / temp_var);
    reg_calib_data->par_t3 = (int8_t)reg_data[4];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_t3 = ((double)reg_calib_data->par_t3 / temp_var);
    reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
    temp_var = 1048576.0f;
    quantized_calib_data->par_p1 = ((double)(reg_calib_data->par_p1 - (16384)) / temp_var);
    reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
    temp_var = 536870912.0f;
    quantized_calib_data->par_p2 = ((double)(reg_calib_data->par_p2 - (16384)) / temp_var);
    reg_calib_data->par_p3 = (int8_t)reg_data[9];
    temp_var = 4294967296.0f;
    quantized_calib_data->par_p3 = ((double)reg_calib_data->par_p3 / temp_var);
    reg_calib_data->par_p4 = (int8_t)reg_data[10];
    temp_var = 137438953472.0f;
    quantized_calib_data->par_p4 = ((double)reg_calib_data->par_p4 / temp_var);
    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);

    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantized_calib_data->par_p5 = ((double)reg_calib_data->par_p5 / temp_var);
    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14], reg_data[13]);
    temp_var = 64.0f;
    quantized_calib_data->par_p6 = ((double)reg_calib_data->par_p6 / temp_var);
    reg_calib_data->par_p7 = (int8_t)reg_data[15];
    temp_var = 256.0f;
    quantized_calib_data->par_p7 = ((double)reg_calib_data->par_p7 / temp_var);
    reg_calib_data->par_p8 = (int8_t)reg_data[16];
    temp_var = 32768.0f;
    quantized_calib_data->par_p8 = ((double)reg_calib_data->par_p8 / temp_var);
    reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p9 = ((double)reg_calib_data->par_p9 / temp_var);
    reg_calib_data->par_p10 = (int8_t)reg_data[19];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p10 = ((double)reg_calib_data->par_p10 / temp_var);
    reg_calib_data->par_p11 = (int8_t)reg_data[20];
    temp_var = 36893488147419103232.0f;
    quantized_calib_data->par_p11 = ((double)reg_calib_data->par_p11 / temp_var);
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 * Returns temperature (deg Celsius) in double.
 * For e.g. Returns temperature 24.26 deg Celsius
 */
static int8_t compensate_temperature(double *temperature,
                                     const struct bmp3_uncomp_data *uncomp_data,
                                     struct bmp3_calib_data *calib_data)
{
    int8_t rslt = BMP3_OK;
    int64_t uncomp_temp = uncomp_data->temperature;
    double partial_data1;
    double partial_data2;

    partial_data1 = (double)(uncomp_temp - calib_data->quantized_calib_data.par_t1);
    partial_data2 = (double)(partial_data1 * calib_data->quantized_calib_data.par_t2);

    /* Update the compensated temperature in calib structure since this is
     * needed for pressure calculation */
    calib_data->quantized_calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) *
                                             calib_data->quantized_calib_data.par_t3;

    /* Returns compensated temperature */
    if (calib_data->quantized_calib_data.t_lin < BMP3_MIN_TEMP_DOUBLE)
    {
        calib_data->quantized_calib_data.t_lin = BMP3_MIN_TEMP_DOUBLE;
        rslt = BMP3_W_MIN_TEMP;
    }

    if (calib_data->quantized_calib_data.t_lin > BMP3_MAX_TEMP_DOUBLE)
    {
        calib_data->quantized_calib_data.t_lin = BMP3_MAX_TEMP_DOUBLE;
        rslt = BMP3_W_MAX_TEMP;
    }

    (*temperature) = calib_data->quantized_calib_data.t_lin;

    return rslt;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 * For e.g. returns pressure in Pascal p = 95305.295
 */
static int8_t compensate_pressure(double *pressure,
                                  const struct bmp3_uncomp_data *uncomp_data,
                                  const struct bmp3_calib_data *calib_data)
{
    int8_t rslt = BMP3_OK;
    const struct bmp3_quantized_calib_data *quantized_calib_data = &calib_data->quantized_calib_data;

    /* Variable to store the compensated pressure */
    double comp_press;

    /* Temporary variables used for compensation */
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    partial_data1 = quantized_calib_data->par_p6 * quantized_calib_data->t_lin;
    partial_data2 = quantized_calib_data->par_p7 * pow_bmp3(quantized_calib_data->t_lin, 2);
    partial_data3 = quantized_calib_data->par_p8 * pow_bmp3(quantized_calib_data->t_lin, 3);
    partial_out1 = quantized_calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = quantized_calib_data->par_p2 * quantized_calib_data->t_lin;
    partial_data2 = quantized_calib_data->par_p3 * pow_bmp3(quantized_calib_data->t_lin, 2);
    partial_data3 = quantized_calib_data->par_p4 * pow_bmp3(quantized_calib_data->t_lin, 3);
    partial_out2 = uncomp_data->pressure *
                   (quantized_calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = pow_bmp3((double)uncomp_data->pressure, 2);
    partial_data2 = quantized_calib_data->par_p9 + quantized_calib_data->par_p10 * quantized_calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + pow_bmp3((double)uncomp_data->pressure, 3) * quantized_calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;

    if (comp_press < BMP3_MIN_PRES_DOUBLE)
    {
        comp_press = BMP3_MIN_PRES_DOUBLE;
        rslt = BMP3_W_MIN_PRES;
    }

    if (comp_press > BMP3_MAX_PRES_DOUBLE)
    {
        comp_press = BMP3_MAX_PRES_DOUBLE;
        rslt = BMP3_W_MAX_PRES;
    }

    (*pressure) = comp_press;

    return rslt;
}

/*!
 * @brief This internal API is used to calculate the power functionality for
 *  floating point values.
 */
static float pow_bmp3(double base, uint8_t power)
{
    float pow_output = 1;

    while (power != 0)
    {
        pow_output = (float) base * pow_output;
        power--;
    }

    return pow_output;
}

void BMP390_createSemaphores(void)
{
	// create binary semaphores for Interrupt Service routine
	xSemaphore_BMP390 = xSemaphoreCreateBinary();
}

void IRAM_ATTR BMP390_isr_handler(void* arg)
{

    // notify the "pcnt_task" to zero counter
	xSemaphoreGiveFromISR(xSemaphore_BMP390, NULL);
}

void BMP390_Task(void *pvParameter)
{
	uint8_t data[6] = {0,0,0,0,0,0};
	struct bmp3_uncomp_data uncomp_data;
	struct bmp3_data comp_data;
	
	printf("Starting BMP390_Task\n");
	while(1)
	{
			// wait for the notification from INT_SNS pin
			if( xSemaphoreTake(xSemaphore_BMP390, portMAX_DELAY ) == pdTRUE)
			{
				if( ( BMP390_read_pressure_temperature_data(data) ) == ESP_OK )
				{
					//printf("Data Read\n");
					parse_sensor_data(data,  &uncomp_data);
					//printf( "Pressure: %llu, Temperature: %llu\n", uncomp_data.pressure, uncomp_data.temperature );
					compensate_data(BMP3_PRESS_TEMP, &uncomp_data, &comp_data, &BMP390_device.calib_data);
					comp_data.temperature = comp_data.temperature * 1.8 + 32;
					BMP_pressure = comp_data.pressure * pascalToInchMg;
					BMP_temperature = comp_data.temperature;
					printf( "\nTemperature,%2.1lf,Pressure,%4.2lf\n", BMP_temperature, BMP_pressure );
					 //print_temp_hum();
				}
				else printf("Data Read Failed\n");
			}
	}
}


esp_err_t BMP390_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle, uint16_t address )
{
	esp_err_t ret;
	
	i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = address,
    .scl_speed_hz = BMP390_CLOCK,
	};

	ESP_ERROR_CHECK(ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &BMP390_handle));
	return ret;
}

esp_err_t BMP390_get_data_no_irq(uint16_t attempts, float *temperature, float *pressure, float *tempC )
{
	uint8_t data[6] = {0,0,0,0,0,0};
	struct bmp3_uncomp_data uncomp_data;
	struct bmp3_data comp_data;
	bool data_good;
	*temperature = -100;
	*pressure = -100;
	esp_err_t ret = -1;
	
	while(attempts)
	{

			if( ( ret = BMP390_get_raw_pressure_temperature(data, &data_good) ) == ESP_OK )
			{
				if(data_good)
				{
					//printf("Data Read\n");
					parse_sensor_data(data,  &uncomp_data);
					//printf( "Pressure: %llu, Temperature: %llu\n", uncomp_data.pressure, uncomp_data.temperature );
					compensate_data(BMP3_PRESS_TEMP, &uncomp_data, &comp_data, &BMP390_device.calib_data);
					*tempC = comp_data.temperature;
					comp_data.temperature = comp_data.temperature * 1.8 + 32;
					*pressure = comp_data.pressure * pascalToInchMg;
					*temperature = comp_data.temperature;
					printf( "\nTemperature,%2.1lf,Pressure,%4.2lf\n", *temperature, *pressure );
					attempts = 0;
					break;
				}
				else printf("Data Not Ready\n");
			}
			else printf("Data Read Failed\n");
			attempts--;
			vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	
	return ret;
}

// check status, read data if ready
esp_err_t BMP390_get_raw_pressure_temperature(uint8_t *data, bool *data_good )
{
	esp_err_t ret;
	uint8_t cmd = BMP390_REG_INT_STATUS; 

	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		if( (ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 1, -1)) == ESP_OK )
		//if( (ret = BMP390_read( I2C_NUM_0, BMP390_REG_INT_STATUS, data, 1 ) ) == ESP_OK )
		{
			if( *data & BMP390_DRDY)
			{
				// read data
				cmd = BMP390_PRESSURE;
				if( (ret = i2c_master_transmit_receive( BMP390_handle, &cmd, sizeof(cmd), data, 6, -1)) == ESP_OK )
				//if( (ret = BMP390_read( I2C_NUM_0, BMP390_PRESSURE, data, 6) ) == ESP_OK )
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
