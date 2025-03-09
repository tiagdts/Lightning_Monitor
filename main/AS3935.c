/*
 * AS3935.c
 *
 *  Created on: Feb 11, 2025
 *      Author: tiagd
 */

#include "AS3935.h"
#include "esp_err.h"
#include <stdint.h>


extern SemaphoreHandle_t xSemaphore_I2C;
extern SemaphoreHandle_t xSemaphore_SPI;

SemaphoreHandle_t xSemaphore_AS3935;

static spi_device_handle_t spi;

esp_err_t get_lightning( uint32_t *energy, uint8_t *distance )
{
	esp_err_t ret = ESP_OK;
	
	if( ( ret = get_AS3935_reg32( REG_X04, energy ) ) != ESP_OK )
	{
		*energy = 0;
	}
	
	if( ( ret = get_AS3935_reg( REG_X07, distance ) ) != ESP_OK )
	{
		*distance = 0;
	}
	else *distance = *distance & DISTANCE;

	return ret;
}

esp_err_t get_lightning_data( uint8_t *status, uint32_t *energy, uint8_t *distance )
{
	uint8_t IRQ;
	esp_err_t ret;

	printf("AS3935 Interrupt: ");
	// delay 2ms before reading register (AS3935 datasheet ([v1-04] 2016-Jan-13) page 34 )
	vTaskDelay(2 / portTICK_PERIOD_MS);
	if( (ret = get_AS3935_reg( REG_X03, &IRQ ) ) == ESP_OK ) 
	{
		*status = IRQ;
		if( IRQ & INT_BITS )
		{
			if( IRQ & INT_NH  )
			{
				printf("\n  Noise level to high.\n");
			}
			
			if( IRQ & INT_D )
			{
				printf("\n  Disturber detected\n");
			}
			
			if( IRQ & INT_L )
			{
				printf("\n  Lightning detected\n");
				*energy = 0;
				*distance = 0;
				get_lightning(energy, distance );
				printf( "Energy: %lu, Distance: %u\n", *energy, *distance );
			}
			
		} else printf("none\n");		
	} else printf("Interrupt Register Read Failed\n");
	
	return ret;
}

void AS3935_createSemaphores(void)
{
	// create binary semaphores for Interrupt Service routine
	xSemaphore_AS3935 = xSemaphoreCreateBinary();
}

void IRAM_ATTR AS3935_isr_handler(void* arg)
{

    // notify the "AS3935_task"
	xSemaphoreGiveFromISR(xSemaphore_AS3935, NULL);
}

void AS3935_Task(void *pvParameter)
{
	uint8_t IRQ;
	
	uint32_t energy = 0;
	uint8_t distance = 0;
	
	printf("Starting AS3935_Task\n");
	printf("Clear AS3935 Interrupt...");
	if( get_AS3935_reg( REG_X03, &IRQ ) == ESP_OK )
	{
		printf(" interrupt cleared\n");
	} else printf(" failed to read IRQ register\n");
	
#define LIGHTNING_READ_TEST
#ifdef LIGHTNING_READ_TEST	
	get_lightning(&energy, &distance );
	printf( "Energy: %lu, Distance: %u\n", energy, distance );
#endif
	while(1)
	{
			// wait for the notification from INT_SNS pin
			if( xSemaphoreTake(xSemaphore_AS3935, portMAX_DELAY ) == pdTRUE)
			{
				printf("AS3935 Interrupt: ");
				// delay 2ms before reading register (AS3935 datasheet ([v1-04] 2016-Jan-13) page 34 )
				vTaskDelay(2 / portTICK_PERIOD_MS);
				if( get_AS3935_reg( REG_X03, &IRQ ) == ESP_OK )
				{
					if( IRQ & INT_BITS )
					{
						if( IRQ & INT_NH  )
						{
							printf("\n  Noise level to high.\n");
						}
						
						if( IRQ & INT_D )
						{
							printf("\n  Disturber detected\n");
						}
						
						if( IRQ & INT_L )
						{
							printf("\n  Lightning detected\n");
							energy = 0;
							distance = 0;
							get_lightning(&energy, &distance );
							printf( "Energy: %lu, Distance: %u\n", energy, distance );
							
						}
						
					} else printf("none\n");		
				} else printf("Interrupt Register Read Failed\n");
			}

	}
}



esp_err_t tune_TCO_AS3935( uint8_t tuning )
{
	esp_err_t ret;
	spi_transaction_t t;
	uint8_t data[2];
	
	memset(&t, 0, sizeof(t));
	t.length = 8 * 2;
	t.tx_buffer = data;
	t.flags = SPI_TRANS_USE_RXDATA;
	
	// write register
	data[0] = REG_X08;
	data[1] = tuning;
	printf( "register pre-write: %x, value: %x\n", data[0], data[1] );
	if( xSemaphoreTake( xSemaphore_SPI, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = spi_device_polling_transmit(spi, &t); //Transmit!
    	assert(ret == ESP_OK);          //Should have had no issues.
		xSemaphoreGive( xSemaphore_SPI );
		if( ret != ESP_OK )return ret;
	}
	else ret = ESP_ERR_TIMEOUT;
	
	return ret;
}

esp_err_t calibrate_AS3935( void )
{
	esp_err_t ret = 0;
	
	// restore default values
	if( direct_command_AS3935(PRESET_DEFAULT ) == ESP_OK )
	{
		printf("AS3935 presets set\n");
	}
	else printf("AS3935 preset setting failed\n");
	
	// set tuning calibration value
	if( tune_TCO_AS3935( LCO_CALIBRATION ) == ESP_OK )
	{
		printf("Set tuning to %x\n", LCO_CALIBRATION );
	}
	else printf("Failed to tune AS3935\n");
	
	// put device in calibration mode
	if( direct_command_AS3935( CALIB_RCO ) == ESP_OK )
	{
		printf("AS3935 in calibration mode\n");
	}else printf("AS3935 not in calibration mode\n");
	
	// set Display SRCO on IRQ pin, keeping LCO calibration
	uint8_t data = DISP_SRCO | LCO_CALIBRATION;
	if( set_AS3935_reg( REG_X08, data ) == ESP_OK )
	{
		printf("Displaying SRCO on interrupt pin\n");
	} else printf("Not displaying SRCO on interrupt pin\n");
	
	// delay for 2 ms
	vTaskDelay(2 / portTICK_PERIOD_MS);
	
	printf("SRCO calibration ");
	if( get_AS3935_reg( REG_X3A, &data ) == ESP_OK )
	{
		if( data & SRCO_CALIB_DONE )
		{
			printf("successful\n");
			ret = ESP_OK;
		}
		else if( data & SRCO_CALIB_NOK )
		{
			printf("unsuccessful\n");
			ret = ESP_FAIL;
		}
	} else printf("not performed\n");
	
	// disable SRCO display on interrupt pin retaining LCD calibration
	data = LCO_CALIBRATION;
	if( set_AS3935_reg( REG_X08, data ) == ESP_OK )
	{
		printf("Disabling SRCO output on interrupt pin\n");
	} else printf("Disabling SRCO output on interrupt pin failed\n");

	return ret;
}

esp_err_t direct_command_AS3935( uint8_t reg )
{

	esp_err_t ret;

	uint8_t data[3] = {0,DIRECT_COMMAND, 0};
	spi_transaction_t t;
	
	memset(&t, 0, sizeof(t));
	t.length = 8 * 2;
	t.tx_buffer = data;
    //t.flags = SPI_TRANS_USE_RXDATA;
	
	data[0] =  reg;

		// read REG_X08 register
	if( xSemaphoreTake( xSemaphore_SPI, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		printf( "Direct Command: %x, value: %x\n", data[0], data[1] );
		ESP_ERROR_CHECK( ret = spi_device_polling_transmit(spi, &t) );
    	assert(ret == ESP_OK);
		xSemaphoreGive( xSemaphore_SPI );
	}
	else ret = ESP_ERR_TIMEOUT;
	
	return ret;
}

esp_err_t get_AS3935_reg32( uint8_t reg, uint32_t *value )
{

	esp_err_t ret;
	//uint8_t cmd = REG_X08;
	uint8_t data[4] = {0,0, 0, 0};
	spi_transaction_t t;
	
	memset(&t, 0, sizeof(t));
	t.length = 8 * 4;
	t.tx_buffer = data;
    t.flags = SPI_TRANS_USE_RXDATA;
	
	data[0] = SPI_READ | reg;


		// read REG_X08 register
	if( xSemaphoreTake( xSemaphore_SPI, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		//printf("SPI read\n");
		//printf( "Pre-read register: %x, value: %x\n", data[0], data[1] );
		ESP_ERROR_CHECK( ret = spi_device_polling_transmit(spi, &t) );
    	assert(ret == ESP_OK);
		xSemaphoreGive( xSemaphore_SPI );
		if( ret == ESP_OK )
		{
			//printf( "Post-read register: %x, value: %x\n", data[0], t.rx_data[1] );
			*value = ( ( t.rx_data[3] & S_LIG_MM ) * 0x10000 )+ ( t.rx_data[2] * 0x100 ) + t.rx_data[1];
		}
		else printf( "register read failed\n" );
	}
	else ret = ESP_ERR_TIMEOUT;
	
	return ret;
}

esp_err_t get_AS3935_reg( uint8_t reg, uint8_t *value )
{

	esp_err_t ret;
	//uint8_t cmd = REG_X08;
	uint8_t data[2] = {0,0};
	spi_transaction_t t;
	
	memset(&t, 0, sizeof(t));
	t.length = 8 * 2;
	t.tx_buffer = data;
    t.flags = SPI_TRANS_USE_RXDATA;
	
	data[0] = SPI_READ | reg;


		// read REG_X08 register
	if( xSemaphoreTake( xSemaphore_SPI, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		//printf("SPI read\n");
		//printf( "Pre-read register: %x, value: %x\n", data[0], data[1] );
		ESP_ERROR_CHECK( ret = spi_device_polling_transmit(spi, &t) );
    	assert(ret == ESP_OK);
		xSemaphoreGive( xSemaphore_SPI );
		if( ret == ESP_OK )
		{
			//printf( "Post-read register: %x, value: %x\n", data[0], t.rx_data[1] );
			*value = t.rx_data[1];
		}
		else printf( "register read failed\n" );
	}
	else ret = ESP_ERR_TIMEOUT;
	
	return ret;
}

esp_err_t set_AS3935_reg( uint8_t reg, uint8_t value )
{

	esp_err_t ret;
	uint8_t data[2];
	spi_transaction_t t;
	
	memset(&t, 0, sizeof(t));
	t.length = 8 * 2;
	t.tx_buffer = data;
	
	data[0] = reg;
	data[1] = value;

		// read REG_X08 register
	if( xSemaphoreTake( xSemaphore_SPI, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		//printf("SPI write\n");
		ESP_ERROR_CHECK( ret = spi_device_polling_transmit(spi, &t) );
    	assert(ret == ESP_OK);
		xSemaphoreGive( xSemaphore_SPI );
	}
	else ret = ESP_ERR_TIMEOUT;
	
	return ret;
}

esp_err_t set_AS3935_irq_pin_function( uint8_t pin_setting, uint8_t tuning )
{

	esp_err_t ret;
	uint8_t data[3] = {0,0, 0};
	spi_transaction_t t;
	
	memset(&t, 0, sizeof(t));
	t.length = 8 * 2;
	t.tx_buffer = data;
    t.flags = SPI_TRANS_USE_RXDATA;
	
	data[0] = SPI_READ | REG_X08;

//*
		// read REG_X08 register
	if( xSemaphoreTake( xSemaphore_SPI, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		//ESP_ERROR_CHECK( ret = i2c_master_transmit_receive( AS3935_handle, &cmd, sizeof(cmd), data, 1, -1) );
		printf("SPI read\n");
		printf( "Pre-read register: %x, value: %x\n", data[0], data[1] );
		ESP_ERROR_CHECK( ret = spi_device_polling_transmit(spi, &t) );
    	assert(ret == ESP_OK);
		xSemaphoreGive( xSemaphore_SPI );
		if( ret == ESP_OK ) printf( "Post-read register: %x, value: %x\n", data[0], t.rx_data[1] );
		else printf( "register read failed\n" );
	}
	else ret = ESP_ERR_TIMEOUT;
//*/		

	memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 2 * 8;             //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;
	
	// write register
	data[0] = REG_X08;
	data[1] = pin_setting | tuning;
	printf( "register pre-write: %x, value: %x\n", data[0], data[1] );
	if( xSemaphoreTake( xSemaphore_SPI, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		ret = spi_device_polling_transmit(spi, &t); //Transmit!
    	assert(ret == ESP_OK);          //Should have had no issues.
		xSemaphoreGive( xSemaphore_SPI );
		if( ret != ESP_OK )return ret;
	}
	else ret = ESP_ERR_TIMEOUT;
	
	return ret;
}


esp_err_t add_AS3935_to_SPI_bus( void )
{
	esp_err_t ret;
	spi_device_interface_config_t devcfg = {

        .clock_speed_hz = 10 * 1000 * 1000,     //Clock out at 10 MHz
        .mode = 1,								 //SPI mode 1
        .queue_size = 1,                            
        .spics_io_num = PIN_NUM_CS,             //CS pin
    };
        //Attach the LCD to the SPI bus
    printf("Add AS3935 to SPI bus\n");
    ESP_ERROR_CHECK( ret = spi_bus_add_device(IC_SPI_HOST, &devcfg, &spi) );
    
    return ret;
}


/*
esp_err_t AS3935_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle )
{
	esp_err_t ret;
	
	i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AS3935,
    .scl_speed_hz = AS3935_CLOCK,
	};

	ESP_ERROR_CHECK(ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &AS3935_handle));
	return ret;
}
*/

/*
	Mode	| 	Register Address 		|  	Direct Command Register Data
	B15 B14 |	B13 B12 B11 B10 B9 B8 	|	B7 B6 B5 B4 B3 B2 B1 B0
	
	B15 B14 		Mode
	0 	0 		WRITE / DIRECT COMMAND
	0 	1 		READ
	
*/