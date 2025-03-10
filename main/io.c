/*
 * IO.c
 *
 *  Created on: May 19, 2021
 *      Author: dschmidt
 */

#include "io.h"
#include "BMP390.h"

//static char* TAG = "IO";

static uint16_t expected_addresses[EXPECTED_I2C_ADDRESSES] = {MAX17048, SHT45, BMP390_0};


SemaphoreHandle_t xSemaphore_I2C;
SemaphoreHandle_t xSemaphore_SPI;


void wifi_connect_LED_off( void )
{
	//gpio_set_level(PIN_BLU_LED, 0);
}

void wifi_connect_LED_on( void )
{
	//gpio_set_level(PIN_BLU_LED, 1);
}

/*
bool get_connection_state( void )
{
	return wifi_connection_state;
}
*/

void io_createSemaphores(void)
{
	// create mutex semaphores to be used for SPI bus access
	// 	between Tasks
	xSemaphore_I2C = xSemaphoreCreateMutex();
	xSemaphore_SPI = xSemaphoreCreateMutex();

}


void init_GPIO( void )
{

#define IO_CONFIG
#ifdef IO_CONFIG
	// configure ADS1115 interrupt pin
	esp_rom_gpio_pad_select_gpio( BMP390_IRQ );
	// set the correct direction
	gpio_set_direction(BMP390_IRQ, GPIO_MODE_INPUT);
	// pull pin high
	//gpio_set_pull_mode(BMP390_IRQ, GPIO_PULLUP_ONLY);
	// enable interrupt on falling (1->0) edge
	gpio_set_intr_type(BMP390_IRQ, GPIO_INTR_NEGEDGE);
	
#endif

}



uint8_t scan_i2c( i2c_master_bus_handle_t *bus_handle)
{
	printf("scanning the bus...\r\n\r\n");
	int devices_found = 0;
	uint16_t address;


	for(uint16_t i = 0; i < EXPECTED_I2C_ADDRESSES; i++)
	{
		address = expected_addresses[i];
		if(i2c_master_probe( *bus_handle, address, I2C_SCAN_WAIT_TIME ) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
			devices_found++;
		}
	}
	return devices_found;
}

// initialize i2c ports
esp_err_t init_I2C(i2c_master_bus_handle_t *bus_handle)
{ 
	esp_err_t ret;
     i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = PIN_NUM_SCL,
        .sda_io_num = PIN_NUM_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    //i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(ret = i2c_new_master_bus(&i2c_bus_config, bus_handle));

	// create I2 and SPI mutex semaphores
	io_createSemaphores();
	return ret;
}

esp_err_t init_SPI( void )
{
	esp_err_t ret;
    printf( "Initializing bus SPI%d...", IC_SPI_HOST + 1);
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    
    //Initialize the SPI bus
    printf("Init SPI buss\n");
    ret = spi_bus_initialize(IC_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    
    return ret;
}


