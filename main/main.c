/*
 * SPDX-FileCopyrightText: 2017-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "Station_Data_Types.h"
#include "esp_attr.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
// #include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "io.h"
#include "BMP390.h"
#include "SHT45.h"
#include "HDC3020.h"
#include "AS3935.h"
#include "espnow_.h"
#include "adc_.h"

#include "driver/gpio.h"

uint16_t sensor_location = KITCHEN;
//uint16_t sensor_location = WEST_SIDE;	
//#if SOC_RTC_FAST_MEM_SUPPORTED
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR uint64_t t_system_time_last_update_us = 0;
static RTC_DATA_ATTR uint8_t sensor_initialized;
static RTC_DATA_ATTR uint8_t noise_status;
static RTC_DATA_ATTR uint8_t noise_count;

static uint8_t read_AS3935 = 0;
//#else
//static struct timeval sleep_enter_time;
//#endif

// semaphore used to signal data ready
SemaphoreHandle_t xSemaphore_DataReady = NULL;

// semaphore used to signal system time set
SemaphoreHandle_t xSemaphore_SystemTimeSet = NULL;

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
bool 
spi_available = false;

#define TIME_UPDATE_INTERVAL 		3600000000	// us (1 HOUR)
// #define TIME_UPDATE_INTERVAL 		300000000	// us (5 minutes) use for testing
#define WAIT_FOR_TIME_UPDATE		30000		// ms waiting for system time update

#define BMP390_INSTALLED
#define AS3935_INITIALIZED	0x01

#if CONFIG_EXAMPLE_GPIO_WAKEUP
#define DEFAULT_WAKEUP_PIN      CONFIG_EXAMPLE_GPIO_WAKEUP_PIN
#ifdef CONFIG_EXAMPLE_GPIO_WAKEUP_HIGH_LEVEL
#define DEFAULT_WAKEUP_LEVEL    ESP_GPIO_WAKEUP_GPIO_HIGH
#else
#define DEFAULT_WAKEUP_LEVEL    ESP_GPIO_WAKEUP_GPIO_LOW
#endif

void example_deep_sleep_register_gpio_wakeup(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = BIT(DEFAULT_WAKEUP_PIN),
        .mode = GPIO_MODE_INPUT,
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT(DEFAULT_WAKEUP_PIN), DEFAULT_WAKEUP_LEVEL));

    printf("Enabling GPIO wakeup on pins GPIO%d\n", DEFAULT_WAKEUP_PIN);
}
#endif

static void deep_sleep_task(void *args)
{
    /**
     * Prefer to use RTC mem instead of NVS to save the deep sleep enter time, unless the chip
     * does not support RTC mem(such as esp32c2). Because the time overhead of NVS will cause
     * the recorded deep sleep enter time to be not very accurate.
     */
    float BMP_temperature = -100, BMP_pressure = -100, BMP_tempC = -100;
	float temperature = -100, humidity = -100, SHT45_tempC = -100;
	lightningData_t data; 
	time_t timestamp;
	double batt_volts = -100, batt_soc = -100;
	uint8_t lightning_status = 0;
	uint8_t distance = 0;
	uint32_t energy = 0;
	esp_err_t ret;
	bool waitForSystemTimeUpdate = false;
     
     esp_err_t err;

    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Open NVS done\n");
    }

    // Get deep sleep enter time
    nvs_get_i32(nvs_handle, "slp_enter_sec", (int32_t *)&sleep_enter_time.tv_sec);
    nvs_get_i32(nvs_handle, "slp_enter_usec", (int32_t *)&sleep_enter_time.tv_usec);
    
    // get system time update
    nvs_get_u64(nvs_handle, "update_usec",(uint64_t *)&t_system_time_last_update_us );
    
    // Get AS3935 initialization status
    nvs_get_u8(nvs_handle, "sensor_status", (uint8_t *)&sensor_initialized);
    nvs_get_u8(nvs_handle, "noise_level", (uint8_t *)&noise_status);
    nvs_get_u8(nvs_handle, "noise_events", (uint8_t *)&noise_count);
//#endif

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;
    
    printf("Last update(us): %llu\n", t_system_time_last_update_us);
    // increment last system update elapse time
    t_system_time_last_update_us += (sleep_time_ms * 1000);
    
     printf("Last update + sleep time(us): %llu\n", t_system_time_last_update_us);
    
    // initiate a system time update if needed
    if( t_system_time_last_update_us > TIME_UPDATE_INTERVAL )
    {
		clrSystemTimeSet();
		waitForSystemTimeUpdate = true;
	}

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
			/* When noise_status is set, the AS3935 interrupt was disabled due to too much noise
				read_AS3935 is normally set from the GPIO interrupt below
				we need to read the AS3935 to see if the noise is still present
			*/
			if(noise_status) read_AS3935 = 1;
			
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }

#if CONFIG_EXAMPLE_GPIO_WAKEUP
        case ESP_SLEEP_WAKEUP_GPIO: {
            uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
            if (wakeup_pin_mask != 0)
            {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
                read_AS3935 = 1;    
            }
            else
            {
                printf("Wake up from GPIO\n");
            }
            break;
        }
#endif //CONFIG_EXAMPLE_GPIO_WAKEUP

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
 			sensor_initialized = 0;
			noise_status = NOISE_LEVEL_GOOD;
			read_AS3935 = 0;
    }

//////////////////// Lightning Detector ///////////////////////
	AS3935_createSemaphores();
	lightning_status = 0;
	distance = 0;
	energy = 0;
	
	/////////////// init ADC /////////////
	if( init_adc( ) == ESP_OK ) printf("adc initialized\n");
		else  printf("adc initialization failed\n"); 
	
	if( spi_available )
	{
		if( add_AS3935_to_SPI_bus( ) == ESP_OK )
		{
			printf("AS3935 added to SPI Bus\n");
			if( read_AS3935 )
			{
				// read lightning data
				printf("Reading Lightning Data\n");
				if( get_lightning_data( &lightning_status, &energy, &distance ) == ESP_OK )
				{
					if(lightning_status & INT_NH )
					{
						printf("Noise level too high\n");
						printf("  Disabling Lightning interrupt\n");
						noise_status = NOISE_LEVEL_HIGH;
					}
					else noise_status = NOISE_LEVEL_GOOD;
					printf( "Status: %x, Distance: %u, Energy: %lu\n", lightning_status, distance, energy );
				}
				else
				{ 
					printf("Lightning Data Read Failed\n");
				}
				
			}
			else
			{
				uint8_t SRCO_status = 0, TRCO_status = 0;
				if( get_calibration_status( &SRCO_status, &TRCO_status ) == ESP_OK )
				{
					if( !( SRCO_status & SRCO_CALIB_DONE ) || !( TRCO_status & TRCO_CALIB_DONE ) )
					{
						if( calibrate_AS3935( sensor_location ) == ESP_OK )
						{
							printf("AS3935 Calibrated.\n");
							// set AS3935 bit while maintaining other bits
							sensor_initialized = sensor_initialized | AS3935_INITIALIZED; 
						}
						else
						{
							printf("AS3935 Not Calibrated.\n");
						}
	#define DISABLE_DISTURBER
	#ifdef DISABLE_DISTURBER			 	
						// disable Disturber Interrupt
						if( set_AS3935_reg( REG_X03, MASK_DIST_BIT ) == ESP_OK )
							printf("Disturber interrupt diabled\n");
						else printf("Disable Disturber interrupt failed\n");
	#endif
					}
					else printf("AS3935 previously calibrated\n");
				}
	 		}
	 	}
	}
	//////////////////// End Lightning Detector ////////////////////////////////
		
	if(noise_status == NOISE_LEVEL_GOOD )
	{
		
		// reset noise event count
		noise_count = 0;
		///////////////////////// I2C Devices //////////////////////////////////////
		if(i2c_bus_handle != NULL)
		{
			    printf( "Adding SHT45 device...\n" );
			if( ( ret = SHT45_add_to_i2c_bus( i2c_bus_handle ) ) == ESP_OK )
			{
				printf("SHT45 added to i2c Bus\n");	
				
				if( (ret = get_SHT45_TMP_RH(&temperature, &humidity, &SHT45_tempC) ) == ESP_OK )
				{
					printf("SHT45 temperature: %2.1f, humidity: %2.1f\n", temperature, humidity );
					printf("SHT45_tempC: %2.2f\n", SHT45_tempC);
				}
				else printf("unable to get SHT45 data\n");
			}
			else printf("Failed to add SHT45 to i2c Bus\n\n");
			
			if(ret != ESP_OK )
			{
				temperature = -100;
				humidity = -100;
				SHT45_tempC = -100;
			}
		#ifdef BMP390_INSTALLED
			printf( "Adding BMP390 device...\n" );
			if( (ret = BMP390_add_to_i2c_bus(i2c_bus_handle, BMP390_0 ) ) == ESP_OK )
			{
				printf("BMP390 added to i2c Bus\n");
				if( ( ret =  BMP390_initialize( ) ) == ESP_OK )
				{
					printf("BMP390 initialized\n");
					ret = BMP390_get_data_no_irq(10, &BMP_temperature, &BMP_pressure, &BMP_tempC );
					printf("BMP_tempC: %2.2f\n", BMP_tempC);
				}
				else printf("BMP390 initialization failed\n");
			}
			else printf("Failed to add BMP390 to i2c Bus\n");
			
			if(ret != ESP_OK )
			{
				BMP_temperature = -100;
				BMP_pressure = -100;
				BMP_tempC = -100;
				
			}
		#endif	
			printf( "Adding MAX17048 device...\n" );
			if( MAX17048_add_to_i2c_bus( i2c_bus_handle ) == ESP_OK )
			{
				printf("MAX17048 added to i2c Bus\n");
				
				if( ( ret = MAX17048_read_SOC_data(&batt_soc) ) == ESP_OK )
				{
		
					printf("Battery SOC: %2.1lf\n",  batt_soc );
				}
				else
				{
					printf("unable to get MAX17048: SOC\n");
					batt_soc = -100;
				}
				
				if( ( ret = MAX17048_read_VCELL_data(&batt_volts) ) == ESP_OK )
				{
					printf("Battery Voltage: %1.3lf\n",  batt_volts );
				}
				else
				{
					printf("unable to get MAX17048: battery voltage\n");
					batt_volts = -100;
				}
		
				// use temperature from SHT45 or BMP390 to compensate MAX17048
				if( SHT45_tempC != -100 )
				{
					MAX17048_set_Rcomp( SHT45_tempC );
					printf("SHT45_tempC: %2.2f\n", SHT45_tempC);
				}
				else if( BMP_tempC != -100 )
				{
					MAX17048_set_Rcomp( BMP_tempC );
					printf("BMP_tempC: %2.2f\n", BMP_tempC);
				}
		
			}
			else
			{
				printf("Failed to add MAX17048 to i2c Bus\n");
				batt_volts = -100;
				batt_soc = -100;
			}
		}
		else printf("I2C Bus not Available\n");
		
		///////////////// End I2C Devices ///////////////////////////
		
		////////////// read battery charge current //////////////////
		// int16_t raw_data = 0;
		int mv_data = 0;	
		if( read_adc( &mv_data ) == ESP_OK)
		{
			data.batt_charge = mV_to_mA(mv_data);
			printf( "Battery Charge Current read: %4.0fmA\n", data.batt_charge );
		}
		else
		{
			printf("Battery Charge Current not read\n");
			data.batt_charge = -100.0;
		}
		
		
		//////////////// get sample time ///////////////////
		time(&timestamp);
		
		// see if time is current
		if( timestamp < 1743269000 ) 
		{
			waitForSystemTimeUpdate = true;
			clrSystemTimeSet( );
		}
	    
	    data.air_humidity = humidity;
	    data.air_temperature = temperature;
	    data.air_pressure = BMP_pressure;
	    data.air_pressure_temp = BMP_temperature;
	    data.batt_volts = batt_volts;
	    data.batt_soc = batt_soc;
	    data.location_id = sensor_location;
	    data.time = timestamp;
	    data.irq_status = lightning_status;
	    data.distance = distance;
	    data.energy = energy;
	    
	    // send data to espnow
	    downloadLightning( &data );
	    
	    // start wifi
		wifi_init();
	
		// start sensor network
		espnow_init();
							
							
			// wait here until data has been sent
			// wait for the notification from espnow that the data has been sent
			printf("Waiting for Lightning data to be sent\n");
			if( xSemaphoreTake(xSemaphore_DataReady, portMAX_DELAY ) == pdTRUE)
			{
				printf("Lightning data sent\n");	
				if( waitForSystemTimeUpdate )
				{
					printf("Waiting for System Time updated\n");
					waitForSystemTimeUpdate = false;
					if( xSemaphoreTake(xSemaphore_SystemTimeSet, WAIT_FOR_TIME_UPDATE/ portTICK_PERIOD_MS  ) == pdTRUE)
					{
						// reset update time
						t_system_time_last_update_us = 0;
						printf("System Time updated\n");
					}
					else
					{
						printf("Sytem Time update failed\n");
						t_system_time_last_update_us = TIME_UPDATE_INTERVAL;
					}
				}
				else printf("No System Time update needed\n");
					
			    printf("Entering deep sleep\n");
			
			    // get deep sleep enter time
			    gettimeofday(&sleep_enter_time, NULL);
			
			//#if !SOC_RTC_FAST_MEM_SUPPORTED
			    // record deep sleep enter time via nvs
			    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_sec", sleep_enter_time.tv_sec));
			    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_usec", sleep_enter_time.tv_usec));
			    ESP_ERROR_CHECK(nvs_set_u64(nvs_handle, "update_usec",t_system_time_last_update_us ));
			    ESP_ERROR_CHECK( nvs_set_u8(nvs_handle, "sensor_status", sensor_initialized) );
			    ESP_ERROR_CHECK( nvs_set_u8(nvs_handle, "noise_level", noise_status) );
			    ESP_ERROR_CHECK( nvs_set_u8(nvs_handle, "noise_events", noise_count) );
			    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
			    nvs_close(nvs_handle);
			//#endif
			
				/* To make sure the complete line is printed before entering sleep mode,
				 * need to wait until UART TX FIFO is empty: */
				 //uart_tx_wait_done(CONFIG_ESP_CONSOLE_UART_NUM, 100);
			
			    // enter deep sleep
			    esp_deep_sleep_start();
			}
		} // noise level too high
		else
		{
			int i;
			
			noise_count++;
			printf("Noise events: %u\n", noise_count);
				
			// wait for 10 seconds then goto sleep - waiting for noise to settle
			for(i=0; i<10; i++)
			{
				printf("Noise: Waiting %d seconds to sleep...\n", 10-i);
				vTaskDelay(1000 / portTICK_PERIOD_MS);
			}
			
			if(noise_count>10)
			{
				// send out notice that noise still present after 10 consecutive noise events
				noise_count = 0;
				memset( &data, 0, sizeof( data ) );
				time(&timestamp);
				data.time = timestamp;
				data.location_id = sensor_location;
				data.irq_status = lightning_status;
				
				// send data to espnow
			    downloadLightning( &data );
			    
			    // start wifi
				wifi_init();
			
				// start sensor network
				espnow_init();
				
							// wait here until data has been sent
				// wait for the notification from espnow that the data has been sent
				if( xSemaphoreTake(xSemaphore_DataReady, portMAX_DELAY ) == pdTRUE)
				{
					printf("espnow data saved after 10 noise events\n");
				}

			}
			ESP_ERROR_CHECK( nvs_set_u8(nvs_handle, "noise_events", noise_count) );
			// enter deep sleep
			printf("Entering deep sleep after noise too high detection\n");
			esp_deep_sleep_start();
		}

	}

static void deep_sleep_register_rtc_timer_wakeup(void)
{
    const int wakeup_time_sec = 60;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}

void app_main(void)
{
	esp_err_t err;
	
	 ESP_ERROR_CHECK( err = nvs_flash_init() );
	
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    
    init_GPIO( );
    
    // Initialize I2C Bus
	if( init_I2C( &i2c_bus_handle ) == ESP_OK )
	{
		printf("i2c Bus Initalized \n");
	}
	else printf("i2c Bus Initalization Failed \n");
	
	// Initialize SPI Bus
    if( init_SPI(  ) == ESP_OK )
    {
		spi_available = true;
		printf("SPI Bus Initalized \n");
	}
	else printf("SPI Bus Initalization Failed \n");
	
	
    // create binary semaphore for signaling espnow data sent 
	xSemaphore_DataReady = xSemaphoreCreateBinary();

	// create binary semaphore signaling for system time set
	xSemaphore_SystemTimeSet = xSemaphoreCreateBinary();
	
	/* Enable wakeup from deep sleep by rtc timer */
    deep_sleep_register_rtc_timer_wakeup();
	
	#if CONFIG_EXAMPLE_GPIO_WAKEUP
	/* Enable wakeup from deep sleep by gpio */
	example_deep_sleep_register_gpio_wakeup();
	#endif
	
	xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);

/////////////////////////////////////////////////////////////////////////
    
    while(1)
    {

		vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}
