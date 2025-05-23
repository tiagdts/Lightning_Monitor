/*
 * adc_.c
 *
 *  Created on: Mar 29, 2025
 *      Author: tiagd
 */

#include "adc_.h"

	const static char *TAG = "adc_c.c";
	adc_oneshot_unit_handle_t adc1_handle;
	adc_cali_handle_t adc1_cali_chan0_handle = NULL;
	bool do_calibration1_chan0 = false;


	static int adc_raw[2][ARRAY_SIZE];
	static int voltage[2][ARRAY_SIZE];
	static uint8_t valid_data[ARRAY_SIZE];
	
	static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

esp_err_t init_adc( void )
{
	esp_err_t ret;
	uint8_t i;
	
	for(i=0;i<ARRAY_SIZE;i++)
	{
		adc_raw[0][i] = 0;
		voltage[0][i] = 0;
		valid_data[i] = 1;
	}
	    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(ret = adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(ret = adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    
        //-------------ADC1 Calibration Init---------------//
//    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
    //bool do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);

    return ret;
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

esp_err_t  read_adc( int *adc_volt )
{
	esp_err_t ret;
	
	uint8_t i;
	
	for(i=0;i<ARRAY_SIZE;i++)
	{
		ESP_ERROR_CHECK(ret = adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][i]));
	    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][i]);
	    
	    //*adc_raw_data = adc_raw[0][0];
	    if (do_calibration1_chan0)
	    {
	        ESP_ERROR_CHECK(ret = adc_cali_raw_to_voltage(adc1_cali_chan0_handle,adc_raw[0][i], &voltage[0][i]));
	        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][i]);
	        *adc_volt = voltage[0][0];
	    }
	    else return ESP_FAIL;  // CALIBRATION NOT AVAILABLE
	    vTaskDelay(pdMS_TO_TICKS(10));
    }
    *adc_volt = calc_overall_mean( );
    return ret;
}    

float mV_to_mA(int16_t mv)
{
	return ( (mv * mv * -0.0032) + ( mv * 1.0976) + 4.7772 );
}

int calc_overall_mean( void )
{
	uint16_t i;
	int sum, sum_count;
	int min, max;
	int smallest;
	int largest;
	int mean;

	// determine smallest and largest

	min = voltage[0][0];
	max = voltage[0][0];
	smallest = 0;
	largest = 0;
	for(i = 1; i < ARRAY_SIZE; i++)
	{

		// check for smallest
		if( voltage[0][i] < min )
		{
			min = voltage[0][i];
			smallest = i;
		}
		// check for largest
		if( voltage[0][i] > max )
		{
			max = voltage[0][i];
			largest = i;
		}
	}
	valid_data[ smallest ] = 0; // mark smallest invalid
	valid_data[ largest ]  = 0;  // mark largest invalid

	// calculate average excluding the smallest and the largest readings
	sum = 0;
	sum_count = 0;
	for(i = 0; i < ARRAY_SIZE; i++)
	{
		if( valid_data[i] )
		{
			sum += voltage[0][i];
			sum_count++;
		}

	}
	if(sum_count>0)
		mean = sum/sum_count;
	else mean = 0;
	
	return mean;
}

