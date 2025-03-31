/*
 * adc_.h
 *
 *  Created on: Mar 29, 2025
 *      Author: tiagd
 */

#ifndef MAIN_ADC__H_
#define MAIN_ADC__H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_0
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

esp_err_t init_adc( void );
void adc_calibration_deinit(adc_cali_handle_t handle);
esp_err_t  read_adc( int16_t *adc_raw_data, int16_t *adc_volt );
float mV_to_mA(int16_t mv);

#endif /* MAIN_ADC__H_ */
