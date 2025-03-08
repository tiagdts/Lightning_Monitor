/*
 * BMP390.h
 *
 *  Created on: Sep 10, 2022
 *      Author: tiagd
 */

#ifndef MAIN_BMP390_H_
#define MAIN_BMP390_H_

#include <string.h>
#include <stddef.h>

// I2C driver
#include "driver/i2c_master.h"

//#include "HDC3020.h"

// Error library

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "io.h"

#define ACK_CHECK_EN 0x01				/*!< I2C master will check ack from slave*/
#define NACK_VAL 0x01					/*!< I2C nack value */

#define BMP390_0 	0x76
#define BMP390_1 	0x77

#define BMP390_CLOCK 	1000000


/*!
 * @brief Interface selection Enums
 */
enum bmp3_intf {
    /*! SPI interface */
    BMP3_SPI_INTF,
    /*! I2C interface */
    BMP3_I2C_INTF
};
/**\name Register Address */

#define BMP390_PRESSURE						0x04		// 24 bits of data

#define BMP390_DEVICE_ID         		 	0x00		//	Device ID byte
#define BMP390_REV_ID						0x01		//	mask revision of the ASIC
#define BMP390_REG_ERR						0x02
#define BMP390_REG_SENS_STATUS				0x03
#define BMP390_REG_DATA						0x04
#define BMP390_REG_EVENT					0x10
#define BMP390_REG_INT_STATUS				0x11
#define BMP390_REG_FIFO_LENGTH				0x12
#define BMP390_REG_FIFO_DATA 				0x14
#define BMP390_REG_FIFO_WM					0x15
#define BMP390_REG_FIFO_CONFIG_1			0x17
#define BMP390_REG_FIFO_CONFIG_2			0x18
#define BMP390_REG_INT_CTRL					0x19
#define BMP390_REG_IF_CONF					0x1A
#define BMP390_REG_PWR_CTRL					0x1B
#define BMP390_REG_OSR						0x1C
#define BMP390_REG_ODR						0x1D
#define BMP390_REG_CONFIG					0x1F
#define BMP390_REG_CALIB_DATA				0x31
#define BMP390_REG_CMD						0x7E


/**\name Error status macros */
#define BMP390_ERR_FATAL					0x01
#define BMP390_ERR_CMD						0x02
#define BMP390_ERR_CONF						0x04

/**\name Status macros */
#define BMP390_CMD_RDY						0x10
#define BMP390_DRDY_PRESS					0x20
#define BMP390_DRDY_TEMP					0x40

/**\name Power mode macros */
#define BMP390_MODE_SLEEP					0x00
#define BMP390_MODE_FORCED					0x10
#define BMP390_MODE_NORMAL					0x30
#define BMP390_PRES_ENA						0x01
#define BMP390_TEMP_ENA						0x02

/**\name FIFO related macros */
/**\name FIFO enable  */
#define BMP390_ENABLE						0x01
#define BMP390_DISABLE						0x00

/**\name Interrupt pin configuration macros */
#define BMP390_INT_PIN_ACTIVE_HIGH			0x02
#define BMP390_INT_PIN_ACTIVE_LOW			0x00

/* interrupt if data ready */
#define BMP390_INT_ON_DRDY					0x40

/* data ready bit in status register */
#define BMP390_DRDY							0x08

/**\name Open drain */
#define BMP390_INT_PIN_OPEN_DRAIN			0x01
#define BMP390_INT_PIN_PUSH_PULL			0x00

/**\name Latch */
#define BMP390_INT_PIN_LATCH				0x04
#define BMP390_INT_PIN_NON_LATCH			0x00

/**\name Advance settings  */
/**\name I2c watch dog timer period selection */
#define BMP390_I2C_WDT_SHORT_1_25_MS		0x00
#define BMP390_I2C_WDT_LONG_40_MS			0x01

/**\name FIFO Sub-sampling macros */
#define BMP390_FIFO_NO_SUBSAMPLING			0x00
#define BMP390_FIFO_SUBSAMPLING_2X			0x01
#define BMP390_FIFO_SUBSAMPLING_4X			0x02
#define BMP390_FIFO_SUBSAMPLING_8X			0x03
#define BMP390_FIFO_SUBSAMPLING_16X			0x04
#define BMP390_FIFO_SUBSAMPLING_32X			0x05
#define BMP390_FIFO_SUBSAMPLING_64X			0x06
#define BMP390_FIFO_SUBSAMPLING_128X		0x07

/**\name Pressure Over sampling macros */
#define BMP390_NO_PRES_OVERSAMPLING			0x00
#define BMP390_PRES_OVERSAMPLING_2X			0x01
#define BMP390_PRES_OVERSAMPLING_4X			0x02
#define BMP390_PRES_OVERSAMPLING_8X			0x03
#define BMP390_PRES_OVERSAMPLING_16X		0x04
#define BMP390_PRES_OVERSAMPLING_32X		0x05

/**\name Temperature Over sampling macros */
#define BMP390_NO_TEMP_OVERSAMPLING			0x00
#define BMP390_TEMP_OVERSAMPLING_2X			0x08
#define BMP390_TEMP_OVERSAMPLING_4X			0x10
#define BMP390_TEMP_OVERSAMPLING_8X			0x18
#define BMP390_TEMP_OVERSAMPLING_16X		0x20
#define BMP390_TEMP_OVERSAMPLING_32X		0x28

/**\name Filter setting macros */
#define BMP390_IIR_FILTER_DISABLE			0x00
#define BMP390_IIR_FILTER_COEFF_1			0x01
#define BMP390_IIR_FILTER_COEFF_3			0x02
#define BMP390_IIR_FILTER_COEFF_7			0x03
#define BMP390_IIR_FILTER_COEFF_15			0x04
#define BMP390_IIR_FILTER_COEFF_31			0x05
#define BMP390_IIR_FILTER_COEFF_63			0x06
#define BMP390_IIR_FILTER_COEFF_127			0x07

/**\name Odr setting macros */
#define BMP390_ODR_200_HZ					0x00
#define BMP390_ODR_100_HZ					0x01
#define BMP390_ODR_50_HZ					0x02
#define BMP390_ODR_25_HZ					0x03
#define BMP390_ODR_12_5_HZ					0x04
#define BMP390_ODR_6_25_HZ					0x05
#define BMP390_ODR_3_1_HZ					0x06
#define BMP390_ODR_1_5_HZ					0x07
#define BMP390_ODR_0_78_HZ					0x08
#define BMP390_ODR_0_39_HZ					0x09
#define BMP390_ODR_0_2_HZ					0x0A
#define BMP390_ODR_0_1_HZ					0x0B
#define BMP390_ODR_0_05_HZ					0x0C
#define BMP390_ODR_0_02_HZ					0x0D
#define BMP390_ODR_0_01_HZ					0x0E
#define BMP390_ODR_0_006_HZ					0x0F
#define BMP390_ODR_0_003_HZ					0x10
#define BMP390_ODR_0_001_HZ					0x11

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.*/
#define BMP3_PRESS                             1
#define BMP3_TEMP                              2
#define BMP3_PRESS_TEMP                        3

/**\name Temperature range values in integer and float */
#define BMP3_MIN_TEMP_INT                       INT64_C(-4000)
#define BMP3_MAX_TEMP_INT                       INT64_C(8500)
#define BMP3_MIN_TEMP_DOUBLE                    -40.0f
#define BMP3_MAX_TEMP_DOUBLE                    85.0f

/**\name Pressure range values in integer and float */
#define BMP3_MIN_PRES_INT                       UINT64_C(3000000)
#define BMP3_MAX_PRES_INT                       UINT64_C(12500000)
#define BMP3_MIN_PRES_DOUBLE                    30000.0f
#define BMP3_MAX_PRES_DOUBLE                    125000.0f

#define pascalToInchMg	 						0.00029529983071f
#define pascalToMillibar						0.0100f

/**\name API error codes */
#define BMP3_E_NULL_PTR                         -1
#define BMP3_E_COMM_FAIL                        -2
#define BMP3_E_INVALID_ODR_OSR_SETTINGS         -3
#define BMP3_E_CMD_EXEC_FAILED               	-4
#define BMP3_E_CONFIGURATION_ERR                -5
#define BMP3_E_INVALID_LEN                    	-6
#define BMP3_E_DEV_NOT_FOUND                    -7
#define BMP3_E_FIFO_WATERMARK_NOT_REACHED       -8

/**\name API warning codes */
#define BMP3_W_SENSOR_NOT_ENABLED               1
#define BMP3_W_INVALID_FIFO_REQ_FRAME_CNT       2
#define BMP3_W_MIN_TEMP                         3
#define BMP3_W_MAX_TEMP                         4
#define BMP3_W_MIN_PRES                         5
#define BMP3_W_MAX_PRES                         6

#define BMP3_INTF_RET_TYPE                      int8_t

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/**\name API success code */
#define BMP3_OK                                 0

typedef void (*bmp3_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

struct BMP390_reg_calib_data
{
    /*! Trim Variables */

    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    int64_t t_lin;
};

/*!
 * @brief Quantized Trim Variables
 */
struct bmp3_quantized_calib_data
{
    /*! Quantized Trim Variables */

    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
};

/*!
 * @brief Register Trim Variables
 */
struct bmp3_reg_calib_data
{
    /*! Trim Variables */

    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    int64_t t_lin;
};



/*!
 * @brief Calibration data
 */
struct bmp3_calib_data
{
    /*! Quantized data */
    struct bmp3_quantized_calib_data quantized_calib_data;

    /*! Register data */
    struct bmp3_reg_calib_data reg_calib_data;
};

struct bmp3_uncomp_data
{
    /*! un-compensated pressure */
    uint64_t pressure;

    /*! un-compensated temperature */
    int64_t temperature;
};

/*!
 * @brief bmp3 sensor structure which comprises of temperature and pressure
 * data.
 */
struct bmp3_data
{
    /*! Compensated temperature */
    double temperature;

    /*! Compensated pressure */
    double pressure;
};

/*!
 * @brief bmp3 device structure
 */
struct bmp3_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
 //   void *intf_ptr;

    /*! Interface Selection
     * For SPI, interface = BMP3_SPI_INTF
     * For I2C, interface = BMP3_I2C_INTF
     **/
//    enum bmp3_intf intf;

    /*! To store interface pointer error */
//    BMP3_INTF_RET_TYPE intf_rslt;

    /*! Decide SPI or I2C read mechanism */
//    uint8_t dummy_byte;

    /*! Read function pointer */
//    bmp3_read_fptr_t read;

    /*! Write function pointer */
//    bmp3_write_fptr_t write;

    /*! Delay function pointer */
 //   bmp3_delay_us_fptr_t delay_us;

    /*! Trim data */
    struct bmp3_calib_data calib_data;
};


esp_err_t BMP390_read(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_rd, size_t size);
esp_err_t BMP390_write(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size);
esp_err_t BMP390_read_deviceID(uint8_t *deviceID);
esp_err_t BMP390_read_revID(uint8_t *revID);
esp_err_t BMP390_read_pressure(uint8_t *data);
esp_err_t BMP390_read_cal_data(struct BMP390_reg_calib_data *data);
esp_err_t BMP390_configure( uint8_t *cal_data );
void BMP390_isr_handler(void* arg);
void BMP390_Task(void *pvParameter);
void BMP390_createSemaphores(void);
esp_err_t BMP390_initialize( void );
void getBMPpressure( float *pressure );
esp_err_t BMP390_start_forced_mode(uint8_t *data);
esp_err_t BMP390_get_pressure_temperature(uint8_t *data, bool *data_good );
esp_err_t BMP390_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle, uint16_t address );
esp_err_t BMP390_get_raw_pressure_temperature(uint8_t *data, bool *data_good );
esp_err_t BMP390_get_data_no_irq(uint16_t attempts, float *temperature, float *pressure, float *tempC);

#endif /* MAIN_BMP390_H_ */
