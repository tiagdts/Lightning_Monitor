#ifndef MAIN_DS3231_H_
#define MAIN_DS3231_H_

#include <time.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include <sys/time.h>

#include "io.h"

#define ACK_CHECK_EN 0x01				/*!< I2C master will check ack from slave*/
#define NACK_VAL 0x01					/*!< I2C nack value */

#define SEC	 			0x00		// Second
#define MINUTE			0x01		// Minute
#define HOUR		 	0x02		// Hour
#define WEEK 			0x03		// Day of Week
#define DAY				0x04		// Day
#define MONTH			0X05		// Month
#define YEAR			0x06		// Year

#define DS3231_ADDR 0x68 //!< I2C address

#define DS3231_STAT_OSF		   0x80		// Oscillator Stop Flag
#define DS3231_STAT_32KHZ      0x08
#define DS3231_STAT_BUSY       0x04
#define DS3231_STAT_ALARM_2    0x02
#define DS3231_STAT_ALARM_1    0x01

#define DS3231_CTRL_OSCILLATOR    0x80
#define DS3231_CTRL_SQUAREWAVE_BB 0x40
#define DS3231_CTRL_TEMPCONV      0x20
#define DS3231_CTRL_ALARM_INTS    0x04
#define DS3231_CTRL_ALARM2_INT    0x02
#define DS3231_CTRL_ALARM1_INT    0x01

#define DS3231_ALARM_WDAY   0x40
#define DS3231_ALARM_NOTSET 0x80

#define DS3231_ADDR_TIME    0x00
#define DS3231_ADDR_ALARM1  0x07
#define DS3231_ADDR_ALARM2  0x0b
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_ADDR_STATUS  0x0f
#define DS3231_ADDR_AGING   0x10
#define DS3231_ADDR_TEMP    0x11

#define DS3231_12HOUR_FLAG  0x40
#define DS3231_12HOUR_MASK  0x1f
#define DS3231_PM_FLAG      0x20
#define DS3231_MONTH_MASK   0x1f

esp_err_t DS3231_read(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_rd, size_t size);
esp_err_t DS3231_write(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size);
uint8_t bcd2dec(uint8_t val);
uint8_t dec2bcd(uint8_t val);
esp_err_t ds3231_set_time(struct tm *time);
esp_err_t ds3231_get_raw_temp(int16_t *temp);
esp_err_t ds3231_get_temp_integer(int8_t *temp);
esp_err_t ds3231_get_temp_float(float *temp);
esp_err_t ds3231_get_time(struct tm *time);
esp_err_t ds3231_get_pwr_status(bool *pwr_interruption);
esp_err_t ds3231_set_pwr_status(void);
void check_RTC( time_t t );
void set_RTC( void );
void setsystemtime( void );
#endif /* MAIN_DS3231_H_ */

