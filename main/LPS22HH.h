/*
 * LPS22HH.h
 *
 *  Created on: Mar 20, 2025
 *      Author: tiagd
 */

#ifndef MAIN_LPS22HH_H_
#define MAIN_LPS22HH_H_

// I2C driver
#include "driver/i2c_master.h"

// Error library
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "io.h"

#define ACK_CHECK_EN 0x01				/*!< I2C master will check ack from slave*/
#define NACK_VAL 0x01					/*!< I2C nack value */

#define LPS22HH_0 	0x5c
#define LPS22HH_1 	0x5d

#define LPS22HH_CLOCK 	1000000


// Reserved 00 – 0A - Reserved
#define INTERRUPT_CFG			0x0b 		// R/W, default:00000000 Interrupt register
/* ----------------------INTERRUPT_CFG bit definitions ----------------------------
	 7      	 6			 5			 4			 3			 2		 1		 0
	AUTOREFP	RESET_ARP	AUTOZERO	RESET_AZ	DIFF_EN		LIR		PLE		PHE	
*/
#define 	AUTOREFP			0x80		// 		Enable AUTOREFP function. Default value: 0
											//		(0: normal mode; 1: AUTOREFP enabled)
#define 	RESET_ARP 			0x40		//		Reset AUTOREFP function. Default value: 0
											//		(0: normal mode; 1: reset AUTOREFP function)
#define		AUTOZERO 			0x20		//		Enable AUTOZERO function. Default value: 0
											// 		(0: normal mode; 1: AUTOZERO enabled)
#define 	RESET_AZ			0x10		//		Reset AUTOZERO function. Default value: 0
											//		(0: normal mode; 1: reset AUTOZERO function)
#define 	DIFF_EN 			0x08		//		Enable interrupt generation. Default value: 0
											// 		(0: interrupt generation disabled; 1: interrupt generation enabled)
#define 	LIR					0x04		//		Latch interrupt request to the INT_SOURCE (24h) register. Default value: 0
											//		(0: interrupt request not latched; 1: interrupt request latched)
#define 	PLE 				0x02		//		Enable interrupt generation on pressure low event. Default value: 0
											//		(0: disable interrupt request;
											//		1: enable interrupt request on pressure value lower than preset threshold)
#define 	PHE 				0x01		//		Enable interrupt generation on pressure high event. Default value: 0
											//		(0: disable interrupt request;
											//		1: enable interrupt request on pressure value higher than preset threshold)

#define THS_P_L					0x0c 		// R/W, default:00000000 Pressure threshold register low (lsb)
//   7    6    5   4    3    2    1    0
// THS7 THS6 THS5 THS4 THS3 THS2 THS1 THS0
// THS[7:0] This register contains the low part of threshold value for pressure interrupt generation.
// Default value: 00h

#define THS_P_H 				0x0d		// R/W, default:00000000 Pressure threshold register high (msb)
// 7   6     5     4     3     2     1    0
// -  THS14 THS13 THS12 THS11 THS10 THS9 THS8
// THS[14:8]
// This register contains the high part of threshold value for pressure interrupt generation.
// Refer to THS_P_L (0Ch).
// Default value: 00h

#define IF_CTRL 				0x0e 		// R/W, default:00000000 Interface control register
//  7           6   5    4         3         2           1           0
// INT_EN_I3C   0   0   SDA_PU_EN SDO_PU_EN PD_DIS_INT1 I3C_DISABLE I2C_DISABLE
#define INT_EN_I3C 0x80
// Enable INT1 pad with MIPI I3CSM. If the INT_EN_I3C bit is set, the INT1 pad is
// polarized as OUT. Default value: 0
// (0: INT1 disabled with MIPI I3CSM; 1: INT1 enabled with MIPI I3CSM)
#define SDA_PU_EN 0x10
// Enable pull-up on the SDA pin. Default value: 0
// (0: SDA pin pull-up disconnected; 1: SDA pin with pull-up)
#define SDO_PU_EN 0x08
// Enable pull-up on the SDO pin. Default value: 0
// (0: SDO pin pull-up disconnected; 1: SDO pin with pull-up)
#define PD_DIS_INT1 0x04
// Disable pull down on the INT1 pin. Default value: 0
// (0: INT1 pin with pull-down; 1: INT1 pin pull-down disconnected)
#define I3C_DISABLE 0x02	// (1)
//  1. I3C_DISABLE bit disables the MIPI I3CSM communication protocol.
// Disable MIPI I3CSM interface. Default value: 0
// (0: MIPI I3CSM enabled; 1: MIPI I3CSM disabled)
#define I2C_DISABLE 0x01	// (2)
// Disable I²C interface. Default value: 0
// (0: I²C enabled; 1: I²C disabled)

// 1. I3C_DISABLE bit disables the MIPI I3CSM communication protocol.
// 2. I2C_DISABLE bit disables the I²C interface, by default both SPI and I²C interfaces are enabled.

#define WHO_AM_I 				0x0f 		// R, 	default:10110011 Who am I
#define WHO_AM_I_VALUE			0b10110011

#define CTRL_REG1				0x10		// R/W, default:00000000 Control registers
// 7   6     5     4    3        2         1    0
// 0  ODR2  ODR1  ODR0 EN_LPFP  LPFP_CFG  BDU  SIM
#define ODR 0x70		// [2:0] Output data rate selection. Default value: 000
// Refer to Table 18.
#define EN_LPFP 0x08	//Enable low-pass filter on pressure data when Continuous mode is used.
// Default value: 0
//(0: Low-pass filter disabled; 1: Low-pass filter enabled)
#define LPFP_CFG 0x04 	// LPFP_CFG: Low-pass configuration register. Default value: 0
//Refer to Table 19.
#define BDU 0x02		//(1)Block data update. Default value: 0
// (0: continuous update;
// 1: output registers not updated until MSB and LSB have been read)
#define SIM 0x01		//SPI Serial Interface Mode selection. Default value: 0
// (0: 4-wire interface; 1: 3-wire interface)
// 1. To guarantee the correct behavior of BDU feature, PRESS_OUT_H (2Ah) must be the last address read.

/*				Table 18. Output data rate bit configurations
						ODR[2:0] Temperature, Pressure 
						*/
#define ONE_SHOT			Ob00000000 		// One-shot
#define ONE_HZ				0b00010000 		// 	1 Hz
#define TEN_HZ				0b00100000 		//  10 Hz
#define TWENTY_FIVE_HZ		0b00110000 		//	25 Hz
#define FIFTY_HZ			0b01000000 		//	50 Hz
#define SEVENTY_FIVE_HZ		0b01010000 		//	75 Hz
#define ONE_HUNDRED_HZ		0b01100000 		//	(1)	100 Hz
#define TWO_HUNDRED_HZ		0b01110000		//	(1) 200 Hz
/*
1. This option disables the low-noise mode automatically.
*/ 

/* Table 19. Low-pass filter configurations */
// EN_LPFP LPFP_CFG 	Additional low-pass filter status Device bandwidth
//  0		x Disabled ODR/2
//  1		0 Enabled ODR/9
//  1		1 Enabled ODR/20 

#define CTRL_REG2				0x11 		// R/W, default:00010000 
//  7      6         5       4           3    2         1              0
// BOOT   INT_H_L   PP_OD   IF_ADD_INC   0   SWRESET   LOW_NOISE_EN   ONE_SHOT
#define BOOT 0x80			// Reboots memory content. Default value: 0
//	(0: normal mode; 1: reboot memory content)
#define INT_H_L Interrupt active-high, active-low. Default value: 0
// (0: active high; 1: active low)
#define PP_OD 0x40  // Push-pull/open-drain selection on interrupt pad. Default value: 0
// (0: push-pull; 1: open-drain)
#define IF_ADD_INC 0x20  // Register address automatically incremented during a multiple byte access with a
// serial interface (I²C or SPI). Default value: 1
// (0: disable; 1: enable)
#define SWRESET 0x04 // Software reset. Default value: 0
// (0: normal mode; 1: software reset).
// The bit is self-cleared when the reset is completed.
#define LOW_NOISE_EN 0x02 //  Enables low noise (used only if ODR is lower than 100 Hz). Default value: 0
// (0: low-current mode; 1: low-noise mode)
#define ONE_SHOT 0x01  // Enables one-shot. Default value: 0
// (0: idle mode; 1: a new dataset is acquired)

#define CTRL_REG3				0x12 		// R/W, default:00000000
//  7   6    5            4           3           2      1        0
//  0   0   INT_F_FULL   INT_F_WTM   INT_F_OVR   DRDY   INT_S1   INT_S0
#define INT_F_FULL 0x20  // FIFO full flag on INT_DRDY pin. Default value: 0
// (0: FIFO empty; 1: FIFO full - 128 unread samples)
#define INT_F_WTM 0x10  // FIFO threshold (watermark) status on INT_DRDY pin. Default value: 0
// (0: FIFO is lower than FTH level; 1: FIFO is equal to or higher than FTH level)
#define INT_F_OVR 0x08 // FIFO overrun status on INT_DRDY pin. Default value: 0
// (0: not overwritten; 1: at least one sample in the FIFO has been overwritten)
#define DRDY 0x04  // Data-ready signal on INT_DRDY pin. Default value: 0
// (0: disable; 1: enable)
#define INT_S  0x03 // [1:0] Data signal on INT_DRDY pin control bits. Default value: 00
// Refer to Table 21.

// Table 21. Interrupt configurations
// INT_S1 INT_S0 	INT_DRDY pin configuration
//  0      0    	Data signal (in order of priority: DRDY or INT_F_WTM or INT_F_OVR or INT_F_FULL)
//  0      1    	Pressure high (P_high)
//  1      0    	Pressure low (P_low)
//  1      1    	Pressure low OR high

#define FIFO_CTRL				0x13 		// R/W, default:00000000 FIFO configuration register
// 7   6   5   4    3             2            1         0
// 0   0   0   0   STOP_ON_WTM   TRIG_MODES   F_MODE1   F_MODE0
#define STOP_ON_WTM 0x08  // Stop-on-FIFO watermark. Enables FIFO watermark level use. Default value: 0
// (0: disable; 1: enable)
#define TRIG_MODES 0x04  // Enables triggered FIFO modes. Default value: 0
#define  F_MODE 0x03     // [1:0] Selects triggered FIFO modes. Default value: 00
// Refer to Table 22.
// Table 22. FIFO mode selection
// TRIG_MODES F_MODE[1:] Mode
//  x 00 Bypass
// 0 01 FIFO mode
// 0 1x Continuous (Dynamic-Stream)
// 1 01 Bypass-to-FIFO
// 1 10 Bypass-to-Continuous (Dynamic-Stream)
// 1 11 Continuous (Dynamic-Stream)-to-FIFO

#define FIFO_WTM				0x14 		// R/W, default:00000000
//  7    6      5      4      3      2      1      0
//  0   WTM6   WTM5   WTM4   WTM3   WTM2   WTM1   WTM0
#define WTM 0x7f // [6:0] FIFO threshold. Watermark level setting. Default value: 0000000

#define REF_P_L					0x15 		// R,   default:00000000 Reference pressure register low
#define REF_P_H					0x16 		// R,   default:00000000 Reference pressure register high
// Reserved 17 - Reserved
#define RPDS_L					0x18		// R/W, default:00000000 Pressure offset register low
#define RPDS_H					0x19		// R/W, default:00000000 Pressure offset register high
// Reserved 1A-23 - Reserved

#define INT_SOURCE 				0x24		// R,					 Interrupt register
//   7        6   5   4   3    2    1    0
//  BOOT_ON   0   0   0   0   IA   PL   PH
#define BOOT_ON 0x80  // Indication of Boot phase.
// (0: Boot phase has ended;
// 1: Boot phase is running).
#define IA 0x04 //  Interrupt active.
// (0: no interrupt has been generated;
// 1: one or more interrupt events have been generated).
#define PL  0x02 // Differential pressure Low.
// (0: no interrupt has been generated;
// 1: low differential pressure event has occurred).
#define PH 0x01  //  Differential pressure High.
// (0: no interrupt has been generated;
// 1: high differential pressure event has occurred).

#define FIFO_STATUS1			0x25		// R, 					 FIFO status registers
//   7      6      5      4      3      2      1      0
//  FSS7   FSS6   FSS5   FSS4   FSS3   FSS2   FSS1   FSS0
//  FSS[7:0] FIFO stored data level, number of unread samples stored in FIFO.
//  (00000000: FIFO empty; 10000000: FIFO full, 128 unread samples)

#define FIFO_STATUS2 			0x26		// R,
//   7             6             5             4   3   2   1   0
//  FIFO_WTM_IA   FIFO_OVR_IA   FIFO_FULL_IA   -   -   -   -   -
#define FIFO_WTM_IA 0x80   // FIFO threshold (watermark) status. Default value: 0
// (0: FIFO filling is lower than treshold level;
// 1: FIFO filling is equal or higher than treshold level).
#define FIFO_OVR_IA 0x40   // FIFO overrun status. Default value: 0
// (0: FIFO is not completely full;
// 1: FIFO is full and at least one sample in the FIFO has been overwritten).
#define FIFO_FULL_IA 0x20   // FIFO full status. Default value: 0
// (0: FIFO is not completely filled;
//  1: FIFO is completely filled, no samples overwritten)


#define STATUS_PRES_TEMP		0x27		// R,					 Status register
//   7    6   5      4      3    2    1      0
//  --   --  T_OR   P_OR   --   --   T_DA   P_DA
#define  T_OR 0x20   //  Temperature data overrun.
//  (0: no overrun has occurred;
// 1: a new data for temperature has overwritten the previous data)
#define  P_OR 0x10   // Pressure data overrun.
//  (0: no overrun has occurred;
//  1: new data for pressure has overwritten the previous data)
#define  T_DA  0x02  //  Temperature data available.
//  (0: new data for temperature is not yet available;
//  1: a new temperature data is generated)
#define  P_DA 0x01   //  Pressure data available.
//  (0: new data for pressure is not yet available;
//  1: a new pressure data is generated)

#define PRESSURE_OUT_XL			0x28		// R,					 Output Pressure output register low
#define PRESSURE_OUT_L			0x29		// R,					 Output Pressure output register mid
#define PRESSURE_OUT_H			0x2a		// R,					 Output Pressure output register high
#define TEMP_OUT_L				0x2b		// R,					 Temperature output register low
#define TEMP_OUT_H				0x2c		// R,					 Temperature output register high
//Reserved 2D - 77 - Reserved
#define FIFO_DATA_OUT_PRESS_XL	0x78		// R,					 FIFO pressure output register low
#define FIFO_DATA_OUT_PRESS_L	0x79		// R,					 FIFO pressure output register mid
#define FIFO_DATA_OUT_PRESS_H	0x7a		// R,					 FIFO pressure output register high
#define FIFO_DATA_OUT_TEMP_L	0x7b		// R,					 FIFO temperature output register low 
#define FIFO_DATA_OUT_TEMP_H	0x7c		// R,					 FIFO temperature output register high

esp_err_t LPS22HH_add_to_i2c_bus( i2c_master_bus_handle_t bus_handle );
esp_err_t LPS22HH_read_deviceID(uint8_t *deviceID);
esp_err_t LPS22HH_configure( void );
esp_err_t  LPS22HH_get_pressure_temperature(uint8_t *data, bool *data_good );

#endif /* MAIN_LPS22HH_H_ */
