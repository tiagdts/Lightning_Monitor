/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)
b
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_H
#define ESPNOW_H

#include "esp_now.h"
#include "Station_Data_Types.h"


/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           10
#define ESPNOW_SEND_LEN 100
// #define CONFIG_ESPNOW_SEND_DELAY	2000
#define CONFIG_ESPNOW_SEND_DELAY	200
#define CONFIG_ESPNOW_CHANNEL		1
#define CONFIG_ESPNOW_SEND_COUNT	100
#define CONFIG_ESPNOW_PMK			"pmk1234567890123"

#define TASK_DATA_WAIT_TIME			50

#define DATA_READ					0
#define DATA_READ_TIMEOUT			-1

//#define WEATHER_DATA_TYPE	0
#define POND_DATA_TYPE		0
#define MPPT_DATA_TYPE		1
//#define PHONE_DATA_TYPE		2
//#define RAIN_DATA_TYPE		3



#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data. */
typedef struct {
    uint8_t payload_type;                 		// data type weather, pump, etc
    uint16_t seq_num;                     		//Sequence number of ESPNOW data.
    uint16_t crc;                         		//CRC16 value of ESPNOW data.
    uint8_t payload[( ESPNOW_SEND_LEN -5 )];	//Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} espnow_send_param_t;

void wifi_init(void);
esp_err_t espnow_init(void);
int16_t  updateWeather( weatherData_t *data );
int16_t updateWeatherloc( weatherData_t *data );
int16_t updateRain( rainData_t *data );
int16_t updateRainloc( rainData_t *data );
int16_t  updatePump( pumpData_t *data );
int16_t  updatePumploc( pumpData_t *data );
int16_t  updatePhone( phoneData_t *data );
int16_t  updateHVAC( HVACdata_t *data );
int16_t  updateRoom( roomData_t *data );
int16_t  updateRoomloc( roomData_t *data );
int16_t  updateTime( timeData_t *data );
int16_t updateSystemTime( systemTimeData_t *data);
int16_t  updateSystemTimeloc( systemTimeData_t *data );
uint32_t getDataReadyStatus( void );
void clearDataReadyStatus( uint32_t clearbits);
int16_t  updateTimeloc( timeData_t *data );
int16_t updateMPPTloc( MPPTdata_t *data );
int16_t updateWeatherloc( weatherData_t *data );
int16_t  updatePhoneloc( phoneData_t *data );
int16_t updateWeatherCalLoc( weatherCalibrationData_t *data );
int16_t updateWeatherCal( weatherCalibrationData_t *data );
int16_t  updatePond( pondData_t *data );
int16_t updatePondloc( pondData_t *data );
int16_t  updateDuct( ductData_t *data );
int16_t updateDuctloc( ductData_t *data );
void downloadDuct( ductData_t *data );
int16_t  updateLightning( lightningData_t *data );
int16_t updateLightningloc( lightningData_t *data );
void downloadLightning( lightningData_t *data );

bool getReadyToSleep(void);
void clrReadyToSleep(void);
void clrDataTypesToSendAll(void);
void clrDataTypesToSendIndividual(uint32_t i);

#endif
