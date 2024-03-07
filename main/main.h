#ifndef _MAIN_H_
#define _MAIN_H_


#include <stdio.h>
#include "driver/uart.h"
#include "sdkconfig.h"
#include "time.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include <string.h>

//==========================================================
#define POWER_TASK_STACK_SIZE          (4096)
#define MAX_ATTEMPS 3

//============================PINs IN/OUT==========================//

#define DEBUG_TX_PIN 			GPIO_NUM_43  //115200
#define DEBUG_RX_PIN 			GPIO_NUM_44

//============================PARAMETERS GPS L86==========================//

#define L86_FORCE_ON_PIN 		GPIO_NUM_6
#define L86_RESET_PIN 			GPIO_NUM_7

#define     L86_UART_NUM        UART_NUM_2
#define     L86_PIN_RXD         GPIO_NUM_16 //9600 
#define     L86_PIN_TXD         GPIO_NUM_15
#define     L86_UART_BUF_SIZE   (1024) 


//============================PARAMETERS M95==========================//
// power key 25

#define INPUT_I1	GPIO_NUM_18

//============================PARAMTERS ADS==========================//
#define ADC_UNIT_BAT_EXT ADC_UNIT_2
#define ADC_UNIT_BAT_INT ADC_UNIT_1
#define ADC_UNIT_PANIC	 ADC_UNIT_1

#define ADC_CHANEL_BAT_EXT ADC_CHANNEL_6
#define ADC_CHANEL_BAT_INT ADC_CHANNEL_3
#define ADC_CHANEL_PANIC	ADC_CHANNEL_4

//============================ STATICS VARIABLES MAIN CONFIG==========================//
#define TIMER_INTERVAL_SECONDS 	60 // cada 1 min
#define TIME_EPOCH_LIMIT 		1696136400
#define INTERVAL_GPS_SEND   	12*60*60	// S

//============================STATICS VARIABLES GPS==========================//
#define SHOW_NMEA_SENTENCE  0
#define PROTOCOL_QUECLINK 

#define PRESICION_GPS  10000000
#define knots_to_kmph  1.582
#define PERIOD_GPS     1500    //(1Hz, cada 1 segundo = 1000 ms)

//============================STRUCTURES==========================//
/**
 * @brief Objeto GPS
 *
 */
typedef struct {
    struct tm time;
    double latitude;                        /*!< Latitud (grados) */
    double longitude;                       /*!< Longitud (grados) */
    bool valid;                             /*!< validación de la data */
} gps_t;


/**
 * @brief Objet record data struct
 *
 */
typedef struct{
	// time.
	time_t time;
	// GPS
	gps_t data_sat;
}record_data_t;


#endif