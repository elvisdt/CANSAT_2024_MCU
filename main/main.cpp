#include "Arduino.h"

extern "C" {

#include <stdio.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "math.h"
#include <sys/time.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_err.h"
#include "main.h"

#include "nmea_uart.c"
#include "nmea.h"
#include "gptxt.h"
#include "gprmc.h"
#include "gpgga.h"

}
/**********************************************
 * DEFINES
***********************************************/
#define TAG "MAIN"


#define DEBUG_MAIN          0           /*1->enable debug   ,  0->disable debug*/
#define LOW_PWER_GPS	    0		    /*1->enable low pwer, 0->disable low power*/
#define STORAGE_NAMESPACE "gps"     /*staorage nvs memory*/

#define WAIS_MS(x)      vTaskDelay(pdMS_TO_TICKS(x))
#define WAIS_S(x)       vTaskDelay(pdMS_TO_TICKS(x*1e3))
/**********************************************
 * VARIABLES
***********************************************/
TaskHandle_t GPS_Task_Handle	= NULL;

gps_t gps_data_l86;



/********************************************
 * FUNCTIONS
*******************************************/
extern "C" {
    double parse_lat_long_deg(nmea_position pos);
    void enable_pins_l86();
    void disable_pins_l86();
    static void Task_Read_GPS(void* pvParameters);
    void app_main(void);
}
/**
 * Enable L86 module pins.
 * 
 * This function enables the pins associated with the L86 module.
 * If LOW_POWER_GPS macro is defined, additional power-saving configuration is performed.
 */
void enable_pins_l86(){
    ESP_LOGI("L86", "GPS ON");
    #if LOW_POWER_GPS
        gpio_hold_dis(L86_RESET_PIN);
        gpio_deep_sleep_hold_dis();
        
        gpio_reset_pin(L86_RESET_PIN);
        gpio_set_direction(L86_RESET_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(L86_RESET_PIN, 1);
        WAIS_MS(500);
    #endif

    gpio_reset_pin(L86_FORCE_ON_PIN);
    gpio_set_direction(L86_FORCE_ON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(L86_FORCE_ON_PIN, 0);
    WAIS_MS(500);
}

/**
 * Disable L86 module pins.
 * 
 * This function disables the pins associated with the L86 module.
 * If DISABLE_GPS macro is defined, additional configuration is performed to turn off the module.
 */
void disable_pins_l86(){
    #if DISABLE_GPS 
        ESP_LOGI("L86", "GPS OFF");
        gpio_set_level(L86_RESET_PIN, 0);
        gpio_hold_en(L86_RESET_PIN);
        gpio_deep_sleep_hold_en();
    #endif
}


/**
 * Read and parse NMEA sentences to extract GPS data.
 * 
 * This Task reads NMEA sentences from the GPS module, parses them, and extracts GPS data.
 * The extracted GPS data is stored in the provided gps_t structure.
 * 
 **/
static void Task_Read_GPS(void* pvParameters){
	const static char *TAG_GPS = "NMEA";
    ESP_LOGI(TAG_GPS,"READ GPS STATUS");
    gps_t data_gps_read; // ={0};

    while (1) {
        // char fmt_buf[32];
        nmea_s *data;

        char *start;
        size_t length;
        nmea_example_read_line(&start, &length, 100 /* ms */);
        if (length == 0) {
            continue;
        }

        /* handle data */
        data = nmea_parse(start, length, 0);
        if (data == NULL) {
            ESP_LOGE(TAG_GPS,"Failed to parse the sentence!");
            printf("  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
        } else {
            if (data->errors != 0) {
                ESP_LOGE(TAG_GPS,"WARN: The sentence struct contains parse errors!\n");
            }
			switch (data->type){
			case NMEA_GPRMC:{
                #if SHOW_NMEA_SENTENCE 
				ESP_LOGI(TAG_GPS,"GPRMC sentence");
				#endif

                nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
                data_gps_read.valid = pos->valid; // validar data

                if (pos->valid){
                    data_gps_read.time = pos->date_time;
                    data_gps_read.latitude = parse_lat_long_deg(pos->latitude);
                    data_gps_read.longitude = parse_lat_long_deg(pos->longitude);
                    data_gps_read.speed = pos->gndspd_knots;    // sp
                    // ESP_LOGI(TAG_GPS,"GPS SIGNAL OK (^_^)");        
                }else{
                    ESP_LOGE(TAG_GPS,"GPS NO SIGNAL (-_-)");
                }
                
				}break;

			case NMEA_GPGGA:{
                ESP_LOGI(TAG_GPS,"GPGGA  sentence");
                nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
                printf("Number of satellites: %d\n", gpgga->n_satellites);
                printf("Altitude: %f %c\n", gpgga->altitude,  gpgga->altitude_unit);
                }break;
			default:
				break;
			}
            nmea_free(data);
        }
        
        if (data_gps_read.valid) {
            // send data//
            ESP_LOGI(TAG_GPS,"== SUCCSESFULL DATA");
        }
        WAIS_MS(200);
    }
}

/**
 * Parse latitude or longitude from NMEA position structure to decimal degrees.
 * 
 * This function parses latitude or longitude from the NMEA position structure to decimal degrees.
 * 
 * @param pos NMEA position structure containing degrees, minutes, and cardinal direction.
 * @return Decimal degrees representation of the latitude or longitude.
 */
double parse_lat_long_deg(nmea_position pos){	
    double ll = pos.degrees + pos.minutes/60.0;
	// Adjust cardinal direction (N, S, E, W)
	if (pos.cardinal == 'S' || pos.cardinal == 'W')
	{
		ll = -ll;
	}
    return ll;
}


/************************************************************
 * MAIN TASK
************************************************************/
void app_main(void){
    ESP_LOGI(TAG,"\n======INIT MAIN======\n"); 

    //=========== NVS==============================//
    // Init memory (NVS).
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    /*--------enable gps--------------*/
    nmea_example_init_interface();
    enable_pins_l86();

    xTaskCreate(Task_Read_GPS, "GPS_task", 1024*4, NULL, 10, &GPS_Task_Handle);
}


