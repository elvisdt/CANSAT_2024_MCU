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

/**********************************************
 * DEFINES
***********************************************/
#define TAG "MAIN"


#define DEBUG_MAIN          0           /*1->enable debug   ,  0->disable debug*/
#define LOW_PWER_GPS	    0		    /*1->enable low pwer, 0->disable low power*/
#define STORAGE_NAMESPACE "gps"     /*staorage nvs memory*/

#define wait_ms(x)      vTaskDelay(x/portTICK_PERIOD_MS)
/**********************************************
 * VARIABLES
***********************************************/

gps_t gps_data_l86;
gps_t gps_data_nvs;


/********************************************
 * FUNCTIONS
*******************************************/
void read_and_parse_nmea(gps_t *data_gps);
float parse_lat_long_deg(nmea_position pos);
void update_gps_data(int lec_max);
void enable_pins_l86();
void disable_pins_l86();

esp_err_t save_gps_data(gps_t* data);
esp_err_t read_gps_data(gps_t* data);
esp_err_t print_what_saved(void);



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
        wait_ms(500);
    #endif

    gpio_reset_pin(L86_FORCE_ON_PIN);
    gpio_set_direction(L86_FORCE_ON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(L86_FORCE_ON_PIN, 0);
    wait_ms(500);
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
 * Save GPS data to non-volatile storage.
 * 
 * This function saves GPS data to the non-volatile storage.
 * 
 * @param data Pointer to the GPS data structure to be saved.
 * @return
 *     - ESP_OK if the operation is successful.
 *     - Other error codes if the operation fails.
 */
esp_err_t save_gps_data(gps_t* data){
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write
    err = nvs_set_blob(my_handle, "gps_data", data, sizeof(gps_t));
    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

/**
 * Read GPS data from non-volatile storage.
 * 
 * This function reads GPS data from the non-volatile storage.
 * 
 * @param data Pointer to the GPS data structure where the read data will be stored.
 * @return
 *     - ESP_OK if the operation is successful.
 *     - ESP_ERR_NVS_NOT_FOUND if the requested key is not found in the storage.
 *     - Other error codes if the operation fails.
 */
esp_err_t read_gps_data(gps_t* data){
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    size_t required_size = sizeof(gps_t);
    err = nvs_get_blob(my_handle, "gps_data", data, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}


/**
 * Print saved GPS data from non-volatile storage.
 * 
 * This function prints the GPS data saved in the non-volatile storage.
 * 
 * @return
 *     - ESP_OK if the operation is successful.
 *     - ESP_ERR_NVS_NOT_FOUND if the requested key is not found in the storage.
 *     - Other error codes if the operation fails.
 */
esp_err_t print_what_saved(void){

    nvs_handle_t my_handle;
    esp_err_t err;
    
    ESP_LOGI(TAG, "=====PRINT DATA TO NVS====");
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    
    // Read run time blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, "gps_data", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    
    printf("GPS data saved:\n");
    if (required_size == 0) {
        printf("  ->Nothing saved yet!\n");
    } else {
        gps_t* data = malloc(required_size);
        err = nvs_get_blob(my_handle, "gps_data", data, &required_size);
        if (err != ESP_OK) {
            free(data);
            return err;
        }
        char buf_time[30];
        strftime(buf_time, sizeof(buf_time), "%d %b %T %Y", &data->time);
        printf("  -> Date & Time: %s\n", buf_time);
        printf("  -> latitud:  %f \n",data->latitude);
        printf("  -> longitud: %f \n",data->longitude);
    }

    printf("\n");
    // Close
    nvs_close(my_handle);
    return ESP_OK;
}


/*---------------------GPS-------------------------------*/
/**
 * Update GPS data and save to non-volatile storage.
 * 
 * This function updates GPS data and saves it to non-volatile storage.
 * If the data is successfully updated, it is saved to NVS.
 * If the update fails or no valid GPS data is obtained, the previous GPS data from NVS is used.
 * 
 * @param lec_max Maximum number of attempts to read GPS data.
 */
void update_gps_data(int lec_max){

    int64_t start_time   = esp_timer_get_time();
    int64_t current_time = start_time ;
    esp_err_t ret;

    int num_read_GPS = 0;
    do{
        read_and_parse_nmea(&gps_data_l86); //gps data
        vTaskDelay(800/portTICK_PERIOD_MS);
        num_read_GPS++; 
    } while (gps_data_l86.valid != 1 && num_read_GPS <= lec_max);

    ESP_LOGI("GPS","READ ATTEMPTS: %d",num_read_GPS);
    vTaskDelay(500/portTICK_PERIOD_MS);

    if (gps_data_l86.valid){
        // Save the new GPS data to NVS.
        ret = save_gps_data(&gps_data_l86);
        if (ret != ESP_OK) {
            printf("Error (%s) saving GPS data to NVS!\n", esp_err_to_name(ret));
        }

    }else{
        // Use previous GPS value from NVS.
        gps_data_l86 = gps_data_nvs;
    }
    current_time = esp_timer_get_time(); 
    printf("  Read data interval: %.2f sec\n\n", (double)(current_time-start_time)/1E6); 

}



/**
 * Read and parse NMEA sentences to extract GPS data.
 * 
 * This function reads NMEA sentences from the GPS module, parses them, and extracts GPS data.
 * The extracted GPS data is stored in the provided gps_t structure.
 * 
 * @param data_gps Pointer to the gps_t structure where the parsed GPS data will be stored.
 */
void read_and_parse_nmea(gps_t *data_gps)
{
	const static char *TAG_GPS = "NMEA";
    ESP_LOGI(TAG_GPS,"READ GPS STATUS");
	bool continue_read = true;
    int num_GPTXT = 0;

    // Initialize data as invalid
    data_gps ->valid = 0;

	// Current time
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xDelay =  (PERIOD_GPS) / portTICK_PERIOD_MS;

    do {
		if (xTaskGetTickCount() - xLastWakeTime >= xDelay) {
            ESP_LOGW(TAG_GPS, "READ BREAK AFTER: %.2f sec",(xTaskGetTickCount() - xLastWakeTime)/100.0);
            continue_read=false;
            break;
        }

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
                data_gps->valid = pos->valid;

                if (pos->valid)
                {
                    data_gps->time = pos->date_time;
                    data_gps->latitude = parse_lat_long_deg(pos->latitude);
                    data_gps->longitude = parse_lat_long_deg(pos->longitude);

                    ESP_LOGI(TAG_GPS,"GPS SIGNAL OK (^_^)");
                    #ifdef PROTOCOL_QUECLINK 
                        continue_read=false;
                    #endif          
                }else{
                    ESP_LOGE(TAG_GPS,"GPS NO SIGNAL (-_-)");
     
                }
                
				}break;

			case NMEA_GPTXT:{
                num_GPTXT +=1;
                #if SHOW_NMEA_SENTENCE 
				ESP_LOGW(TAG_GPS,"GPTXT sentence");
				nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;
                printf("  ID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
                printf("  %s\n", gptxt->text);
				#endif

                if (num_GPTXT>=2)
                {
                    ESP_LOGI(TAG_GPS,"COMPLETE_READ");
                    continue_read=false;
                }
                
                }break;
			default:
				break;
			}
            nmea_free(data);
        }
    }while(continue_read);

    vTaskDelay(100/portTICK_PERIOD_MS);
}

/**
 * Parse latitude or longitude from NMEA position structure to decimal degrees.
 * 
 * This function parses latitude or longitude from the NMEA position structure to decimal degrees.
 * 
 * @param pos NMEA position structure containing degrees, minutes, and cardinal direction.
 * @return Decimal degrees representation of the latitude or longitude.
 */
float parse_lat_long_deg(nmea_position pos){	
    float ll = pos.degrees + pos.minutes/60.0;
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

    // Lee los datos del GPS desde NVS.
    printf("\n Read NVS data \n");
    ret = read_gps_data(&gps_data_nvs);
    if (ret != ESP_OK) {
        printf("Error (%s) al leer datos de GPS desde NVS!\n", esp_err_to_name(ret));
    } else {
        printf("Datos de GPS leídos correctamente desde NVS \n\n");
    }

    //------------active read GPS-------------/
    for (;;) {
        update_gps_data(1);
        wait_ms(1000);
    }
}


