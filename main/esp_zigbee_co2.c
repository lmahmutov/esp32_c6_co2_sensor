#include "esp_zigbee_co2.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "sensair_s8.h"
#include "driver/i2c.h"
#include "bmx280.h"
#include "ssd1306.h"
#include "zigbee_logo.h"
#include "zigbee_connected.h"
#include "zigbee_disconnected.h"
#include "zigbee_image.h"
#include "iot_button.h"
#include <time.h>
#include <sys/time.h>

/*------ Clobal definitions -----------*/
static char manufacturer[16], model[16], firmware_version[16];
bool time_updated = false, connected = false;
int lcd_timeout = 30;
uint8_t screen_number = 0; 
uint16_t temperature = 0, humidity = 0, pressure = 0, CO2_value = 0;
float temp = 0, pres = 0, hum = 0;
static ssd1306_handle_t ssd1306_dev = NULL;
SemaphoreHandle_t i2c_semaphore = NULL;
static const char *TAG = "ESP_HA_CO2_SENSOR";

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

static void button_single_click_cb(void *arg,void *usr_data)
{
    ESP_LOGI("Button boot", "Single click, change screen to %d", screen_number);
    lcd_timeout = 30;
    screen_number = screen_number + 1;
    if ( screen_number == 3)
    {
        screen_number = 0;
    }
}

static void button_long_press_cb(void *arg,void *usr_data)
{
	ESP_LOGI("Button boot", "Long press, leave & reset");
	esp_zb_factory_reset();
}

void register_button()
{
	// create gpio button
	button_config_t gpio_btn_cfg = {
		.type = BUTTON_TYPE_GPIO,
		.long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
		.short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
		.gpio_button_config = {
			.gpio_num = 9,
			.active_level = 0,
		},
	};

	button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
	if(NULL == gpio_btn) {
		ESP_LOGE("Button boot", "Button create failed");
	}

	iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb,NULL);
	iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_cb, NULL);

}

esp_err_t i2c_master_init()
{
	// Don't initialize twice
    if (i2c_semaphore != NULL)
	{
		return ESP_FAIL;
	}
        
    i2c_semaphore = xSemaphoreCreateMutex();
    if (i2c_semaphore == NULL)
	{
		return ESP_FAIL;
	}
        
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_NUM_6,
		.scl_io_num = GPIO_NUM_7,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};

	esp_err_t ret;

	ret = i2c_param_config(I2C_NUM_0, &i2c_config);
	if(ret != ESP_OK)
	{
		return ret;
	}
        
	ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	if(ret != ESP_OK)
	{
		return ret;
	}
        
    return ESP_OK;
}

/* --------- User task section -----------------*/
static void lcd_task(void *pvParameters)
{
	/* Start lcd */
	ssd1306_dev = ssd1306_create(I2C_NUM_0, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    ssd1306_draw_bitmap(ssd1306_dev, 0, 16, zigbee, 128, 32);
    ssd1306_refresh_gram(ssd1306_dev);
	vTaskDelay(1500 / portTICK_PERIOD_MS);
	
	while (1)
	{	
        switch (screen_number) {
        case 0:
            ESP_LOGI(TAG, "Screen number 0 ");
            if (CO2_value != 0)
            {
                ESP_LOGI("LCD", "data updating");
                ssd1306_refresh_gram(ssd1306_dev);
                ssd1306_clear_screen(ssd1306_dev, 0x00);
                char co2_data_str[16] = {0};
                char temp_data_str[16] = {0};
                char pres_data_str[16] = {0};
                char hum_data_str[16] = {0};
                sprintf(co2_data_str, "CO2 : %d", CO2_value);
                sprintf(temp_data_str, "Temp: %.2f", temp);
                sprintf(pres_data_str, "Pres: %.1f", pres/100);
                sprintf(hum_data_str, "Hum : %.1f", hum);
                ssd1306_draw_string(ssd1306_dev, 5, 0, (const uint8_t *)co2_data_str, 16, 1);
                ssd1306_draw_string(ssd1306_dev, 5, 16, (const uint8_t *)temp_data_str, 16, 1);
                ssd1306_draw_string(ssd1306_dev, 5, 32, (const uint8_t *)pres_data_str, 16, 1);
                ssd1306_draw_string(ssd1306_dev, 5, 48, (const uint8_t *)hum_data_str, 16, 1);
                ssd1306_draw_bitmap(ssd1306_dev, 112, 48, zigbee_image, 16, 16);
                if (connected)
                {
                    ssd1306_draw_bitmap(ssd1306_dev, 112, 0, zigbee_connected, 16, 16);
                }
                else
                {
                    ssd1306_draw_bitmap(ssd1306_dev, 112, 0, zigbee_disconnected, 16, 16);
                }
                ssd1306_refresh_gram(ssd1306_dev);
                
            }
            else
            {
                ESP_LOGI("LCD", "Waiting data");
            }
            break;
        case 1:
            ESP_LOGI(TAG, "Screen number 1 ");
            ssd1306_refresh_gram(ssd1306_dev);
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_draw_bitmap(ssd1306_dev, 112, 48, zigbee_image, 16, 16);
            if (connected)
            {
                if (time_updated)
                {
                    time_t now;
                    char strftime_buf[64];
                    struct tm timeinfo;
                    time(&now);
                    localtime_r(&now, &timeinfo);
                    strftime(strftime_buf, sizeof(strftime_buf), "%a %H:%M:%S", &timeinfo);
                    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
                    ssd1306_draw_string(ssd1306_dev, 5, 48, (const uint8_t *)strftime_buf, 16, 1);
                }

                char connected_str[16] = {0};
                char PAN_ID[16] = {0};
                char Channel[16] = {0};
                sprintf(connected_str, "  Connected");
                sprintf(PAN_ID, "PAN ID : 0x%04hx", esp_zb_get_pan_id());
                sprintf(Channel, "Channel: %d", esp_zb_get_current_channel());
                ssd1306_draw_string(ssd1306_dev, 5, 0, (const uint8_t *)connected_str, 16, 1);
                ssd1306_draw_string(ssd1306_dev, 5, 16, (const uint8_t *)PAN_ID, 16, 1);
                ssd1306_draw_string(ssd1306_dev, 5, 32, (const uint8_t *)Channel, 16, 1);
                ssd1306_draw_bitmap(ssd1306_dev, 112, 0, zigbee_connected, 16, 16);
            }
            else
            {
                char disconnected_str[16] = {0};
                sprintf(disconnected_str, " Disconnected");
                ssd1306_draw_string(ssd1306_dev, 5, 16, (const uint8_t *)disconnected_str, 16, 1);
                ssd1306_draw_bitmap(ssd1306_dev, 112, 0, zigbee_disconnected, 16, 16);
            }
            ssd1306_refresh_gram(ssd1306_dev);
            break;
        case 2:
            ESP_LOGI(TAG, "Screen number 2 ");
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_refresh_gram(ssd1306_dev);
            break;
        default:
            ESP_LOGW(TAG, "Default screen --------");
            break;	
        }
        lcd_timeout = lcd_timeout - 1;
        if (lcd_timeout <= 0) 
        {
            screen_number = 2;
        }
        else
        {
            lcd_timeout = lcd_timeout - 1;
            ESP_LOGI(TAG, "lcd_timeout %d ", lcd_timeout);
        }
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static void bmx280_task(void *pvParameters)
{
	bmx280_t* bmx280 = bmx280_create(I2C_NUM_0);

    if (!bmx280) { 
        ESP_LOGE("BMX280", "Could not create bmx280 driver.");
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    while (1)
    {
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        do {
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        } while(bmx280_isSampling(bmx280));
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));
        ESP_LOGI("BMX280", "Read Values: temp = %.2f, pres = %.1f, hum = %.1f", temp, pres/100, hum);
        temperature = (uint16_t)(temp * 100);
        humidity = (uint16_t)(hum * 100);
        pressure = (uint16_t)(pres/100);
    }
}
/*----------------------------------------*/

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

/* Manual reporting atribute to coordinator */
static void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

/* Task for update attribute value */
void update_attribute()
{
    while(1)
    {
        if (connected)
        {
            /* Write new temperature value */
            esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature, false);
            
            /* Check for error */
            if(state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Setting temperature attribute failed!");
            }
            
            /* Write new humidity value */
            esp_zb_zcl_status_t state_hum = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity, false);
            
            /* Check for error */
            if(state_hum != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Setting humidity attribute failed!");
            }

            /* Write new pressure value */
            esp_zb_zcl_status_t state_press = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pressure, false);
            
            /* Check for error */
            if(state_press != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Setting pressure attribute failed!");
            }
            
            if (CO2_value != 0)
            {
                /* Write new CO2_value value */
                esp_zb_zcl_status_t state_co2 = esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, CO2_CUSTOM_CLUSTER, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 0, &CO2_value, false);
                
                /* Check for error */
                if(state_co2 != ESP_ZB_ZCL_STATUS_SUCCESS)
                {
                    ESP_LOGE(TAG, "Setting CO2_value attribute failed!");
                }

                /* CO2 Cluster is custom and we must report it manually*/
                reportAttribute(SENSOR_ENDPOINT, CO2_CUSTOM_CLUSTER, 0, &CO2_value, 2);
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == SENSOR_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
            ESP_LOGI(TAG, "Identify pressed");
            break;
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", message->info.status,
             message->info.cluster, message->attribute.id, message->attribute.data.type,
             message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0);
    if (message->info.dst_endpoint == SENSOR_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_TIME:
            ESP_LOGI(TAG, "Server time recieved %lu", *(uint32_t*) message->attribute.data.value);
            struct timeval tv;
            tv.tv_sec = *(uint32_t*) message->attribute.data.value + 946684800 - 1080; //after adding OTA cluster time shifted to 1080 sec... strange issue ... 
            settimeofday(&tv, NULL);
            time_updated = true;
            break;
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void read_server_time()
{
    esp_zb_zcl_read_attr_cmd_t read_req;
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    read_req.attributeID = ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME;
    read_req.zcl_basic_cmd.dst_endpoint = 1;
    read_req.zcl_basic_cmd.src_endpoint = 1;
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status != ESP_OK) {
            connected = false;
            ESP_LOGW(TAG, "Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        } else {
            /* device auto start successfully and on a formed network */
            connected = true;
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            read_server_time();
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Reset device");
            esp_zb_factory_reset();
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void set_zcl_string(char *buffer, char *value)
{
    buffer[0] = (char) strlen(value);
    memcpy(buffer + 1, value, buffer[0]);
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    uint16_t undefined_value;
    undefined_value = 0x8000;
   /* basic cluster create with fully customized */
    set_zcl_string(manufacturer, MANUFACTURER_NAME);
    set_zcl_string(model, MODEL_NAME);
    set_zcl_string(firmware_version, FIRMWARE_VERSION);
    uint8_t dc_power_source;
    dc_power_source = 4;
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, firmware_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &dc_power_source);  /**< DC source. */

    /* identify cluster create with fully customized */
    uint8_t identyfi_id;
    identyfi_id = 0;
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_CMD_IDENTIFY_IDENTIFY_ID, &identyfi_id);

    /* Temperature cluster */
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    /* Humidity cluster */
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    /* Presure cluster */
    esp_zb_attribute_list_t *esp_zb_press_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_press_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_press_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_press_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    /* Time cluster */
    esp_zb_attribute_list_t *esp_zb_server_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    
    /* Custom cluster for CO2 ( standart cluster not working), solution only for HOMEd */
    const uint16_t attr_id = 0;
    const uint8_t attr_type = ESP_ZB_ZCL_ATTR_TYPE_U16;
    const uint8_t attr_access = ESP_ZB_ZCL_ATTR_MANUF_SPEC | ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;

    esp_zb_attribute_list_t *custom_co2_attributes_list = esp_zb_zcl_attr_list_create(CO2_CUSTOM_CLUSTER);
    esp_zb_custom_cluster_add_custom_attr(custom_co2_attributes_list, attr_id, attr_type, attr_access, &undefined_value);

    /** Create ota client cluster with attributes.
     *  Manufacturer code, image type and file version should match with configured values for server.
     *  If the client values do not match with configured values then it shall discard the command and
     *  no further processing shall continue.
     */
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_downloaded_file_ver = OTA_UPGRADE_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    /** add client parameters to ota client cluster */
    esp_zb_ota_upgrade_client_parameter_t ota_client_parameter_config = {
        .query_timer = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,          /* time interval for query next image request command */
        .hardware_version = OTA_UPGRADE_HW_VERSION,                           /* version of hardware */
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,                           /* maximum data size of query block image */
    };
    void *ota_client_parameters = esp_zb_ota_client_parameter(&ota_client_parameter_config);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_PARAMETER_ID, ota_client_parameters);

    /* Create full cluster list enabled on device */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list, esp_zb_press_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list, esp_zb_server_time_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, custom_co2_attributes_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, SENSOR_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);

    /* END */
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
	register_button();
    ESP_ERROR_CHECK(i2c_master_init());
	uart_init();
    //sensair_get_info();
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
	xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 1, NULL);
	xTaskCreate(bmx280_task, "bmx280_task",  4096, NULL, 2, NULL);
	xTaskCreate(sensair_tx_task, "sensor_tx_task", 4096, NULL, 3, NULL);
    xTaskCreate(sensair_rx_task, "sensor_rx_task", 4096, NULL, 4, NULL);
    xTaskCreate(update_attribute, "Update_attribute_value", 4096, NULL, 5, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 6, NULL);
}
