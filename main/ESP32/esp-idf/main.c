#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "time.h"
#include "cJSON.h"
#include <inttypes.h> // Add this at the top of your file

// Wi-Fi Configuration
#define WIFI_SSID "Virus"
#define WIFI_PASS "1234567890"

// MQTT Configuration
#define MQTT_BROKER_URI "mqtt://192.168.35.210"
#define MQTT_TOPIC "esp32/test"

// Konfigurasi Flowmeter
#define FLOW_SENSOR_PIN1 GPIO_NUM_34
#define FLOW_SENSOR_PIN2 GPIO_NUM_33
#define PULSES_PER_LITER 450

// Konfigurasi Ultrasonik
#define TRIGGER_PIN GPIO_NUM_27
#define ECHO_PIN GPIO_NUM_32

// Konfigurasi SD Card
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_4
#define MOUNT_POINT "/sdcard"

static const char *TAG = "IRRIGATION";

volatile uint32_t pulse_count1 = 0;
volatile uint32_t pulse_count2 = 0;
float flow_rate1 = 0.0;
float flow_rate2 = 0.0;
float total_volume1 = 0.0;
float total_volume2 = 0.0;
sdmmc_card_t *card;

// Deklarasi Fungsi
esp_err_t init_sdcard();
float measure_distance();
void mqtt_publish_task(void *pvParameters);

// Threshold Jarak
#define DISTANCE_THRESHOLD 20 // 20 cm
#define RELAY_PIN GPIO_NUM_26 // Relay pada PIN GPIO 4
char relay_status[10] = "OFF"; // Variable untuk menyimpan status Relay

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            esp_mqtt_client_publish(event->client, MQTT_TOPIC, "ESP32 Connected", 0, 1, 0);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Subscribed to Topic: %d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Message Published");
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Received Data: %.*s", event->data_len, event->data);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT Event Error");
            break;

        default:
            ESP_LOGI(TAG, "Other Event: %" PRId32, event_id); // Use PRId32 for int32_t
            break;
    }
}

// MQTT Configuration
void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    // Membuat task untuk publish data ke MQTT
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, client, 10, NULL);
}


// Wi-Fi Event Handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying WiFi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        esp_ip4_addr_t ip_addr = event->ip_info.ip; // Dereference the pointer

        char ip_str[16]; // Buffer to store the IP address string
        esp_ip4addr_ntoa(&ip_addr, ip_str, sizeof(ip_str)); // Convert IP to string
        ESP_LOGI(TAG, "Got IP: %s", ip_str); // Log the IP address
    }
}

// Initialize Wi-Fi
void wifi_init_sta() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        NULL);
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler,
                                        NULL,
                                        NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

// ISR untuk Flowmeter
static void IRAM_ATTR flow_isr_handler1(void *arg) {
    pulse_count1++;
}

static void IRAM_ATTR flow_isr_handler2(void *arg) {
    pulse_count2++;
}

// Task Menghitung Flow Rate dan Volume
void calculate_flow_task(void *pvParameters) {
    uint32_t last_pulse_count1 = 0;
    uint32_t last_pulse_count2 = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000);  // Set interval 1 detik

    while (1) {
        vTaskDelayUntil(&lastWakeTime, interval);

        // Ambil selisih pulsa dalam 1 detik
        uint32_t pulses1 = pulse_count1 - last_pulse_count1;
        last_pulse_count1 = pulse_count1;

        uint32_t pulses2 = pulse_count2 - last_pulse_count2;
        last_pulse_count2 = pulse_count2;

        // Menghitung Flow Rate dan Volume Total untuk Flow Meter 1
        float flow_rate1 = (pulses1 / (float)PULSES_PER_LITER) * 60.0;
        total_volume1 += (pulses1 / (float)PULSES_PER_LITER);

        // Menghitung Flow Rate dan Volume Total untuk Flow Meter 2
        float flow_rate2 = (pulses2 / (float)PULSES_PER_LITER) * 60.0;
        total_volume2 += (pulses2 / (float)PULSES_PER_LITER);

        // Tampilkan hanya jika ada aliran
        if (flow_rate1 > 0) {
            ESP_LOGI(TAG, "Flow Meter 1: %.2f L/min, Total Volume: %.2f L", flow_rate1, total_volume1);
        }

        if (flow_rate2 > 0) {
            ESP_LOGI(TAG, "Flow Meter 2: %.2f L/min, Total Volume: %.2f L", flow_rate2, total_volume2);
        }
    }
}

// Task Logging ke SD Card
void log_to_sdcard_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        char message[64];
        int64_t timestamp = esp_timer_get_time() / 1000;
        
        snprintf(message, sizeof(message), "%lld;%.2f;%.2f;%.2f;%.2f;%.2f;%s", 
                 timestamp, flow_rate1, total_volume1, flow_rate2, total_volume2, measure_distance(), relay_status);

        FILE *f = fopen(MOUNT_POINT"/log.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            continue;
        }
        fprintf(f, "%s\n", message);
        fclose(f);
        ESP_LOGI(TAG, "Logged: %s", message);
    }
}

void mqtt_publish_task(void *pvParameters) {
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t) pvParameters;

    while (1) {
        cJSON *json_data = cJSON_CreateObject();
        cJSON_AddNumberToObject(json_data, "flow_rate1", flow_rate1);
        cJSON_AddNumberToObject(json_data, "total_volume1", total_volume1);
        cJSON_AddNumberToObject(json_data, "flow_rate2", flow_rate2);
        cJSON_AddNumberToObject(json_data, "total_volume2", total_volume2);
        cJSON_AddNumberToObject(json_data, "distance", measure_distance());
        cJSON_AddStringToObject(json_data, "relay_status", relay_status);

        char *json_string = cJSON_Print(json_data);

        esp_mqtt_client_publish(client, "esp32/data", json_string, 0, 1, 0);

        cJSON_Delete(json_data);
        free(json_string);

        vTaskDelay(pdMS_TO_TICKS(5000));  // Kirim setiap 5 detik
    }
}

// Fungsi untuk menyimpan status ke variable string
void update_relay_status(bool is_on) {
    if (is_on && strcmp(relay_status, "OFF") == 0) {
        strcpy(relay_status, "ON");
        ESP_LOGI(TAG, "Relay Status Updated: %s", relay_status);
    }
    else if (!is_on && strcmp(relay_status, "ON") == 0) {
        strcpy(relay_status, "OFF");
        ESP_LOGI(TAG, "Relay Status Updated: %s", relay_status);
    }
}

// Task Ultrasonik
void ultrasonic_task(void *pvParameters) {
    gpio_set_level(TRIGGER_PIN, 0);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        float distance = measure_distance();
        ESP_LOGI(TAG, "Distance: %.2f cm", distance);
        // Kontrol Relay Berdasarkan Jarak
        if (distance > DISTANCE_THRESHOLD) {
            gpio_set_level(RELAY_PIN, 1); // Matikan Relay
            update_relay_status(false); // Update status ke OFF
            printf("Relay OFF\n");
        } else {
            gpio_set_level(RELAY_PIN, 0); // Hidupkan Relay
            update_relay_status(true);
            printf("Relay ON\n");
        }
    }
}

// Fungsi Mengukur Jarak Ultrasonik
float measure_distance() {
    int64_t start_time = 0, end_time = 0;  // Inisialisasi dengan 0
    float distance = 0.0;

    // Mulai pengukuran
    gpio_set_level(TRIGGER_PIN, 0);
    esp_rom_delay_us(2);

    // Beri pulsa trigger
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIGGER_PIN, 0);

    // Tunggu echo naik
    while (gpio_get_level(ECHO_PIN) == 0) {
        start_time = esp_timer_get_time();
    }

    // Tunggu echo turun
    while (gpio_get_level(ECHO_PIN) == 1) {
        end_time = esp_timer_get_time();
    }

    // Hitung jarak
    distance = (float)(end_time - start_time) * 0.034 / 2;
    return distance;
}


// Inisialisasi SD Card
esp_err_t init_sdcard() {
    ESP_LOGI(TAG, "Initializing SD card");
    vTaskDelay(pdMS_TO_TICKS(2000));
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
    };

    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");
    sdmmc_card_print_info(stdout, card);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return ESP_OK;
}

// Fungsi Utama
void app_main(void) {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi
    wifi_init_sta();

    mqtt_app_start();

    // Inisialisasi GPIO Flowmeter
    // Konfigurasi GPIO untuk Flow Meter 1
    gpio_config_t io_conf1 = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FLOW_SENSOR_PIN1),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf1);

    // Konfigurasi GPIO untuk Flow Meter 2
    gpio_config_t io_conf2 = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FLOW_SENSOR_PIN2),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf2);

    // Instal ISR untuk kedua sensor
    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_SENSOR_PIN1, flow_isr_handler1, (void*) FLOW_SENSOR_PIN1);
    gpio_isr_handler_add(FLOW_SENSOR_PIN2, flow_isr_handler2, (void*) FLOW_SENSOR_PIN2);

    // Inisialisasi GPIO Ultrasonik
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

    // Setup Relay Pin as Output
    gpio_reset_pin(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_PIN, 1); // Matikan relay di awal (OFF)

    // Inisialisasi SD Card
    if (init_sdcard() != ESP_OK) {
        return;
    }

    // Buat Task
    xTaskCreate(calculate_flow_task, "calculate_flow_task", 2048, NULL, 10, NULL);
    xTaskCreate(log_to_sdcard_task, "log_to_sdcard_task", 4096, NULL, 5, NULL);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 10, NULL);
}
