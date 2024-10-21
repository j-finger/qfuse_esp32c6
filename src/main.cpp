#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h" 
#include "driver/uart.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_system.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_sntp.h"

#include "mqtt_client.h"


#include <stdio.h>
#include <string>
#include <string.h>
#include <time.h>
#include <ctime>
#include <cstring>
#include <cstdio>

#define JSON_NOEXCEPTION
#include "json.hpp"
using json = nlohmann::json;

// Declare device ID
static const char *TAG = "ESP_coms";


/* ----- WIFI MACROS ----- */

#include "config.h" // Include Wi-Fi SSID and Password
#define MAXIMUM_RETRY  5
static int s_retry_num = 0;

/* ----- MQTT MACROS ----- */
static esp_mqtt_client_handle_t mqtt_client = NULL;
#define MQTT_URI       "mqtt://192.168.86.20:1883"
// #define MQTT_URI       "mqtt://192.168.4.1:1883"

/* ----- UART MACROS ----- */
// Declare UART Pins, Baud Rate, Buffer Size and Queue
#define UART_NUM UART_NUM_1
#define TXD_PIN GPIO_NUM_16
#define RXD_PIN GPIO_NUM_17
#define BAUD_RATE 921600
#define UART_BUFFER_SIZE (16384) // UART Buffer Sizes

static QueueHandle_t uart_queue; // UART Event Queue
static QueueHandle_t json_queue; // JSON Queue


#define LED_PIN GPIO_NUM_15
/* ----- WIFI ----- */

// Wi-Fi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to the AP...");
        } else {
            ESP_LOGI(TAG, "Failed to connect to the AP");
        }
        ESP_LOGI(TAG, "Disconnected from Wi-Fi");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// Initialize Wi-Fi in Station Mode
void wifi_init_sta(void){
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create a default Wi-Fi station
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi with default configurations
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register the event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler,
                                               NULL));

    // Configure the Wi-Fi connection and security
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    // Set the Wi-Fi mode and configuration
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization finished.");
}

// Function to Wait for IP Address
void wait_for_ip()
{
    ESP_LOGI(TAG, "Waiting for IP address...");

    esp_netif_ip_info_t ip_info;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif == NULL) {
            ESP_LOGE(TAG, "Failed to get network interface handle.");
            continue;
        }
        esp_netif_get_ip_info(netif, &ip_info);
        if (ip_info.ip.addr != 0) {
            break;
        }
        ESP_LOGI(TAG, "IP not yet obtained.");
    }

    ESP_LOGI(TAG, "Connected with IP Address: " IPSTR, IP2STR(&ip_info.ip));
}


/* ----- TIME ----- */

// Set Timezone to AEST-10
void set_timezone(void)
{
    setenv("TZ", "AEST-10", 1);
    tzset();
}

// Initialize SNTP
void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");  // You can specify other NTP servers if needed
    esp_sntp_init();
}

// Obtain Time from NTP Server
void obtain_time(void)
{
    initialize_sntp();

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {};
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2023 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == retry_count) {
        ESP_LOGI(TAG, "Failed to get time over NTP.");
    } else {
        ESP_LOGI(TAG, "System time is set.");
    }
}

// Print the Current Time in POSIX format
void print_current_time(void)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
}

/* ----- JSON ----- */

// Function to append the current UNIX epoch time to a JSON object
void json_append_time(json& json_obj) {
    // Get current time as UNIX epoch
    time_t current_time = time(nullptr);
    json_obj["time"] = current_time;
}



/* ----- MQTT ----- */

// Function to subscribe to required MQTT topics
void mqtt_sub_topics(esp_mqtt_client_handle_t client) {
    // Subscribe to necessary topics
    esp_mqtt_client_subscribe(client, "sensor/settings", 1);
    esp_mqtt_client_subscribe(client, "sensor/data", 1);
    esp_mqtt_client_subscribe(client, "sensor/logs", 1);

    ESP_LOGI(TAG, "Subscribed to topics: 'sensor/settings', 'sensor/data', 'sensor/logs'");
}

// Function to publish JSON data to a specified MQTT topic
void mqtt_publish(esp_mqtt_client_handle_t client, const std::string& topic, json& json_obj) {
    // Append current time to the JSON object
    json_append_time(json_obj);

    // Serialize JSON object to string
    std::string json_str = json_obj.dump();

    // Publish JSON string to the specified MQTT topic
    esp_mqtt_client_publish(client, topic.c_str(), json_str.c_str(), 0, 1, 0);
    ESP_LOGI(TAG, "Published data to topic '%s'", topic.c_str());
}

// Function to handle MQTT errors
void mqtt_handle_error(esp_mqtt_event_handle_t event) {
    ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
        ESP_LOGE(TAG, "Last tls stack error: 0x%x", event->error_handle->esp_tls_stack_err);
        ESP_LOGE(TAG, "Last tls certificate verification flags: 0x%x", event->error_handle->esp_tls_cert_verify_flags);
        ESP_LOGE(TAG, "Last socket errno: %d", event->error_handle->esp_transport_sock_errno);
    }
    // Add additional error handling as needed
}

// Function to handle incoming MQTT messages
void mqtt_handle_message(esp_mqtt_event_handle_t event) {
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    std::string topic(event->topic, event->topic_len);
    std::string data(event->data, event->data_len);

    ESP_LOGI(TAG, "Received message on topic: %s", topic.c_str());
    ESP_LOGI(TAG, "Message data: %s", data.c_str());

    // Process the message as needed
    // For now, we can just log it or handle settings updates
}

// Function to handle MQTT connection events
void mqtt_handle_connection(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;

    if (event->event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        // Subscribe to MQTT topics
        mqtt_sub_topics(client);

        // Optionally publish initial data if needed
        // For example, send a connection log
        json log_json;
        log_json["device"] = "E46338809B472231"; // Replace with your device ID
        log_json["log"] = "Device connected to MQTT broker";
        json_append_time(log_json);
        mqtt_publish(client, "sensor/logs", log_json);

    } else if (event->event_id == MQTT_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");

        // Handle disconnection logic if necessary
    }
}

// MQTT Event Handler Callback
static void mqtt_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
        case MQTT_EVENT_DISCONNECTED:
            mqtt_handle_connection(event);
            break;

        case MQTT_EVENT_DATA:
            mqtt_handle_message(event);
            break;

        case MQTT_EVENT_ERROR:
            mqtt_handle_error(event);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        default:
            ESP_LOGI(TAG, "Unhandled MQTT event id: %d", event->event_id);
            break;
    }
}

// Start MQTT Client
void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_URI;  // Set each field individually

    mqtt_cfg.outbox.limit = 65536;

    mqtt_cfg.buffer.size = 16384;     // Input buffer size
    mqtt_cfg.buffer.out_size = 16384; // Output buffer size

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    // Register the event handler for specific MQTT events
    esp_err_t err;
    err = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_CONNECTED, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT_EVENT_CONNECTED handler");
    }

    err = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT_EVENT_DISCONNECTED handler");
    }

    err = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_PUBLISHED, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT_EVENT_PUBLISHED handler");
    }


    // Start the MQTT client
    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
    }
}


/* ----- UART ----- */

// Function to initialize UART
void uart_initialize(void) {
    const uart_port_t uart_num = UART_NUM;

    uart_config_t uart_config = {}; // Zero-initialize the uart_config_t structure

    // Set the required UART parameters
    uart_config.baud_rate = BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config)); // Configure UART parameters
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // Set UART pins
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_queue, 0)); // Install UART driver using an event queue here

    uart_enable_rx_intr(uart_num); // Configure UART to receive interrupt on RX full
}

// Function to send data over UART
void uart_send(const char* data) {
    const uart_port_t uart_num = UART_NUM;
    size_t len = strlen(data);
    uart_write_bytes(uart_num, data, len);
}

// Function to read data over UART
void uart_read(char* message, size_t max_length) {
    const uart_port_t uart_num = UART_NUM;
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));

    if (length > 0) { // If data is available, read it
        int read_len = uart_read_bytes(uart_num, (uint8_t*)message, length > max_length ? max_length : length, 100 / portTICK_PERIOD_MS);
        message[read_len] = '\0'; // Null-terminate the string
    } else {
        message[0] = '\0'; // No data received
    }
}


static void json_processing_task(void *pvParameters)
{
    std::string* json_str;
    while (true) {
        if (xQueueReceive(json_queue, &json_str, portMAX_DELAY)) {
            // Optional: Limit logging
            ESP_LOGI(TAG, "Processing JSON of length %d", json_str->length());

            if (json::accept(*json_str)) {
                json received_json = json::parse(*json_str);
                mqtt_publish(mqtt_client, "sensor/data", received_json);
            } else {
                ESP_LOGE(TAG, "JSON Parse Error: Invalid JSON format");
            }
            delete json_str; // Free allocated memory
        }
    }
}


// // Function to handle UART events
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    char incoming_byte;
    std::string uart_buffer;

    while (true) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int rx_bytes = event.size;
                    uint8_t* data = (uint8_t*) malloc(rx_bytes);
                    int read_bytes = uart_read_bytes(UART_NUM, data, rx_bytes, portMAX_DELAY);
                    for (int i = 0; i < read_bytes; i++) {
                        incoming_byte = data[i];
                        if (incoming_byte == '\n') {
                            if (!uart_buffer.empty()) {
                                // Allocate memory for the JSON string
                                std::string* json_str = new std::string(uart_buffer);
                                if (xQueueSend(json_queue, &json_str, portMAX_DELAY) != pdPASS) {
                                    ESP_LOGE(TAG, "Failed to enqueue JSON string");
                                    delete json_str; // Free memory if enqueue fails
                                }
                                uart_buffer.clear();
                            }
                        } else {
                            uart_buffer += incoming_byte;
                        }
                    }
                    free(data);
                    break;
                }
                // Handle other UART events as needed
                default:
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}



// // Function to handle UART events
// static void uart_event_task(void *pvParameters)
// {
//     uart_event_t event;
//     char incoming_byte;
//     std::string uart_buffer = ""; // Initialize an empty buffer to accumulate incoming data

//     while (true) {
//         // Wait for UART event.
//         if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
//             switch (event.type) {
//                 case UART_DATA: {
//                     // Read incoming bytes one by one
//                     while (uart_read_bytes(UART_NUM, (uint8_t *)&incoming_byte, 1, 0) > 0) {
//                         if (incoming_byte == '\n') {
//                             // End of JSON message
//                             if (!uart_buffer.empty()) {
//                                 // ESP_LOGI(TAG, "Received complete JSON: %s", uart_buffer.c_str());
//                                 printf("Received complete JSON\n");
//                                 // Check if the received string is valid JSON
//                                 if (json::accept(uart_buffer)) {
//                                     // Parse the JSON string
//                                     json received_json = json::parse(uart_buffer);
                                    
//                                     // Process the JSON object as needed
//                                     // For example, publish to MQTT
//                                     mqtt_publish(mqtt_client, "sensor/data", received_json);
//                                 }
//                                 else {
//                                     // ESP_LOGE(TAG, "JSON Parse Error: Invalid JSON format");
//                                     printf("JSON Parse Error: Invalid JSON format\n");
//                                 }
                                
//                                 // Clear the buffer for the next message
//                                 uart_buffer.clear();
//                             }
//                         }
//                         else {
//                             // Accumulate the incoming byte
//                             uart_buffer += incoming_byte;
//                         }
//                     }
//                     break;
//                 }
//                 // Handle other UART events as needed
//                 case UART_FIFO_OVF:
//                     ESP_LOGI(TAG, "UART FIFO Overflow");
//                     uart_flush_input(UART_NUM);
//                     xQueueReset(uart_queue);
//                     break;

//                 case UART_BUFFER_FULL:
//                     ESP_LOGI(TAG, "UART Buffer Full");
//                     uart_flush_input(UART_NUM);
//                     xQueueReset(uart_queue);
//                     break;

//                 case UART_BREAK:
//                     ESP_LOGI(TAG, "UART Break");
//                     break;

//                 case UART_PARITY_ERR:
//                     ESP_LOGI(TAG, "UART Parity Error");
//                     break;

//                 case UART_FRAME_ERR:
//                     ESP_LOGI(TAG, "UART Frame Error");
//                     break;

//                 default:
//                     ESP_LOGI(TAG, "UART event type: %d", event.type);
//                     break;
//             }
//         }
//     }
//     vTaskDelete(NULL);
// }


// Function to initialize the LED pin
void init_led_pin(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << LED_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    gpio_config(&io_conf);
}

// Function to blink the LED
void blink_led(void) {
    gpio_set_level(LED_PIN, 0);  // Pull low
    vTaskDelay(250 / portTICK_PERIOD_MS);  // Delay 1 second
    gpio_set_level(LED_PIN, 1);  // Pull high
}

/* ----- Main ----- */


extern "C" void app_main(void)
{
    
    init_led_pin();
    



    blink_led();
    // Sleep for 8 seconds before starting
    vTaskDelay(8000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Starting ESP32C6 UART Example");
    
    // Wi-Fi Initialization
    blink_led();
    wifi_init_sta(); // Initialize Wi-Fi
    blink_led();
    wait_for_ip();   // Wait for IP address

    // Time Synchronization
    blink_led();
    set_timezone();  // Set the timezone
    obtain_time();   // Obtain time from NTP server
    print_current_time(); // Print the current time

    // MQTT Initialization
    blink_led();
    mqtt_app_start(); // Start the MQTT client

    // Log MQTT initialization to sensor/log
    blink_led();
    json log_json;
    log_json["device"] = "E46338809B472231"; // Replace with your device ID
    log_json["time"] = time(nullptr);
    log_json["log"] = "MQTT client started";
    mqtt_publish(mqtt_client, "sensor/logs", log_json);
    ESP_LOGI(TAG, "MQTT client started");

    // UART Initialization
    blink_led();
    uart_initialize(); // Initialize UART

    blink_led();
    // Initialize the queue before starting tasks
    json_queue = xQueueCreate(10, sizeof(std::string*)); // Adjust the queue length as needed
    if (json_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON queue");
    }
    
    // Start UART Event Task
    blink_led();
    xTaskCreate(json_processing_task, "json_processing_task", 16384, NULL, 12, NULL);
    // Start UART Event Task
    blink_led();
    xTaskCreate(uart_event_task, "uart_event_task", 16384, NULL, 10, NULL);
    
    gpio_set_level(LED_PIN, 0);  // Pull low

    uart_send("Clear\nConnected\n");  // Implementation of the TODO    printf("Clear\n");
    printf("Clear\nConnected\n");

    // Delete the main task if not needed
    vTaskDelete(NULL);
}
