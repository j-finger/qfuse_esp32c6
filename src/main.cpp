// main.cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include <stdio.h>
#include <string>
#include <string.h>
#include <time.h>
#include <ctime>
#include <cstring>
#include <cstdio>

#include "wifi/wifi.h"
#include "time/timesync.h"

// Declare device ID
static const char *TAG = "ESP_coms";

/* ----- MQTT MACROS ----- */
static esp_mqtt_client_handle_t mqtt_client = NULL;
#define MQTT_URI       "mqtt://192.168.86.20:1883"
#define MQTT_BUFF_IN_SIZE (2048)
#define MQTT_BUFF_OUT_SIZE (26384)
#define MQTT_OUTBOX_SIZE (16384)

/* ----- UART MACROS ----- */
// Declare UART Pins, Baud Rate, Buffer Size and Queue
#define UART_NUM UART_NUM_1
#define TXD_PIN GPIO_NUM_16
#define RXD_PIN GPIO_NUM_17
#define BAUD_RATE 921600
#define UART_RX_BUFFER_SIZE (16384) // Reduced RX buffer size
#define UART_TX_BUFFER_SIZE (16384)  // Reduced TX buffer size

static QueueHandle_t uart_queue; // UART Event Queue
static QueueHandle_t json_queue; // JSON Queue

#define LED_PIN GPIO_NUM_15

#include <sys/time.h>  // Include necessary header for settimeofday

bool time_synced = false;  // Global variable to indicate if time is synchronized

void set_system_time(int64_t epoch_time) {
    struct timeval tv;
    tv.tv_sec = epoch_time;
    tv.tv_usec = 0;

    int res = settimeofday(&tv, NULL);
    if (res != 0) {
        ESP_LOGE(TAG, "Failed to set system time");
    } else {
        ESP_LOGI(TAG, "System time set to: %lld", (long long)epoch_time);
        time_synced = true;
    }
}

/* ----- JSON Manipulation ----- */

// Function to append the current UNIX epoch time to a JSON string
void json_append_time(std::string& json_str) {
    // Get current time as UNIX epoch
    time_t current_time = time(nullptr);
    // Find the last occurrence of '}'
    size_t pos = json_str.rfind('}');
    if (pos != std::string::npos) {
        // Remove the last '}'
        json_str.erase(pos, 1);
        // Append ',"time":<current_time>}'
        char time_str[64];
        snprintf(time_str, sizeof(time_str), ",\"time\":%lld}", (long long)current_time);
        json_str += time_str;
    } else {
        // No '}' found, append ',"time":<current_time>'
        char time_str[64];
        snprintf(time_str, sizeof(time_str), ",\"time\":%lld}", (long long)current_time);
        json_str += time_str;
    }
}

/* ----- MQTT ----- */

// Function to subscribe to required MQTT topics
void mqtt_sub_topics(esp_mqtt_client_handle_t client) {
    esp_mqtt_client_subscribe(client, "time/response", 1);
    ESP_LOGI(TAG, "Subscribed to topic: 'time/response'");
}

// Function to publish JSON data to a specified MQTT topic
void mqtt_publish(esp_mqtt_client_handle_t client, const std::string& topic, std::string& json_str) {
    // Append current time to the JSON string
    json_append_time(json_str);

    // Publish JSON string to the specified MQTT topic
    esp_mqtt_client_publish(client, topic.c_str(), json_str.c_str(), json_str.length(), 1, 0);
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

    if (topic == "time/response") {
        ESP_LOGI(TAG, "Handling time response");
        int64_t time_epoch = 0;
        const char* time_key = "\"time\":";
        const char* pos = strstr(data.c_str(), time_key);
        if (pos != NULL) {
            pos += strlen(time_key);
            // Parse the time value
            time_epoch = atoll(pos);
            set_system_time(time_epoch);
        } else {
            ESP_LOGE(TAG, "Time response does not contain 'time' field");
        }
    }
    // Handle other topics as needed
}

// Function to handle MQTT connection events
void mqtt_handle_connection(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;

    if (event->event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        // Subscribe to MQTT topics
        mqtt_sub_topics(client);

        // Publish to 'time/request' to request current time
        esp_mqtt_client_publish(client, "time/request", "", 0, 1, 0);
        ESP_LOGI(TAG, "Published time request to 'time/request'");

        // Publish a log message to 'sensor/logs'
        std::string log_json_str = "{\"device\":\"E46338809B472231\",\"log\":\"Device connected to MQTT broker\"}";
        mqtt_publish(client, "sensor/logs", log_json_str);
        printf("Device connected to MQTT broker\n");

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

    mqtt_cfg.outbox.limit = MQTT_OUTBOX_SIZE;

    mqtt_cfg.buffer.size = MQTT_BUFF_IN_SIZE;     // Input buffer size
    mqtt_cfg.buffer.out_size = MQTT_BUFF_OUT_SIZE; // Output buffer size

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    // // Register the event handler for specific MQTT events
    esp_err_t err;
    // err = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_CONNECTED, mqtt_event_handler, NULL);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to register MQTT_EVENT_CONNECTED handler");
    // }

    // err = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, NULL);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to register MQTT_EVENT_DISCONNECTED handler");
    // }

    // err = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_PUBLISHED, mqtt_event_handler, NULL);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to register MQTT_EVENT_PUBLISHED handler");
    // }

    err = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler");
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
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, 5, &uart_queue, 0)); // Install UART driver using an event queue here

    uart_enable_rx_intr(uart_num); // Configure UART to receive interrupt on RX full
}

// Function to send data over UART
void uart_send(const char* data) {
    const uart_port_t uart_num = UART_NUM;
    size_t len = strlen(data);
    uart_write_bytes(uart_num, data, len);
}

// JSON Processing Task
static void json_processing_task(void *pvParameters)
{
    std::string* json_str;
    while (true) {
        if (xQueueReceive(json_queue, &json_str, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Processing JSON of length %d", json_str->length());

            // Directly publish the JSON string
            mqtt_publish(mqtt_client, "sensor/data", *json_str);

            delete json_str; // Free allocated memory
        }
    }
}

// Function to handle UART events
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    char incoming_byte;
    std::string uart_buffer;
    const size_t max_buffer_size = 16384; // Limit the buffer size to prevent memory issues

    while (true) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int rx_bytes = event.size;
                    uint8_t* data = (uint8_t*) malloc(rx_bytes);
                    if (data == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate memory for UART data");
                        break;
                    }
                    int read_bytes = uart_read_bytes(UART_NUM, data, rx_bytes, portMAX_DELAY);
                    for (int i = 0; i < read_bytes; i++) {
                        incoming_byte = data[i];
                        if (incoming_byte == '\n') {
                            if (!uart_buffer.empty()) {
                                // Limit the buffer size to prevent excessive memory usage
                                if (uart_buffer.length() > max_buffer_size) {
                                    ESP_LOGE(TAG, "UART buffer overflow, discarding data");
                                    uart_buffer.clear();
                                    continue;
                                }
                                // Allocate memory for the JSON string
                                std::string* json_str = new(std::nothrow) std::string(uart_buffer);
                                if (json_str == NULL) {
                                    ESP_LOGE(TAG, "Failed to allocate memory for JSON string");
                                    uart_buffer.clear();
                                    continue;
                                }
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
    vTaskDelay(250 / portTICK_PERIOD_MS);  // Delay 250 milliseconds
    gpio_set_level(LED_PIN, 1);  // Pull high
}

/* ----- Main ----- */

extern "C" void app_main(void)
{
    init_led_pin();

    blink_led();
    // Sleep for 8 seconds before starting
    vTaskDelay(8000 / portTICK_PERIOD_MS);
    printf("Starting ESP32C6 UART Example\n");

    // Wi-Fi Initialization
    blink_led();
    wifi_init_sta(); // Initialize Wi-Fi
    blink_led();
    wait_for_ip();   // Wait for IP address

    // MQTT Initialization
    blink_led();
    mqtt_app_start(); // Start the MQTT client

    // Time Synchronization
    blink_led();
    set_timezone();  // Set the timezone
    while (!time_synced) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    print_current_time(); // Print the current time

    // UART Initialization
    blink_led();
    uart_initialize(); // Initialize UART

    blink_led();
    // Initialize the queue before starting tasks
    json_queue = xQueueCreate(10, sizeof(std::string*)); // Adjust the queue length as needed
    if (json_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON queue");
    }

    // Start JSON Processing Task
    blink_led();
    xTaskCreate(json_processing_task, "json_processing_task", 4096, NULL, 12, NULL);
    // Start UART Event Task
    blink_led();
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);

    gpio_set_level(LED_PIN, 0);  // Pull low

    uart_send("Clear\nConnected\n");  // Implementation of the TODO
    printf("Clear\nConnected\n");

    // Delete the main task if not needed
    vTaskDelete(NULL);
}
