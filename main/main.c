
/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2018 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

#include <stdio.h>
#include <nvs_flash.h>
#include <string.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_timer.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "db_parameters.h"
#include "db_serial.h"

// Include FastMAVLink library
#include "db_mavlink_msgs.h"

#define UART_BUFFER_SIZE 1024
#define LOG_TAG "UART_MONITOR"

// UART pins configuration
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 4  // GPIO4
#define UART_RX_PIN 5  // GPIO5
#define UART_BAUD_RATE 57600

static const char *TAG = "DB_ESP32";

/**
 * Initialize UART with the specified configuration
 */
void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

/**
 * Format and print a buffer as hex and ASCII
 */
void print_buffer(const uint8_t *data, int len) {
    if (len <= 0) return;
    
    printf("\n--- UART Message (%d bytes) ---\n", len);
    
    // Print in hex with 16 bytes per line
    for (int i = 0; i < len; i += 16) {
        printf("%04X: ", i);
        
        // Hex portion
        for (int j = 0; j < 16; j++) {
            if (i + j < len) {
                printf("%02X ", data[i + j]);
            } else {
                printf("   ");
            }
            if (j == 7) printf(" ");  // Extra space after 8 bytes
        }
        
        // ASCII portion
        printf(" |");
        for (int j = 0; j < 16; j++) {
            if (i + j < len) {
                char c = data[i + j];
                // Print only printable ASCII characters
                if (c >= 32 && c <= 126) {
                    printf("%c", c);
                } else {
                    printf(".");
                }
            } else {
                printf(" ");
            }
        }
        printf("|\n");
    }
    printf("-----------------------------\n");
}

/**
 * Parse MAVLink message using FastMAVLink library
 */
void parse_mavlink_message(const uint8_t *data, int len) {
    // Setup parser status
    fmav_status_t status;
    fmav_status_reset(&status);
    
    // Setup message structure
    fmav_message_t msg;

    int the_one = 0;  // Reset the message counter
    // Parse byte by byte
    for (int i = 0; i < len; i++) {
        if (fmav_parse_to_msg(&msg, &status, data[i])) {
            // Check if it's a heartbeat with sysid=200 and compid=200
            if (msg.msgid == FASTMAVLINK_MSG_ID_HEARTBEAT && 
                msg.sysid == 200 && 
                msg.compid == 200) {
                the_one = 1;  // Set the flag to indicate a heartbeat was received
                // Print a single, concise message for this heartbeat
                
            }
        }
    }
    if (the_one) {
        ESP_LOGI(TAG, "Heartbeat received from sysid=200, compid=200");
        printf("\n--- Heartbeat received from sysid=200, compid=200---\n");
    } else {
        ESP_LOGI(TAG, "No heartbeat received from sysid=200, compid=200");
    }
}
/**
 * UART monitoring task
 */
void uart_monitor_task(void *pvParameters) {
    uint8_t buffer[UART_BUFFER_SIZE];
    int rx_bytes = 0;
    
    ESP_LOGI(TAG, "UART monitoring task started");
    
    while (1) {
        // Read data from UART
        rx_bytes = uart_read_bytes(UART_NUM, buffer, UART_BUFFER_SIZE, pdMS_TO_TICKS(100));
        
        if (rx_bytes > 0) {
            // Print timestamp
            // int64_t time = esp_timer_get_time();
            
            // Print formatted buffer
            // print_buffer(buffer, rx_bytes);
            
            // Parse MAVLink messages using the official library
            parse_mavlink_message(buffer, rx_bytes);
            
            // Echo data back if needed
            // uart_write_bytes(UART_NUM, buffer, rx_bytes);
        }
    }
}

/**
 * Main entry point - simplified for UART monitoring only
 */
void app_main() {
    // Initialize NVS (still needed for parameters)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Starting DroneBridge UART Monitor");
    
    // Initialize UART
    init_uart();
    
    // Create UART monitoring task
    xTaskCreate(uart_monitor_task, "uart_monitor_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "UART monitor initialized and running");
    ESP_LOGI(TAG, "Configured for UART%d: TX=%d, RX=%d, Baud=%d", UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}