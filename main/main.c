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

// Heartbeat configuration
#define HEARTBEAT_INTERVAL_MS 1000
#define CUSTOM_SYS_ID 200
#define CUSTOM_COMP_ID 200

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
 * Heartbeat sending task
 */
void heartbeat_task(void *pvParameters) {
    uint8_t buffer[UART_BUFFER_SIZE];
    fmav_status_t fmav_status = {0};
    fmav_message_t msg;
    
    ESP_LOGI(TAG, "Heartbeat task started, sending every %d ms", HEARTBEAT_INTERVAL_MS);
    
    // Log first heartbeat explicitly
    ESP_LOGI(TAG, "Sending first heartbeat...");
    
    while (1) {
        // Use the fmav_msg_heartbeat_pack function directly to create a heartbeat message
        fmav_msg_heartbeat_pack(
            &msg,
            CUSTOM_SYS_ID,        // System ID
            CUSTOM_COMP_ID,       // Component ID
            MAV_TYPE_ONBOARD_CONTROLLER,  // Type
            MAV_AUTOPILOT_INVALID,        // Autopilot
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  // Base mode
            0,                             // Custom mode
            MAV_STATE_ACTIVE,              // System status
            &fmav_status
        );
        
        // Pack the message into a buffer for transmission
        // CORRECTED: fmav_msg_to_frame_buf only takes 2 parameters
        uint16_t len = fmav_msg_to_frame_buf(buffer, &msg);
        
        // Send the heartbeat via UART
        int bytes_sent = uart_write_bytes(UART_NUM, buffer, len);
        
        if (bytes_sent == len) {
            ESP_LOGI(TAG, "Heartbeat sent successfully (sysid=%d, compid=%d, %d bytes)", 
                    CUSTOM_SYS_ID, CUSTOM_COMP_ID, bytes_sent);
        } else {
            ESP_LOGW(TAG, "Failed to send complete heartbeat (%d/%d bytes sent)", 
                    bytes_sent, len);
        }
        
        // Wait for the next interval
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
    }
}
/**
 * Main entry point
 */
void app_main() {
    // Initialize NVS (still needed for parameters)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Starting DroneBridge Heartbeat Sender");
    
    // Initialize UART
    init_uart();
    
    // Create heartbeat sending task
    TaskHandle_t heartbeatTaskHandle = NULL;
    if (xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 5, &heartbeatTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heartbeat task!");
    } else {
        ESP_LOGI(TAG, "Heartbeat task created successfully");
    }
    
    ESP_LOGI(TAG, "Heartbeat sender initialized and running");
    ESP_LOGI(TAG, "Configured for UART%d: TX=%d, RX=%d, Baud=%d", UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
    ESP_LOGI(TAG, "Sending heartbeats with sysid=%d, compid=%d every %d ms", CUSTOM_SYS_ID, CUSTOM_COMP_ID, HEARTBEAT_INTERVAL_MS);
}