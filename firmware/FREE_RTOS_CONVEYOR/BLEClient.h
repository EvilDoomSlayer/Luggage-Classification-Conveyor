/**
 * @file BLEClient.h
 * @author M. Alejandro Sáchez R.
 * @brief Public interface for the FreeRTOS-based BLE Client library.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This library provides a simple way to set up an ESP32 as a BLE client that runs
 * its own dedicated task on Core 0. It handles scanning, connecting, and receiving
 * notifications from a specified BLE server and characteristic.
 *
 * It uses two queues for communication:
 * 1.  `bleValueQueue`: Passes data (as std::string) received from the server *out* to the main application.
 * 2.  `smQueue`: A handle to the main application's state machine queue, used to send connection status events *in* to the application.
 */

#ifndef BLE_CLIENT_H
#define BLE_CLIENT_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "LedController.h" // Assumes LedController is part of the project for status indication

/**
 * @brief Sets up and starts the BLE client task.
 *
 * This function initializes the necessary queues and creates the dedicated BLE task,
 * pinning it to Core 0 of the ESP32. This is essential to prevent conflicts between
 * the Bluetooth stack and other application logic running on Core 1.
 * This must be called from your main `setup()` function.
 *
 * @param serviceUUID The UUID of the remote BLE service to connect to.
 * @param characteristicUUID The UUID of the remote BLE characteristic for notifications.
 * @param smQueue The queue handle for the main state machine, used to send connection/disconnection events.
 */
void setupBLEClient(const char* serviceUUID, const char* characteristicUUID, QueueHandle_t smQueue);

/**
 * @brief Public handle for the FreeRTOS queue that carries incoming BLE data.
 *
 * After calling setupBLEClient(), other tasks can use this handle with `xQueueReceive()`
 * to get the string values (e.g., "none", "light", "heavy") sent from the BLE
 * server via notifications. This queue passes `std::string` objects.
 */
extern QueueHandle_t bleValueQueue;

#endif // BLE_CLIENT_H