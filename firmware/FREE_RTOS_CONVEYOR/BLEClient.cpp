/**
 * @file BLEClient.cpp
 * @author M. Alejandro Sáchez R.
 * @brief Implementation of the FreeRTOS-based BLE Client library.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This file implements the logic for the BLE client. It uses the ESP32 BLE libraries
 * within a FreeRTOS task pinned to Core 0. Callbacks are used to handle events
 * like finding a device, connecting, disconnecting, and receiving notifications.
 */

#include "system_config.h"
#include "BLEClient.h"
#include "Events.h"  // Defines state machine event types like EVT_CONNECTED_TO_SERVER
#include "ble_definitions.h" // Shared definitions with the ESP32 CAM
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <string>

// --- Public Variable Definition ---
QueueHandle_t bleValueQueue; // Queue to send received data strings to other tasks

// --- Private Library Variables ---
static QueueHandle_t stateMachineQueueHandle; // Handle to the main app's state machine queue
static BLEUUID serviceUUID;                   // UUID of the service to look for
static BLEUUID charUUID;                      // UUID of the characteristic to subscribe to
static BLEUUID stateCommandCharUUID;          // UUID of the characteristic to subscribe to 
static boolean doConnect = false;             // Flag set to true when a valid device is found
static boolean connected = false;             // Flag to track the current connection status
static BLEAdvertisedDevice* myDevice;         // Pointer to the found BLE device
static BLERemoteCharacteristic* pRemoteCharacteristic; // Pointer to the remote characteristic
static BLERemoteCharacteristic* pStateCommandCharacteristic; // Pointer to the command characteristic

// --- Forward Declarations for Internal Functions and Classes ---
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
bool connectToServer();
void BLEClientTask(void* pvParameters);

/**
 * @brief Callback class for handling BLE client connection events.
 */
class MyClientCallback : public BLEClientCallbacks {
    /**
     * @brief Called when the client successfully connects to the server.
     * @param pclient Pointer to the BLE client.
     */
    void onConnect(BLEClient* pclient) {
      #ifdef BLE_DEBUG_LOGS
        DEBUG_PRINTLN("onConnect: Client connected to server!");
      #endif
      connected = true; // Set connection flag
      // Send a connection event to the main state machine
      event_t event = EVT_CONNECTED_TO_SERVER;
      if (stateMachineQueueHandle != NULL) {
          xQueueSend(stateMachineQueueHandle, &event, portMAX_DELAY);
      }
    }

    /**
     * @brief Called when the client disconnects from the server.
     * @param pclient Pointer to the BLE client.
     */
    void onDisconnect(BLEClient* pclient) {
      #ifdef BLE_DEBUG_LOGS
        DEBUG_PRINTLN("onDisconnect: Client disconnected");
      #endif
      connected = false; // Clear connection flag
      pRemoteCharacteristic = nullptr;
      pStateCommandCharacteristic = nullptr;
      // Send a disconnection event to the main state machine
      event_t event = EVT_DISCONECTED_FROM_SERVER;
      if (stateMachineQueueHandle != NULL) {
          xQueueSend(stateMachineQueueHandle, &event, portMAX_DELAY);
      }
    }
};

/**
 * @brief Callback class for handling BLE scan results.
 * Finds a device that is advertising the specific service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    /**
     * @brief Called for each advertising device found.
     * @param advertisedDevice The device that was found.
     */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        // Check if the device has the service UUID we want
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            BLEDevice::getScan()->stop(); // Stop scanning once we find the device
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true; // Set the flag to trigger a connection attempt
            #ifdef BLE_DEBUG_LOGS
              DEBUG_PRINTLN("Found our device! Now connecting...");
            #endif
        }
    }
};

/**
 * @brief Callback function triggered when a notification is received.
 * @note This function is called in the context of the BLE stack, so it should
 * execute quickly. It copies the data and sends it to a queue for
 * processing by another task.
 *
 * @param pBLERemoteCharacteristic The characteristic that sent the notification.
 * @param pData A pointer to the data payload.
 * @param length The length of the data payload.
 * @param isNotify True if it is a notification, false if an indication.
 */
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // Convert the raw data to a C++ string
    std::string receivedValue = std::string((char*)pData, length);
    // Send the received string to the data queue for the consumer task
    if (xQueueSend(bleValueQueue, &receivedValue, pdMS_TO_TICKS(10)) != pdPASS) {
        // Failed to send, the queue is likely full.
        // DEBUG_PRINTLN("Failed to send to queue!");
    }
}

/**
 * @brief Establishes a connection to the BLE server.
 * @return true if the connection and characteristic registration were successful, false otherwise.
 */
bool connectToServer() {
    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
    
    #ifdef BLE_DEBUG_LOGS
      DEBUG_PRINT("Connecting to ");
      DEBUG_PRINTLN(myDevice->getAddress().toString().c_str());
    #endif
    
    if (!pClient->connect(myDevice)) {
        #ifdef BLE_DEBUG_LOGS
          DEBUG_PRINTLN("Failed to connect.");
        #endif
        return false;
    }
    setLedMode(BLINK_SLOW); // Indicate connection attempt

    // Get the service we are interested in
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        #ifdef BLE_DEBUG_LOGS
          DEBUG_PRINT("Failed to find service UUID: ");
          DEBUG_PRINTLN(serviceUUID.toString().c_str());
        #endif
        pClient->disconnect();
        return false;
    }

    // Get the status characteristic
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
        #ifdef BLE_DEBUG_LOGS
          DEBUG_PRINT("Failed to find characteristic UUID: ");
          DEBUG_PRINTLN(charUUID.toString().c_str());
        #endif
        pClient->disconnect();
        return false;
    }

    // Get the write command characteristic
    pStateCommandCharacteristic = pRemoteService->getCharacteristic(stateCommandCharUUID);
    if (pStateCommandCharacteristic == nullptr) {
        #ifdef BLE_DEBUG_LOGS
          DEBUG_PRINT("Failed to find characteristic UUID: ");
          DEBUG_PRINTLN(stateCommandCharUUID.toString().c_str());
        #endif
        pClient->disconnect();
        return false;
    }
    if (!pStateCommandCharacteristic->canWrite()) {
        #ifdef BLE_DEBUG_LOGS
          DEBUG_PRINTLN("Command characteristic is not writable!");
        #endif
        pClient->disconnect();
        return false;
    }

    // Register for notifications from the characteristic
    if (pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }
    #ifdef BLE_DEBUG_LOGS
      DEBUG_PRINTLN("Successfully connected and registered for notifications.");
    #endif
    return true;
}

/**
 * @brief The main FreeRTOS task for managing the BLE client.
 * @param pvParameters Unused task parameters.
 * @note This task handles the entire BLE lifecycle: scanning, connecting, and idling.
 * It's designed to be pinned to Core 0.
 */
void BLEClientTask(void* pvParameters) {
    #ifdef BLE_DEBUG_LOGS
      DEBUG_PRINTLN("BLE Client Task started on Core 0");
    #endif
    BLEDevice::init("");

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);

    while (1) {
        // If a device is found, connect to it
        if (doConnect && !connected) {
            if (connectToServer()) {
                // Connection successful
                // The onConnect callback will set 'connected' to true
            } else {
                #ifdef BLE_DEBUG_LOGS
                  DEBUG_PRINTLN("Failed to connect to the server.");
                #endif
            }
            doConnect = false; // Reset the connection trigger flag
        } 
        // If disconnected, start scanning again
        else if (!connected) {
            #ifdef BLE_DEBUG_LOGS
              DEBUG_PRINTLN("Not connected, starting scan...");
            #endif
            BLEDevice::getScan()->start(5, false); // Rescan
        }

        // If connected, this loop will just idle until a disconnect occurs.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Public function to write a command to the server.
 */
void BLEClientWriteCommand(state_command_t command) {
    if (!connected || pStateCommandCharacteristic == nullptr) {
        #ifdef BLE_DEBUG_LOGS
          DEBUG_PRINTLN("BLEClientWriteCommand: Not connected or char handle is null.");
        #endif
        return;
    }

    uint8_t data[1];
    data[0] = (uint8_t)command;

    #ifdef BLE_DEBUG_LOGS
      DEBUG_PRINT("BLEClientWriteCommand: Sending 0x");
      DEBUG_PRINTLN(data[0], HEX);
    #endif

    // Write the value. 'false' means Write Without Response.
    pStateCommandCharacteristic->writeValue(data, 1, false);
}

/**
 * @brief Public setup function to initialize and launch the BLE client.
 */
void setupBLEClient(const char* serviceUUID_str, const char* characteristicUUID_str, QueueHandle_t smQueue) {
    // 1. Store the application's queue handle and UUIDs
    stateMachineQueueHandle = smQueue;
    serviceUUID = BLEUUID(serviceUUID_str);
    charUUID = BLEUUID(characteristicUUID_str);
    stateCommandCharUUID = BLEUUID(CHAR_UUID_STATE_COMMAND);

    // 2. Create the queue that will hold incoming string data from BLE notifications
    bleValueQueue = xQueueCreate(5, sizeof(std::string));
    if (bleValueQueue == NULL) {
        Serial.println("Error creating the bleValueQueue");
        return; // Halt initialization
    }

    // 3. Create the BLE client task and pin it to Core 0
    //    Core 0 is reserved for the radio stack (Wi-Fi/Bluetooth) for stability.
    xTaskCreatePinnedToCore(
        BLEClientTask,      // Function to implement the task
        "BLEClient",        // A descriptive name for debugging
        10000,              // Stack size in bytes
        NULL,               // Task input parameter
        1,                  // Priority of the task
        NULL,               // Task handle
        0);                 // Core where the task should run
}