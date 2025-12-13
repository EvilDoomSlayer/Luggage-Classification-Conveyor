/**
 * @file FREE_RTOS_CONVEYOR.ino
 * @author M. Alejandro Sáchez R.
 * @brief Main firmware for an automated conveyor belt system controlled by an ESP32 using FreeRTOS.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This project implements a state machine to manage a luggage sorting conveyor belt.
 * It uses multiple sensors and actuators, each managed by a dedicated FreeRTOS task.
 * The system communicates with a remote device (e.g., an ESP32-CAM) via BLE to
 * receive luggage classification data.
 *
 * Core Components:
 * - State Machine: Manages the overall system logic (pairing, idle, running, weighing, etc.).
 * - BLE Client: Connects to a server to receive commands/data.
 * - DC Motor: Drives the conveyor belt.
 * - Servos: Two servos for sorting luggage based on classification and weight.
 * - HX711 Load Cell: Weighs luggage.
 * - E18-D80NK IR Sensor: Detects the presence of objects at the start of the conveyor.
 * - KY-024 Magnetic Sensor: Detects magnetic fields in luggage.
 *
 * Task Architecture:
 * - Core 0: BLE communication tasks (handled by BLEClient library and bleMessageHandlerTask).
 * - Core 1: Main application logic including the state machine, sensor monitoring, and actuator control.
 */

// --- System & Library Headers ---
#include "system_config.h" // Project-specific configurations (like DEBUG_PRINT macros)
#include "BLEClient.h"     // Custom BLE client library
#include "Events.h"        // Defines state machine events (event_t) and states (state_t)
#include <string>          // For using std::string
#include "HX711_RTOS.h"    // Custom HX711 library modified for FreeRTOS
#include "DCMotor.h"       // Custom DC motor controller library
#include <ESP32Servo.h>    // Library for controlling servo motors on ESP32
#include "ble_definitions.h"  // Shared definitions with the ESP32 CAM

// --- IMPORTANT: CONFIGURE YOUR SERVER's UUIDs HERE ---
// These UUIDs must match the UUIDs on the BLE server (e.g., the ESP32-CAM).
const char* SERVICE_UUID = SERVICE_UUID_CONVEYOR;
const char* CHARACTERISTIC_UUID = CHAR_UUID_DETECTION;
// ---------------------------------------------------

// --- Pin Definitions ---
// On-board LED for status indication
const uint8_t LED_BUILTIN = 2;
// E18-D80NK IR Proximity Sensor: Detects objects entering the conveyor
const uint8_t E18_D80NKPin = 34;
// L293D Motor Driver pins for the conveyor belt motor
const uint8_t L293D_1A_PIN = 16;
const uint8_t L293D_2A_PIN = 17;
// HX711 Load Cell Amplifier pins
const uint8_t HX711_DOUT_PIN = 23; // Data Out
const uint8_t HX711_SCK_PIN = 19;  // Serial Clock
// KY-024 Linear Magnetic Hall Sensor: Detects magnetic fields
const uint8_t KY_024_PIN = 32;
// Servo Motor pins
const uint8_t SERVO1_PIN = 21; // Controls the "heavy" vs "light" luggage diverter
const uint8_t SERVO2_PIN = 22; // Controls the "accept" vs "discard" luggage diverter

// --- Global Object Instances ---
HX711 scale;                               // Instance for the load cell
DCMotor motor(L293D_1A_PIN, L293D_2A_PIN); // Instance for the conveyor motor
Servo servo1;                              // Instance for the classification servo
Servo servo2;                              // Instance for the sorting servo

// --- System Constants ---
// Scale Calibration & Thresholds
const float CALIBRATION_FACTOR = 240.075; // Value obtained from a calibration script
const float THRESHOLD_OBJECT_IN_SCALE_GRAMS = 5.0; // Min weight to consider an object is on the scale
const float THRESHOLD_ACCEPTED_WEIGHT_GRAMS = 30.0; // Max allowed weight for "heavy" luggage
// Servo 1 (Classification Diverter) Angles
const uint8_t SERVO1_HEAVY_LUGGAGE_ANGLE = 0;   // Angle to route heavy luggage
const uint8_t SERVO1_LIGHT_LUGGAGE_ANGLE = 70;  // Angle to route light luggage
// Servo 2 (Sorting Gate) Angles
const uint8_t SERVO2_IDLE_ANGLE = 85;       // Default position, allowing luggage to pass to scale
const uint8_t SERVO2_ACCEPTED_ANGLE = 135;  // Angle to release accepted luggage
const uint8_t SERVO2_DISCARTED_ANGLE = 45;  // Angle to release discarded luggage
// Magnetic Sensor Threshold
const int THRESHOLD_MAGNETIC_FIELD_DETECTED = 60; // Change from baseline to trigger detection

// --- FreeRTOS Handles & State Variables ---
// Queue for passing events from sensor tasks to the state machine task
QueueHandle_t stateMachineQueue;
// Global variable to hold the current state of the state machine
volatile state_t currentState = PAIRING; // Start in PAIRING state

// --- Function Prototypes ---
void servo1Move(uint8_t angle);
void servo2Move(uint8_t angle);

/**
 * @brief Handles incoming BLE messages and generates state machine events.
 * @param pvParameters Pointer to task parameters (unused).
 * @note This task runs on Core 0 to avoid interfering with the main application logic.
 *
 * It waits for messages from the `bleValueQueue`. When it receives "light" or "heavy",
 * it immediately sends a classification event to the state machine.
 *
 * To handle cases where an object leaves the camera's view before being classified,
 * this task implements a timeout. When a "none" message is received, it starts a
 * 5-second timer. If no "light" or "heavy" message arrives within this period,
 * it concludes that no luggage was detected and sends the `EVT_NO_LUGGAGE_DETECTED`
 * event. This prevents the system from waiting indefinitely.
 */
void bleMessageHandlerTask(void* pvParameters) {
    event_t event;
    std::string receivedString;

    // Define the timeout duration in milliseconds for the "no luggage" event.
    const TickType_t NO_LUGGAGE_TIMEOUT_MS = 5000;
    
    // Stores the timestamp (in system ticks) when a "none" message is received.
    TickType_t noLuggageTimestamp = 0;
    
    // A flag to track if we are currently waiting for the timeout to expire.
    bool isWaitingForNoLuggageTimeout = false;

    int multi_error_tries = 0;

    while (1) {
        // Only process messages if the system is in the RUNNING state.
        if (currentState == RUNNING) {
            #ifdef BLE_DEBUG_LOGS
              DEBUG_PRINTLN("Reciving BLE Messages");
            #endif
            // Check the queue for new messages with a short, non-blocking delay.
            // This allows the loop to continue and check the timeout logic below.
            if (xQueueReceive(bleValueQueue, &receivedString, pdMS_TO_TICKS(100)) == pdPASS) {
                #ifdef BLE_DEBUG_LOGS
                  DEBUG_PRINT("Conveyor received: '");
                  DEBUG_PRINT(receivedString.c_str());
                  DEBUG_PRINTLN("'");
                #endif

                // If a "light" or "heavy" message is received, handle it immediately.
                if (receivedString == "light" || receivedString == "heavy") {
                    // Cancel any pending "no luggage" timeout since we now have luggage info.
                    isWaitingForNoLuggageTimeout = false; 
                    
                    event = (receivedString == "light") ? EVT_LIGHT_LUGGAGE_CLASIFICATED : EVT_HEAVY_LUGGAGE_CLASIFICATED;
                    xQueueSend(stateMachineQueue, &event, portMAX_DELAY);

                } else if (receivedString == "none") {
                    // If a "none" message is received, start the timeout process if not already running.
                    if (!isWaitingForNoLuggageTimeout) {
                        isWaitingForNoLuggageTimeout = true;
                        noLuggageTimestamp = xTaskGetTickCount(); // Record the current time.
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("Started 'no luggage' timeout.");
                        #endif
                    }
                } else if (receivedString == "multi_error") {
                     multi_error_tries = multi_error_tries + 1;
                     if (multi_error_tries >= 6) {
                        isWaitingForNoLuggageTimeout = false;
                        event = EVT_MULTI_ERROR; 
                        xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
                     }
                }
            }

            // Continuously check if the timeout needs to be triggered.
            if (isWaitingForNoLuggageTimeout) {
                // If the elapsed time is greater than the defined timeout...
                if ((xTaskGetTickCount() - noLuggageTimestamp) >= pdMS_TO_TICKS(NO_LUGGAGE_TIMEOUT_MS)) {
                    #ifdef BLE_DEBUG_LOGS
                      DEBUG_PRINTLN("'No luggage' timeout expired. Sending event.");
                    #endif
                    event = EVT_NO_LUGGAGE_DETECTED;
                    xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
                    
                    // Reset the flag to prevent sending the event multiple times.
                    isWaitingForNoLuggageTimeout = false;
                }
            }
        } else {
            // If not in the RUNNING state, pause briefly to yield CPU time.
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

/**
 * @brief Manages the main system logic using a state machine.
 * @param pvParameters Pointer to task parameters (unused).
 * @note This is the central task of the application, running on Core 1 with the highest priority.
 *
 * It waits for events on the `stateMachineQueue` and transitions the system
 * between different operational states (e.g., PAIRING, RUNNING, WEIGHING).
 * Based on the current state and the received event, it controls the motors,
 * servos, and status LED. It also handles error conditions like BLE disconnection
 * and object overlap on the conveyor.
 */
void stateMachineTask(void *pvParameters) {
    state_t lastState = currentState;
    event_t receivedEvent;
    uint8_t magneticFieldDetectedFlag = 0;

    // --- LIGHT_LUGGAGE timeout ---
    TickType_t lightLuggageTimerStart = 0;
    bool isLightLuggageTimerRunning = false;
    const TickType_t LIGHT_LUGGAGE_MAGNETIC_TIMEOUT_MS = 5000; // 5 second timeout
    TickType_t waitTicks; // Will hold the wait time for the queue

    while (1) {
        // Determine the queue wait time based on the current state
        if (currentState == LIGHT_LUGGAGE) {
            // Use a short, non-blocking timeout to allow timer checking
            waitTicks = pdMS_TO_TICKS(100); 
        } else {
            // Wait indefinitely for an event in all other states
            waitTicks = portMAX_DELAY;
        }
        
        // Wait indefinitely for an event to arrive in the queue
        if (xQueueReceive(stateMachineQueue, &receivedEvent, waitTicks)) {
            // --- An event WAS received from the queue ---
            #ifdef STATE_MACHINE_DEBUG_LOGS
              DEBUG_PRINT("Received event ");
              DEBUG_PRINT(receivedEvent);
              DEBUG_PRINT(" in state ");
              DEBUG_PRINTLN(currentState);
            #endif

            switch (currentState) {
                case PAIRING:
                    // Waiting to establish the initial BLE connection
                    if (receivedEvent == EVT_CONNECTED_TO_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("CONNECTED TO ESP32CAM");
                        #endif
                        setLedMode(SOLID_ON); // Solid LED indicates connection
                        BLEClientWriteCommand(STATE_CMD_IDLE);
                        lastState = currentState;
                        currentState = CONNECTED_IDLE;
                    }
                    break;
                
                case DISCONNECTED:
                    // In a disconnected state, waiting to reconnect
                    if (receivedEvent == EVT_CONNECTED_TO_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("CONNECTED TO ESP32CAM");
                        #endif
                        setLedMode(SOLID_ON);
                        currentState = lastState; // Return to the state before disconnection
                        lastState = DISCONNECTED;
                    }
                    break;

                case CONNECTED_IDLE:
                    // System is connected and waiting for an object
                    if (receivedEvent == EVT_CONVEYOR_OBJECT_ENTERED) {
                        #ifdef E18_D80NK_DEBUG_LOGS
                          DEBUG_PRINTLN("OBJECT DETECTED");
                        #endif
                        BLEClientWriteCommand(STATE_CMD_RUN);
                        servo2Move(SERVO2_IDLE_ANGLE); // Ensure scale is in idle position
                        motor.forward(); // Start the conveyor belt
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        motor.brake();
                        lastState = currentState;
                        currentState = RUNNING;
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("CHANGING TO THE RUNNING STATE");
                        #endif
                    }
                    // Handle disconnection
                    if (receivedEvent == EVT_DISCONECTED_FROM_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("DISCONNECTED FROM ESP32CAM");
                        #endif
                        setLedMode(BLINK_RAPID); // Rapid blink indicates error/disconnection
                        motor.brake();
                        lastState = currentState;
                        currentState = DISCONNECTED;
                    }
                    break;
                
                case RUNNING:
                    // Object is on the conveyor, waiting for classification from the camera
                    if (receivedEvent == EVT_LIGHT_LUGGAGE_CLASIFICATED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Light luggage detected");
                        #endif
                        motor.forward();
                        servo1Move(SERVO1_LIGHT_LUGGAGE_ANGLE); // Set diverter for light path
                        lastState = currentState;
                        currentState = LIGHT_LUGGAGE;
                        // --- Start the 5-second magnetic sensor timer ---
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("LIGHT_LUGGAGE: Starting 5s magnetic check timer.");
                        #endif
                        lightLuggageTimerStart = xTaskGetTickCount();
                        isLightLuggageTimerRunning = true;
                    }

                    if (receivedEvent == EVT_HEAVY_LUGGAGE_CLASIFICATED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Heavy luggage detected");
                        #endif
                        motor.forward();
                        servo1Move(SERVO1_HEAVY_LUGGAGE_ANGLE); // Set diverter for heavy path
                        lastState = currentState;
                        currentState = HEAVY_LUGGAGE;
                    }
                    // Handle case where object passes cam with no detection
                    if (receivedEvent == EVT_NO_LUGGAGE_DETECTED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Object passed camera, no classification. Stopping.");
                          DEBUG_PRINTLN("RETURNNING IDLE STATE");
                        #endif
                        BLEClientWriteCommand(STATE_CMD_IDLE); // <-- TELL CAM TO STOP
                        lastState = currentState;
                        currentState = CONNECTED_IDLE; // Go back to waiting
                    }
                    // Handle two objects detected at the same time
                    if (receivedEvent == EVT_MULTI_ERROR) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("TWO OBJECTS DETECTED AT THE SAME TIME - EMERGENCY STOP");
                          DEBUG_PRINTLN("RETURNNING IDLE STATE");
                        #endif
                        BLEClientWriteCommand(STATE_CMD_IDLE); // <-- TELL CAM TO STOP
                        lastState = currentState;
                        currentState = CONNECTED_IDLE; // Go back to waiting
                    }
                    // Handle disconnection
                    if (receivedEvent == EVT_DISCONECTED_FROM_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("DISCONNECTED FROM ESP32CAM");
                        #endif
                        setLedMode(BLINK_RAPID);
                        lastState = currentState;
                        currentState = DISCONNECTED;
                    }
                    // Handle another object entering while one is already processing
                    if (receivedEvent == EVT_CONVEYOR_OBJECT_ENTERED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("OBJECT DETECTED WHILE RUNNING - EMERGENCY STOP");
                          DEBUG_PRINTLN("CHANGING TO THE OVERLAP_DETECTED STATE");
                        #endif
                        lastState = currentState;
                        currentState = OVERLAP_DETECTED;
                    }
                    break;

                case LIGHT_LUGGAGE:
                    // Object classified as "light" is moving towards the magnetic sensor
                    if (receivedEvent == EVT_MAGNETIC_FIELD_DETECTED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Radioactive Luggage detected");
                        #endif
                        magneticFieldDetectedFlag = 1;
                        servo1Move(SERVO1_HEAVY_LUGGAGE_ANGLE); // Re-route to heavy/discard path
                        lastState = currentState;
                        currentState = HEAVY_LUGGAGE;
                        isLightLuggageTimerRunning = false; // Magnetic object found, cancel timer
                    }
                    // Handle disconnection
                    if (receivedEvent == EVT_DISCONECTED_FROM_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("DISCONNECTED FROM ESP32CAM");
                        #endif
                        motor.brake();
                        setLedMode(BLINK_RAPID);
                        lastState = currentState;
                        currentState = DISCONNECTED;
                        isLightLuggageTimerRunning = false; // Cancel timer on disconnect
                    }
                    // Handle object overlap
                    if (receivedEvent == EVT_CONVEYOR_OBJECT_ENTERED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("OBJECT DETECTED WHILE RUNNING - EMERGENCY STOP");
                          DEBUG_PRINTLN("CHANGING TO THE OVERLAP_DETECTED STATE");
                        #endif
                        motor.brake();
                        lastState = currentState;
                        currentState = OVERLAP_DETECTED;
                        isLightLuggageTimerRunning = false; // Cancel timer on overlap
                    }
                    break;

                case HEAVY_LUGGAGE:
                    // Object classified as "heavy" or magnetic is moving towards the scale
                    if (receivedEvent == EVT_MAGNETIC_FIELD_DETECTED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Radioactive Luggage detected");
                        #endif
                        magneticFieldDetectedFlag = 1;
                    }
                    // Event from scale task indicates object has arrived
                    if (receivedEvent == EVT_OBJECT_IN_SCALE) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Somethig arrived to the scale");
                        #endif
                        motor.brake();  // Stop conveyor for weighing
                        lastState = currentState;

                        // If magnetic, discard immediately without weighing
                        if (magneticFieldDetectedFlag) {
                            #ifdef STATE_MACHINE_DEBUG_LOGS
                              DEBUG_PRINTLN("Magnetic field was detected");
                              DEBUG_PRINTLN("Magnetic luggage discarted");
                            #endif
                            servo2Move(SERVO2_DISCARTED_ANGLE);
                            magneticFieldDetectedFlag = 0; // Reset flag
                            currentState = CONNECTED_IDLE; // Return to idle
                            BLEClientWriteCommand(STATE_CMD_IDLE);
                        } else {
                            currentState = WEIGHING;
                        }
                    }
                    // Handle disconnection
                    if (receivedEvent == EVT_DISCONECTED_FROM_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("DISCONNECTED FROM ESP32CAM");
                        #endif
                        motor.brake();
                        setLedMode(BLINK_RAPID);
                        lastState = currentState;
                        currentState = DISCONNECTED;
                    }
                    // Handle object overlap
                    if (receivedEvent == EVT_CONVEYOR_OBJECT_ENTERED) {
                       #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("OBJECT DETECTED WHILE RUNNING - EMERGENCY STOP");
                          DEBUG_PRINTLN("CHANGING TO THE OVERLAP_DETECTED STATE");
                        #endif
                        motor.brake();
                        lastState = currentState;
                        currentState = OVERLAP_DETECTED;
                    }
                    break;

                case WEIGHING:
                    // Object is on the scale being weighed
                    if (receivedEvent == EVT_ACCEPTED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Accepted");
                        #endif
                        servo2Move(SERVO2_ACCEPTED_ANGLE);
                        BLEClientWriteCommand(STATE_CMD_IDLE);
                        lastState = currentState;
                        currentState = CONNECTED_IDLE; // Cycle complete, return to idle
                    }

                    if (receivedEvent == EVT_DISCARTED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("Discarted");
                        #endif
                        servo2Move(SERVO2_DISCARTED_ANGLE);
                        BLEClientWriteCommand(STATE_CMD_IDLE);
                        lastState = currentState;
                        currentState = CONNECTED_IDLE; // Cycle complete, return to idle
                    }
                    // Handle disconnection
                    if (receivedEvent == EVT_DISCONECTED_FROM_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("DISCONNECTED FROM ESP32CAM");
                        #endif
                        motor.brake();
                        setLedMode(BLINK_RAPID);
                        lastState = currentState;
                        currentState = DISCONNECTED;
                    }
                    break;
                
                case OVERLAP_DETECTED:
                    // Emergency stop state, waiting for extra object to be removed
                    if (receivedEvent == EVT_DISCONECTED_FROM_SERVER) {
                        #ifdef BLE_DEBUG_LOGS
                          DEBUG_PRINTLN("DISCONNECTED FROM ESP32CAM");
                        #endif
                        motor.brake();
                        setLedMode(BLINK_RAPID);
                        lastState = currentState;
                        currentState = DISCONNECTED;
                    }
                    // Event indicates the overlapping object has been removed
                    if (receivedEvent == EVT_CONVEYOR_OBJECT_EXITED) {
                        #ifdef STATE_MACHINE_DEBUG_LOGS
                          DEBUG_PRINTLN("OBJECT REMOVED. CONTINUING CURRENT PROCESS");
                          DEBUG_PRINT("RETURNNING TO THE STATE: ");
                          DEBUG_PRINTLN(lastState);
                        #endif
                        if (lastState > CONNECTED_IDLE && lastState < WEIGHING) {
                            motor.forward();
                        }
                        currentState = lastState; // Resume previous operation
                        lastState = OVERLAP_DETECTED;
                    }
                    break;
            }
        } else {
            // --- No event was received from the queue (timeout occurred) ---

            // This block will only be entered if waitTicks was not portMAX_DELAY,
            // which we've set to only happen in the LIGHT_LUGGAGE state.
            if (currentState == LIGHT_LUGGAGE && isLightLuggageTimerRunning) {
                // Check if the 5-second timer has expired
                DEBUG_PRINTLN(xTaskGetTickCount() - lightLuggageTimerStart);
                if ((xTaskGetTickCount() - lightLuggageTimerStart) >= pdMS_TO_TICKS(LIGHT_LUGGAGE_MAGNETIC_TIMEOUT_MS)) {
                    #ifdef STATE_MACHINE_DEBUG_LOGS
                      DEBUG_PRINTLN("LIGHT_LUGGAGE: 5s timeout. No magnetic field detected.");
                      DEBUG_PRINTLN("CYCLE COMPLETE");
                      DEBUG_PRINTLN("RETURNNING TO IDLE STATE");
                    #endif
                    motor.brake(); 
                    // Return to the idle state
                    BLEClientWriteCommand(STATE_CMD_IDLE);
                    lastState = currentState;
                    currentState = CONNECTED_IDLE; 
                    // Stop the timer
                    isLightLuggageTimerRunning = false; 
                }
            }
        }
    }
}

/**
 * @brief Monitors an E18-D80NK IR sensor to detect objects entering the conveyor.
 * @param pvParameters Pointer to task parameters (unused).
 * @note This task runs on Core 1. It incorporates logic to prevent false triggers.
 *
 * State-dependent behavior:
 * - In `CONNECTED_IDLE` state, it waits for an object to be present for a confirmation
 * period (`OBJECT_CONFIRMATION_TIMEOUT_MS`) before sending `EVT_CONVEYOR_OBJECT_ENTERED`.
 * This acts as a debounce mechanism to avoid false starts.
 * - In active states (e.g., `RUNNING`), it sends `EVT_CONVEYOR_OBJECT_ENTERED` immediately
 * to signal an emergency stop due to object overlap.
 * - In `OVERLAP_DETECTED` state, it waits for the object to be removed and sends
 * `EVT_CONVEYOR_OBJECT_EXITED` to resume operation.
 */
void E18_D80NKTask(void *pvParameters) {
    pinMode(E18_D80NKPin, INPUT);
    event_t event;

    // The time in milliseconds an object must be present before the "entered" event is sent.
    const TickType_t OBJECT_CONFIRMATION_TIMEOUT_MS = 3000;
    // Stores the timestamp when an object is first detected.
    TickType_t objectDetectionTimestamp = 0;
    // A flag to track if we are waiting to confirm the object's presence.
    bool isObjectDetectionPending = false;

    while(1) {
        if (currentState == CONNECTED_IDLE) {
            // Check if the sensor detects an object (LOW signal means object is present).
            #ifdef E18_D80NK_DEBUG_LOGS
              bool E18_D80NK_READ = digitalRead(E18_D80NKPin);
              DEBUG_PRINT("E18_D80NK read: ");
              DEBUG_PRINTLN(E18_D80NK_READ);
            #endif
            if (!digitalRead(E18_D80NKPin)) {
                // An object is detected. If we aren't already timing it, start the timer now.
                if (!isObjectDetectionPending) {
                    isObjectDetectionPending = true;
                    objectDetectionTimestamp = xTaskGetTickCount(); // Record the start time.
                }
            } else {
                // No object is detected, so cancel any pending confirmation timer.
                isObjectDetectionPending = false;
            }

            // If a confirmation timer is active...
            if (isObjectDetectionPending) {
                // Check if the required time has passed since the initial detection.
                if ((xTaskGetTickCount() - objectDetectionTimestamp) >= pdMS_TO_TICKS(OBJECT_CONFIRMATION_TIMEOUT_MS)) {
                    // Timeout expired. Final check to ensure the object is still there.
                    if (!digitalRead(E18_D80NKPin)) {
                        event = EVT_CONVEYOR_OBJECT_ENTERED;
                        xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
                        // Delay to prevent this event from being sent multiple times if the
                        // object remains for a long time.
                        vTaskDelay(pdMS_TO_TICKS(3000));
                    }
                    // Reset the flag to be ready for the next event.
                    isObjectDetectionPending = false;
                }
            }
        } else if (currentState > CONNECTED_IDLE && currentState != OVERLAP_DETECTED) { 
            // When the system is active, an object detection is an emergency (overlap).
            // Act immediately without debouncing.
            if (!digitalRead(E18_D80NKPin)) {
                event = EVT_CONVEYOR_OBJECT_ENTERED;
                xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
            }
        } else if (currentState == OVERLAP_DETECTED) {
            // In the overlap error state, wait for the object to be cleared.
            if (digitalRead(E18_D80NKPin)) {
                event = EVT_CONVEYOR_OBJECT_EXITED;
                xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
            }
        }
        // Delay to prevent this task from consuming 100% CPU. This sets the polling rate.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Manages the HX711 load cell to weigh luggage.
 * @param pvParameters Pointer to task parameters (unused).
 * @note This task runs on Core 1.
 *
 * It initializes and calibrates the scale. The task's behavior depends on the
 * system state:
 * - In `HEAVY_LUGGAGE` state, it waits for a weight above a minimum threshold
 * to confirm the object is on the scale, then sends `EVT_OBJECT_IN_SCALE`.
 * - In `WEIGHING` state, it takes a final reading and sends `EVT_ACCEPTED` or
 * `EVT_DISCARTED` based on whether the weight is within the allowed limit.
 */
void HX771Task(void *pvParameters) {
    #ifdef HX711_DEBUG_LOGS
      DEBUG_PRINT("HX711 task running on core ");
      DEBUG_PRINTLN(xPortGetCoreID());
    #endif

    // Initialize the scale
    scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    scale.set_scale(CALIBRATION_FACTOR); // Set the calibration factor
    scale.tare();                        // Reset the scale to 0

    #ifdef HX711_DEBUG_LOGS
      DEBUG_PRINTLN("Scale initialized and tared. Ready to measure.");
    #endif
    event_t event;

    while(1) {
        if (scale.is_ready()) {
            // Get an average of 5 readings for stability
            float reading = scale.get_units(5);
            #ifdef HX711_DEBUG_LOGS
              DEBUG_PRINT("HX711 meassurements: ");
              DEBUG_PRINT(reading);
            #endif

            if (currentState == HEAVY_LUGGAGE) {
                // Wait for an object to be physically present on the scale
                if (reading >= THRESHOLD_OBJECT_IN_SCALE_GRAMS) {
                    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for the reading to stabilize
                    event = EVT_OBJECT_IN_SCALE;
                    xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
                }
            } else if (currentState == WEIGHING) {
                // Perform the final weight check
                #ifdef HX711_DEBUG_LOGS
                  DEBUG_PRINT("Final Weight: ");
                  DEBUG_PRINT(reading, 2); // Print with 2 decimal places
                  DEBUG_PRINTLN(" g");
                #endif
                event = (reading <= THRESHOLD_ACCEPTED_WEIGHT_GRAMS) ? EVT_ACCEPTED : EVT_DISCARTED;
                xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
            }
        } else {
            #ifdef HX711_DEBUG_LOGS
              DEBUG_PRINTLN("HX711 not found.");
            #endif
        }
        // Delay to prevent constant readings and yield CPU time
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

/**
 * @brief Monitors the KY-024 magnetic field sensor to detect specific items.
 * @param pvParameters Pointer to task parameters (unused).
 * @note This task runs on Core 1.
 *
 * At startup, it performs a calibration routine to establish a baseline reading
 * with no magnetic field present.
 *
 * During operation (in `LIGHT_LUGGAGE` or `HEAVY_LUGGAGE` states), it continuously
 * reads the sensor. If the reading deviates from the baseline by more than a
 * defined threshold, it sends the `EVT_MAGNETIC_FIELD_DETECTED` event.
 */
void KY_024Task(void *pvParameters) {
    pinMode(KY_024_PIN, INPUT);
    event_t event;
    
    // --- Calibration Step ---
    int baselineValue = 0;
    #ifdef KY_024_DEBUG_LOGS
      DEBUG_PRINTLN("Calibrating magnetic sensor... Keep magnet away for 2 seconds.");
    #endif
    long total = 0;
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    // Take an average of 100 readings to get a stable baseline with no magnetic field.
    for (int i = 0; i < 200; i++) {
        total += analogRead(KY_024_PIN);
        vTaskDelay(pdMS_TO_TICKS(20)); // Small delay between readings
    }
    baselineValue = total / 200;
    #ifdef KY_024_DEBUG_LOGS
      DEBUG_PRINT("Calibration complete. Baseline value: ");
      DEBUG_PRINTLN(baselineValue);
    #endif
    
    while(1) {
        // Only monitor the sensor when luggage is actively being processed.
        if (currentState == LIGHT_LUGGAGE || currentState == HEAVY_LUGGAGE) {
            int currentValue = analogRead(KY_024_PIN);
            // Calculate the absolute difference from the no-field baseline
            int difference = abs(currentValue - baselineValue);
            #ifdef KY_024_DEBUG_LOGS
              DEBUG_PRINT("Reading of KY_024: ");
              DEBUG_PRINTLN(difference);
            #endif
            // If the change in magnetic field is significant, send an event.
            if (difference > THRESHOLD_MAGNETIC_FIELD_DETECTED) {
                event = EVT_MAGNETIC_FIELD_DETECTED;
                xQueueSend(stateMachineQueue, &event, portMAX_DELAY);
            }
        }
        // Poll the sensor at a regular interval.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Controls the classification servo (Servo 1).
 * @param angle The target angle to move the servo to.
 * @note This function attaches the servo, moves it, waits briefly for the move
 * to complete, and then detaches it to prevent jitter and reduce power consumption.
 */
void servo1Move(uint8_t angle) {
    #ifdef SERVO_DEBUG_LOGS
      DEBUG_PRINT("Moving servo 1 to: ");
    #endif
    if (angle == SERVO1_HEAVY_LUGGAGE_ANGLE) {
        #ifdef SERVO_DEBUG_LOGS
          DEBUG_PRINTLN("Heavy Luggage Position");
        #endif
    } else {
        #ifdef SERVO_DEBUG_LOGS
          DEBUG_PRINTLN("Light Luggage Position");
        #endif
    }
    servo1.attach(SERVO1_PIN);
    servo1.write(angle);
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for the move to complete
    servo1.detach();
}

/**
 * @brief Controls the sorting servo (Servo 2).
 * @param angle The target angle to move the servo to.
 * @note This function attaches the servo, moves it, waits briefly for the move
 * to complete, and then detaches it to prevent jitter and reduce power consumption.
 */
void servo2Move(uint8_t angle) {
    #ifdef SERVO_DEBUG_LOGS
      DEBUG_PRINT("Moving servo 2 to: ");
    #endif
    if (angle == SERVO2_IDLE_ANGLE) {
        #ifdef SERVO_DEBUG_LOGS
          DEBUG_PRINTLN("Idle Position");
        #endif
    } else if (angle == SERVO2_ACCEPTED_ANGLE) {
        #ifdef SERVO_DEBUG_LOGS
          DEBUG_PRINTLN("Accepted Position");
        #endif
    } else {
        #ifdef SERVO_DEBUG_LOGS
          DEBUG_PRINTLN("Discarded Position");
        #endif
    }
    servo2.attach(SERVO2_PIN);
    servo2.write(angle);
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for the move to complete
    servo2.detach();
}

/**
 * @brief Initializes the system.
 *
 * This function performs the following steps:
 * 1. Starts Serial communication for debugging.
 * 2. Allocates hardware timers required for the ESP32Servo library.
 * 3. Creates the FreeRTOS queue for state machine events.
 * 4. Initializes hardware modules (LED controller, BLE client, motor).
 * 5. Creates and pins all the application tasks to their respective CPU cores.
 */
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("--- FreeRTOS Conveyor System Initializing ---");

    // Allow allocation of all timers for servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Create the queue that will pass events to the state machine task
    stateMachineQueue = xQueueCreate(10, sizeof(event_t));
    if (stateMachineQueue == NULL) {
        Serial.println("Error creating state machine queue");
        while (1); // Halt on error
    }
    
    // Initialize peripheral libraries/modules
    setupLedController(LED_BUILTIN, 1, 1);
    setupBLEClient(SERVICE_UUID, CHARACTERISTIC_UUID, stateMachineQueue);
    motor.begin();

    // --- Task Creation ---
    // Core 0 is used for wireless communication (Bluetooth).
    // Core 1 is used for the main application logic.

    // Create the BLE message handler task on Core 0
    xTaskCreatePinnedToCore(
        bleMessageHandlerTask,
        "BLE_Handler",
        2048,
        NULL,
        1, // Low priority
        NULL,
        0);

    // Create the main state machine task with the highest priority on Core 1
    xTaskCreatePinnedToCore(
        stateMachineTask,
        "State_Machine",
        4096,
        NULL,
        3, // Highest priority
        NULL,
        1);

    // Create the IR sensor monitoring task on Core 1
    xTaskCreatePinnedToCore(
        E18_D80NKTask,
        "IR_Sensor",
        2048,
        NULL,
        2, // Medium priority
        NULL,
        1);

    // Create the load cell (scale) task on Core 1
    xTaskCreatePinnedToCore(
        HX771Task,
        "Scale_Task",
        4096,
        NULL,
        1, // Low priority
        NULL,
        1);
    
    // Create the magnetic sensor task on Core 1
    xTaskCreatePinnedToCore(
        KY_024Task,
        "Magnetic_Sensor",
        4096,
        NULL,
        1, // Low priority
        NULL,
        1);
}

/**
 * @brief Main loop - intentionally left empty.
 *
 * Since this project uses the FreeRTOS real-time operating system, all functionality
 * is handled by independent tasks. The `loop()` task has the lowest priority and
 * simply yields CPU time by sleeping.
 */
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}