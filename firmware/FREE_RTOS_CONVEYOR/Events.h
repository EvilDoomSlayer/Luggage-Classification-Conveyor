/**
 * @file Events.h
 * @author M. Alejandro Sáchez R.
 * @brief Defines the states and events for the application's main state machine.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This header acts as a central registry for the state machine's vocabulary.
 * Using enums for states and events makes the code more readable and prevents
 * errors from using incorrect or misspelled string values.
 */

#ifndef APP_EVENTS_H
#define APP_EVENTS_H

/**
 * @brief Defines all possible operational states of the conveyor system.
 */
typedef enum {
    PAIRING,            // Initial state, waiting for the first BLE connection.
    DISCONNECTED,       // An error state entered when the BLE connection is lost.
    CONNECTED_IDLE,     // Connected and waiting for an object to enter the conveyor.
    RUNNING,            // Conveyor is moving, waiting for luggage classification.
    LIGHT_LUGGAGE,      // Light luggage is on the conveyor, moving to the magnetic sensor.
    HEAVY_LUGGAGE,      // Heavy luggage is on the conveyor, moving to the scale.
    WEIGHING,           // Luggage is on the scale being weighed.
    OVERLAP_DETECTED    // Emergency stop state due to a second object entering the system.
} state_t;

/**
 * @brief Defines all events that can trigger a state transition.
 * These events are sent from various sensor tasks to the main state machine task.
 */
typedef enum {
    EVT_CONNECTED_TO_SERVER,        // BLE client has connected.
    EVT_DISCONECTED_FROM_SERVER,    // BLE connection lost.
    EVT_CONVEYOR_OBJECT_ENTERED,    // IR sensor detects an object at the start.
    EVT_CONVEYOR_OBJECT_EXITED,     // IR sensor confirms overlapping object was removed.
    EVT_LIGHT_LUGGAGE_CLASIFICATED, // BLE message "light" received.
    EVT_HEAVY_LUGGAGE_CLASIFICATED, // BLE message "heavy" received.
    EVT_NO_LUGGAGE_DETECTED,        // BLE message "none" received and timed out.
    EVT_MULTI_ERROR,                // BLE message "multi_error" received.
    EVT_MAGNETIC_FIELD_DETECTED,    // Magnetic sensor has been triggered.
    EVT_OBJECT_IN_SCALE,            // Load cell detects a significant weight.
    EVT_DISCARTED,                  // Luggage is overweight or magnetic.
    EVT_ACCEPTED                    // Luggage passed all checks.
} event_t;

#endif // APP_EVENTS_H