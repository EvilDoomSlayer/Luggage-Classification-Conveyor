/**
 * @file DCMotor.cpp
 * @author M. Alejandro Sáchez R.
 * @brief Implementation of the simple DC motor controller class.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 */

#include "DCMotor.h"

/**
 * @brief Constructor: Initializes the private pin variables.
 */
DCMotor::DCMotor(int pin1, int pin2) {
    _pin1 = pin1;
    _pin2 = pin2;
}

/**
 * @brief Sets the pin modes to OUTPUT and ensures the motor is stopped.
 */
void DCMotor::begin() {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    stop(); // Default to a safe, stopped state on startup.
}

/**
 * @brief Sets pin states for forward motion (e.g., IN1=HIGH, IN2=LOW).
 */
void DCMotor::forward() {
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, LOW);
}

/**
 * @brief Sets pin states for reverse motion (e.g., IN1=LOW, IN2=HIGH).
 */
void DCMotor::reverse() {
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, HIGH);
}

/**
 * @brief Sets both pins LOW, disconnecting power and letting the motor coast.
 */
void DCMotor::stop() {
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
}

/**
 * @brief Sets both pins HIGH, shorting the motor terminals for a quick brake.
 */
void DCMotor::brake() {
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, HIGH);
}