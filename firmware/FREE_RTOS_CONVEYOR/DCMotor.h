/**
 * @file DCMotor.h
 * @author M. Alejandro Sáchez R.
 * @brief Public interface for a simple DC motor controller class.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This class provides a basic abstraction for controlling a DC motor using a
 * standard H-bridge motor driver like the L293D or TB6612FNG. It simplifies
 * motor control to functions like forward(), reverse(), stop(), and brake().
 * It does not include PWM for speed control.
 */

#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

class DCMotor {
public:
    /**
     * @brief Construct a new DCMotor object.
     *
     * @param pin1 The first control pin connected to the H-bridge (e.g., IN1).
     * @param pin2 The second control pin connected to the H-bridge (e.g., IN2).
     */
    DCMotor(int pin1, int pin2);

    /**
     * @brief Initializes the motor control pins. Must be called in setup().
     */
    void begin();

    /**
     * @brief Drives the motor forward at full speed.
     */
    void forward();

    /**
     * @brief Drives the motor in reverse at full speed.
     */
    void reverse();

    /**
     * @brief Stops the motor by letting it coast (low-power stop).
     * This disconnects power from the motor, allowing it to spin down freely.
     */
    void stop();
    
    /**
     * @brief Stops the motor using active braking (high-power stop).
     * This shorts the motor terminals, causing it to stop much more quickly
     * than coasting.
     */
    void brake();

private:
    int _pin1; // Stores the GPIO pin number for the first motor input
    int _pin2; // Stores the GPIO pin number for the second motor input
};

#endif // DCMOTOR_H