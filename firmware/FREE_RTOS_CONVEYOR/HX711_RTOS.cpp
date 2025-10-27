/**
 * @file HX711_RTOS.cpp
 * @author Bogdan Necula, Andreas Motl, M. Alejandro Sáchez R.
 * @brief Implementation of the HX711 ADC library for FreeRTOS (ESP32).
 * @version 0.8 (RTOS Modified)
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2018 Bogdan Necula
 */

#include <Arduino.h>
#include "HX711_RTOS.h"

// --- FreeRTOS Specific Modifications ---
#if defined(ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// A mutex for ensuring the time-sensitive read operation is not interrupted.
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif

// Define yield() for older Arduino cores where it might be missing.
#if ARDUINO_VERSION <= 106 && !defined(ESP32)
    void yield(void) {};
#endif


HX711::HX711(byte dout, byte pd_sck, byte gain) {
    begin(dout, pd_sck, gain);
}

HX711::HX711() {
}

HX711::~HX711() {
}

void HX711::begin(byte dout, byte pd_sck, byte gain) {
    PD_SCK = pd_sck;
    DOUT = dout;

    pinMode(PD_SCK, OUTPUT);
    pinMode(DOUT, INPUT);

    set_gain(gain);
}

bool HX711::is_ready() {
    return digitalRead(DOUT) == LOW;
}

void HX711::set_gain(byte gain) {
    switch (gain) {
        case 128: // channel A, gain factor 128
            GAIN = 1;
            break;
        case 64:  // channel A, gain factor 64
            GAIN = 3;
            break;
        case 32:  // channel B, gain factor 32
            GAIN = 2;
            break;
    }

    digitalWrite(PD_SCK, LOW);
    read(); // Read once to apply the new gain setting
}

long HX711::read() {
    // Wait for the chip to become ready.
    while (!is_ready()) {
        // In a FreeRTOS environment, vTaskDelay(1) is used instead of yield().
        // This properly yields control to the RTOS scheduler, allowing other
        // tasks to run and preventing a busy-wait loop.
        #if defined(ESP32)
            vTaskDelay(1);
        #else
            yield();
        #endif
    }

    unsigned long value = 0;
    uint8_t data[3] = { 0 };
    uint8_t filler = 0x00;

    // --- Start of Critical Section ---
    // The following bit-banging code is time-sensitive. The portENTER_CRITICAL
    // macro disables task switching on the current core to ensure that the
    // sequence of clock pulses is not interrupted, which would corrupt the reading.
    portENTER_CRITICAL(&mux);

    // Pulse the clock pin 24 times to read the data.
    data[2] = shiftIn(DOUT, PD_SCK, MSBFIRST);
    data[1] = shiftIn(DOUT, PD_SCK, MSBFIRST);
    data[0] = shiftIn(DOUT, PD_SCK, MSBFIRST);

    // Set the channel and gain factor for the next reading using the clock pin.
    // The number of pulses (1, 2, or 3) determines the gain for the next conversion.
    for (unsigned int i = 0; i < GAIN; i++) {
        digitalWrite(PD_SCK, HIGH);
        digitalWrite(PD_SCK, LOW);
    }

    // --- End of Critical Section ---
    portEXIT_CRITICAL(&mux);

    // The HX711 returns a 24-bit two's complement value. We need to perform
    // sign extension to correctly convert it to a 32-bit signed long.
    // If the most significant bit (bit 23) is 1, the value is negative.
    if (data[2] & 0x80) {
        filler = 0xFF; // Pad the most significant byte with 1s for a negative number.
    } else {
        filler = 0x00; // Pad with 0s for a positive number.
    }

    // Construct the 32-bit signed integer value.
    value = ( static_cast<unsigned long>(filler) << 24
            | static_cast<unsigned long>(data[2]) << 16
            | static_cast<unsigned long>(data[1]) << 8
            | static_cast<unsigned long>(data[0]) );

    return static_cast<long>(value);
}

long HX711::read_average(byte times) {
    long sum = 0;
    for (byte i = 0; i < times; i++) {
        sum += read();
        // Yield to other tasks between readings
        #if defined(ESP32)
            vTaskDelay(1);
        #else
            yield();
        #endif
    }
    return sum / times;
}

double HX711::get_value(byte times) {
    return read_average(times) - OFFSET;
}

float HX711::get_units(byte times) {
    return get_value(times) / SCALE;
}

void HX711::tare(byte times) {
    // Read an average value to establish a stable zero point.
    double sum = read_average(times);
    set_offset(sum);
}

void HX711::set_scale(float scale) {
    SCALE = scale;
}

float HX711::get_scale() {
    return SCALE;
}

void HX711::set_offset(long offset) {
    OFFSET = offset;
}

long HX711::get_offset() {
    return OFFSET;
}

void HX711::power_down() {
    digitalWrite(PD_SCK, LOW);
    digitalWrite(PD_SCK, HIGH);
}

void HX711::power_up() {
    digitalWrite(PD_SCK, LOW);
}