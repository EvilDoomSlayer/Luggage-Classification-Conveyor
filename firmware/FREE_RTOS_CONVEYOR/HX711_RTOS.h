/**
 * @file HX711_RTOS.h
 * @author Bogdan Necula, Andreas Motl, M. Alejandro Sáchez R.
 * @brief Public interface for an HX711 ADC library, modified for FreeRTOS (ESP32).
 * @version 0.8 (RTOS Modified)
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2018 Bogdan Necula
 *
 * This is a modified version of the popular HX711 library, specifically adapted
 * for use with an ESP32 running FreeRTOS. Key modifications include:
 * - Replacing `yield()` with `vTaskDelay(1)` to ensure proper task scheduling
 * and prevent CPU hogging in a multitasking environment.
 * - Using a critical section (mutex) during the bit-banging read operation to
 * prevent the RTOS scheduler from interrupting the time-sensitive communication,
 * which would otherwise corrupt the data.
 */

#ifndef HX711_RTOS_h
#define HX711_RTOS_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class HX711
{
private:
    byte PD_SCK;  // Power Down and Serial Clock Input Pin
    byte DOUT;    // Serial Data Output Pin
    byte GAIN;    // Amplification factor, configured by clock pulses
    long OFFSET = 0;  // Stores the tare weight (raw reading at zero).
    float SCALE = 1;  // The calibration factor used to convert raw data to user units (e.g., grams).

public:
    /**
     * @brief Construct a new HX711 object and initialize it.
     * @param dout The GPIO pin for the Data Out (DOUT) line.
     * @param pd_sck The GPIO pin for the Serial Clock (PD_SCK) line.
     * @param gain The gain factor. 128 or 64 for channel A, 32 for channel B.
     */
    HX711(byte dout, byte pd_sck, byte gain = 128);

    /**
     * @brief Construct a new, uninitialized HX711 object.
     * @note You must call begin() before using the object.
     */
    HX711();

    virtual ~HX711();

    /**
     * @brief Initialize the HX711 with pins and gain.
     * @param dout The GPIO pin for the Data Out (DOUT) line.
     * @param pd_sck The GPIO pin for the Serial Clock (PD_SCK) line.
     * @param gain The gain factor. 128 or 64 for channel A, 32 for channel B.
     */
    void begin(byte dout, byte pd_sck, byte gain = 128);

    /**
     * @brief Check if the HX711 is ready to send data.
     * @note As per the datasheet, DOUT is LOW when data is ready.
     * @return true if data is ready, false otherwise.
     */
    bool is_ready();

    /**
     * @brief Set the gain factor for the next reading.
     * @note This takes effect after the next call to read().
     * @param gain 128 or 64 for channel A; 32 for channel B.
     */
    void set_gain(byte gain = 128);

    /**
     * @brief Waits for the chip to be ready and returns a raw 24-bit reading.
     * @note This is a "blocking" but RTOS-friendly function, as it uses vTaskDelay.
     * @return The raw ADC value as a long.
     */
    long read();

    /**
     * @brief Returns an average of multiple raw readings.
     * @param times The number of readings to average.
     * @return The average raw value.
     */
    long read_average(byte times = 10);

    /**
     * @brief Returns the current raw value with the tare offset subtracted.
     * @param times The number of readings to average.
     * @return The tare-adjusted value.
     */
    double get_value(byte times = 1);

    /**
     * @brief Returns the final calibrated weight in your chosen units.
     * @note This is typically the main function you will use to get a weight.
     * It's equivalent to `(read_average() - OFFSET) / SCALE`.
     * @param times The number of readings to average.
     * @return The weight in user-defined units (e.g., grams).
     */
    float get_units(byte times = 1);

    /**
     * @brief Sets the zero offset (tare).
     * Call this with no load on the scale to zero it out.
     * @param times The number of readings to average for a stable tare value.
     */
    void tare(byte times = 10);

    /**
     * @brief Sets the calibration scale factor.
     * @param scale The factor calculated from a known weight. `scale = raw_reading / known_weight`.
     */
    void set_scale(float scale = 1.f);

    /**
     * @brief Gets the current calibration scale factor.
     * @return The current SCALE value.
     */
    float get_scale();

    /**
     * @brief Manually sets the tare offset value.
     * @param offset The raw ADC value to use as the zero point.
     */
    void set_offset(long offset = 0);

    /**
     * @brief Gets the current tare offset value.
     * @return The current OFFSET value.
     */
    long get_offset();

    /**
     * @brief Puts the HX711 into a low-power sleep mode.
     */
    void power_down();

    /**
     * @brief Wakes up the HX711 from sleep mode.
     */
    void power_up();
};

#endif /* HX711_RTOS_h */