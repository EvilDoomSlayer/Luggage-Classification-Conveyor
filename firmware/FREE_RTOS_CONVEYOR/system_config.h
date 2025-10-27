/**
 * @file system_config.h
 * @author  M. Alejandro Sáchez R.
 * @brief System-wide configuration, primarily for controlling debug serial output.
 * @version 0.2
 * @date 2025-10-26
 *
 * @copyright Copyright (c) 2025
 *
 * This file uses preprocessor macros to create a simple and efficient way to
 * enable or disable all Serial.print() statements across the entire project.
 * When debugging is disabled, the macros compile to nothing, saving program
 * space and execution time.
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// --- DEBUG MODE SWITCH ---
// Set to 1 to activate all DEBUG_PRINT/LN statements for debugging.
// Set to 0 to compile them out for a production release.
#if 1
  #define DEBUG_MODE
#endif

#ifdef DEBUG_MODE
  // If DEBUG_MODE is defined, the macros are aliases for Serial.print functions.
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  // If DEBUG_MODE is not defined, the macros compile to nothing.
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

#endif // SYSTEM_CONFIG_H