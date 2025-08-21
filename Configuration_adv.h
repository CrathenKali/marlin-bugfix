/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Configuration_adv.h
 *
 * Advanced settings.
 * Only change these if you know exactly what you're doing.
 * Some of these settings can damage your printer if improperly set!
 *
 * Basic settings can be found in Configuration.h
 */
#define CONFIGURATION_ADV_H_VERSION 02010300

// @section develop

/**
 * Configuration Export
 *
 * Export the configuration as part of the build. (See signature.py)
 * Output files are saved with the build (e.g., .pio/build/mega2560).
 *
 * See `build_all_examples --ini` as an example of config.ini archiving.
 *
 *  1 = marlin_config.json - Dictionary containing the configuration.
 *      This file is also generated for CONFIGURATION_EMBEDDING.
 *  2 = config.ini - File format for PlatformIO preprocessing.
 *  3 = schema.json - The entire configuration schema. (13 = pattern groups)
 *  4 = schema.yml - The entire configuration schema.
 *  5 = Config.h - Minimal configuration by popular demand.
 */
 //#define CONFIG_EXPORT 105 // :[1:'JSON', 2:'config.ini', 3:'schema.json', 4:'schema.yml', 5:'Config.h']

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================
// @section temperature

/**
 * Thermocouple sensors are quite sensitive to noise.  Any noise induced in
 * the sensor wires, such as by stepper motor wires run in parallel to them,
 * may result in the thermocouple sensor reporting spurious errors.  This
 * value is the number of errors which can occur in a row before the error
 * is reported.  This allows us to ignore intermittent error conditions while
 * still detecting an actual failure, which should result in a continuous
 * stream of errors from the sensor.
 *
 * Set this value to 0 to fail on the first error to occur.
 */
#define THERMOCOUPLE_MAX_ERRORS 15

//
// Custom Thermistor 1000 parameters
//
#if TEMP_SENSOR_0 == 1000
  #define HOTEND0_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND0_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND0_BETA                    3950 // Beta value
  #define HOTEND0_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_1 == 1000
  #define HOTEND1_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND1_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND1_BETA                    3950 // Beta value
  #define HOTEND1_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_2 == 1000
  #define HOTEND2_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND2_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND2_BETA                    3950 // Beta value
  #define HOTEND2_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_3 == 1000
  #define HOTEND3_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND3_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND3_BETA                    3950 // Beta value
  #define HOTEND3_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_4 == 1000
  #define HOTEND4_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND4_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND4_BETA                    3950 // Beta value
  #define HOTEND4_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_5 == 1000
  #define HOTEND5_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND5_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND5_BETA                    3950 // Beta value
  #define HOTEND5_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_6 == 1000
  #define HOTEND6_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND6_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND6_BETA                    3950 // Beta value
  #define HOTEND6_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_7 == 1000
  #define HOTEND7_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND7_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND7_BETA                    3950 // Beta value
  #define HOTEND7_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_BED == 1000
  #define BED_PULLUP_RESISTOR_OHMS        4700 // Pullup resistor
  #define BED_RESISTANCE_25C_OHMS       100000 // Resistance at 25C
  #define BED_BETA                        3950 // Beta value
  #define BED_SH_C_COEFF                     0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_CHAMBER == 1000
  #define CHAMBER_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define CHAMBER_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define CHAMBER_BETA                    3950 // Beta value
  #define CHAMBER_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_COOLER == 1000
  #define COOLER_PULLUP_RESISTOR_OHMS     4700 // Pullup resistor
  #define COOLER_RESISTANCE_25C_OHMS    100000 // Resistance at 25C
  #define COOLER_BETA                     3950 // Beta value
  #define COOLER_SH_C_COEFF                  0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_PROBE == 1000
  #define PROBE_PULLUP_RESISTOR_OHMS      4700 // Pullup resistor
  #define PROBE_RESISTANCE_25C_OHMS     100000 // Resistance at 25C
  #define PROBE_BETA                      3950 // Beta value
  #define PROBE_SH_C_COEFF                   0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_BOARD == 1000
  #define BOARD_PULLUP_RESISTOR_OHMS      4700 // Pullup resistor
  #define BOARD_RESISTANCE_25C_OHMS     100000 // Resistance at 25C
  #define BOARD_BETA                      3950 // Beta value
  #define BOARD_SH_C_COEFF                   0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_REDUNDANT == 1000
  #define REDUNDANT_PULLUP_RESISTOR_OHMS  4700 // Pullup resistor
  #define REDUNDANT_RESISTANCE_25C_OHMS 100000 // Resistance at 25C
  #define REDUNDANT_BETA                  3950 // Beta value
  #define REDUNDANT_SH_C_COEFF               0 // Steinhart-Hart C coefficient
#endif

/**
 * Thermocouple Options — for MAX6675 (-2), MAX31855 (-3), and MAX31865 (-5).
 */
 //#define TEMP_SENSOR_FORCE_HW_SPI                // Ignore SCK/MOSI/MISO pins; use CS and the default SPI bus.
 //#define MAX31865_SENSOR_WIRES_0   2             // (2-4) Number of wires for the probe connected to a MAX31865 board.
 //#define MAX31865_SENSOR_WIRES_1   2
 //#define MAX31865_SENSOR_WIRES_2   2
 //#define MAX31865_SENSOR_WIRES_BED 2

 //#define MAX31865_50HZ_FILTER                    // Use a 50Hz filter instead of the default 60Hz.
 //#define MAX31865_USE_READ_ERROR_DETECTION       // Treat value spikes (20°C delta in under 1s) as read errors.

 //#define MAX31865_USE_AUTO_MODE                  // Read faster and more often than 1-shot; bias voltage always on; slight effect on RTD temperature.
 //#define MAX31865_MIN_SAMPLING_TIME_MSEC     100 // (ms) 1-shot: minimum read interval. Reduces bias voltage effects by leaving sensor unpowered for longer intervals.
 //#define MAX31865_IGNORE_INITIAL_FAULTY_READS 10 // Ignore some read faults (keeping the temperature reading) to work around a possible issue (#23439).

 //#define MAX31865_WIRE_OHMS_0              0.95f // For 2-wire, set the wire resistances for more accurate readings.
 //#define MAX31865_WIRE_OHMS_1              0.0f
 //#define MAX31865_WIRE_OHMS_2              0.0f
 //#define MAX31865_WIRE_OHMS_BED            0.0f

/**
 * Hephestos 2 24V heated bed upgrade kit.
 * https://www.en3dstudios.com/product/bq-hephestos-2-heated-bed-kit/
 */
 //#define HEPHESTOS2_HEATED_BED_KIT
#if ENABLED(HEPHESTOS2_HEATED_BED_KIT)
  #define HEATER_BED_INVERTING true
#endif

//===========================================================================
//============================= Mechanical Settings =========================
//===========================================================================
// @section homing

/**
 * Sensorless Homing
 * Enable sensorless homing for the Octopus Max EZ with EZ-TMC2209 drivers.
 * Requires stallGuard support and proper tuning of stall sensitivity.
 */
#define SENSORLESS_HOMING // Enable sensorless homing
#if ENABLED(SENSORLESS_HOMING)
  #define X_STALL_SENSITIVITY  50 // Tune for X axis (adjust based on testing)
  #define Y_STALL_SENSITIVITY  50 // Tune for Y axis (adjust based on testing)
  #define Z_STALL_SENSITIVITY  30 // Tune for Z axis (adjust based on testing)
  #define SENSORLESS_BACKOFF_MM 5 // Backoff distance after stall detection
#endif

//===========================================================================
//============================= Additional Features =========================
//===========================================================================
// @section extras

/**
 * ESP3D WiFi Support
 * Configure for ESP32 DevKit as a web interface.
 */
#define ESP3D_WIFISUPPORT // Enable ESP3D WiFi support
#if ENABLED(ESP3D_WIFISUPPORT)
  #define ESP3D_HOSTNAME "FlashForgeESP3D" // Custom hostname
  #define ESP3D_WEBSUPPORT // Enable web interface
  #define ESP3D_SD // Enable SD card support via ESP32
  #define ESP3D_TELNET // Enable Telnet for remote control
  #define ESP3D_SERIAL_PORT 1 // Use UART1 (adjust based on ESP32 connection)
  #define ESP3D_BAUDRATE 115200 // Match baud rate with ESP32
#endif

//===========================================================================
//============================= MMU Settings ================================
//===========================================================================
// @section mmu

// No MMU configured for this setup
//#define HAS_PRUSA_MMU2
//#define HAS_PRUSA_MMU3

/**
 * Advanced Print Counter settings
 * @section stats
 */
#if ENABLED(PRINTCOUNTER)
  #define SERVICE_WARNING_BUZZES  3
  // Activate up to 3 service interval watchdogs
  //#define SERVICE_NAME_1      "Service S"
  //#define SERVICE_INTERVAL_1  100 // print hours
  //#define SERVICE_NAME_2      "Service L"
  //#define SERVICE_INTERVAL_2  200 // print hours
  //#define SERVICE_NAME_3      "Service 3"
  //#define SERVICE_INTERVAL_3    1 // print hours
#endif

// @section develop

//
// M100 Free Memory Watcher to debug memory usage
//
//#define M100_FREE_MEMORY_WATCHER

//
// M42 - Set pin states
//
//#define DIRECT_PIN_CONTROL

//
// M43 - display pin status, toggle pins, watch pins, watch endstops & toggle LED, test servo probe
//
//#define PINS_DEBUGGING

// Enable Tests that will run at startup and produce a report
//#define MARLIN_TEST_BUILD

// Enable Marlin dev mode which adds some special commands
//#define MARLIN_DEV_MODE

#if ENABLED(MARLIN_DEV_MODE)
  /**
   * D576 - Buffer Monitoring
   * To help diagnose print quality issues stemming from empty command buffers.
   */
  //#define BUFFER_MONITORING
#endif

/**
 * Postmortem Debugging captures misbehavior and outputs the CPU status and backtrace to serial.
 * When running in the debugger it will break for debugging. This is useful to help understand
 * a crash from a remote location. Requires ~400 bytes of SRAM and 5Kb of flash.
 */
//#define POSTMORTEM_DEBUGGING

/**
 * Software Reset options
 */
//#define SOFT_RESET_VIA_SERIAL         // 'KILL' and '^X' commands will soft-reset the controller
//#define SOFT_RESET_ON_KILL            // Use a digital button to soft-reset the controller after KILL

// Report uncleaned reset reason from register r2 instead of MCUSR. Supported by Optiboot on AVR.
//#define OPTIBOOT_RESET_REASON

// Shrink the build for smaller boards by sacrificing some serial feedback
//#define MARLIN_SMALL_BUILD