/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Configuration.h for FlashForge Creator Pro 2 with BTT Octopus Max EZ
 * IDEX setup with EZ2209 drivers and ESP3D support
 */
#pragma once

#define CONFIGURATION_H_VERSION 02010200

//===========================================================================
//============================= Getting Started =============================
//===========================================================================

// @section info
#define STRING_CONFIG_H_AUTHOR "(crathenkali, FlashForge Creator Pro 2 IDEX)"
#define SHOW_BOOTSCREEN

// @section machine
#ifndef MOTHERBOARD
    #define MOTHERBOARD BOARD_BTT_OCTOPUS_MAX_EZ_V1_0
#endif

// @section serial
#define SERIAL_PORT 1
#define BAUDRATE 115200
#define SERIAL_PORT_2 2 // For ESP3D communication
#define NUM_SERIAL 2

// @section extruder
#define EXTRUDERS 2

// Enable IDEX (Independent Dual Extruders)
#define DUAL_X_CARRIAGE
#if ENABLED(DUAL_X_CARRIAGE)
// Positions of the two X carriages
#define X1_MIN_POS X_MIN_POS   // Set to X_MIN_POS
#define X1_MAX_POS X_BED_SIZE  // Set to X_BED_SIZE
#define X2_MIN_POS 80          // Set to a minimum, typically > 0
#define X2_MAX_POS 353         // Set to the maximum of the 2nd X carriage
#define X2_HOME_DIR 1          // Set to 1. The 2nd X carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // Default X2 home position. Set to X2_MAX_POS
// However: In this mode the HOTEND_OFFSET_X value for the second extruder provides a software
// override for X2_HOME_POS. This also allow recalibration of the distance between the two front-ends
// without modifying the firmware. Remember, you also have to set the second extruder x-offset to 0 in your slicer.
// There are a few selectable movement modes for dual x-carriages using M605 S<mode>
//    Mode 0 (DXC_FULL_CONTROL_MODE): Full user control. The slicer has full control over both x-carriages and can achieve optimal travel results
//                                   as long as it supports dual x-carriages. (M605 S0)
//    Mode 1 (DXC_AUTO_PARK_MODE)   : Auto-park mode. The firmware will automatically park and unpark the x-carriages on tool changes so
//                                   that additional slicer support is not required. (M605 S1)
//    Mode 2 (DXC_DUPLICATION_MODE) : Duplication mode. The firmware will transparently make the second x-carriage and extruder copy all
//                                   actions of the first x-carriage. This allows the printer to print 2 arbitrary items at
//                                   once. (M605 S2)
//    Mode 3 (DXC_MIRRORED_MODE)    : Mirrored mode. Formbot/Vivedino-inspired mirrored mode in which the second extruder duplicates
//                                   the movement of the first except the second extruder is reversed in the X axis.
//                                   (M605 S3)
#define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_AUTO_PARK_MODE
#define TOOLCHANGE_ZRAISE 2.0 // (mm)
// #define TOOLCHANGE_NO_RETURN              // Never return to previous position on toolchange
#define PARK_HEAD_ON_PAUSE // Park the dual x-carriage on pause
#define DXC_AUTO_PARK_X_POS_L X1_MIN_POS
#define DXC_AUTO_PARK_X_POS_R X2_HOME_POS
#define DXC_AUTO_PARK_Y_POS 50
#define DXC_AUTO_PARK_Z_RAISE_FRACTION 0.25 // Auto-park Z raise as a fraction of Z_MAX_POS
#endif

// @section machine
// Travel limits (mm) after homing, corresponding to endstop positions.
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS X_BED_SIZE
#define Y_MAX_POS Y_BED_SIZE
#define Z_MAX_POS 150

// @section geometry
#define X_BED_SIZE 200
#define Y_BED_SIZE 148
#define Z_MAX_POS 150

// @section homing
// Direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

// @section machine
// The size of the print bed
#define BED_CENTER_AT_0_0 // If enabled, the bed center is at (X=0, Y=0)

// @section stepper drivers
#define X_DRIVER_TYPE TMC2209
#define Y_DRIVER_TYPE TMC2209
#define Z_DRIVER_TYPE TMC2209
#define X2_DRIVER_TYPE TMC2209 // Second X carriage
#define E0_DRIVER_TYPE TMC2209
#define E1_DRIVER_TYPE TMC2209

// @section stepper motors
// Default steps per unit for FlashForge Creator Pro 2
#define DEFAULT_AXIS_STEPS_PER_UNIT {80, 80, 400, 93, 93}

// Maximum feedrate (units/s)
#define DEFAULT_MAX_FEEDRATE {300, 300, 5, 25, 25}

// Default max acceleration (units/s^2)
#define DEFAULT_MAX_ACCELERATION {3000, 3000, 100, 10000, 10000}

/**
 * Default Acceleration (units/s^2) for moves that change velocity
 */
#define DEFAULT_ACCELERATION 3000         // X, Y, Z and E acceleration for printing moves
#define DEFAULT_RETRACT_ACCELERATION 3000 // E acceleration for retracts
#define DEFAULT_TRAVEL_ACCELERATION 3000  // X, Y, Z acceleration for travel moves

/**
 * Default Jerk limits (units/s)
 */
#define CLASSIC_JERK
#if ENABLED(CLASSIC_JERK)
#define DEFAULT_XJERK 10.0
#define DEFAULT_YJERK 10.0
#define DEFAULT_ZJERK 0.4
#define DEFAULT_EJERK 5.0
#endif

// @section homing
// Sensorless homing settings
#define SENSORLESS_HOMING // StallGuard capable drivers are recommended
#if ENABLED(SENSORLESS_HOMING)
// TMC2209 stallguard sensitivity
#define X_STALL_SENSITIVITY 8
#define X2_STALL_SENSITIVITY 8
#define Y_STALL_SENSITIVITY 8
// #define Z_STALL_SENSITIVITY  8  // Enable if using sensorless Z homing
// #define SPI_ENDSTOPS              // TMC2130 only
// #define IMPROVE_HOMING_RELIABILITY
#endif

// @section calibrate
// The homing speeds (feedrates) may also be customized for each axis
#define HOMING_FEEDRATE_MM_M {(50 * 60), (50 * 60), (4 * 60)}

// Validate that endstops are triggered on homing moves
#define VALIDATE_HOMING_ENDSTOPS

// @section temperature
#define TEMP_SENSOR_0 1   // 100k thermistor
#define TEMP_SENSOR_1 1   // 100k thermistor
#define TEMP_SENSOR_BED 1 // 100k thermistor

// Extruder temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10 // (seconds) M109 waits for the temperature to stabilize
#define TEMP_WINDOW 1          // (°C) Temperature proximity for the "temperature reached" timer
#define TEMP_HYSTERESIS 3      // (°C) Temperature proximity considered "close enough" to the target

// Below this temperature the heater will be switched off
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define BED_MINTEMP 5

// Above this temperature the heater will be switched off
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define BED_MAXTEMP 150

// @section PID
// Enable PID for hotends
#define PIDTEMP     // See the PID Tuning Guide at https://reprap.org/wiki/PID_Tuning
#define PID_MAX 255 // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_K1 0.95 // Smoothing factor within any PID loop

#if ENABLED(PIDTEMP)
// #define PID_DEBUG             // Print PID debug data to the serial port. Use 'M303 D' to toggle activation.
// #define PID_OPENLOOP          // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
// #define SLOW_PWM_HEATERS      // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
#define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

// Default PID values for Creator Pro 2 (will be tuned during setup)
#define DEFAULT_Kp_LIST {22.20, 22.20}
#define DEFAULT_Ki_LIST {1.08, 1.08}
#define DEFAULT_Kd_LIST {114.00, 114.00}
#else
#define BANG_MAX 255     // Limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#endif

// @section extruder
// Prevent extrusion if the temperature is below EXTRUDE_MINTEMP
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170

// Prevent a single extrusion longer than EXTRUDE_MAXLENGTH
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 200

// @section thermistors
// For hotend 0 thermistor
// #define HEATER_0_USER_THERMISTOR // Enable this if using a custom thermistor
// For hotend 1 thermistor
// #define HEATER_1_USER_THERMISTOR // Enable this if using a custom thermistor
// For bed thermistor
// #define BED_USER_THERMISTOR      // Enable this if using a custom thermistor

// @section safety
// Thermal Protection provides additional protection to your printer from damage
// and fire. Marlin always includes safe min and max temperature ranges which
// protect against a broken or disconnected thermistor wire.
#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders
#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed
// #define THERMAL_PROTECTION_CHAMBER // Enable thermal protection for the heated chamber
// #define THERMAL_PROTECTION_COOLER  // Enable thermal protection for the laser cooling

// @section endstops
// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define X_MIN_ENDSTOP_INVERTING false       // Set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_INVERTING false       // Set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_INVERTING false       // Set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_INVERTING false       // Set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_INVERTING false       // Set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_INVERTING false       // Set to true to invert the logic of the endstop.
#define Z_MIN_PROBE_ENDSTOP_INVERTING false // Set to true to invert the logic of the probe.

// Enable pullup for all endstops to prevent a disconnect
#define ENDSTOPPULLUPS
#if DISABLED(ENDSTOPPULLUPS)
// Disable ENDSTOPPULLUPS to set pullups individually
// #define ENDSTOPPULLUP_XMAX
// #define ENDSTOPPULLUP_YMAX
// #define ENDSTOPPULLUP_ZMAX
// #define ENDSTOPPULLUP_XMIN
// #define ENDSTOPPULLUP_YMIN
// #define ENDSTOPPULLUP_ZMIN
// #define ENDSTOPPULLUP_ZMIN_PROBE
#endif

// @section extras
// Hotend offsets (mm): The last hotend should be set to (0.0, 0.0)
// Based on FlashForge Creator Pro 2 specifications from manual
#define HOTEND_OFFSET_X {0.0, 35.0} // (mm) relative X-offset for each nozzle
#define HOTEND_OFFSET_Y {0.0, 0.0}  // (mm) relative Y-offset for each nozzle
#define HOTEND_OFFSET_Z {0.0, 0.0}  // (mm) relative Z-offset for each nozzle

// @section lcd
// LCD / Controller Selection
#define DWIN_CREALITY_LCD // Creality UI
// #define DWIN_LCD_PROUI        // Pro UI

// @section sdcard
#define SDSUPPORT
#define SDCARD_CONNECTION ONBOARD

// @section safety
#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
  #define MIN_SOFTWARE_ENDSTOP_X
  #define MIN_SOFTWARE_ENDSTOP_Y
  #define MIN_SOFTWARE_ENDSTOP_Z
  #define MIN_SOFTWARE_ENDSTOP_I
  #define MIN_SOFTWARE_ENDSTOP_J
  #define MIN_SOFTWARE_ENDSTOP_K
  #define MIN_SOFTWARE_ENDSTOP_U
  #define MIN_SOFTWARE_ENDSTOP_V
  #define MIN_SOFTWARE_ENDSTOP_W
#endif

// Max software endstops constrain movement within maximum coordinate bounds

#define MAX_SOFTWARE_ENDSTOPS
#if EITHER(MIN_SOFTWARE_ENDSTOPS, MAX_SOFTWARE_ENDSTOPS)
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
  #define MAX_SOFTWARE_ENDSTOP_X
  #define MAX_SOFTWARE_ENDSTOP_Y
  #define MAX_SOFTWARE_ENDSTOP_Z
  #define MAX_SOFTWARE_ENDSTOP_I
  #define MAX_SOFTWARE_ENDSTOP_J
  #define MAX_SOFTWARE_ENDSTOP_K
  #define MAX_SOFTWARE_ENDSTOP_U
  #define MAX_SOFTWARE_ENDSTOP_V
  #define MAX_SOFTWARE_ENDSTOP_W
#define SOFT_ENDSTOPS_MENU_ITEM // Enable/Disable software endstops from the LCD
#endif

// @section interface
#define HOST_ACTION_COMMANDS
#if ENABLED(HOST_ACTION_COMMANDS)
#define HOST_PROMPT_SUPPORT
#endif

// @section extras
// Power loss recovery
#define POWER_LOSS_RECOVERY
#if ENABLED(POWER_LOSS_RECOVERY)
#define PLR_ENABLED_DEFAULT false // Power Loss Recovery enabled by default. (Set with 'M413 Sn' & M500)
// #define BACKUP_POWER_SUPPLY       // Backup power / UPS to move the steppers on power loss
// #define POWER_LOSS_ZRAISE       2 // (mm) Z axis raise on resume (on power loss with UPS)
// #define POWER_LOSS_PIN         44 // Pin to detect power loss. Set to -1 to disable default pin on boards without module.
// #define POWER_LOSS_STATE     HIGH // State of pin indicating power loss
// #define POWER_LOSS_PULLUP         // Set pullup / pulldown as appropriate for your sensor
// #define POWER_LOSS_PULLDOWN
// #define POWER_LOSS_PURGE_LEN   20 // (mm) Length of filament to purge on resume
// #define POWER_LOSS_RETRACT_LEN 10 // (mm) Length of filament to retract on fail. Requires backup power.

// Without a POWER_LOSS_PIN the following option helps reduce wear on the SD card,
// especially with "vase mode" printing. Set too high and vases cannot be continued.
#define POWER_LOSS_MIN_Z_CHANGE 0.05 // (mm) Minimum Z change before saving power-loss data
#endif

// @section interface
// Add 'M73' to set print progress percentage
#define SET_PROGRESS_MANUALLY
#if ENABLED(SET_PROGRESS_MANUALLY)
#define SET_PROGRESS_PERCENT // Display progress percentage on LCD
#define SET_REMAINING_TIME   // Display remaining time on LCD
// #define SET_INTERACTION_TIME     // Display time to next user interaction on LCD
#endif

// @section host
#define EMERGENCY_PARSER                // Break out of compatibility-breaking command lines
#define SERIAL_STATS_MAX_RX_QUEUED 64   // Maximum number of bytes queued for RX
#define SERIAL_STATS_RX_BUFFER_OVERRUNS // Enable RX overrun debugging

// @section interface
#define NO_TIMEOUTS 1000 // Milliseconds

// @section tmc
/**
 * TMC2130, TMC2160, TMC2208, TMC2209, TMC5130 and TMC5160 only
 * Use Trinamic's ultra quiet stepping mode.
 * When disabled, Marlin will use spreadCycle stepping mode.
 */
#define STEALTHCHOP_XY
#define STEALTHCHOP_Z
#define STEALTHCHOP_E

/**
 * Optimize spreadCycle chopper parameters by using predefined parameter sets
 * or with the help of an example included in the library.
 * Provided parameter sets are
 * CHOPPER_DEFAULT_12V
 * CHOPPER_DEFAULT_19V
 * CHOPPER_DEFAULT_24V
 * CHOPPER_DEFAULT_36V
 * CHOPPER_PRUSAMK3_24V // Imported parameters from the official Průša firmware for MK3 (24V)
 * CHOPPER_MARLIN_119   // Old defaults from Marlin v1.1.9
 */
#define CHOPPER_TIMING CHOPPER_DEFAULT_24V

#if AXIS_IS_TMC(X)
#define X_CURRENT 800
#define X_CURRENT_HOME 400
#define X_MICROSTEPS 16
#define X_RSENSE 0.11
#define X_CHAIN_POS -1
#define X_SLEW_RATE 1
#endif

#if AXIS_IS_TMC(X2)
#define X2_CURRENT 800
#define X2_CURRENT_HOME 400
#define X2_MICROSTEPS 16
#define X2_RSENSE 0.11
#define X2_CHAIN_POS -1
#define X2_SLEW_RATE 1
#endif

#if AXIS_IS_TMC(Y)
#define Y_CURRENT 800
#define Y_CURRENT_HOME 400
#define Y_MICROSTEPS 16
#define Y_RSENSE 0.11
#define Y_CHAIN_POS -1
#define Y_SLEW_RATE 1
#endif

#if AXIS_IS_TMC(Z)
#define Z_CURRENT 800
#define Z_CURRENT_HOME 400
#define Z_MICROSTEPS 16
#define Z_RSENSE 0.11
#define Z_CHAIN_POS -1
#define Z_SLEW_RATE 1
#endif

#if AXIS_IS_TMC(E0)
#define E0_CURRENT 800
#define E0_MICROSTEPS 16
#define E0_RSENSE 0.11
#define E0_CHAIN_POS -1
#define E0_SLEW_RATE 1
#endif

#if AXIS_IS_TMC(E1)
#define E1_CURRENT 800
#define E1_MICROSTEPS 16
#define E1_RSENSE 0.11
#define E1_CHAIN_POS -1
#define E1_SLEW_RATE 1
#endif

/**
 * Monitor Trinamic drivers
 * Continually watch all enabled Trinamic drivers and report any error state or timeout.
 * To find out the cause of the error (like overtemperature, power outage, etc.)
 * you have to examine the 'drv_status' byte as reported by TMC_DEBUG.
 */
#define MONITOR_DRIVER_STATUS

#if ENABLED(MONITOR_DRIVER_STATUS)
#define CURRENT_STEP_DOWN 50 // [mA]
#define REPORT_CURRENT_CHANGE
#define STOP_ON_ERROR
#endif

/**
 * TMC2209 stepper drivers have an integrated UART serial interface which can be used
 * to communicate with the driver. Hardware serial communication ports are recommended.
 */
#define TMC_USE_SW_SPI // Use software SPI to communicate with TMC drivers

// @section extras
// Enable G26 Mesh Validation Pattern tool.
// #define G26_MESH_VALIDATION
#if ENABLED(G26_MESH_VALIDATION)
#define MESH_TEST_NOZZLE_SIZE 0.4  // (mm) Diameter of primary nozzle.
#define MESH_TEST_LAYER_HEIGHT 0.2 // (mm) Default layer height for G26.
#define MESH_TEST_HOTEND_TEMP 205  // (°C) Default nozzle temperature for G26.
#define MESH_TEST_BED_TEMP 60      // (°C) Default bed temperature for G26.
#define G26_XY_FEEDRATE 20         // (mm/s) Feedrate for XY Moves for the G26 Mesh Validation Tool.
#define G26_RETRACT_MULTIPLIER 1.0 // G26 Q (retraction) used by default between mesh test elements.
#endif