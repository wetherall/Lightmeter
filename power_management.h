/**
 * Power Management Functions
 * 
 * This header file defines functions for managing power consumption,
 * including sleep modes and wake-up handling. Power management is
 * critical for battery-powered devices to extend operational life.
 * 
 * The AVR microcontroller has several sleep modes with different
 * power-saving levels. In this project, we use the deepest sleep
 * mode (Power-Down) to minimize power consumption when idle.
 */

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include <stdint.h>

/**
 * FUNCTION DECLARATIONS
 */

/**
 * Enter sleep mode
 * 
 * This function puts the device into a low-power sleep state to conserve
 * battery life. In sleep mode, most peripherals are disabled, and the CPU
 * stops executing instructions. The device will wake up when an interrupt
 * occurs (e.g., a button press).
 * 
 * Before entering sleep mode, this function:
 * 1. Disables unnecessary peripherals
 * 2. Turns off all LEDs
 * 3. Puts the VEML7700 in power save mode
 * 4. Configures the AVR's sleep mode
 * 
 * The device remains in sleep mode until an interrupt occurs.
 */
void enter_sleep_mode(void);

/**
 * Wake from sleep mode
 * 
 * This function restores normal operation after waking from sleep mode.
 * It is called automatically when the device wakes up from sleep.
 * 
 * This function:
 * 1. Disables sleep mode
 * 2. Re-enables peripherals
 * 3. Restores the shift registers to normal operation
 * 4. Puts the VEML7700 back into normal mode
 */
void wake_from_sleep(void);

/**
 * Configure sleep mode
 * 
 * This function prepares the microcontroller for sleep mode by:
 * 1. Setting the desired sleep mode (Power-Down)
 * 2. Enabling the sleep mode functionality
 * 
 * This is called by enter_sleep_mode() before actually entering sleep.
 */
void configure_sleep_mode(void);

/**
 * Get the battery level
 * 
 * This function measures the current battery voltage and
 * converts it to a percentage (0-100).
 * 
 * Returns:
 *   A value from 0 to 100 representing the battery percentage
 *   (0 = empty, 100 = full)
 */
uint8_t get_battery_level(void);

#endif /* POWER_MANAGEMENT_H */