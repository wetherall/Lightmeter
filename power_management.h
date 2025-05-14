/**
 * Power Management Functions
 * 
 * This header file defines functions for managing power consumption,
 * including sleep modes, wake-up handling, and battery monitoring.
 * 
 * The design now uses a single LIR2450 rechargeable lithium coin cell
 * instead of the previous 3 LR44 batteries, requiring appropriate voltage
 * monitoring and power management.
 */

 #ifndef POWER_MANAGEMENT_H
 #define POWER_MANAGEMENT_H
 
 #include <stdint.h>
 
 /**
  * CONSTANTS AND DEFINITIONS
  */
 
 /* Battery monitoring */
 #define BATTERY_ADC_PIN      PIN2_bm    // Battery level ADC input on PC2
 #define BATTERY_ADC_PORT     PORTC      // Port for battery ADC pin
 
 /* Battery voltage thresholds for LIR2450 cell */
 #define BATTERY_FULL_MV      4200       // Fully charged LIR2450 (4.2V)
 #define BATTERY_NOMINAL_MV   3700       // Nominal voltage
 #define BATTERY_LOW_MV       3300       // Low battery warning threshold
 #define BATTERY_CRITICAL_MV  3000       // Critical level - protect battery from overdischarge
 
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
  * 3. Restores LED matrix to normal operation
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
  * Initialize battery monitoring
  * 
  * This function initializes the ADC for battery voltage measurement:
  * 1. Configures the ADC pin
  * 2. Sets up the ADC with proper reference and settings
  * 3. Enables the ADC peripheral
  */
 void init_battery_monitoring(void);
 
 /**
  * Get battery voltage in millivolts
  * 
  * This function measures the actual battery voltage by:
  * 1. Taking an ADC reading
  * 2. Converting the ADC value to a voltage
  * 3. Applying any calibration factors
  * 
  * Returns:
  *   The battery voltage in millivolts
  */
 uint16_t get_battery_voltage(void);
 
 /**
  * Check if battery is critically low
  * 
  * This function checks if the battery voltage is below the critical
  * threshold where operation should stop to prevent damage to the battery.
  * 
  * Returns:
  *   true if battery is critically low, false otherwise
  */
 bool is_battery_critical(void);
 
 /**
  * Check if battery is low but not critical
  * 
  * Returns:
  *   true if battery is low but not critical, false otherwise
  */
 bool is_battery_low(void);
 
 /**
  * Flash the bottom two LEDs three times to indicate low battery
  */
 void flash_low_battery_warning(void);
 
 #endif /* POWER_MANAGEMENT_H */