/**
 * Power Management Implementation
 * 
 * This file contains the implementation of power management functions
 * for the light meter device, including sleep modes and battery monitoring.
 * 
 * Effective power management is crucial for extending battery life in portable
 * devices, especially with a single LIR2450 lithium coin cell power source.
 * 
 * These functions control:
 * - Sleep modes (low power states)
 * - Wake-up behavior
 * - Battery voltage monitoring via ADC
 * 
 * Updated to fix ADC averaging, consistent voltage references, and improved
 * sleep mode power consumption.
 * 
 * IMPROVED: Removed mode button hold references for cleaner code
 */

 #include <avr/io.h>
 #include <avr/sleep.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include <stdbool.h>
 #include "power_management.h"
 #include "led_matrix.h"       // For LED control
 #include "veml7700.h"         // For light sensor power management
 
 /* Define the actual VDD voltage in millivolts 
  * This should match your STNS01 regulator output
  * Common values are 3100mV or 3300mV
  */
 #define VDD_VOLTAGE_MV 3100   // 3.1V from STNS01 regulator
 
 // External references to time tracking variables (defined in main.c)
 extern volatile uint32_t current_time;
 extern volatile uint32_t last_activity_time;
 extern volatile bool critical_battery_shutdown;
 extern volatile bool displaying_battery_warning;
 extern volatile bool battery_warning_active;
 extern void save_led_state(void);
 extern void restore_led_state(void);
 
 // External button flags that need to be cleared on wake
 extern volatile bool wake_flag;
 extern volatile bool mode_button_flag;
 extern volatile bool up_button_flag;
 extern volatile bool down_button_flag;
 
 /**
  * Initialize battery monitoring
  * 
  * This function sets up the ADC (Analog to Digital Converter) to measure
  * the battery voltage. The LIR2450 battery voltage is connected to PC2.
  * 
  * NOTE: There should be a voltage divider on the PCB to bring the battery voltage
  * (up to 4.2V fully charged) within the ADC reference range (typically 3.1V).
  * A 470kΩ + 220kΩ divider gives a division factor of 0.319.
  */
 void init_battery_monitoring(void) {
     // Configure PC2 as an input for ADC
     BATTERY_ADC_PORT.DIRCLR = BATTERY_ADC_PIN;
     
     // Disable digital input buffer on the ADC pin to save power
     PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
     
     // Configure ADC
     // Enable ADC
     ADC0.CTRLA = ADC_ENABLE_bm;
     
     // Set ADC resolution to 10-bit and use VDD as reference
     // The VDD reference uses whatever voltage is powering the chip
     ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
     
     // Configure for single-ended input on AIN2 (PC2)
     ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
     
     // Low-pass filter to reduce noise - accumulate 16 samples
     // When reading, we'll need to divide by 16 or right-shift by 4
     ADC0.CTRLB = ADC_SAMPNUM_ACC16_gc;
 }
 
 /**
  * Get battery voltage in millivolts
  * 
  * This function reads the battery voltage using the ADC, applies
  * calibration factors, and returns the actual battery voltage in millivolts.
  * 
  * It assumes a voltage divider circuit that brings the battery voltage
  * within the ADC reference range.
  * 
  * Returns:
  *   Battery voltage in millivolts (e.g., 3700 for 3.7V)
  */
 uint16_t get_battery_voltage(void) {
     // Start ADC conversion
     ADC0.COMMAND = ADC_STCONV_bm;
     
     // Wait for conversion to complete
     while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
     
     // Read ADC result - this is the sum of 16 samples due to ACC16 mode
     // We need to divide by 16 (or right-shift by 4) to get the average
     uint16_t adc_result = ADC0.RES >> 4;  // Average of 16 samples
     
     // Calculate voltage in millivolts
     // With VDD reference and 10-bit ADC: each step = VDD/1024
     // For a 690kΩ divider (220kΩ + 470kΩ), division factor is 220/(470+220) = 0.319
     // So actual battery voltage = ADC voltage * (1/0.319) = ADC voltage * 3.13
     
     // Step 1: Convert ADC reading to voltage at the ADC pin (in mV)
     float adc_voltage_mv = (adc_result * (float)VDD_VOLTAGE_MV) / 1024.0;
     
     // Step 2: Calculate actual battery voltage accounting for voltage divider
     // The divider ratio is (470+220)/220 = 3.13
     uint16_t battery_voltage_mv = (uint16_t)(adc_voltage_mv * 3.13);
     
     return battery_voltage_mv;
 }
 
 /**
  * Check if battery is low but not critical
  * 
  * This function checks if the battery voltage is below the low threshold
  * but still above the critical threshold.
  * 
  * Returns:
  *   true if battery is low but not critical, false otherwise
  */
 bool is_battery_low(void) {
     uint16_t voltage = get_battery_voltage();
     return (voltage < BATTERY_LOW_MV && voltage >= BATTERY_CRITICAL_MV);
 }
 
 /**
  * Check if battery is critically low
  * 
  * This function checks if the battery voltage is below the critical threshold.
  * When the battery is critically low, the device should enter a deep sleep mode
  * to protect the battery from over-discharge, which can damage lithium batteries.
  * 
  * Returns:
  *   true if battery is critically low, false otherwise
  */
 bool is_battery_critical(void) {
     uint16_t voltage = get_battery_voltage();
     return (voltage < BATTERY_CRITICAL_MV);
 }
 
 /**
  * Flash the bottom two LEDs three times to indicate low battery
  * 
  * This function uses the battery_warning_active flag to prevent
  * other display updates from interfering during the warning.
  */
 void flash_low_battery_warning(void) {
     // Save current LED state if not already saved
     save_led_state();
     
     // Clear all LEDs first
     for (int i = 0; i < LEDS_PER_COLUMN; i++) {
         set_led(i, 0, false);
         set_led(i, 1, false);
     }
     
     // Flash the bottom two LEDs three times
     for (int i = 0; i < 3; i++) {
        // Turn on the f/1 and 1s LEDs (position 0 in both columns)
        set_led(0, 0, true);  // f/1 LED in ISO/aperture column
        set_led(0, 1, true);  // 1s LED in shutter speed column
        _delay_ms(200);  // On for 200ms
        
        // Turn off the LEDs
        set_led(0, 0, false);
        set_led(0, 1, false);
        _delay_ms(200);  // Off for 200ms
    }
     
     // Restore the original LED state
     restore_led_state();
 }
 
 /**
  * Configure sleep mode settings
  * 
  * This function prepares the microcontroller for entering sleep mode
  * by configuring which sleep mode to use.
  */
 void configure_sleep_mode(void) {
     /* Set sleep mode to Power-Down for minimal power consumption
      * 
      * The AVR microcontroller has several sleep modes with different
      * levels of power saving. Power-Down is the deepest sleep mode,
      * which turns off almost everything except:
      * - The external interrupt system (for wake-up)
      * - The watchdog timer (if enabled)
      * 
      * In Power-Down mode, the CPU stops executing and most peripherals
      * are disabled, resulting in current consumption of typically
      * less than 1 microamp from the microcontroller itself.
      */
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
     
     /* Enable sleep mode
      * 
      * This sets the SE (Sleep Enable) bit in the SMCR (Sleep Mode Control Register)
      * but doesn't actually enter sleep mode yet. That happens when sleep_mode()
      * is called.
      */
     sleep_enable();
 }
 
 /**
  * Enter sleep mode to save power
  * 
  * This function puts the device into a low-power state to extend battery life.
  * The device will wake up when a button is pressed (via interrupt).
  * 
  * Our complete sleep strategy involves:
  * 1. Turning off all LEDs (by disabling rows and enabling columns)
  * 2. Setting I2C pins to a defined state to prevent floating
  * 3. Putting the VEML7700 light sensor into power saving mode
  * 4. Disabling the TWI peripheral
  * 5. Putting the microcontroller into Power-Down sleep mode
  * 
  * This combination minimizes power consumption to extend battery life.
  */
 void enter_sleep_mode(void) {
     /* Turn off all LEDs to save power
      * 
      * In our matrix setup, we need to:
      * 1. Disable all rows (set LOW)
      * 2. Set all columns HIGH (cathodes)
      * This ensures no current flows through any LED
      */
     // Disable all rows (anodes) by setting them LOW
     ROWS_PORT_A.OUTCLR = ROW1_PIN | ROW2_PIN | ROW3_PIN;
     ROWS_PORT_C.OUTCLR = ROW4_PIN | ROW5_PIN | ROW6_PIN;
     
     // Set all columns (cathodes) HIGH to ensure LEDs are off
     COLS_PORT.OUTSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
     
     /* Configure I2C pins to prevent floating during sleep
      * 
      * Floating inputs can cause increased power consumption due to
      * the input buffer switching states. By driving them to a defined
      * level, we eliminate this source of power waste.
      */
     // Set I2C pins as outputs temporarily
     PORTB.DIRSET = PIN0_bm | PIN1_bm;  // SDA and SCL as outputs
     // Drive them low (could also drive high, but low is typically preferred)
     PORTB.OUTCLR = PIN0_bm | PIN1_bm;  // Drive low
     
     /* Disable the TWI peripheral completely
      * 
      * This saves additional power as the peripheral won't consume
      * any current while disabled.
      */
     TWI0.MCTRLA = 0;  // Disable TWI
     
     /* Put VEML7700 into power saving mode
      * 
      * The light sensor has a power saving mode that significantly
      * reduces its current consumption when not taking readings.
      * Mode 3 is the maximum power saving setting.
      */
     veml7700_power_save_enable(3); // Maximum power saving
     
     /* Configure sleep mode
      * 
      * This sets up which sleep mode we'll use (Power-Down)
      */
     configure_sleep_mode();
     
     /* Enter sleep mode
      * 
      * The sleep_mode() function:
      * 1. Disables interrupts temporarily
      * 2. Sets the sleep bit
      * 3. Re-enables interrupts
      * 4. Executes the SLEEP instruction
      * 
      * The CPU stops executing at this point and enters sleep mode.
      * Execution will resume from the next instruction when an interrupt occurs.
      * 
      * In our case, button presses generate interrupts that wake the device.
      */
     sleep_mode();
     
     /* Code resumes here after wake-up (button interrupt)
      * 
      * When an interrupt occurs (like a button press), the CPU wakes up,
      * executes the interrupt service routine, and then continues from
      * this point in the code.
      */
     
     /* After waking up, re-enable peripherals
      * 
      * This function handles all the post-wakeup configuration
      */
     wake_from_sleep();
 }
 
 /**
  * Wake up from sleep mode
  * 
  * This function restores normal operation after waking from sleep.
  * It re-enables all peripherals that were disabled before sleeping.
  * 
  * IMPROVED: Removed mode button hold flag clearing
  */
 void wake_from_sleep(void) {
     /* Disable sleep mode
      * 
      * This clears the SE (Sleep Enable) bit in the SMCR register,
      * preventing accidental re-entry into sleep mode.
      */
     sleep_disable();
 
     /* Re-configure I2C pins back to their normal state
      * 
      * We need to restore them as inputs with pull-ups for I2C operation
      */
     PORTB.DIRCLR = PIN0_bm | PIN1_bm;  // SDA and SCL back to inputs
     PORTB.PIN0CTRL = PORT_PULLUPEN_bm;  // Re-enable pull-up on SDA
     PORTB.PIN1CTRL = PORT_PULLUPEN_bm;  // Re-enable pull-up on SCL
     
     /* Re-enable I2C/TWI peripheral */
     TWI0.MCTRLA = TWI_ENABLE_bm;
     
     /* Re-configure TWI for smart mode */
     TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
     
     /* Re-enable ADC for battery monitoring */
     ADC0.CTRLA |= ADC_ENABLE_bm;
     
     /* Take VEML7700 out of power saving mode
      * 
      * This returns the light sensor to normal operation mode
      * so it can take readings again.
      */
     veml7700_power_save_disable();
     
     /* Clear any pending button flags
      * 
      * The wake interrupt might have set button flags that we don't
      * want to process immediately after waking up.
      */
     wake_flag = false;
     mode_button_flag = false;
     up_button_flag = false;
     down_button_flag = false;
     
     /* Reset the activity timer
      * 
      * This ensures we have a full timeout period after waking up
      * before going back to sleep.
      */
     last_activity_time = current_time;
     
     /* Allow time for devices to wake up
      * 
      * Some devices need a short time to fully wake up and stabilize.
      * 10ms is typically enough for I2C devices.
      */
     _delay_ms(10);
 
     /* Check battery level immediately after wake */
     // If critically low, set flag to go back to sleep (will be handled in main loop)
     if (is_battery_critical()) {
         critical_battery_shutdown = true;
     }
     // If low but not critical, show warning
     else if (is_battery_low()) {
         displaying_battery_warning = true;
     }
 }