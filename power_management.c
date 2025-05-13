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
 */

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "power_management.h"
#include "led_matrix.h"       // Updated to use direct LED matrix control
#include "veml7700.h"         // For light sensor power management

// External references to time tracking variables (defined in main.c)
extern volatile uint32_t current_time;
extern volatile uint32_t last_activity_time;

/**
 * Initialize battery monitoring
 * 
 * This function sets up the ADC (Analog to Digital Converter) to measure
 * the battery voltage. The LIR2450 battery voltage is connected to PC2.
 * 
 * NOTE: There should be a voltage divider on the PCB to bring the battery voltage
 * (up to 4.2V fully charged) within the ADC reference range (typically 3.3V).
 * A 100kΩ + 220kΩ divider gives a division factor of 0.3125, bringing 4.2V down to ~1.31V.
 */
void init_battery_monitoring(void) {
    // Configure PC2 as an input for ADC
    BATTERY_ADC_PORT.DIRCLR = BATTERY_ADC_PIN;
    
    // Disable digital input buffer on the ADC pin to save power
    PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    
    // Configure ADC
    // Enable ADC
    ADC0.CTRLA = ADC_ENABLE_bm;
    
    // Set ADC resolution to 10-bit and use VDD (3.3V) as reference
    ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
    
    // Configure for single-ended input on AIN2 (PC2)
    ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
    
    // Low-pass filter to reduce noise
    ADC0.CTRLB = ADC_SAMPNUM_ACC16_gc;  // Accumulate 16 samples for filtering
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
    
    // Read ADC result (10-bit, 0-1023)
    uint16_t adc_result = ADC0.RES;
    
    // Calculate voltage in millivolts
    // With 3.3V reference and 10-bit ADC: each step = 3.3V/1024 = 3.22mV
    // For a 320kΩ divider (100kΩ + 220kΩ), division factor is 100/(100+220) = 0.3125
    // So actual battery voltage = ADC voltage * (1/0.3125) = ADC voltage * 3.2
    
    // Step 1: Convert ADC reading to voltage at the ADC pin (in mV)
    float adc_voltage_mv = (adc_result * 3300.0) / 1024.0;
    
    // Step 2: Calculate actual battery voltage accounting for voltage divider
    uint16_t battery_voltage_mv = (uint16_t)(adc_voltage_mv * 3.2);
    
    return battery_voltage_mv;
}

/**
 * Get the battery level as a percentage
 * 
 * This function converts the battery voltage to a percentage from 0-100%,
 * where 100% is fully charged (4.2V) and 0% is the critical level (3.0V).
 * 
 * Returns:
 *   Battery level as a percentage (0-100)
 */
uint8_t get_battery_level(void) {
    // Get current battery voltage
    uint16_t voltage = get_battery_voltage();
    
    // Convert to percentage (map voltage range to 0-100%)
    int16_t percentage = ((int32_t)(voltage - BATTERY_PERCENT_0) * 100) / 
                        (BATTERY_PERCENT_100 - BATTERY_PERCENT_0);
    
    // Constrain to valid range (0-100%)
    if (percentage > 100) {
        percentage = 100;
    } else if (percentage < 0) {
        percentage = 0;
    }
    
    return (uint8_t)percentage;
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
 * 2. Putting the VEML7700 light sensor into power saving mode
 * 3. Putting the microcontroller into Power-Down sleep mode
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
    // Turn off all LEDs using the matrix control functions
    for (int i = 0; i < LEDS_PER_COLUMN; i++) {
        set_led(i, 0, false);  // Clear ISO/aperture column
        set_led(i, 1, false);  // Clear shutter speed column
    }
    
    // Also directly set row/column pins to ensure LEDs are off
    // Disable all rows (anodes) by setting them LOW
    ROWS_PORT_A.OUTCLR = ROW1_PIN | ROW2_PIN | ROW3_PIN;
    ROWS_PORT_C.OUTCLR = ROW4_PIN | ROW5_PIN | ROW6_PIN;
    
    // Set all columns (cathodes) HIGH to ensure LEDs are off
    COLS_PORT.OUTSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;

    /* Disable power-hungry peripherals before sleeping */
    // Disable the ADC completely to save power
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
    
    // Disable I2C/TWI module when not needed
    TWI0.MCTRLA &= ~TWI_ENABLE_bm;
    
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
 */
void wake_from_sleep(void) {
    /* Disable sleep mode
     * 
     * This clears the SE (Sleep Enable) bit in the SMCR register,
     * preventing accidental re-entry into sleep mode.
     */
    sleep_disable();

    /* Re-enable all necessary peripherals */
    // Re-enable I2C for the light sensor
    TWI0.MCTRLA |= TWI_ENABLE_bm;
    
    // Re-enable ADC for battery monitoring
    ADC0.CTRLA |= ADC_ENABLE_bm;
    
    /* Take VEML7700 out of power saving mode
     * 
     * This returns the light sensor to normal operation mode
     * so it can take readings again.
     */
    veml7700_power_save_disable();
    
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
}/**
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
 */

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "power_management.h"
#include "led_matrix.h"       // For LED control
#include "veml7700.h"         // For light sensor power management

// External references to time tracking variables (defined in main.c)
extern volatile uint32_t current_time;
extern volatile uint32_t last_activity_time;

/**
 * Initialize battery monitoring
 * 
 * This function sets up the ADC (Analog to Digital Converter) to measure
 * the battery voltage. The LIR2450 battery voltage is connected to PC2.
 * 
 * NOTE: There should be a voltage divider on the PCB to bring the battery voltage
 * (up to 4.2V fully charged) within the ADC reference range (typically 3.3V).
 * A 100kΩ + 220kΩ divider gives a division factor of 0.3125, bringing 4.2V down to ~1.31V.
 */
void init_battery_monitoring(void) {
    // Configure PC2 as an input for ADC
    BATTERY_ADC_PORT.DIRCLR = BATTERY_ADC_PIN;
    
    // Disable digital input buffer on the ADC pin to save power
    PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    
    // Configure ADC
    // Enable ADC
    ADC0.CTRLA = ADC_ENABLE_bm;
    
    // Set ADC resolution to 10-bit and use VDD (3.3V) as reference
    ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
    
    // Configure for single-ended input on AIN2 (PC2)
    ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
    
    // Low-pass filter to reduce noise
    ADC0.CTRLB = ADC_SAMPNUM_ACC16_gc;  // Accumulate 16 samples for filtering
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
    
    // Read ADC result (10-bit, 0-1023)
    uint16_t adc_result = ADC0.RES;
    
    // Calculate voltage in millivolts
    // With 3.3V reference and 10-bit ADC: each step = 3.3V/1024 = 3.22mV
    // For a 320kΩ divider (100kΩ + 220kΩ), division factor is 100/(100+220) = 0.3125
    // So actual battery voltage = ADC voltage * (1/0.3125) = ADC voltage * 3.2
    
    // Step 1: Convert ADC reading to voltage at the ADC pin (in mV)
    float adc_voltage_mv = (adc_result * 3300.0) / 1024.0;
    
    // Step 2: Calculate actual battery voltage accounting for voltage divider
    uint16_t battery_voltage_mv = (uint16_t)(adc_voltage_mv * 3.2);
    
    return battery_voltage_mv;
}

/**
 * Get the battery level as a percentage
 * 
 * This function converts the battery voltage to a percentage from 0-100%,
 * where 100% is fully charged (4.2V) and 0% is the critical level (3.0V).
 * 
 * Returns:
 *   Battery level as a percentage (0-100)
 */
uint8_t get_battery_level(void) {
    // Get current battery voltage
    uint16_t voltage = get_battery_voltage();
    
    // Convert to percentage (map voltage range to 0-100%)
    int16_t percentage = ((int32_t)(voltage - BATTERY_PERCENT_0) * 100) / 
                        (BATTERY_PERCENT_100 - BATTERY_PERCENT_0);
    
    // Constrain to valid range (0-100%)
    if (percentage > 100) {
        percentage = 100;
    } else if (percentage < 0) {
        percentage = 0;
    }
    
    return (uint8_t)percentage;
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
 * 2. Putting the VEML7700 light sensor into power saving mode
 * 3. Putting the microcontroller into Power-Down sleep mode
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
 */
void wake_from_sleep(void) {
    /* Disable sleep mode
     * 
     * This clears the SE (Sleep Enable) bit in the SMCR register,
     * preventing accidental re-entry into sleep mode.
     */
    sleep_disable();
    
    /* Take VEML7700 out of power saving mode
     * 
     * This returns the light sensor to normal operation mode
     * so it can take readings again.
     */
    veml7700_power_save_disable();
    
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
}
