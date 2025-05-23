/**
 * VEML7700 Light Sensor Implementation
 * 
 * This file contains the implementation of functions for
 * interfacing with the VEML7700 ambient light sensor via I2C.
 * 
 * Updated to use a lookup table for integration time delays,
 * reducing excessive wait times and improving responsiveness.
 */

#include <avr/io.h>
#include <util/delay.h>
#include "veml7700.h"
#include "i2c.h"

// Global variable to store the last lux reading
float last_lux_reading = 0.0;

// Current sensor settings
static uint8_t current_gain = VEML7700_GAIN_1_8;
static uint8_t current_integration_time = VEML7700_IT_100MS;

/**
 * Lookup table for integration time delays in milliseconds
 * 
 * This table maps integration time settings to their actual delays.
 * Using a lookup table is more efficient than a switch statement
 * and makes it easy to add safety margins if needed.
 */
static const uint16_t integration_delays[] = {
    100,  // VEML7700_IT_100MS (0x00)
    200,  // VEML7700_IT_200MS (0x01)
    400,  // VEML7700_IT_400MS (0x02)
    800,  // VEML7700_IT_800MS (0x03)
    0,    // Reserved (0x04)
    0,    // Reserved (0x05)
    0,    // Reserved (0x06)
    0,    // Reserved (0x07)
    50,   // VEML7700_IT_50MS  (0x08)
    0,    // Reserved (0x09)
    0,    // Reserved (0x0A)
    0,    // Reserved (0x0B)
    25    // VEML7700_IT_25MS  (0x0C)
};

/**
 * Get the delay in milliseconds for a given integration time setting
 * 
 * This helper function safely retrieves the delay time from our lookup table.
 * It includes bounds checking to prevent array access errors.
 */
static uint16_t get_integration_delay(uint8_t integration_time) {
    // Extract just the integration time bits (bits 6-9)
    uint8_t it_value = integration_time & 0x0F;
    
    // Bounds check
    if (it_value > 12) {
        // Invalid integration time, return default
        return 100;
    }
    
    uint16_t delay = integration_delays[it_value];
    
    // If the delay is 0 (reserved value), return default
    if (delay == 0) {
        return 100;
    }
    
    // Add a small safety margin (5ms) to ensure conversion completes
    return delay + 5;
}

/**
 * Write a register to the VEML7700 sensor
 * 
 * Now includes error checking from the updated I2C functions.
 */
static uint8_t veml7700_write_register(uint8_t reg, uint16_t value) {
    uint8_t data[2];
    
    // Prepare data array (low byte first, then high byte)
    data[0] = value & 0xFF;         // Low byte
    data[1] = (value >> 8) & 0xFF;  // High byte
    
    // Use centralized I2C function with error checking
    return i2c_write(VEML7700_I2C_ADDR, reg, data, 2);
}

/**
 * Read a register from the VEML7700 sensor
 * 
 * Now includes error handling. Returns 0xFFFF on error.
 */
static uint16_t veml7700_read_register(uint8_t reg) {
    uint8_t data[2];
    uint16_t value;
    
    // Use centralized I2C function with error checking
    if (i2c_read(VEML7700_I2C_ADDR, reg, data, 2) != 0) {
        // Error occurred, return maximum value to indicate error
        return 0xFFFF;
    }
    
    // Combine bytes (low byte first, then high byte)
    value = data[0] | (data[1] << 8);
    
    return value;
}

/**
 * Initialize the VEML7700 sensor
 */
void veml7700_init(void) {
    // Configure sensor with default settings
    // ALS integration time: 100ms
    // Gain: 1/8
    // Persistence: 1
    // Interrupts: Disabled
    // Power: Normal mode (not shutdown)
    uint16_t config = current_integration_time | (current_gain << 11);
    veml7700_write_register(VEML7700_REG_CONFIG, config);
    
    // Configure power-saving mode (disabled by default)
    veml7700_write_register(VEML7700_REG_PSM, 0x0000);
    
    // Allow sensor to stabilize
    _delay_ms(50);
}

/**
 * Read the ambient light sensor value
 */
uint16_t veml7700_read_als(void) {
    uint16_t value = veml7700_read_register(VEML7700_REG_ALS);
    
    // Check for error (0xFFFF indicates I2C error)
    if (value == 0xFFFF) {
        // Return 0 to indicate error in light reading
        return 0;
    }
    
    return value;
}

/**
 * Convert raw ALS value to LUX based on gain and integration time
 */
float veml7700_get_lux(void) {
    uint16_t raw_als = veml7700_read_als();
    
    // Check for error
    if (raw_als == 0) {
        return 0.0;
    }
    
    float lux = 0.0;
    
    // Apply gain factor
    float gain_factor = 1.0;
    switch (current_gain) {
        case VEML7700_GAIN_1:
            gain_factor = 1.0;
            break;
        case VEML7700_GAIN_2:
            gain_factor = 0.5;
            break;
        case VEML7700_GAIN_1_8:
            gain_factor = 8.0;
            break;
        case VEML7700_GAIN_1_4:
            gain_factor = 4.0;
            break;
    }
    
    // Apply integration time factor
    float it_factor = 1.0;
    switch (current_integration_time) {
        case VEML7700_IT_25MS:
            it_factor = 4.0;
            break;
        case VEML7700_IT_50MS:
            it_factor = 2.0;
            break;
        case VEML7700_IT_100MS:
            it_factor = 1.0;
            break;
        case VEML7700_IT_200MS:
            it_factor = 0.5;
            break;
        case VEML7700_IT_400MS:
            it_factor = 0.25;
            break;
        case VEML7700_IT_800MS:
            it_factor = 0.125;
            break;
    }
    
    // Calculate lux value
    // The factor 0.0576 is based on datasheet conversion for default settings
    lux = (float)raw_als * 0.0576 * gain_factor * it_factor;
    
    return lux;
}

/**
 * Set the gain of the sensor
 */
void veml7700_set_gain(uint8_t gain) {
    uint16_t config = veml7700_read_register(VEML7700_REG_CONFIG);
    
    // Check for error
    if (config == 0xFFFF) {
        return;
    }
    
    config &= ~(0x03 << 11); // Clear gain bits
    config |= (gain << 11);   // Set new gain
    veml7700_write_register(VEML7700_REG_CONFIG, config);
    current_gain = gain;
}

/**
 * Set the integration time of the sensor
 */
void veml7700_set_integration_time(uint8_t it) {
    uint16_t config = veml7700_read_register(VEML7700_REG_CONFIG);
    
    // Check for error
    if (config == 0xFFFF) {
        return;
    }
    
    config &= ~(0x0F << 6); // Clear integration time bits
    config |= it;           // Set new integration time
    veml7700_write_register(VEML7700_REG_CONFIG, config);
    current_integration_time = it;
}

/**
 * Enable power save mode
 */
void veml7700_power_save_enable(uint8_t mode) {
    // Mode is 0-3, with 3 being the most power saving
    veml7700_write_register(VEML7700_REG_PSM, 0x0001 | (mode << 1));
}

/**
 * Disable power save mode
 */
void veml7700_power_save_disable(void) {
    veml7700_write_register(VEML7700_REG_PSM, 0x0000);
}

/**
 * Wait for integration to complete with optimized delay
 * 
 * This helper function uses our lookup table to wait the appropriate
 * amount of time for the current integration time setting.
 */
static void wait_for_integration(void) {
    uint16_t delay_ms = get_integration_delay(current_integration_time);
    
    // Use _delay_ms in a loop for delays > 255ms
    while (delay_ms > 255) {
        _delay_ms(255);
        delay_ms -= 255;
    }
    
    // Delay the remaining time
    if (delay_ms > 0) {
        _delay_ms(delay_ms);
    }
}

/**
 * Measure light with automatic gain and integration time adjustment
 * 
 * This function has been optimized to use the lookup table for delays,
 * reducing wait times and improving responsiveness.
 */
float measure_light(void) {
    // Start with middle settings
    veml7700_set_gain(VEML7700_GAIN_1_8);
    veml7700_set_integration_time(VEML7700_IT_100MS);
    
    // Wait for the integration to complete using optimized delay
    wait_for_integration();
    
    // Take initial reading
    float lux = veml7700_get_lux();
    uint16_t raw_als = veml7700_read_als();
    
    // Check for I2C error
    if (raw_als == 0) {
        // Communication error, return last known good value
        return last_lux_reading;
    }
    
    // Check if we need to adjust settings for very bright light
    if (raw_als > 10000) {
        // Light is very bright, reduce sensitivity
        veml7700_set_gain(VEML7700_GAIN_1_8);
        veml7700_set_integration_time(VEML7700_IT_25MS);
        
        // Wait for integration using optimized delay
        wait_for_integration();
        
        lux = veml7700_get_lux();
        raw_als = veml7700_read_als();
    }
    
    // Check if we need to adjust settings for very low light
    if (raw_als < 100 && raw_als > 0) {
        // Light is very dim, increase sensitivity
        veml7700_set_gain(VEML7700_GAIN_2);
        veml7700_set_integration_time(VEML7700_IT_800MS);
        
        // Wait for integration using optimized delay
        wait_for_integration();
        
        lux = veml7700_get_lux();
    }
    
    // Store the reading
    last_lux_reading = lux;
    
    return lux;
}