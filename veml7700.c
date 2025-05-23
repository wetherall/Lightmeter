/**
 * VEML7700 Light Sensor Implementation
 * 
 * This file contains the implementation of functions for
 * interfacing with the VEML7700 ambient light sensor via I2C.
 * 
 * Updated to use compile-time constants for integration time delays,
 * reducing code size and fixing compilation errors.
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
 * This helper function waits the appropriate amount of time for the current 
 * integration time setting using compile-time constants only.
 * This is the most efficient approach and avoids all variable delay issues.
 */
static void wait_for_integration(void) {
    // Switch on the actual integration time setting to use compile-time constants
    // Each delay includes the 5ms safety margin from get_integration_delay()
    switch (current_integration_time) {
        case VEML7700_IT_25MS:
            _delay_ms(30);  // 25ms + 5ms safety margin
            break;
            
        case VEML7700_IT_50MS:
            _delay_ms(55);  // 50ms + 5ms safety margin
            break;
            
        case VEML7700_IT_100MS:
            _delay_ms(105); // 100ms + 5ms safety margin
            break;
            
        case VEML7700_IT_200MS:
            _delay_ms(205); // 200ms + 5ms safety margin
            break;
            
        case VEML7700_IT_400MS:
            // For delays > 255ms, break into chunks
            _delay_ms(255);
            _delay_ms(150); // Total: 405ms (400ms + 5ms safety margin)
            break;
            
        case VEML7700_IT_800MS:
            // For delays > 255ms, break into chunks  
            _delay_ms(255);
            _delay_ms(255);
            _delay_ms(255);
            _delay_ms(40);  // Total: 805ms (800ms + 5ms safety margin)
            break;
            
        default:
            // Fallback to 100ms + safety margin for unknown settings
            _delay_ms(105);
            break;
    }
}

/**
 * Measure light with automatic gain and integration time adjustment
 * 
 * This function has been optimized to use compile-time constants for delays,
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