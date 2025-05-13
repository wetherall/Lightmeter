/**
 * VEML7700 Light Sensor Implementation
 * 
 * This file contains the implementation of functions for
 * interfacing with the VEML7700 ambient light sensor via I2C.
 */

#include <avr/io.h>
#include <util/delay.h>
#include "veml7700.h"
#include "i2c.h"  // Added i2c.h include

// Global variable to store the last lux reading
float last_lux_reading = 0.0;

// Current sensor settings
static uint8_t current_gain = VEML7700_GAIN_1_8;
static uint8_t current_integration_time = VEML7700_IT_100MS;

/**
 * Write a register to the VEML7700 sensor
 */
static void veml7700_write_register(uint8_t reg, uint16_t value) {
    uint8_t data[2];
    
    // Prepare data array (low byte first, then high byte)
    data[0] = value & 0xFF;         // Low byte
    data[1] = (value >> 8) & 0xFF;  // High byte
    
    // Use centralized I2C function
    i2c_write(VEML7700_I2C_ADDR, reg, data, 2);
}

/**
 * Read a register from the VEML7700 sensor
 */
static uint16_t veml7700_read_register(uint8_t reg) {
    uint8_t data[2];
    uint16_t value;
    
    // Use centralized I2C function
    i2c_read(VEML7700_I2C_ADDR, reg, data, 2);
    
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
    return veml7700_read_register(VEML7700_REG_ALS);
}

/**
 * Convert raw ALS value to LUX based on gain and integration time
 */
float veml7700_get_lux(void) {
    uint16_t raw_als = veml7700_read_als();
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
 * Measure light with automatic gain and integration time adjustment
 */
float measure_light(void) {
    // Start with middle settings
    veml7700_set_gain(VEML7700_GAIN_1_8);
    veml7700_set_integration_time(VEML7700_IT_100MS);
    
    // Take initial reading
    float lux = veml7700_get_lux();
    uint16_t raw_als = veml7700_read_als();
    
    // Check if we need to adjust settings for very bright light
    if (raw_als > 10000) {
        // Light is very bright, reduce sensitivity
        veml7700_set_gain(VEML7700_GAIN_1_8);
        veml7700_set_integration_time(VEML7700_IT_25MS);
        _delay_ms(50); // Allow sensor to adjust
        lux = veml7700_get_lux();
        raw_als = veml7700_read_als();
    }
    
    // Check if we need to adjust settings for very low light
    if (raw_als < 100) {
        // Light is very dim, increase sensitivity
        veml7700_set_gain(VEML7700_GAIN_2);
        veml7700_set_integration_time(VEML7700_IT_800MS);
        _delay_ms(850); // Allow sensor to adjust and complete integration
        lux = veml7700_get_lux();
    }
    
    // Store the reading
    last_lux_reading = lux;
    
    return lux;
}