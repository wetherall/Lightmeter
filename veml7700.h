/**
 * VEML7700 Light Sensor Interface
 * 
 * This header file defines functions and constants for interfacing with
 * the VEML7700 ambient light sensor via I2C communication.
 * 
 * The VEML7700 is a high-accuracy ambient light sensor with:
 * - 16-bit resolution
 * - Wide dynamic range (0.004 lux to 120,000 lux)
 * - Excellent sensitivity to human eye response
 * - Adjustable gain and integration time
 * - I2C interface
 * - Ultra-low power consumption
 * 
 * We use this sensor to measure light levels for calculating
 * appropriate camera exposure settings (shutter speed, aperture, and ISO).
 */

#ifndef VEML7700_H
#define VEML7700_H

#include <stdint.h>

/**
 * CONSTANTS AND DEFINITIONS
 */

/* I2C address of the VEML7700 sensor
 * This is a fixed address that can't be changed
 */
#define VEML7700_I2C_ADDR 0x10

/* Register addresses within the VEML7700
 * These define specific functions we can access
 */
#define VEML7700_REG_CONFIG      0x00  // Configuration register
#define VEML7700_REG_PSM         0x03  // Power save mode register
#define VEML7700_REG_ALS         0x04  // Ambient light sensor data register
#define VEML7700_REG_WHITE       0x05  // White channel data register
#define VEML7700_REG_ALS_INT_TH_H 0x06  // High threshold register (not used in this project)
#define VEML7700_REG_ALS_INT_TH_L 0x07  // Low threshold register (not used in this project)

/* VEML7700 Configuration bits - Gain settings
 * These control the sensitivity of the sensor
 * Higher gain = more sensitivity in low light, but easier to saturate in bright light
 */
#define VEML7700_GAIN_1          0x00  // Gain x1
#define VEML7700_GAIN_2          0x01  // Gain x2
#define VEML7700_GAIN_1_8        0x02  // Gain x1/8
#define VEML7700_GAIN_1_4        0x03  // Gain x1/4

/* VEML7700 Configuration bits - Integration time settings
 * These control how long the sensor collects light for each reading
 * Longer time = more accuracy in low light, but slower readings and
 * easier to saturate in bright light
 */
#define VEML7700_IT_25MS         0x0C  // 25 milliseconds
#define VEML7700_IT_50MS         0x08  // 50 milliseconds
#define VEML7700_IT_100MS        0x00  // 100 milliseconds (default)
#define VEML7700_IT_200MS        0x01  // 200 milliseconds
#define VEML7700_IT_400MS        0x02  // 400 milliseconds
#define VEML7700_IT_800MS        0x03  // 800 milliseconds

/* VEML7700 Configuration bits - Persistence settings
 * These control how many consecutive readings outside threshold
 * are needed to trigger an interrupt (not used in this project)
 */
#define VEML7700_PERS_1          0x00  // 1 reading
#define VEML7700_PERS_2          0x01  // 2 readings
#define VEML7700_PERS_4          0x02  // 4 readings
#define VEML7700_PERS_8          0x03  // 8 readings

/* VEML7700 Configuration bits - Miscellaneous settings */
#define VEML7700_INT_EN          0x01  // Enable interrupts (not used in this project)
#define VEML7700_SD_ON           0x01  // Shutdown enabled (power saving)

/**
 * FUNCTION DECLARATIONS
 * 
 * These functions allow us to communicate with and control the VEML7700
 */

/* Initialize the VEML7700 sensor
 * This sets up the sensor with default configuration.
 */
void veml7700_init(void);

/* Read raw ambient light sensor value
 * Returns the raw 16-bit value from the sensor.
 */
uint16_t veml7700_read_als(void);

/* Convert raw ALS value to lux
 * Returns the light level in lux (standard unit of illuminance).
 */
float veml7700_get_lux(void);

/* Set the gain of the sensor
 * Adjusts sensitivity for different lighting conditions.
 */
void veml7700_set_gain(uint8_t gain);

/* Set the integration time of the sensor
 * Adjusts the sampling time for different lighting conditions.
 */
void veml7700_set_integration_time(uint8_t it);

/* Enable power save mode
 * Reduces power consumption when continuous readings aren't needed.
 */
void veml7700_power_save_enable(uint8_t mode);

/* Disable power save mode
 * Returns to normal operation for taking readings.
 */
void veml7700_power_save_disable(void);

/* Measure light with automatic adjustment
 * Takes a reading and automatically adjusts gain and integration time
 * for optimal results in current lighting conditions.
 */
float measure_light(void);

/**
 * GLOBAL VARIABLES
 */

/* Variable to store last reading
 * This is declared as external because it's defined in veml7700.c
 * but needs to be accessible from other files.
 */
extern float last_lux_reading;

#endif /* VEML7700_H */