/**
 * I2C Communication Interface
 * 
 * This header file defines functions for I2C communication
 * used by multiple components in the light meter.
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/**
 * Initialize I2C communication
 * 
 * Sets up the TWI peripheral for standard speed (100kHz)
 * for communication with I2C-based components.
 */
void init_i2c(void);

/**
 * Write data to an I2C device
 * 
 * @param device_addr The 7-bit I2C address of the device
 * @param reg The register address to write to
 * @param data Pointer to data buffer
 * @param length Number of bytes to write
 * @return 0 if successful, non-zero if error
 */
uint8_t i2c_write(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length);

/**
 * Read data from an I2C device
 * 
 * @param device_addr The 7-bit I2C address of the device
 * @param reg The register address to read from
 * @param data Pointer to data buffer to store read bytes
 * @param length Number of bytes to read
 * @return 0 if successful, non-zero if error
 */
uint8_t i2c_read(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length);

#endif /* I2C_H */