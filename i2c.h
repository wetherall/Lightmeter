#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/**
 * @brief Initialize I2C communication.
 * Sets up the TWI peripheral for master mode operation.
 */
void init_i2c(void);

/**
 * @brief Write data to an I2C device.
 * * @param device_addr The 7-bit I2C address of the slave device.
 * @param reg The register address to write to.
 * @param data Pointer to the data buffer to write.
 * @param length Number of bytes to write.
 * @return uint8_t 0 if successful, 
 * 1 if NACK received during address phase,
 * 2 if NACK received during data/register phase.
 */
uint8_t i2c_write(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length);

/**
 * @brief Read data from an I2C device.
 * * @param device_addr The 7-bit I2C address of the slave device.
 * @param reg The register address to read from.
 * @param data Pointer to the buffer to store read data.
 * @param length Number of bytes to read.
 * @return uint8_t 0 if successful,
 * 1 if NACK received during address phase (write part),
 * 2 if NACK received during register write phase,
 * 3 if NACK received during address phase (read part).
 */
uint8_t i2c_read(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length);

#endif // I2C_H
