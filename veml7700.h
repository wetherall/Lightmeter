#ifndef VEML7700_H
#define VEML7700_H

#include <stdint.h>

// I2C address of the VEML7700 sensor
#define VEML7700_I2C_ADDR 0x10

// Register addresses
#define VEML7700_REG_CONFIG         0x00 // ALS_GAIN, ALS_IT, ALS_PERS, ALS_INT_EN, ALS_SD
#define VEML7700_REG_ALS_WH         0x01 // ALS high threshold window
#define VEML7700_REG_ALS_WL         0x02 // ALS low threshold window
#define VEML7700_REG_PSM            0x03 // Power saving mode (PSM_EN, PSM_MODE)
#define VEML7700_REG_ALS            0x04 // ALS sensor data output
#define VEML7700_REG_WHITE          0x05 // White channel data output
#define VEML7700_REG_ALS_INT        0x06 // ALS interrupt status

// Configuration register bits (Register 0x00)
// Gain settings (ALS_GAIN, bits 12:11)
#define VEML7700_GAIN_1             (0x00 << 11) // Gain x1
#define VEML7700_GAIN_2             (0x01 << 11) // Gain x2
#define VEML7700_GAIN_1_8           (0x02 << 11) // Gain x1/8
#define VEML7700_GAIN_1_4           (0x03 << 11) // Gain x1/4
#define VEML7700_GAIN_MASK          (0x03 << 11)

// Integration time settings (ALS_IT, bits 9:6)
#define VEML7700_IT_25MS            (0x0C << 6) // 25 ms
#define VEML7700_IT_50MS            (0x08 << 6) // 50 ms
#define VEML7700_IT_100MS           (0x00 << 6) // 100 ms (default)
#define VEML7700_IT_200MS           (0x01 << 6) // 200 ms
#define VEML7700_IT_400MS           (0x02 << 6) // 400 ms
#define VEML7700_IT_800MS           (0x03 << 6) // 800 ms
#define VEML7700_IT_MASK            (0x0F << 6)

// Persistence settings (ALS_PERS, bits 5:4) - Not used in this project
#define VEML7700_PERS_1             (0x00 << 4) // 1 reading
#define VEML7700_PERS_2             (0x01 << 4) // 2 readings
#define VEML7700_PERS_4             (0x02 << 4) // 4 readings
#define VEML7700_PERS_8             (0x03 << 4) // 8 readings

// Interrupt enable (ALS_INT_EN, bit 1) - Not used in this project
#define VEML7700_INT_ENABLE         (0x01 << 1)
#define VEML7700_INT_DISABLE        (0x00 << 1)

// Shutdown (ALS_SD, bit 0)
#define VEML7700_POWER_ON           (0x00 << 0)
#define VEML7700_SHUTDOWN           (0x01 << 0)

// Power Save Mode Register (0x03)
// PSM_EN (bit 0), PSM_MODE (bits 2:1)
#define VEML7700_PSM_EN             (1 << 0)
#define VEML7700_PSM_MODE1          (0x00 << 1) // Not recommended by datasheet
#define VEML7700_PSM_MODE2          (0x01 << 1) // Refresh once per 1s
#define VEML7700_PSM_MODE3          (0x02 << 1) // Refresh once per 2s
#define VEML7700_PSM_MODE4          (0x03 << 1) // Refresh once per 4s

// Global variable to store the last successful lux reading
extern float last_lux_reading;

/**
 * @brief Initialize the VEML7700 sensor with default settings.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_init(void);

/**
 * @brief Read the raw 16-bit ambient light sensor (ALS) value.
 * @param raw_value Pointer to store the raw ALS value.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_read_als_raw(uint16_t *raw_value);

/**
 * @brief Convert raw ALS value to lux based on current gain and integration time.
 * @param raw_als The raw ALS value from the sensor.
 * @return float Calculated lux value. Returns -1.0f on error or invalid input.
 */
float veml7700_convert_to_lux(uint16_t raw_als);

/**
 * @brief Set the gain of the sensor.
 * @param gain_setting Use VEML7700_GAIN_x macros.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_set_gain(uint16_t gain_setting);

/**
 * @brief Set the integration time of the sensor.
 * @param it_setting Use VEML7700_IT_xMS macros.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_set_integration_time(uint16_t it_setting);

/**
 * @brief Enable power save mode on the VEML7700.
 * @param psm_mode Use VEML7700_PSM_MODEx macros.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_power_save_enable(uint16_t psm_mode);

/**
 * @brief Disable power save mode on the VEML7700.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_power_save_disable(void);

/**
 * @brief Puts the VEML7700 sensor into shutdown mode.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_shutdown(void);

/**
 * @brief Powers on the VEML7700 sensor from shutdown mode.
 * @return uint8_t 0 on success, non-zero on I2C error.
 */
uint8_t veml7700_power_on(void);


/**
 * @brief Measure light with automatic adjustment of gain and integration time.
 * This function attempts to get an optimal reading by adjusting sensor settings.
 * @return float Measured lux value. Returns a negative value on error or if no valid reading obtained.
 */
float measure_light_auto_adjust(void);

#endif // VEML7700_H
