/**
 * VEML7700 Light Sensor Implementation
 * * Provides functions to interface with the VEML7700 ambient light sensor
 * via I2C, including automatic gain and integration time adjustment.
 */

 #include <avr/io.h>
 #include <util/delay.h> // For _delay_ms()
 #include <math.h>       // For powf
 #include "veml7700.h"
 #include "i2c.h"
 
 // Global variable to store the last successful lux reading
 float last_lux_reading = 0.0f;
 
 // Current sensor settings, cached locally
 static uint16_t current_config_reg = VEML7700_GAIN_1_8 | VEML7700_IT_100MS | VEML7700_PERS_1 | VEML7700_POWER_ON;
 static uint16_t current_gain_val = VEML7700_GAIN_1_8;
 static uint16_t current_it_val = VEML7700_IT_100MS;
 
 /**
  * @brief Write a 16-bit value to a VEML7700 register.
  */
 static uint8_t veml7700_write_word_to_register(uint8_t reg, uint16_t value) {
     uint8_t data[2];
     data[0] = value & 0xFF;         // Low byte
     data[1] = (value >> 8) & 0xFF;  // High byte
     return i2c_write(VEML7700_I2C_ADDR, reg, data, 2);
 }
 
 /**
  * @brief Read a 16-bit value from a VEML7700 register.
  */
 static uint8_t veml7700_read_word_from_register(uint8_t reg, uint16_t *value) {
     uint8_t data[2];
     uint8_t error = i2c_read(VEML7700_I2C_ADDR, reg, data, 2);
     if (error == 0) {
         *value = ((uint16_t)data[1] << 8) | data[0]; // Low byte first, then high byte
     } else {
         *value = 0xFFFF; // Indicate error
     }
     return error;
 }
 
 /**
  * @brief Initialize the VEML7700 sensor.
  */
 uint8_t veml7700_init(void) {
     // Default configuration: Gain 1/8, IT 100ms, Persistence 1, Power ON
     current_gain_val = VEML7700_GAIN_1_8;
     current_it_val = VEML7700_IT_100MS;
     current_config_reg = current_gain_val | current_it_val | VEML7700_PERS_1 | VEML7700_POWER_ON;
     
     uint8_t error = veml7700_write_word_to_register(VEML7700_REG_CONFIG, current_config_reg);
     if (error) return error;
 
     error = veml7700_write_word_to_register(VEML7700_REG_PSM, 0x0000); // Ensure PSM is off
     if (error) return error;
     
     _delay_ms(5); // Allow sensor to stabilize after initial config (datasheet recommends >2.5ms after power on)
     return 0; // Success
 }
 
 /**
  * @brief Read the raw ALS value.
  */
 uint8_t veml7700_read_als_raw(uint16_t *raw_value) {
     return veml7700_read_word_from_register(VEML7700_REG_ALS, raw_value);
 }
 
 /**
  * @brief Get integration time in milliseconds for delay calculation.
  */
 static uint16_t get_integration_time_ms(uint16_t it_setting) {
     switch (it_setting) {
         case VEML7700_IT_25MS:  return 25;
         case VEML7700_IT_50MS:  return 50;
         case VEML7700_IT_100MS: return 100;
         case VEML7700_IT_200MS: return 200;
         case VEML7700_IT_400MS: return 400;
         case VEML7700_IT_800MS: return 800;
         default: return 100; // Default to 100ms
     }
 }
 
 /**
  * @brief Perform a delay based on integration time.
  */
 static void wait_for_current_integration(void) {
     uint16_t delay_ms = get_integration_time_ms(current_it_val);
     // Add a small buffer (e.g., 5-10ms or a percentage) to ensure integration is complete.
     // Datasheet: "The ALS conversion time depends on the programmed integration time."
     // "After the ALS command register is written, the result can be read out after the programmed integration time."
     // A small extra delay is prudent.
     delay_ms += (delay_ms / 10) + 5; // Add 10% + 5ms, e.g. 800ms -> 800 + 80 + 5 = 885ms
 
     if (delay_ms == 0) return;
 
     // _delay_ms can only handle up to 262.14 ms / F_CPU_MHZ. For 4MHz, it's ~65ms.
     // Need to loop for longer delays.
     for (uint16_t i = 0; i < delay_ms; i++) {
         _delay_ms(1);
     }
 }
 
 
 /**
  * @brief Set the gain of the sensor.
  */
 uint8_t veml7700_set_gain(uint16_t gain_setting) {
     current_config_reg = (current_config_reg & ~VEML7700_GAIN_MASK) | gain_setting;
     current_gain_val = gain_setting;
     return veml7700_write_word_to_register(VEML7700_REG_CONFIG, current_config_reg);
 }
 
 /**
  * @brief Set the integration time of the sensor.
  */
 uint8_t veml7700_set_integration_time(uint16_t it_setting) {
     current_config_reg = (current_config_reg & ~VEML7700_IT_MASK) | it_setting;
     current_it_val = it_setting;
     return veml7700_write_word_to_register(VEML7700_REG_CONFIG, current_config_reg);
 }
 
 /**
  * @brief Convert raw ALS value to lux.
  */
 float veml7700_convert_to_lux(uint16_t raw_als) {
     float resolution; // lux per count
     float lux;
 
     // Determine resolution based on gain and integration time
     // These values are from the VEML7700 datasheet application note/examples
     // The base resolution is for Gain x1, IT 100ms.
     // Example: For IT 100ms, Gain x1, resolution is 0.0576 lux/count
     // Lux = RawALS * Resolution_Factor * (Default_IT / Actual_IT) * (Default_Gain_Multiplier / Actual_Gain_Multiplier)
 
     // Let's use the table from the datasheet for resolution values.
     // Gain | IT (ms) | Resolution (lux/step)
     //---------------------------------------
     // x2   | 800     | 0.0036
     // x1   | 800     | 0.0072
     // x1/4 | 800     | 0.0288
     // x1/8 | 800     | 0.0576
     // ... and scale by IT. For example, for IT 100ms, multiply above by 8.
 
     float gain_factor_mult = 1.0f;
     if (current_gain_val == VEML7700_GAIN_1) gain_factor_mult = 1.0f;
     else if (current_gain_val == VEML7700_GAIN_2) gain_factor_mult = 0.5f; // Gain x2 means each count is worth half as much lux
     else if (current_gain_val == VEML7700_GAIN_1_4) gain_factor_mult = 4.0f;
     else if (current_gain_val == VEML7700_GAIN_1_8) gain_factor_mult = 8.0f;
 
     float it_factor_mult = 1.0f; // Relative to 100ms
     uint16_t it_ms = get_integration_time_ms(current_it_val);
     if (it_ms > 0) {
         it_factor_mult = 100.0f / (float)it_ms;
     }
 
     // Base resolution for Gain x1/8, IT 100ms is often stated around 0.0576 * (1/8 gain factor) = 0.4608
     // Or, more simply, use the max lux for each setting.
     // Max lux = Resolution * 65535.
     // The datasheet provides a formula: Lux = (ALS_Counts / (Gain_Factor * Integration_Time_Factor)) * Magic_Number
     // A common approach is to find the resolution for a known setting (e.g., Gain 1/8, IT 100ms)
     // and then scale.
     // Resolution for G=1/8, IT=100ms is 0.0576 lux/count according to some sources.
     // This implies for G=1, IT=100ms, resolution = 0.0576 / 8 = 0.0072 lux/count.
 
     resolution = 0.0072f; // Base resolution for Gain x1, IT 100ms.
 
     lux = (float)raw_als * resolution * (1.0f / gain_factor_mult) * (1.0f / it_factor_mult);
     
     return lux;
 }
 
 
 /**
  * @brief Enable power save mode.
  */
 uint8_t veml7700_power_save_enable(uint16_t psm_mode) {
     return veml7700_write_word_to_register(VEML7700_REG_PSM, VEML7700_PSM_EN | psm_mode);
 }
 
 /**
  * @brief Disable power save mode.
  */
 uint8_t veml7700_power_save_disable(void) {
     uint8_t error = veml7700_write_word_to_register(VEML7700_REG_PSM, 0x0000); // Disable PSM
     _delay_ms(3); // Datasheet: Min 2.5ms wake-up time from PSM
     return error;
 }
 
 /**
  * @brief Puts the VEML7700 sensor into shutdown mode.
  */
 uint8_t veml7700_shutdown(void) {
     current_config_reg |= VEML7700_SHUTDOWN;
     return veml7700_write_word_to_register(VEML7700_REG_CONFIG, current_config_reg);
 }
 
 /**
  * @brief Powers on the VEML7700 sensor from shutdown mode.
  */
 uint8_t veml7700_power_on(void) {
     current_config_reg &= ~VEML7700_SHUTDOWN;
     uint8_t error = veml7700_write_word_to_register(VEML7700_REG_CONFIG, current_config_reg);
     _delay_ms(3); // Datasheet: Min 2.5ms wake-up time from SD
     return error;
 }
 
 
 /**
  * @brief Measure light with automatic gain and integration time adjustment.
  */
 float measure_light_auto_adjust(void) {
     uint16_t raw_als;
     uint8_t error;
     float lux_val;
 
     // Ensure sensor is powered on from general shutdown
     error = veml7700_power_on(); 
     if (error) return -1.0f; // I2C error during power on
 
     // Start with a moderate sensitivity setting
     veml7700_set_gain(VEML7700_GAIN_1_8);
     veml7700_set_integration_time(VEML7700_IT_100MS);
     wait_for_current_integration();
     error = veml7700_read_als_raw(&raw_als);
     if (error) return -2.0f; // I2C error reading ALS
 
     // Auto-ranging logic
     // If saturated (counts near max), decrease sensitivity
     if (raw_als > 60000) { // High saturation threshold
         if (current_it_val != VEML7700_IT_25MS) {
             veml7700_set_integration_time(VEML7700_IT_25MS);
         } else if (current_gain_val != VEML7700_GAIN_1_8) { // Should already be 1/8 if IT is 25ms
              veml7700_set_gain(VEML7700_GAIN_1_8); // Lowest gain
         }
         wait_for_current_integration();
         error = veml7700_read_als_raw(&raw_als);
         if (error) return -3.0f;
     } 
     // If too low (counts very small), increase sensitivity
     else if (raw_als < 1000) { // Low light threshold
         if (current_it_val != VEML7700_IT_800MS) {
             veml7700_set_integration_time(VEML7700_IT_800MS);
         } else if (current_gain_val != VEML7700_GAIN_2) {
             veml7700_set_gain(VEML7700_GAIN_2); // Highest gain
         }
         wait_for_current_integration();
         error = veml7700_read_als_raw(&raw_als);
         if (error) return -4.0f;
     }
 
     lux_val = veml7700_convert_to_lux(raw_als);
     
     if (lux_val >= 0) { // Check if conversion was valid
         last_lux_reading = lux_val;
     }
     return lux_val;
 }
 