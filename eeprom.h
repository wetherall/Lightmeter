#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stdbool.h>

// EEPROM Addresses for settings
#define EEPROM_FIRST_RUN_ADDR    0x00 // Byte to check if settings were ever saved
#define EEPROM_ISO_ADDR          0x01 // 2 bytes for ISO (uint16_t)
#define EEPROM_APERTURE_ADDR     0x03 // 4 bytes for aperture (float)
#define EEPROM_BRIGHTNESS_ADDR   0x07 // 1 byte for brightness (uint8_t) - now binary

/**
 * @brief Write a single byte to EEPROM.
 * @param addr EEPROM address (0 to EEPROM_SIZE-1).
 * @param data Byte to write.
 */
void eeprom_write_byte(uint16_t addr, uint8_t data);

/**
 * @brief Read a single byte from EEPROM.
 * @param addr EEPROM address.
 * @return uint8_t Byte read from EEPROM.
 */
uint8_t eeprom_read_byte(uint16_t addr);

/**
 * @brief Write a 16-bit word to EEPROM.
 * @param addr Starting EEPROM address.
 * @param data Word to write.
 */
void eeprom_write_word(uint16_t addr, uint16_t data);

/**
 * @brief Read a 16-bit word from EEPROM.
 * @param addr Starting EEPROM address.
 * @return uint16_t Word read from EEPROM.
 */
uint16_t eeprom_read_word(uint16_t addr);

/**
 * @brief Write a float value to EEPROM.
 * @param addr Starting EEPROM address.
 * @param data Float value to write.
 */
void eeprom_write_float(uint16_t addr, float data);

/**
 * @brief Read a float value from EEPROM.
 * @param addr Starting EEPROM address.
 * @return float Float value read from EEPROM.
 */
float eeprom_read_float(uint16_t addr);

/**
 * @brief Save current settings (ISO, aperture, brightness) to EEPROM.
 * Only writes if settings have changed to preserve EEPROM life.
 */
void save_settings_to_eeprom(void);

/**
 * @brief Load settings from EEPROM into global variables.
 * If no settings are found (first run), uses default values and saves them.
 */
void load_settings_from_eeprom(void);

#endif // EEPROM_H
