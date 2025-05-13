/**
 * EEPROM Interface
 * 
 * This header file defines functions for reading from and writing to
 * the ATtiny3216's internal EEPROM memory. This allows the light meter
 * to remember settings between power cycles.
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

/* EEPROM memory address constants */
#define EEPROM_START 0x1400        // EEPROM start address for ATtiny3216
#define EEPROM_FIRST_RUN_ADDR 0    // Address for the first-run flag
#define EEPROM_ISO_ADDR 1          // Address for the ISO setting (16-bit)
#define EEPROM_APERTURE_ADDR 3     // Address for the aperture setting (32-bit float)
#define EEPROM_BRIGHTNESS_ADDR 7   // Address for the brightness setting (8-bit)

/**
 * FUNCTION DECLARATIONS
 */

/**
 * Write a single byte to EEPROM
 * 
 * @param addr - The address within EEPROM to write to
 * @param data - The byte to write
 */
void eeprom_write_byte(uint16_t addr, uint8_t data);

/**
 * Read a single byte from EEPROM
 * 
 * @param addr - The address within EEPROM to read from
 * @return The byte read from EEPROM
 */
uint8_t eeprom_read_byte(uint16_t addr);

/**
 * Write a 16-bit word to EEPROM
 * 
 * @param addr - The address within EEPROM to write to
 * @param data - The 16-bit word to write
 */
void eeprom_write_word(uint16_t addr, uint16_t data);

/**
 * Read a 16-bit word from EEPROM
 * 
 * @param addr - The address within EEPROM to read from
 * @return The 16-bit word read from EEPROM
 */
uint16_t eeprom_read_word(uint16_t addr);

/**
 * Write a floating-point value to EEPROM
 * 
 * @param addr - The address within EEPROM to write to
 * @param data - The floating-point value to write
 */
void eeprom_write_float(uint16_t addr, float data);

/**
 * Read a floating-point value from EEPROM
 * 
 * @param addr - The address within EEPROM to read from
 * @return The floating-point value read from EEPROM
 */
float eeprom_read_float(uint16_t addr);

/**
 * Save current settings to EEPROM
 * 
 * This function saves the current ISO, aperture, and brightness settings
 * to EEPROM for retrieval on next power-up.
 */
void save_settings(void);

/**
 * Load settings from EEPROM
 * 
 * This function loads and validates ISO, aperture, and brightness
 * settings from EEPROM. If no valid settings are found, defaults are used.
 */
void load_settings(void);

#endif /* EEPROM_H */