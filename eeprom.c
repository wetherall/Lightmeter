/**
 * EEPROM Implementation
 * 
 * This file implements functions for reading from and writing to
 * the ATtiny3216's internal EEPROM memory. This allows the light meter
 * to remember settings between power cycles.
 */

 #include <avr/io.h>
 #include <string.h>  // For memcpy
 #include "eeprom.h"
 #include "shift_register.h"
 
 // External references to the global settings variables
 extern uint16_t iso_setting;
 extern float aperture_setting;
 extern uint8_t current_brightness;  // Global reference to LED brightness (now 0-15)
 
 /**
  * Write a single byte to EEPROM
  */
 void eeprom_write_byte(uint16_t addr, uint8_t data) {
     /* Wait for any previous EEPROM operations to complete */
     while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
     
     /* Set up address and data */
     *((volatile uint8_t *)(EEPROM_START + addr)) = data;
     
     /* Execute EEPROM write command */
     NVMCTRL.CTRLA = NVMCTRL_CMD_EEERWR_gc;
 }
 
 /**
  * Read a single byte from EEPROM
  */
 uint8_t eeprom_read_byte(uint16_t addr) {
     /* Wait for any previous EEPROM operations to complete */
     while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
     
     /* Read data from EEPROM */
     return *((volatile uint8_t *)(EEPROM_START + addr));
 }
 
 /**
  * Write a 16-bit word to EEPROM
  */
 void eeprom_write_word(uint16_t addr, uint16_t data) {
     /* Write the low byte (bits 0-7) */
     eeprom_write_byte(addr, (uint8_t)(data & 0xFF));
     
     /* Write the high byte (bits 8-15) */
     eeprom_write_byte(addr + 1, (uint8_t)(data >> 8));
 }
 
 /**
  * Read a 16-bit word from EEPROM
  */
 uint16_t eeprom_read_word(uint16_t addr) {
     /* Read both bytes and combine them */
     uint16_t low_byte = eeprom_read_byte(addr);
     uint16_t high_byte = eeprom_read_byte(addr + 1);
     
     /* Return the combined 16-bit value */
     return (high_byte << 8) | low_byte;
 }
 
 /**
  * Write a floating-point value to EEPROM
  */
 void eeprom_write_float(uint16_t addr, float data) {
     /* Create a union to allow access to the float as bytes */
     union {
         float f;
         uint8_t bytes[4];
     } converter;
     
     /* Copy the float value to the union */
     converter.f = data;
     
     /* Write each byte to EEPROM */
     for (uint8_t i = 0; i < 4; i++) {
         eeprom_write_byte(addr + i, converter.bytes[i]);
     }
 }
 
 /**
  * Read a floating-point value from EEPROM
  */
 float eeprom_read_float(uint16_t addr) {
     /* Create a union to allow access to the float as bytes */
     union {
         float f;
         uint8_t bytes[4];
     } converter;
     
     /* Read each byte from EEPROM */
     for (uint8_t i = 0; i < 4; i++) {
         converter.bytes[i] = eeprom_read_byte(addr + i);
     }
     
     /* Return the float value */
     return converter.f;
 }
 
 /**
  * Save current settings to EEPROM
  */
 void save_settings(void) {
     /* Save ISO setting (16-bit value) */
     eeprom_write_word(EEPROM_ISO_ADDR, iso_setting);
     
     /* Save aperture setting (32-bit float) */
     eeprom_write_float(EEPROM_APERTURE_ADDR, aperture_setting);
     
     /* Save LED brightness setting (8-bit value) */
     eeprom_write_byte(EEPROM_BRIGHTNESS_ADDR, current_brightness);
     
     /* Mark that settings have been saved */
     eeprom_write_byte(EEPROM_FIRST_RUN_ADDR, 0x42);
 }
 
 /**
  * Load settings from EEPROM with improved validation
  */
 void load_settings(void) {
     /* Check if this is first run (no saved settings) */
     if (eeprom_read_byte(EEPROM_FIRST_RUN_ADDR) != 0x42) {
         /* First run - use defaults */
         iso_setting = 400;
         aperture_setting = 11.0;
         current_brightness = DEFAULT_BRIGHTNESS;  // Now 0-15 scale
         
         /* Save defaults to EEPROM */
         save_settings();
         return;
     }
     
     /* Load and validate ISO setting */
     uint16_t loaded_iso = eeprom_read_word(EEPROM_ISO_ADDR);
     if (loaded_iso >= 25 && loaded_iso <= 3200) {  // Max ISO now 3200 (was 6400)
         iso_setting = loaded_iso;
     } else {
         iso_setting = 400;  // Default if invalid
     }
     
     /* Load and validate aperture setting */
     float loaded_aperture = eeprom_read_float(EEPROM_APERTURE_ADDR);
     if (loaded_aperture >= 1.0 && loaded_aperture <= 45.0) {  // Max aperture now 45 (was 64)
         aperture_setting = loaded_aperture;
     } else {
         aperture_setting = 11.0;  // Default if invalid
     }
     
     /* Load and validate brightness setting */
     uint8_t loaded_brightness = eeprom_read_byte(EEPROM_BRIGHTNESS_ADDR);
     
     /* Check if converting from old brightness scale (0-255) to new (0-15) */
     if (loaded_brightness > MAX_BRIGHTNESS) {
         /* FIXED: Improved conversion from old scale to new scale to avoid truncation issues */
         float scale_factor = (float)MAX_BRIGHTNESS / 255.0f;
         loaded_brightness = (uint8_t)(loaded_brightness * scale_factor + 0.5f); // Add 0.5 for rounding
     }
     
     if (loaded_brightness >= MIN_BRIGHTNESS && loaded_brightness <= MAX_BRIGHTNESS) {
         current_brightness = loaded_brightness;
     } else {
         current_brightness = DEFAULT_BRIGHTNESS;  // Default if invalid
     }
     
     /* If any setting was invalid, save the corrected values */
     if (loaded_iso != iso_setting || 
         loaded_aperture != aperture_setting || 
         loaded_brightness != current_brightness) {
         save_settings();
     }
 }