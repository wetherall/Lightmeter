/**
 * EEPROM Implementation
 * * Functions for reading from and writing to the ATtiny3217's internal EEPROM.
 * Includes change detection to minimize EEPROM writes.
 * * Corrected eeprom_write_byte for ATtiny3217 NVM controller.
 */

 #include <avr/io.h>
 #include <avr/interrupt.h> 
 #include <string.h>        
 #include <stdbool.h>
 #include "eeprom.h"
 #include "led_matrix.h"     
 
 // External global settings variables (defined in main.c)
 extern uint16_t iso_setting;
 extern float aperture_setting;
 extern uint8_t current_led_matrix_brightness_level;
 
 // Cache for last saved settings to detect changes
 typedef struct {
     uint16_t iso;
     float aperture;
     uint8_t brightness_level;
     bool initialized;
 } settings_cache_t;
 
 static settings_cache_t last_saved_settings_cache = {0, 0.0f, 0, false};
 
 /**
  * @brief Write a single byte to EEPROM.
  * For ATtiny3217, this involves writing to the memory-mapped EEPROM address
  * which loads the page buffer, then issuing an NVM command.
  */
 void eeprom_write_byte(uint16_t addr, uint8_t data) {
     // Wait for NVM controller to be ready (no previous EEPROM operation busy)
     while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
     
     // Write data to memory mapped EEPROM address. This loads the EEPROM page buffer.
     // Note: EEPROM_START is the base address of the EEPROM memory map.
     *( (volatile uint8_t *)(EEPROM_START + addr) ) = data;
     
     // Execute EEPROM Page Erase Write command.
     // This command is protected, so CCP write is needed first.
     CCP = CCP_SPM_gc; // Unlock Configuration Change Protection for NVMCTRL.CTRLA
     NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc; // Erase page and write buffer contents
 
     // The NVMCTRL automatically clears EEBUSY when done.
     // No need for an explicit wait here if subsequent operations also check EEBUSY.
 }
 
 /**
  * @brief Read a single byte from EEPROM.
  */
 uint8_t eeprom_read_byte(uint16_t addr) {
     while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
     return *((volatile uint8_t *)(EEPROM_START + addr));
 }
 
 /**
  * @brief Write a 16-bit word to EEPROM.
  */
 void eeprom_write_word(uint16_t addr, uint16_t data) {
     eeprom_write_byte(addr, (uint8_t)(data & 0xFF));
     eeprom_write_byte(addr + 1, (uint8_t)(data >> 8));
 }
 
 /**
  * @brief Read a 16-bit word from EEPROM.
  */
 uint16_t eeprom_read_word(uint16_t addr) {
     uint16_t low_byte = eeprom_read_byte(addr);
     uint16_t high_byte = eeprom_read_byte(addr + 1);
     return (high_byte << 8) | low_byte;
 }
 
 /**
  * @brief Write a float value to EEPROM.
  */
 void eeprom_write_float(uint16_t addr, float data) {
     union {
         float f;
         uint8_t bytes[sizeof(float)];
     } converter;
     converter.f = data;
     for (uint8_t i = 0; i < sizeof(float); i++) {
         eeprom_write_byte(addr + i, converter.bytes[i]);
     }
 }
 
 /**
  * @brief Read a float value from EEPROM.
  */
 float eeprom_read_float(uint16_t addr) {
     union {
         float f;
         uint8_t bytes[sizeof(float)];
     } converter;
     for (uint8_t i = 0; i < sizeof(float); i++) {
         converter.bytes[i] = eeprom_read_byte(addr + i);
     }
     return converter.f;
 }
 
 /**
  * @brief Check if settings have changed since last save.
  */
 static bool settings_have_changed(void) {
     if (!last_saved_settings_cache.initialized) return true;
     if (iso_setting != last_saved_settings_cache.iso) return true;
     if (memcmp(&aperture_setting, &last_saved_settings_cache.aperture, sizeof(float)) != 0) return true;
     if (current_led_matrix_brightness_level != last_saved_settings_cache.brightness_level) return true;
     return false;
 }
 
 /**
  * @brief Save current settings to EEPROM.
  */
 void save_settings_to_eeprom(void) {
     if (!settings_have_changed()) {
         return; 
     }
 
     eeprom_write_word(EEPROM_ISO_ADDR, iso_setting);
     eeprom_write_float(EEPROM_APERTURE_ADDR, aperture_setting);
     
     uint8_t brightness_to_save = (current_led_matrix_brightness_level == FULL_BRIGHTNESS) ? 12 : 4;
     eeprom_write_byte(EEPROM_BRIGHTNESS_ADDR, brightness_to_save);
     
     eeprom_write_byte(EEPROM_FIRST_RUN_ADDR, 0x42); 
 
     last_saved_settings_cache.iso = iso_setting;
     last_saved_settings_cache.aperture = aperture_setting;
     last_saved_settings_cache.brightness_level = current_led_matrix_brightness_level;
     last_saved_settings_cache.initialized = true;
 }
 
 /**
  * @brief Load settings from EEPROM.
  */
 void load_settings_from_eeprom(void) {
     if (eeprom_read_byte(EEPROM_FIRST_RUN_ADDR) != 0x42) {
         iso_setting = 400;
         aperture_setting = 11.0f;
         current_led_matrix_brightness_level = FULL_BRIGHTNESS; 
         save_settings_to_eeprom(); 
     } else {
         uint16_t loaded_iso = eeprom_read_word(EEPROM_ISO_ADDR);
         if (loaded_iso >= 25 && loaded_iso <= 3200) {
             iso_setting = loaded_iso;
         } else {
             iso_setting = 400; 
         }
 
         float loaded_aperture = eeprom_read_float(EEPROM_APERTURE_ADDR);
         if (loaded_aperture >= 1.0f && loaded_aperture <= 45.0f) {
             aperture_setting = loaded_aperture;
         } else {
             aperture_setting = 11.0f; 
         }
 
         uint8_t loaded_brightness_compat = eeprom_read_byte(EEPROM_BRIGHTNESS_ADDR);
         if (loaded_brightness_compat <= 7) {
             current_led_matrix_brightness_level = HALF_BRIGHTNESS;
         } else {
             current_led_matrix_brightness_level = FULL_BRIGHTNESS;
         }
     }
 
     last_saved_settings_cache.iso = iso_setting;
     last_saved_settings_cache.aperture = aperture_setting;
     last_saved_settings_cache.brightness_level = current_led_matrix_brightness_level;
     last_saved_settings_cache.initialized = true;
 }
 