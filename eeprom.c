/**
 * EEPROM Implementation
 * 
 * This file implements functions for reading from and writing to
 * the ATtiny3216's internal EEPROM memory. This allows the light meter
 * to remember settings between power cycles.
 * 
 * Updated to work with the new LED matrix implementation and binary brightness levels.
 * 
 * IMPROVED: Added change detection to prevent unnecessary EEPROM writes,
 * extending the lifetime of the EEPROM memory (typically rated for 100,000 write cycles).
 */

#include <avr/io.h>
#include <string.h>  // For memcpy
#include "eeprom.h"
#include "led_matrix.h"  // Updated from shift_register.h to led_matrix.h

// External references to the global settings variables
extern uint16_t iso_setting;
extern float aperture_setting;
extern uint8_t current_brightness;  // Now a binary value (0=half, 1=full)

/**
 * Structure to cache the last saved settings
 * This allows us to detect when settings have actually changed
 * and avoid unnecessary EEPROM writes
 */
typedef struct {
    uint16_t iso;           // Last saved ISO value
    float aperture;         // Last saved aperture value
    uint8_t brightness;     // Last saved brightness value
    bool initialized;       // Whether we've loaded/saved at least once
} settings_cache_t;

// Static variable to store the last saved settings
// This persists for the duration of the program but is lost on power cycle
static settings_cache_t last_saved_settings = {
    .iso = 0,
    .aperture = 0.0,
    .brightness = 0,
    .initialized = false
};

/**
 * Write a single byte to EEPROM
 */
void eeprom_write_byte(uint16_t addr, uint8_t data) {
    /* Wait for any previous EEPROM operations to complete */
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
    
    /* Set up address and data */
    *((volatile uint8_t *)(EEPROM_START + addr)) = data;
    
    /* Execute EEPROM write command */
    NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
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
 * Check if settings have changed since last save
 * 
 * This function compares current settings with the cached values
 * to determine if an EEPROM write is necessary.
 * 
 * For floating point comparison, we use memcmp to avoid issues
 * with floating point equality testing.
 * 
 * @return true if settings have changed, false if they're the same
 */
static bool settings_have_changed(void) {
    // If we haven't initialized yet, settings have "changed"
    if (!last_saved_settings.initialized) {
        return true;
    }
    
    // Check if ISO has changed
    if (iso_setting != last_saved_settings.iso) {
        return true;
    }
    
    // Check if aperture has changed
    // Use memcmp for safe float comparison
    if (memcmp(&aperture_setting, &last_saved_settings.aperture, sizeof(float)) != 0) {
        return true;
    }
    
    // Check if brightness has changed
    if (current_brightness != last_saved_settings.brightness) {
        return true;
    }
    
    // No changes detected
    return false;
}

/**
 * Save current settings to EEPROM
 * 
 * IMPROVED: Now only writes to EEPROM if settings have actually changed,
 * preventing unnecessary wear on the EEPROM memory.
 */
void save_settings(void) {
    /* Check if settings have actually changed */
    if (!settings_have_changed()) {
        // Settings haven't changed, no need to write to EEPROM
        return;
    }
    
    /* Settings have changed, proceed with saving */
    
    /* Save ISO setting (16-bit value) */
    eeprom_write_word(EEPROM_ISO_ADDR, iso_setting);
    
    /* Save aperture setting (32-bit float) */
    eeprom_write_float(EEPROM_APERTURE_ADDR, aperture_setting);
    
    /* Save LED brightness setting (8-bit value) 
     * We're using binary brightness (0=half, 1=full) internally,
     * but we'll save as a compatible value:
     * - HALF_BRIGHTNESS (0) -> save as 4 (compatible with old scale)
     * - FULL_BRIGHTNESS (1) -> save as 12 (compatible with old scale)
     */
    uint8_t compat_brightness = (current_brightness == FULL_BRIGHTNESS) ? 12 : 4;
    eeprom_write_byte(EEPROM_BRIGHTNESS_ADDR, compat_brightness);
    
    /* Mark that settings have been saved */
    eeprom_write_byte(EEPROM_FIRST_RUN_ADDR, 0x42);
    
    /* Update the cache with the newly saved values */
    last_saved_settings.iso = iso_setting;
    last_saved_settings.aperture = aperture_setting;
    last_saved_settings.brightness = current_brightness;
    last_saved_settings.initialized = true;
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
        current_brightness = FULL_BRIGHTNESS;  // Default to full brightness
        
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
    
    /* Brightness compatibility layer:
     * - For values from old 0-15 scale, convert to binary (0-1)
     * - If value is <= 7, use HALF_BRIGHTNESS (0)
     * - If value is > 7, use FULL_BRIGHTNESS (1)
     */
    if (loaded_brightness <= 7) {
        current_brightness = HALF_BRIGHTNESS;
    } else {
        current_brightness = FULL_BRIGHTNESS;
    }
    
    /* Update the cache with loaded values */
    last_saved_settings.iso = iso_setting;
    last_saved_settings.aperture = aperture_setting;
    last_saved_settings.brightness = current_brightness;
    last_saved_settings.initialized = true;
    
    /* If any setting was invalid and got corrected, save the corrected values */
    if (loaded_iso != iso_setting || 
        loaded_aperture != aperture_setting || 
        (loaded_brightness <= 7 && current_brightness != HALF_BRIGHTNESS) ||
        (loaded_brightness > 7 && current_brightness != FULL_BRIGHTNESS)) {
        save_settings();
    }
}