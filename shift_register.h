/**
 * Shift Register LED Control Interface
 * 
 * This header file defines functions and constants for controlling LEDs
 * using 74LV595 shift registers and MOSFETs. This is a low-power alternative
 * to the previous IS31FL3731 LED driver implementation.
 * 
 * LED Matrix Organization:
 * Our light meter has 24 LEDs in a logical 2×12 arrangement, with each LED
 * individually controlled by a MOSFET driven by a shift register output.
 */

#ifndef SHIFT_REGISTER_H
#define SHIFT_REGISTER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * CONSTANTS AND DEFINITIONS
 */

/* Shift register control pins */
#define SR_DATA_PIN    PIN0_bm   // Data pin on PA0
#define SR_CLOCK_PIN   PIN1_bm   // Clock pin on PA1
#define SR_LATCH_PIN   PIN2_bm   // Latch pin on PA2

#define SR_DATA_PORT   PORTA     // Port for data pin
#define SR_CLOCK_PORT  PORTA     // Port for clock pin
#define SR_LATCH_PORT  PORTA     // Port for latch pin

/* Number of shift registers and LEDs */
#define NUM_SHIFT_REGISTERS 3    // Using 3 shift registers (24 outputs)
#define LEDS_PER_COLUMN 12       // 12 LEDs per column
#define TOTAL_LEDS 24            // Total of 24 LEDs

/* PWM settings */
#define PWM_LEVELS 16            // 4-bit PWM (16 brightness levels)
#define DEFAULT_BRIGHTNESS 12    // Default brightness level (0-15)

/* Display mode options */
#define DISPLAY_APERTURE  0      // Display mode for aperture
#define DISPLAY_ISO       1      // Display mode for ISO
#define DISPLAY_BRIGHTNESS 2     // Display mode for brightness adjustment

/* Min/Max brightness values */
#define MIN_BRIGHTNESS    3      // Minimum allowable brightness (0-15)
#define MAX_BRIGHTNESS   15      // Maximum allowable brightness (0-15)
#define BRIGHTNESS_STEP   1      // Increment/decrement step for brightness

/**
 * FUNCTION DECLARATIONS
 */

/**
 * Initialize shift register control pins and timer for PWM
 */
void init_shift_registers(void);

/**
 * Send current LED states to all shift registers
 */
void update_shift_registers(void);

/**
 * Set an LED on or off
 * 
 * @param row - LED row (0-11)
 * @param col - LED column (0-1)
 * @param state - true to turn on, false to turn off
 */
void set_led(uint8_t row, uint8_t col, bool state);

/**
 * Set an LED's brightness level
 * 
 * @param row - LED row (0-11)
 * @param col - LED column (0-1)
 * @param brightness - brightness level (0-15, 0 = off)
 */
void set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness);

/**
 * Map a logical LED position to its shift register and bit position
 * 
 * @param row - LED row (0-11)
 * @param col - LED column (0-1)
 * @param reg - Pointer to store the shift register index (0-2)
 * @param bit - Pointer to store the bit position within the register (0-7)
 */
void map_led_to_register(uint8_t row, uint8_t col, uint8_t *reg, uint8_t *bit);

/**
 * Initialize the PWM timer for software PWM
 */
void init_pwm_timer(void);

/**
 * Update the display to show current settings
 */
void update_settings_display(void);

/**
 * Update the shutter speed display
 * 
 * @param shutter_speed - The calculated shutter speed in seconds
 */
void update_shutter_display(float shutter_speed);

/**
 * Display brightness level using LEDs
 */
void display_brightness(void);

/**
 * Set the active brightness level (affects all LED brightness levels)
 *
 * @param brightness - The new brightness level (0-15)
 */
void set_global_brightness(uint8_t brightness);

/* External variable declarations */
extern uint8_t shift_register_data[NUM_SHIFT_REGISTERS];
extern uint8_t led_brightness[TOTAL_LEDS];
extern uint8_t current_brightness;

#endif /* SHIFT_REGISTER_H */
