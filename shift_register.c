/**
 * Shift Register LED Control Implementation
 * 
 * This file contains the implementation of functions for controlling LEDs
 * using 74LV595 shift registers and MOSFETs. This approach provides a
 * low-power alternative to the previous IS31FL3731 LED driver.
 * 
 * The implementation uses:
 * - 3 × 74LV595 shift registers (24 outputs total)
 * - 24 × DMG1012T MOSFETs for individual LED control
 * - Software PWM for brightness control (4-bit, 16 levels)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>  // For memcpy/memset
#include <math.h>    // For fabsf
#include "shift_register.h"
#include "light_meter.h"

/* External references to global variables */
extern uint16_t iso_setting;
extern float aperture_setting;
extern uint8_t display_mode;
extern bool showing_seconds;
extern float last_lux_reading;

/* Global variables */
uint8_t shift_register_data[NUM_SHIFT_REGISTERS] = {0}; // Current state of shift registers
uint8_t led_brightness[TOTAL_LEDS] = {0};              // Brightness level for each LED (0-15)
volatile uint8_t pwm_counter = 0;                      // Counter for software PWM
uint8_t current_brightness = DEFAULT_BRIGHTNESS;       // Global brightness level (0-15)

/* External reference to arrays of values */
extern const uint16_t iso_values[ISO_COUNT];
extern const float aperture_values[APERTURE_COUNT];
extern const uint16_t shutter_speed_values[SHUTTER_COUNT];

/**
 * Initialize shift register control pins
 */
void init_shift_registers(void) {
    // Configure pins as outputs
    PORTA.DIRSET = SR_DATA_PIN | SR_CLOCK_PIN | SR_LATCH_PIN;
    
    // Set initial pin states
    SR_DATA_PORT.OUTCLR = SR_DATA_PIN;   // Data low
    SR_CLOCK_PORT.OUTCLR = SR_CLOCK_PIN; // Clock low
    SR_LATCH_PORT.OUTCLR = SR_LATCH_PIN; // Latch low
    
    // Clear all shift register data
    memset(shift_register_data, 0, sizeof(shift_register_data));
    memset(led_brightness, 0, sizeof(led_brightness));
    
    // Initialize the PWM timer (Timer B1)
    init_pwm_timer();
    
    // Update the shift registers with initial values (all LEDs off)
    update_shift_registers();
}

/**
 * Initialize the PWM timer for software PWM
 */
void init_pwm_timer(void) {
    // Configure Timer/Counter B1 for PWM (separate from millisecond timer)
    TCB1.CTRLB = TCB_CNTMODE_INT_gc;
    
    // Set timer period for fast PWM (~4kHz with 16 levels)
    // With 3.3MHz clock and 16 PWM levels, we want around 200 cycles per level
    TCB1.CCMP = 200;
    
    // Enable timer interrupt
    TCB1.INTCTRL = TCB_CAPT_bm;
    
    // Enable timer with 1:1 prescaler
    TCB1.CTRLA = TCB_ENABLE_bm;
}

/**
 * PWM Timer interrupt service routine
 * 
 * This function implements software PWM for LED brightness control.
 * It runs at ~4kHz and manages a 4-bit PWM cycle (16 brightness levels).
 */
ISR(TCB1_INT_vect) {
    uint8_t updated_registers[NUM_SHIFT_REGISTERS];
    
    // Make a copy of the current register data
    memcpy(updated_registers, shift_register_data, sizeof(updated_registers));
    
    // For each LED, check if it should be on for this PWM cycle
    for (uint8_t i = 0; i < TOTAL_LEDS; i++) {
        uint8_t reg = i / 8;
        uint8_t bit = i % 8;
        
        // LED should be on if its brightness is greater than the PWM counter
        if (led_brightness[i] > pwm_counter) {
            updated_registers[reg] |= (1 << bit);
        } else {
            updated_registers[reg] &= ~(1 << bit);
        }
    }
    
    // Only update the shift registers if the data has changed
    // This avoids unnecessary SPI traffic
    if (memcmp(updated_registers, shift_register_data, sizeof(updated_registers)) != 0) {
        // Copy the updated values to the global data
        memcpy(shift_register_data, updated_registers, sizeof(shift_register_data));
        
        // Send the new data to the shift registers
        update_shift_registers();
    }
    
    // Increment PWM counter with rollover at PWM_LEVELS
    pwm_counter = (pwm_counter + 1) % PWM_LEVELS;
    
    // Clear the interrupt flag
    TCB1.INTFLAGS = TCB_CAPT_bm;
}

/**
 * Send data to shift registers
 * 
 * This function shifts out the current LED states to all shift registers
 * in the correct order (rightmost register first).
 */
void update_shift_registers(void) {
    // Ensure latch is low before shifting data
    SR_LATCH_PORT.OUTCLR = SR_LATCH_PIN;
    
    // Shift out data for each register, starting with the last one
    for (int8_t i = NUM_SHIFT_REGISTERS - 1; i >= 0; i--) {
        // Shift out each bit of the current register
        for (int8_t b = 7; b >= 0; b--) {
            // Set data pin based on current bit
            if (shift_register_data[i] & (1 << b)) {
                SR_DATA_PORT.OUTSET = SR_DATA_PIN;  // Data high
            } else {
                SR_DATA_PORT.OUTCLR = SR_DATA_PIN;  // Data low
            }
            
            // Pulse clock to shift in the bit
            SR_CLOCK_PORT.OUTSET = SR_CLOCK_PIN;  // Clock high
            _delay_us(1);                         // Small delay
            SR_CLOCK_PORT.OUTCLR = SR_CLOCK_PIN;  // Clock low
            _delay_us(1);                         // Small delay
        }
    }
    
    // Pulse latch to update outputs
    SR_LATCH_PORT.OUTSET = SR_LATCH_PIN;  // Latch high
    _delay_us(1);                         // Small delay
    SR_LATCH_PORT.OUTCLR = SR_LATCH_PIN;  // Latch low
}

/**
 * Map a logical LED position to its shift register and bit position
 */
void map_led_to_register(uint8_t row, uint8_t col, uint8_t *reg, uint8_t *bit) {
    // Calculate the linear LED index (0-23)
    uint8_t led_index = row + (col * LEDS_PER_COLUMN);
    
    // Map to shift register index and bit position
    *reg = led_index / 8;        // Which shift register
    *bit = led_index % 8;        // Which bit in that register
}

/**
 * Set an LED on or off
 */
void set_led(uint8_t row, uint8_t col, bool state) {
    // Validate coordinates
    if (row >= LEDS_PER_COLUMN || col >= 2) {
        return;
    }
    
    // Find which shift register and bit corresponds to this LED
    uint8_t reg, bit;
    map_led_to_register(row, col, &reg, &bit);
    
    // Set or clear the appropriate bit
    if (state) {
        shift_register_data[reg] |= (1 << bit);
        led_brightness[row + (col * LEDS_PER_COLUMN)] = current_brightness;
    } else {
        shift_register_data[reg] &= ~(1 << bit);
        led_brightness[row + (col * LEDS_PER_COLUMN)] = 0;
    }
}

/**
 * Set an LED's brightness
 */
void set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness) {
    // Validate coordinates and brightness
    if (row >= LEDS_PER_COLUMN || col >= 2 || brightness > MAX_BRIGHTNESS) {
        return;
    }
    
    // Store brightness value for PWM control
    led_brightness[row + (col * LEDS_PER_COLUMN)] = brightness;
    
    // Find which shift register and bit corresponds to this LED
    uint8_t reg, bit;
    map_led_to_register(row, col, &reg, &bit);
    
    // Set or clear the bit based on whether brightness is non-zero
    // Actual brightness control happens in the PWM interrupt
    if (brightness > 0) {
        shift_register_data[reg] |= (1 << bit);
    } else {
        shift_register_data[reg] &= ~(1 << bit);
    }
}

/**
 * Set the global brightness level
 */
void set_global_brightness(uint8_t brightness) {
    // Validate brightness value
    if (brightness > MAX_BRIGHTNESS) {
        brightness = MAX_BRIGHTNESS;
    } else if (brightness < MIN_BRIGHTNESS) {
        brightness = MIN_BRIGHTNESS;
    }
    
    // Store the new brightness setting
    current_brightness = brightness;
    
    // Update all LEDs that are currently on to use the new brightness
    for (uint8_t i = 0; i < TOTAL_LEDS; i++) {
        if (led_brightness[i] > 0) {
            led_brightness[i] = current_brightness;
        }
    }
}

/**
 * Update the display to show current settings
 */
void update_settings_display(void) {
    // Clear all LEDs in the settings column
    for (int i = 0; i < LEDS_PER_COLUMN; i++) {
        set_led(i, 0, false);
    }
    
    // Set LEDs based on current mode and setting
    if (display_mode == DISPLAY_APERTURE) {
        // Aperture mode - find the nearest standard aperture value
        int index = find_nearest_aperture_index(aperture_setting);
        
        // Check if we're displaying a half-stop
        float difference = aperture_setting - aperture_values[index];
        
        if (difference < 0.1 && difference > -0.1) {
            // Full stop - just light one LED
            set_led(index, 0, true);
        } else {
            // Half stop - light two adjacent LEDs
            if (difference > 0 && index < (APERTURE_COUNT - 1)) {
                // Between this and next higher value
                set_led(index, 0, true);
                set_led(index + 1, 0, true);
            } else if (difference < 0 && index > 0) {
                // Between this and next lower value
                set_led(index, 0, true);
                set_led(index - 1, 0, true);
            }
        }
    } else {
        // ISO mode
        int index = find_nearest_iso_index(iso_setting);
        
        // Set the LED
        set_led(index, 0, true);
    }
    
    // Update the shift registers
    update_shift_registers();
}

/**
 * Update the shutter speed display
 */
void update_shutter_display(float shutter_speed) {
    // Clear all LEDs in the shutter speed column
    for (int i = 0; i < LEDS_PER_COLUMN; i++) {
        set_led(i, 1, false);
    }
    
    // Determine if showing seconds or fractions of a second
    showing_seconds = (shutter_speed >= 1.0);
    
    // Prepare the value for comparison with our standard values
    float display_speed = showing_seconds ? shutter_speed : 1.0 / shutter_speed;
    
    // Find the nearest standard shutter speed
    int index = -1;
    float min_diff = 1000000.0;
    
    for (int i = 0; i < SHUTTER_COUNT; i++) {
        float diff = fabsf(display_speed - shutter_speed_values[i]);
        if (diff < min_diff) {
            min_diff = diff;
            index = i;
        }
    }
    
    // Light up the main LED for the nearest shutter speed
    set_led(index, 1, true);
    
    // Check if we need to show an intermediate value
    float percent_to_next = 0.0;
    bool show_next_led = false;
    
    if (index < (SHUTTER_COUNT - 1)) {
        // Calculate how far we are to the next standard value
        float range = shutter_speed_values[index+1] - shutter_speed_values[index];
        percent_to_next = (display_speed - shutter_speed_values[index]) / range;
        
        // If more than 25% to next value, we'll indicate this
        show_next_led = (percent_to_next > 0.25);
    }
    
    // If showing an intermediate value and not at the top end
    if (show_next_led && index < (SHUTTER_COUNT - 1)) {
        // Turn on the next LED at half brightness
        set_led_brightness(index + 1, 1, current_brightness / 2);
    }
    
    // Update the shift registers with the new LED states
    update_shift_registers();
}

/**
 * Display brightness level using LEDs
 */
void display_brightness(void) {
    // Clear all LEDs
    for (int i = 0; i < LEDS_PER_COLUMN; i++) {
        set_led(i, 0, false);
        set_led(i, 1, false);
    }
    
    // Calculate how many LEDs to light based on current brightness
    // Map 0-15 brightness to 1-12 LEDs
    int num_leds = (int)((float)(current_brightness - MIN_BRIGHTNESS) / 
                         (float)(MAX_BRIGHTNESS - MIN_BRIGHTNESS) * LEDS_PER_COLUMN);
    
    // Ensure at least one LED is lit
    if (num_leds < 1) num_leds = 1;
    if (num_leds > LEDS_PER_COLUMN) num_leds = LEDS_PER_COLUMN;
    
    // Light up LEDs to show brightness level
    for (int i = 0; i < num_leds; i++) {
        set_led(i, 0, true);  // Use the left column
    }
    
    // Set current LED on right column to indicate current brightness level visually
    set_led(num_leds - 1, 1, true);
    
    // Update the shift registers
    update_shift_registers();
}
