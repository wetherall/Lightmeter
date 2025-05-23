/**
 * LED Matrix Control Implementation - Optimized for 2 Brightness Levels
 * 
 * This file contains the implementation of functions for controlling LEDs
 * using matrix scanning with simplified brightness control. Instead of
 * 16 brightness levels, we now use just 2 levels (full and half brightness)
 * for better efficiency while maintaining necessary functionality.
 * 
 * Updated for 4MHz clock frequency operation.
 * 
 * The implementation uses:
 * - 6 row pins (anodes) and 4 column pins (cathodes) for 24 LEDs
 * - Simple 2-phase PWM for brightness control (full and half brightness)
 * - Matrix scanning technique to multiplex the LEDs
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>  // For memcpy/memset
#include "led_matrix.h"

/* Global variables */
uint8_t led_matrix_data[TOTAL_LEDS] = {0};  // Current state of each LED (on/off)
uint8_t led_brightness[TOTAL_LEDS] = {0};   // Brightness level for each LED (0=half, 1=full)

/* Current active row for matrix scanning */
volatile uint8_t current_row = 0;

/* PWM phase tracker - 0 for first half of cycle, 1 for second half */
volatile uint8_t pwm_phase = 0;

/* Arrays of pins for easier iteration */
const uint8_t row_pins[6] = {
    ROW1_PIN, ROW2_PIN, ROW3_PIN,  // Rows 1-3 on PORTA
    ROW4_PIN, ROW5_PIN, ROW6_PIN   // Rows 4-6 on PORTC
};

/* Array to track which port each row belongs to */
const uint8_t row_ports[6] = {
    ROW_PORT_A, ROW_PORT_A, ROW_PORT_A,  // Rows 1-3 on PORTA
    ROW_PORT_C, ROW_PORT_C, ROW_PORT_C   // Rows 4-6 on PORTC
};

/* Array of column pins for easier iteration */
const uint8_t column_pins[4] = {
    COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN
};

/**
 * Initialize LED matrix control pins
 * 
 * This function sets up the GPIO pins needed to control the LED matrix.
 * Row pins are set as outputs (initially LOW)
 * Column pins are set as outputs (initially HIGH, since they're active-low)
 */
void init_led_matrix(void) {
    // Configure row pins (anodes) as outputs, initially LOW
    // Rows 1-3 on PORTA
    ROWS_PORT_A.DIRSET = ROW1_PIN | ROW2_PIN | ROW3_PIN;
    ROWS_PORT_A.OUTCLR = ROW1_PIN | ROW2_PIN | ROW3_PIN;
    
    // Rows 4-6 on PORTC
    ROWS_PORT_C.DIRSET = ROW4_PIN | ROW5_PIN | ROW6_PIN;
    ROWS_PORT_C.OUTCLR = ROW4_PIN | ROW5_PIN | ROW6_PIN;
    
    // Configure column pins (cathodes) as outputs, initially HIGH (LEDs off)
    // All columns on PORTB
    COLS_PORT.DIRSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
    COLS_PORT.OUTSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
    
    // Clear all LED data
    memset(led_matrix_data, 0, sizeof(led_matrix_data));
    memset(led_brightness, 0, sizeof(led_brightness));
    
    // Initialize PWM timer for matrix scanning
    init_matrix_timer();
}

/**
 * Initialize the timer for LED matrix scanning
 * 
 * This function sets up Timer/Counter B1 to generate interrupts
 * at a rate fast enough for matrix scanning and basic PWM control.
 * Updated for 4MHz clock frequency.
 */
void init_matrix_timer(void) {
    // Configure Timer/Counter B1 for matrix scanning
    TCB1.CTRLB = TCB_CNTMODE_INT_gc;
    
    // Set timer period for fast scanning with 4MHz clock
    // With 4MHz clock, we want around 667 cycles per phase (2 phases per row)
    // This gives us approximately (4000000 / 667 / 2 / 6) = 500Hz refresh rate
    // Each row gets scanned at about 83Hz, which is fast enough to avoid flicker
    TCB1.CCMP = 667;
    
    // Enable timer interrupt
    TCB1.INTCTRL = TCB_CAPT_bm;
    
    // Enable timer with 1:1 prescaler
    TCB1.CTRLA = TCB_ENABLE_bm;
}

/**
 * Map a logical LED position to its physical matrix position
 * 
 * This function converts the logical LED coordinates (row 0-11, col 0-1)
 * to the physical matrix coordinates (row 0-5, col 0-3).
 * 
 * The logical layout is:
 * - Left column (col 0): ISO/Aperture LEDs (12 LEDs)
 * - Right column (col 1): Shutter speed LEDs (12 LEDs)
 * 
 * The physical layout is a 6×4 matrix:
 * - Col 0-1: ISO/Aperture (split 6 LEDs per physical column)
 * - Col 2-3: Shutter speed (split 6 LEDs per physical column)
 */
void map_led_to_matrix(uint8_t row, uint8_t col, uint8_t *matrix_row, uint8_t *matrix_col) {
    // Validate coordinates
    if (row >= LEDS_PER_COLUMN || col >= 2) {
        // Invalid coordinates, set to defaults
        *matrix_row = 0;
        *matrix_col = 0;
        return;
    }
    
    // Map logical row to physical row
    // Rows 0-5 go to matrix row 0-5
    // Rows 6-11 also go to matrix row 0-5
    *matrix_row = row % ROWS_PER_COLUMN;
    
    // Map logical column to physical column
    // For rows 0-5:
    //   Col 0 (ISO/Aperture) maps to physical column 0
    //   Col 1 (Shutter Speed) maps to physical column 2
    // For rows 6-11:
    //   Col 0 (ISO/Aperture) maps to physical column 1
    //   Col 1 (Shutter Speed) maps to physical column 3
    if (row < ROWS_PER_COLUMN) {
        // Upper half of LEDs (rows 0-5)
        *matrix_col = col * 2;  // Col 0 → 0, Col 1 → 2
    } else {
        // Lower half of LEDs (rows 6-11)
        *matrix_col = col * 2 + 1;  // Col 0 → 1, Col 1 → 3
    }
}

/**
 * Set an LED on or off
 * 
 * @param row Logical LED row (0-11)
 * @param col Logical LED column (0-1)
 * @param state true to turn on, false to turn off
 */
void set_led(uint8_t row, uint8_t col, bool state) {
    // Validate coordinates
    if (row >= LEDS_PER_COLUMN || col >= 2) {
        return;
    }
    
    // Calculate the LED index in our arrays
    uint8_t led_idx = row + (col * LEDS_PER_COLUMN);
    
    // Update the LED state
    if (state) {
        led_matrix_data[led_idx] = 1;
        led_brightness[led_idx] = FULL_BRIGHTNESS;
    } else {
        led_matrix_data[led_idx] = 0;
        led_brightness[led_idx] = 0;
    }
}

/**
 * Set an LED's brightness level
 * 
 * @param row Logical LED row (0-11)
 * @param col Logical LED column (0-1)
 * @param brightness Either FULL_BRIGHTNESS or HALF_BRIGHTNESS
 */
void set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness) {
    // Validate coordinates and brightness
    if (row >= LEDS_PER_COLUMN || col >= 2 || brightness > FULL_BRIGHTNESS) {
        return;
    }
    
    // Calculate the LED index in our arrays
    uint8_t led_idx = row + (col * LEDS_PER_COLUMN);
    
    // Store brightness value
    led_brightness[led_idx] = brightness;
    
    // Update the LED state based on brightness
    led_matrix_data[led_idx] = (brightness > 0) ? 1 : 0;
}

/**
 * The matrix scanning and PWM interrupt service routine
 * 
 * This function is called by the timer interrupt and handles:
 * 1. Turning off the previous row
 * 2. Managing the PWM phase (for half brightness)
 * 3. Setting the column outputs based on LED states for this row
 * 4. Implementing simplified 2-level PWM
 * 
 * Note: This ISR is defined in main.c, but the implementation is provided here
 */
/* ISR implementation moved to main.c */