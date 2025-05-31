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
 * 
 * IMPROVED: Added lookup table for faster ISR execution and clearer mapping functions
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

/* Lookup table for fast LED index calculation in ISR
 * This table maps [physical_row][physical_col] -> led_index
 * Pre-calculated during initialization to avoid repeated calculations
 */
uint8_t led_index_lookup[6][4];

/**
 * Get the physical position for a logical LED position
 * 
 * This function clarifies the mapping between what the user thinks of as
 * LED positions (0-11 for ISO/aperture, 0-11 for shutter speed) and the
 * actual physical matrix layout.
 * 
 * @param logical_row - The LED row in logical terms (0-11)
 * @param logical_col - The LED column in logical terms (0=ISO/Aperture, 1=Shutter)
 * @return A structure containing the physical row and column in the matrix
 */
physical_position_t get_physical_position(uint8_t logical_row, uint8_t logical_col) {
    physical_position_t pos;
    
    // Validate input parameters
    if (logical_row >= LEDS_PER_COLUMN || logical_col >= 2) {
        // Invalid input, return zero position
        pos.physical_row = 0;
        pos.physical_col = 0;
        return pos;
    }
    
    // Physical row is always the logical row modulo 6
    // This is because we have 6 physical rows that are reused
    pos.physical_row = logical_row % ROWS_PER_COLUMN;
    
    // Physical column depends on both logical column and which half of the logical rows
    if (logical_col == 0) {  // ISO/Aperture column
        // Logical rows 0-5 go to physical column 0
        // Logical rows 6-11 go to physical column 1
        pos.physical_col = (logical_row < ROWS_PER_COLUMN) ? 0 : 1;
    } else {  // Shutter speed column
        // Logical rows 0-5 go to physical column 2
        // Logical rows 6-11 go to physical column 3
        pos.physical_col = (logical_row < ROWS_PER_COLUMN) ? 2 : 3;
    }
    
    return pos;
}

/**
 * Get the LED array index from physical matrix position
 * 
 * This function is the inverse of get_physical_position. It's used by the ISR
 * to quickly determine which LED's data to check when scanning a particular
 * physical row and column.
 * 
 * @param phys_row - The physical row being scanned (0-5)
 * @param phys_col - The physical column being checked (0-3)
 * @return The index into the LED data arrays
 */
uint8_t get_led_index_from_physical(uint8_t phys_row, uint8_t phys_col) {
    uint8_t logical_row, logical_col;
    
    // Validate input parameters
    if (phys_row >= ROWS_PER_COLUMN || phys_col >= 4) {
        return 0;  // Return first LED index on invalid input
    }
    
    // Determine logical column from physical column
    // Physical columns 0-1 are ISO/Aperture (logical column 0)
    // Physical columns 2-3 are Shutter Speed (logical column 1)
    logical_col = (phys_col < 2) ? 0 : 1;
    
    // Determine logical row from physical row and column
    logical_row = phys_row;  // Base row (0-5)
    
    // If we're in the second physical column of each logical column,
    // we're displaying the bottom half of that logical column (rows 6-11)
    if (phys_col == 1 || phys_col == 3) {
        logical_row += ROWS_PER_COLUMN;  // Add 6 to get to bottom half
    }
    
    // Calculate the array index
    // ISO/Aperture LEDs are indices 0-11
    // Shutter Speed LEDs are indices 12-23
    return logical_row + (logical_col * LEDS_PER_COLUMN);
}

/**
 * Initialize the LED lookup table
 * 
 * This function pre-calculates all the LED indices for each physical position
 * in the matrix. This is called once during initialization and saves many
 * calculations during the interrupt service routine.
 */
void init_led_lookup_table(void) {
    // For each physical position in the matrix
    for (uint8_t row = 0; row < 6; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            // Calculate and store the LED index for this position
            led_index_lookup[row][col] = get_led_index_from_physical(row, col);
        }
    }
}

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
    
    // Initialize the lookup table for fast ISR operation
    init_led_lookup_table();
    
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
 * This function is maintained for backwards compatibility but now uses
 * the cleaner get_physical_position function internally.
 * 
 * @param row - Logical LED row (0-11)
 * @param col - Logical LED column (0-1)
 * @param matrix_row - Pointer to store the physical matrix row (0-5)
 * @param matrix_col - Pointer to store the physical matrix column (0-3)
 */
void map_led_to_matrix(uint8_t row, uint8_t col, uint8_t *matrix_row, uint8_t *matrix_col) {
    // Use the new function to get the mapping
    physical_position_t pos = get_physical_position(row, col);
    
    // Return the values through the pointers
    *matrix_row = pos.physical_row;
    *matrix_col = pos.physical_col;
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
 * IMPROVED: Now uses lookup table for much faster execution
 */
/* ISR implementation moved to main.c */