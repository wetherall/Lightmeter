/**
 * LED Matrix Control Interface - Optimized for 2 Brightness Levels
 * 
 * This header file defines functions and constants for controlling LEDs
 * using matrix scanning. This approach directly drives the LEDs
 * from the microcontroller pins with an optimized 2-level brightness control.
 * 
 * LED Matrix Organization:
 * Our light meter has 24 LEDs in a 6×4 matrix arrangement:
 * - 6 rows (anodes)
 * - 4 columns (cathodes)
 * - Column 1-2: ISO/Aperture values (12 LEDs total)
 * - Column 3-4: Shutter speed values (12 LEDs total)
 */

#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>
#include <stdbool.h>

/**
 * CONSTANTS AND DEFINITIONS
 */

/* LED Matrix Pins */
/* Row pins (anodes) */
#define ROW1_PIN    PIN5_bm    // Row 1 on PA5
#define ROW2_PIN    PIN6_bm    // Row 2 on PA6
#define ROW3_PIN    PIN7_bm    // Row 3 on PA7
#define ROW4_PIN    PIN0_bm    // Row 4 on PC0
#define ROW5_PIN    PIN1_bm    // Row 5 on PC1
#define ROW6_PIN    PIN3_bm    // Row 6 on PC3

/* Column pins (cathodes) */
#define COL1_PIN    PIN2_bm    // Column 1 on PB2 (ISO/Aperture 0-5)
#define COL2_PIN    PIN3_bm    // Column 2 on PB3 (ISO/Aperture 6-11)
#define COL3_PIN    PIN4_bm    // Column 3 on PB4 (Shutter Speed 0-5)
#define COL4_PIN    PIN5_bm    // Column 4 on PB5 (Shutter Speed 6-11)

/* Port definitions */
#define ROWS_PORT_A PORTA      // Port A for rows 1-3
#define ROWS_PORT_C PORTC      // Port C for rows 4-6
#define COLS_PORT   PORTB      // Port B for all columns

/* Array indices for row and column ports */
#define ROW_PORT_A  0          // Index for Port A
#define ROW_PORT_C  1          // Index for Port C

/* LED Matrix Dimensions */
#define ROWS_PER_COLUMN 6      // 6 rows per column
#define COLS_PER_SIDE   2      // 2 columns per side (left/right)
#define LEDS_PER_COLUMN 12     // Total 12 LEDs per logical column (split across 2 physical columns)
#define TOTAL_LEDS      24     // Total of 24 LEDs

/* Brightness levels - easily adjustable during development */
#define FULL_BRIGHTNESS 1      // Full brightness (100% duty cycle)
#define HALF_BRIGHTNESS 0      // Half brightness (50% duty cycle)

/* PWM duty cycle adjustments (as percentages) - tune during development */
#define FULL_DUTY_CYCLE 100    // 100% duty cycle
#define HALF_DUTY_CYCLE 50     // 50% duty cycle  

/* Display mode options */
#define DISPLAY_APERTURE  0    // Display mode for aperture
#define DISPLAY_ISO       1    // Display mode for ISO

/**
 * FUNCTION DECLARATIONS
 */

/**
 * Initialize LED matrix control pins and timer for PWM
 */
void init_led_matrix(void);

/**
 * Set an LED on or off
 * 
 * @param row - LED row (0-11) in logical arrangement
 * @param col - LED column (0-1) in logical arrangement
 * @param state - true to turn on, false to turn off
 */
void set_led(uint8_t row, uint8_t col, bool state);

/**
 * Set an LED's brightness level
 * 
 * @param row - LED row (0-11) in logical arrangement
 * @param col - LED column (0-1) in logical arrangement
 * @param brightness - Either FULL_BRIGHTNESS or HALF_BRIGHTNESS
 */
void set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness);

/**
 * Initialize the timer for LED matrix scanning
 */
void init_matrix_timer(void);

/**
 * Map a logical LED position to its physical matrix position
 * 
 * @param row - Logical LED row (0-11)
 * @param col - Logical LED column (0-1)
 * @param matrix_row - Pointer to store the physical matrix row (0-5)
 * @param matrix_col - Pointer to store the physical matrix column (0-3)
 */
void map_led_to_matrix(uint8_t row, uint8_t col, uint8_t *matrix_row, uint8_t *matrix_col);

/* External variable declarations */
extern uint8_t led_matrix_data[TOTAL_LEDS];     // Current state of each LED (on/off)
extern uint8_t led_brightness[TOTAL_LEDS];      // Brightness level for each LED (0=half, 1=full)


extern uint8_t led_matrix_data[TOTAL_LEDS];     // Current state of each LED (on/off)
extern uint8_t led_brightness[TOTAL_LEDS];      // Brightness level for each LED (0=half, 1=full)
extern volatile uint8_t current_row;            // Current active row for matrix scanning
extern const uint8_t column_pins[4];            // Array of column pins for LED matrix
extern const uint8_t row_pins[6];               // Array of row pins for LED matrix
extern const uint8_t row_ports[6];              // Array to track which port each row belongs to
extern volatile uint8_t pwm_phase;       // PWM phase tracker for brightness control

#endif /* LED_MATRIX_H */