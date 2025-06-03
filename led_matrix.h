#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>
#include <stdbool.h>

// Constants for LED matrix
#define TOTAL_LEDS 24
#define LEDS_PER_COLUMN 12
#define ROWS_PER_COLUMN 6 // Physical rows used for each logical column half

// Brightness levels
#define HALF_BRIGHTNESS 0
#define FULL_BRIGHTNESS 1

// Pin definitions for LED Matrix (Anodes - Rows, Cathodes - Columns)
// Match these with your PCB layout and README
// Rows (Anodes)
#define ROW1_PIN         PIN5_bm // PA5
#define ROW2_PIN         PIN6_bm // PA6
#define ROW3_PIN         PIN7_bm // PA7
#define ROW4_PIN         PIN0_bm // PC0
#define ROW5_PIN         PIN1_bm // PC1
#define ROW6_PIN         PIN3_bm // PC3

// <<<< CHANGED VPORTx to PORTx HERE >>>>
#define ROWS_PORT_A      PORTA // PORTA for rows 1-3
#define ROWS_PORT_C      PORTC // PORTC for rows 4-6

// Columns (Cathodes) - All on PORTB
#define COL1_PIN         PIN2_bm // PB2
#define COL2_PIN         PIN3_bm // PB3
#define COL3_PIN         PIN4_bm // PB4
#define COL4_PIN         PIN5_bm // PB5
#define COLS_PORT        PORTB   // <<<< CHANGED VPORTB to PORTB HERE >>>>


// Structure to represent physical position in the matrix
typedef struct {
    uint8_t physical_row; // 0-5
    uint8_t physical_col; // 0-3
} physical_position_t;

// External declarations for LED data arrays (defined in led_matrix.c)
extern volatile uint8_t led_matrix_data[TOTAL_LEDS];
extern volatile uint8_t led_brightness[TOTAL_LEDS];

// External declaration for ISR-modified variables (defined in led_matrix.c)
extern volatile uint8_t current_row; // Physical row index for ISR
extern volatile uint8_t pwm_phase;   // PWM phase for ISR


/**
 * @brief Initializes the LED matrix control pins and lookup table.
 * Timer initialization is separate.
 */
void init_led_matrix(void);

/**
 * @brief Initializes the timer (TCB1) for LED matrix scanning and PWM.
 */
void init_matrix_timer(void);

/**
 * @brief Sets an LED on or off.
 * @param row Logical LED row (0-11).
 * @param col Logical LED column (0=ISO/Aperture, 1=Shutter).
 * @param state True to turn on, false to turn off.
 */
void set_led(uint8_t row, uint8_t col, bool state);

/**
 * @brief Sets an LED's brightness level.
 * @param row Logical LED row (0-11).
 * @param col Logical LED column (0=ISO/Aperture, 1=Shutter).
 * @param brightness_level Either FULL_BRIGHTNESS or HALF_BRIGHTNESS.
 */
void set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness_level);

/**
 * @brief Turns all LEDs in the matrix off immediately.
 * This function directly manipulates ports to turn off LEDs.
 */
void led_matrix_all_off_immediate(void);

/**
 * @brief Clears all LED data in the software buffers.
 * This sets all LEDs to off in the `led_matrix_data` and `led_brightness` arrays.
 * The matrix scanning ISR will then turn them off on the next refresh cycle.
 */
void led_matrix_clear_all_data(void);

// This static array is used by ISR and led_matrix.c, so it's better kept in led_matrix.c
// extern const uint8_t row_pins_array[ROWS_PER_COLUMN]; // Not needed as extern if only used in led_matrix.c
// extern const uint8_t col_pins_array[4];             // Not needed as extern if only used in led_matrix.c
extern uint8_t led_index_lookup[ROWS_PER_COLUMN][4]; // This needs to be accessible by ISR in main.c


#endif // LED_MATRIX_H

