/**
 * LED Matrix Control Implementation
 * * Controls a 6x4 LED matrix with 2 brightness levels (full/half)
 * using direct GPIO manipulation and timer-based scanning.
 */

 #include <avr/io.h>
 #include <string.h> // For memset
 #include "led_matrix.h"
 
 // Global variables for LED state and matrix scanning
 volatile uint8_t led_matrix_data[TOTAL_LEDS];
 volatile uint8_t led_brightness[TOTAL_LEDS];
 volatile uint8_t current_row = 0; // Current active physical row for scanning
 volatile uint8_t pwm_phase = 0;   // PWM phase (0 or 1)
 
 // Lookup table for fast LED index calculation in ISR
 // Maps [physical_row][physical_col] -> led_index
uint8_t led_index_lookup[ROWS_PER_COLUMN][4];
 
 // Arrays of pins for easier iteration in ISR (already defined in .h as macros for port access)
 // For lookup table generation, we use these arrays
 static const uint8_t row_pins_array[ROWS_PER_COLUMN] = {ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN, ROW5_PIN, ROW6_PIN};
 static const uint8_t col_pins_array[4] = {COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN};
 
 
 /**
  * @brief Get the physical matrix position for a logical LED position.
  * @param logical_row Logical row (0-11).
  * @param logical_col Logical column (0 for ISO/Aperture, 1 for Shutter).
  * @return physical_position_t Struct containing physical row and column.
  */
 static physical_position_t get_physical_position(uint8_t logical_row, uint8_t logical_col) {
     physical_position_t pos;
     
     if (logical_row >= LEDS_PER_COLUMN || logical_col >= 2) {
         pos.physical_row = 0;
         pos.physical_col = 0;
         return pos;
     }
     
     pos.physical_row = logical_row % ROWS_PER_COLUMN;
     
     if (logical_col == 0) { // ISO/Aperture column
         pos.physical_col = (logical_row < ROWS_PER_COLUMN) ? 0 : 1;
     } else { // Shutter speed column
         pos.physical_col = (logical_row < ROWS_PER_COLUMN) ? 2 : 3;
     }
     return pos;
 }
 
 /**
  * @brief Get the LED array index from physical matrix position.
  * Used by ISR for quick lookup.
  * @param phys_row Physical row (0-5).
  * @param phys_col Physical column (0-3).
  * @return uint8_t Index into led_matrix_data/led_brightness arrays.
  */
 static uint8_t get_led_index_from_physical(uint8_t phys_row, uint8_t phys_col) {
     uint8_t logical_row, logical_col_group;
     
     if (phys_row >= ROWS_PER_COLUMN || phys_col >= 4) {
         return 0; // Invalid input, return first LED index
     }
     
     logical_col_group = (phys_col < 2) ? 0 : 1; // 0 for ISO/Aperture, 1 for Shutter
     logical_row = phys_row;
     
     if (phys_col == 1 || phys_col == 3) { // Second physical column of each group
         logical_row += ROWS_PER_COLUMN;
     }
     
     return logical_row + (logical_col_group * LEDS_PER_COLUMN);
 }
 
 /**
  * @brief Initialize the LED lookup table for fast ISR operation.
  */
 static void init_led_lookup_table(void) {
     for (uint8_t r = 0; r < ROWS_PER_COLUMN; r++) {
         for (uint8_t c = 0; c < 4; c++) {
             led_index_lookup[r][c] = get_led_index_from_physical(r, c);
         }
     }
 }
 
 /**
  * @brief Initializes the LED matrix control pins.
  */
 void init_led_matrix(void) {
     // Configure row pins (anodes) as outputs, initially LOW
     ROWS_PORT_A.DIRSET = ROW1_PIN | ROW2_PIN | ROW3_PIN;
     ROWS_PORT_A.OUTCLR = ROW1_PIN | ROW2_PIN | ROW3_PIN;
     
     ROWS_PORT_C.DIRSET = ROW4_PIN | ROW5_PIN | ROW6_PIN;
     ROWS_PORT_C.OUTCLR = ROW4_PIN | ROW5_PIN | ROW6_PIN;
     
     // Configure column pins (cathodes) as outputs, initially HIGH (LEDs off)
     COLS_PORT.DIRSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
     COLS_PORT.OUTSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
     
     // Clear all LED data
     led_matrix_clear_all_data();
     
     // Initialize the lookup table
     init_led_lookup_table();
     
     // Timer initialization is separate (called by system_init)
     // init_matrix_timer(); 
 }
 
 /**
  * @brief Initializes the timer for LED matrix scanning (TCB1).
  * F_CPU = 4MHz. Target refresh rate ~500Hz for the whole matrix.
  * 6 rows, 2 PWM phases per row = 12 steps per full scan.
  * Interrupt frequency = 500Hz * 12 = 6000 Hz.
  * Timer period = F_CPU / Interrupt_Freq = 4,000,000 / 6000 = 666.66 cycles.
  * Use 667.
  */
 void init_matrix_timer(void) {
     TCB1.CTRLB = TCB_CNTMODE_INT_gc; // Periodic Interrupt Mode
     TCB1.CCMP = 667;                 // Compare value for interrupt frequency
     TCB1.INTCTRL = TCB_CAPT_bm;      // Enable Capture/Compare Interrupt
     // Timer is enabled as the last step in this function
     TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Use peripheral clock (4MHz), Enable Timer
 }
 
 /**
  * @brief Sets an LED on or off in the data arrays.
  */
 void set_led(uint8_t row, uint8_t col, bool state) {
     if (row >= LEDS_PER_COLUMN || col >= 2) {
         return; // Invalid coordinates
     }
     
     uint8_t led_idx = row + (col * LEDS_PER_COLUMN);
     if (led_idx >= TOTAL_LEDS) return;
 
     if (state) {
         led_matrix_data[led_idx] = 1;
         // Default to full brightness if just turning on
         if (led_brightness[led_idx] == 0 && led_matrix_data[led_idx] == 0) { // if it was previously off
              led_brightness[led_idx] = FULL_BRIGHTNESS;
         }
     } else {
         led_matrix_data[led_idx] = 0;
         led_brightness[led_idx] = 0; // Off means no brightness
     }
 }
 
 /**
  * @brief Sets an LED's brightness level in the data arrays.
  */
 void set_led_brightness(uint8_t row, uint8_t col, uint8_t brightness_level) {
     if (row >= LEDS_PER_COLUMN || col >= 2) {
         return; // Invalid coordinates
     }
     if (brightness_level > FULL_BRIGHTNESS) {
         brightness_level = FULL_BRIGHTNESS; // Cap brightness
     }
     
     uint8_t led_idx = row + (col * LEDS_PER_COLUMN);
     if (led_idx >= TOTAL_LEDS) return;
 
     led_brightness[led_idx] = brightness_level;
     
     // If brightness is set, ensure LED data is also on
     if (brightness_level > 0) {
         led_matrix_data[led_idx] = 1;
     } else {
         led_matrix_data[led_idx] = 0; // No brightness means LED is off
     }
 }
 
 /**
  * @brief Turns all LEDs in the matrix off immediately by setting port states.
  */
 void led_matrix_all_off_immediate(void) {
     // Disable all rows (anodes) by setting them LOW
     ROWS_PORT_A.OUTCLR = ROW1_PIN | ROW2_PIN | ROW3_PIN;
     ROWS_PORT_C.OUTCLR = ROW4_PIN | ROW5_PIN | ROW6_PIN;
     
     // Set all columns (cathodes) HIGH to ensure LEDs are off
     COLS_PORT.OUTSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
 }
 
 /**
  * @brief Clears all LED data in the software buffers.
  */
 void led_matrix_clear_all_data(void) {
     memset((void*)led_matrix_data, 0, sizeof(led_matrix_data));
     memset((void*)led_brightness, 0, sizeof(led_brightness));
 }
 
 
 // ISR for TCB1 (LED Matrix Scanning) is defined in main.c
 // This is because ISRs are typically kept in the file where interrupts are globally managed (main.c)
 // or in a dedicated interrupts.c file. For this project structure, main.c is suitable.
 