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
 #include <util/delay.h>
 #include <stdbool.h>
 #include <string.h>  // For memcpy/memset
 #include "shift_register.h"
 
 /* External references to global variables */
 extern uint8_t current_brightness;
 
 /* Global variables */
 uint8_t shift_register_data[NUM_SHIFT_REGISTERS] = {0}; // Current state of shift registers
 uint8_t led_brightness[TOTAL_LEDS] = {0};              // Brightness level for each LED (0-15)
 
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
  * Set an LED on or off
  * Uses the main.c implementation of map_led_to_register
  */
 void set_led(uint8_t row, uint8_t col, bool state) {
     // Validate coordinates
     if (row >= LEDS_PER_COLUMN || col >= 2) {
         return;
     }
     
     // Find which shift register and bit corresponds to this LED
     uint8_t reg, bit;
     
     // Call the map_led_to_register function from main.c via function pointer
     extern void map_led_to_register(uint8_t row, uint8_t col, uint8_t *reg, uint8_t *bit);
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
  * Uses the main.c implementation of map_led_to_register
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
     
     // Call the map_led_to_register function from main.c via function pointer
     extern void map_led_to_register(uint8_t row, uint8_t col, uint8_t *reg, uint8_t *bit);
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
     
     // Store the new brightness setting - uses the extern from main.c
     current_brightness = brightness;
     
     // Update all LEDs that are currently on to use the new brightness
     for (uint8_t i = 0; i < TOTAL_LEDS; i++) {
         if (led_brightness[i] > 0) {
             led_brightness[i] = current_brightness;
         }
     }
 }