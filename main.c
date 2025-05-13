/**
 * Light Meter Main Program - Shift Register Implementation
 * 
 * This file contains the main program for a light meter using 74LV595 shift registers
 * to control 24 LEDs (2 columns of 12) via MOSFETs. This implementation replaces
 * the previous IS31FL3731 driver with a lower-power solution.
 * 
 * Overview of how the program works:
 * 1. The system initializes all components (shift registers, light sensor, buttons)
 * 2. It loads saved settings from EEPROM (or uses defaults if first run)
 * 3. It enters a main loop that checks for button presses
 * 4. When the read button is pressed, it takes a light reading and displays the appropriate shutter speed
 * 5. When up/down buttons are pressed, it changes settings and updates the display
 * 6. After 20 seconds of inactivity, it saves settings and enters low-power sleep mode
 * 
 * Hardware Configuration:
 * - ATtiny3216 microcontroller
 * - 3× 74LV595 shift registers (24 outputs total)
 * - 24× DMG1012T MOSFETs (driving individual LEDs)
 * - VEML7700 light sensor via I2C
 * - 4 pushbuttons for user control
 * - TPS62203 switching regulator for power efficiency
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>  // For memset

/* Include our custom header files */
#include "veml7700.h"
#include "light_meter.h"
#include "user_interface.h"
#include "power_management.h"
#include "eeprom.h"
#include "i2c.h"

/**
 * SHIFT REGISTER DEFINITIONS
 * 
 * These define the pins and constants used for interfacing with the 74LV595 shift registers.
 */

/* Shift register control pins */
#define SR_DATA_PIN    PIN3_bm   // Data pin on PA3 (changed from PA0)
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

/**
 * GLOBAL VARIABLES
 */

/* LED state variables */
uint8_t shift_register_data[NUM_SHIFT_REGISTERS] = {0}; // Current state of shift registers
uint8_t led_brightness[TOTAL_LEDS] = {0};               // Brightness level for each LED (0-15)
volatile uint8_t pwm_counter = 0;                       // Counter for software PWM

/* Button flags */
volatile bool wake_flag = false;          // Flag for wake button press
volatile bool mode_button_flag = false;   // Flag for mode button press
volatile bool up_button_flag = false;     // Flag for up button press
volatile bool down_button_flag = false;   // Flag for down button press
volatile bool mode_button_held = false;   // Flag for mode button being held

/* Button state tracking (for hold detection) */
volatile uint32_t mode_button_press_time = 0;  // When mode button was pressed
volatile bool mode_button_down = false;        // Whether mode button is currently down

/* Time constants for button hold detection */
#define BUTTON_HOLD_THRESHOLD 500  // Time in ms to count as a "hold" (500ms = 0.5 seconds)

/* Timing variables for sleep timeout and blink control */
volatile uint32_t current_time = 0;       // Current system time in milliseconds
volatile uint32_t last_activity_time = 0; // Time of last button activity
volatile uint32_t last_blink_time = 0;    // Time of last blink toggle

/* Separate blink state tracking for different functions */
volatile bool iso_blink_state = false;      // Blink state for ISO mode
volatile bool seconds_blink_state = false;  // Blink state for whole seconds display

/* Default settings (will be overridden by loaded settings if available) */
uint16_t iso_setting = 400;               // Default ISO: 400
float aperture_setting = 11.0;            // Default f-stop: f/11
uint8_t display_mode = DISPLAY_APERTURE;  // Default display mode: aperture
bool showing_seconds = false;             // Whether we're showing shutter speeds in seconds

/* Brightness adjustment mode */
bool in_brightness_mode = false;          // Whether we're in brightness adjustment mode
uint8_t current_brightness = DEFAULT_BRIGHTNESS; // Current global brightness level (0-15)

/* Sleep timeout in milliseconds (20 seconds) */
#define SLEEP_TIMEOUT_MS 20000

/* Blink intervals in milliseconds (for software-based blinking) */
#define BLINK_SLOW_MS 1000  // 0.5Hz - once per 2 seconds
#define BLINK_MED_MS  500   // 1Hz - once per second (ISO mode indicator)
#define BLINK_FAST_MS 250   // 2Hz - twice per second (whole seconds indicator)

/* Button pins */
#define READ_BUTTON_PIN  PIN4_bm   // Read button on PA4
#define MODE_BUTTON_PIN  PIN5_bm   // Mode button on PA5
#define UP_BUTTON_PIN    PIN6_bm   // Up button on PA6
#define DOWN_BUTTON_PIN  PIN7_bm   // Down button on PA7

/* Button debouncing constants and variables */
#define DEBOUNCE_TIME_MS  20    // 20ms debounce time

/* Variables to track the last time each button was pressed */
volatile uint32_t last_read_press_time = 0;
volatile uint32_t last_mode_press_time = 0;
volatile uint32_t last_up_press_time = 0;
volatile uint32_t last_down_press_time = 0;

/**
 * APPLICATION CONSTANTS
 * 
 * These define the available settings and values used in the light meter.
 */

/* Number of available ISO and aperture values */
#define ISO_COUNT        12      // Reduced from 13 to 12 (removed ISO 6400)
#define APERTURE_COUNT   12      // Reduced from 13 to 12 (no change in values)
#define SHUTTER_COUNT    12      // Reduced from 13 to 12 (removed 1/4000)

/* Display mode options */
#define DISPLAY_APERTURE  0      // Display mode for aperture
#define DISPLAY_ISO       1      // Display mode for ISO
#define DISPLAY_BRIGHTNESS 2     // Display mode for brightness adjustment

/* Min/Max brightness values */
#define MIN_BRIGHTNESS    3      // Minimum allowable brightness (0-15)
#define MAX_BRIGHTNESS   15      // Maximum allowable brightness (0-15)
#define BRIGHTNESS_STEP   1      // Increment/decrement step for brightness

/* Arrays of supported values */
// ISO values (light sensitivity) from lowest to highest - removed 6400
const uint16_t iso_values[ISO_COUNT] = {25, 50, 100, 125, 160, 200, 250, 400, 500, 800, 1600, 3200};

// Aperture values (f-stops) from lowest to highest
const float aperture_values[APERTURE_COUNT] = {1.0, 1.4, 2.0, 2.8, 4.0, 5.6, 8.0, 11.0, 16.0, 22.0, 32.0, 45.0};

// Shutter speed values in fractions of a second (displayed as 1/n) - removed 4000
const uint16_t shutter_speed_values[SHUTTER_COUNT] = {1, 2, 4, 8, 15, 30, 60, 125, 250, 500, 1000, 2000};

/**
 * LED MAPPING FUNCTIONS
 * 
 * These functions handle the mapping between logical LED positions (row, column)
 * and their corresponding shift register outputs.
 */

/**
 * Map a logical LED position to its shift register and bit position
 * 
 * Parameters:
 *   row - LED row (0-11)
 *   col - LED column (0-1)
 *   reg - Pointer to store the shift register index (0-2)
 *   bit - Pointer to store the bit position within the register (0-7)
 */
void map_led_to_register(uint8_t row, uint8_t col, uint8_t *reg, uint8_t *bit) {
    // Validate coordinates
    if (row >= LEDS_PER_COLUMN || col >= 2) {
        // Invalid coordinates, set to defaults
        *reg = 0;
        *bit = 0;
        return;
    }
    
    // Map logical position to alternating column pattern
    // SR3 handles top rows, SR2 middle rows, SR1 bottom rows
    *reg = 2 - (row / 4);  // Rows 0-3 -> reg 2, rows 4-7 -> reg 1, rows 8-11 -> reg 0
    
    // Determine bit based on position within register group and column
    // Column 0 (ISO) maps to even bits (6,4,2,0)
    // Column 1 (Shutter) maps to odd bits (7,5,3,1)
    uint8_t row_in_register = row % 4;  // Position within register (0-3)
    uint8_t bit_pair = 3 - row_in_register;  // Reverse order within register (3,2,1,0)
    *bit = bit_pair * 2 + col;  // col 0 -> even bits (6,4,2,0), col 1 -> odd bits (7,5,3,1)
}

/**
 * SHIFT REGISTER CONTROL FUNCTIONS
 * 
 * These functions handle the low-level control of the shift registers.
 */

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
    
    // Initialize the PWM timer (Timer B0)
    init_pwm_timer();
    
    // Update the shift registers with initial values (all LEDs off)
    update_shift_registers();
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
 * 
 * Parameters:
 *   row - LED row (0-11)
 *   col - LED column (0-1)
 *   state - true to turn on, false to turn off
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
 * 
 * Parameters:
 *   row - LED row (0-11)
 *   col - LED column (0-1)
 *   brightness - brightness level (0-15, 0 = off)
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
 * Initialize GPIO pins
 */
void init_gpio(void) {
    /* Configure button pins as inputs with pull-ups */
    PORTA.DIRCLR = READ_BUTTON_PIN;  // Read button (PA4)
    PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; 
    
    PORTA.DIRCLR = MODE_BUTTON_PIN;  // Mode button (PA5)
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;  // Both edges for press and release
    
    PORTA.DIRCLR = UP_BUTTON_PIN;    // Up button (PA6)
    PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    
    PORTA.DIRCLR = DOWN_BUTTON_PIN;  // Down button (PA7)
    PORTA.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    
    /* Configure I2C pins as inputs with pull-ups (corrected from DIRSET) */
    PORTB.DIRCLR = PIN0_bm;  // SDA pin
    PORTB.DIRCLR = PIN1_bm;  // SCL pin
    
    /* Enable pull-ups on I2C pins */
    PORTB.PIN0CTRL = PORT_PULLUPEN_bm;
    PORTB.PIN1CTRL = PORT_PULLUPEN_bm;
    
    /* ADD THIS NEW SECTION: Configure unused pins as outputs driven LOW */
    PORTB.DIRSET = PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm;  // Unused PB2-PB5
    PORTB.OUTCLR = PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm;
    
    PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;  // Unused PC0-PC3
    PORTC.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
}

/**
 * Initialize millisecond timer
 */
void init_timer(void) {
    /* Configure Timer/Counter B0 for periodic interrupts */
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;
    
    /* Set timer period for 1ms with 3.3MHz clock */
    TCB0.CCMP = 3300;
    
    /* Enable timer interrupt */
    TCB0.INTCTRL = TCB_CAPT_bm;
    
    /* Enable timer with 1:1 prescaler */
    TCB0.CTRLA = TCB_ENABLE_bm;
}

/**
 * Function to check if enough time has passed for debouncing
 */
static bool debounce_check(uint32_t *last_time) {
    uint32_t current = current_time;
    
    if (current - *last_time >= DEBOUNCE_TIME_MS) {
        *last_time = current;
        return true;
    }
    
    return false;
}

/**
 * Initialize all system components
 */
void system_init(void) {
    // Configure GPIO pins
    init_gpio();
    
    // Initialize I2C communication
    init_i2c();
    
    // Initialize the shift registers for LED control
    init_shift_registers();
    
    // Initialize the VEML7700 light sensor
    veml7700_init();
    
    // Initialize the millisecond timer
    init_timer();
    
    // Load settings from EEPROM
    load_settings();
    
    // Enable global interrupts
    sei();
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
 * 
 * This function shows the calculated shutter speed using LEDs.
 * - For fractional shutter speeds (<1 second): 
 *   - Main LED at full brightness
 *   - If >25% to next value, next LED at half brightness
 * - For whole second shutter speeds (≥1 second):
 *   - Main LED and potentially next LED (if >25% to next) blink at 2Hz
 * 
 * Parameters:
 *   shutter_speed - The calculated shutter speed in seconds
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
 * 
 * This function uses the LEDs to display the current brightness level
 * as a visual indicator during brightness adjustment mode.
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

/**
 * Timer interrupt service routine
 */
ISR(TCB0_INT_vect) {
    // Increment millisecond counter
    current_time++;
    
    // Check if mode button is being held
    if (mode_button_down && !mode_button_held) {
        if (current_time - mode_button_press_time >= BUTTON_HOLD_THRESHOLD) {
            // Mode button has been held down long enough
            mode_button_held = true;
            
            // Enter brightness adjustment mode
            in_brightness_mode = true;
            display_brightness();
        }
    }
    
    // Handle ISO mode blinking (separate from seconds blinking)
    if (display_mode == DISPLAY_ISO && !in_brightness_mode) {
        static uint32_t last_iso_blink_time = 0;
        
        if (current_time - last_iso_blink_time >= BLINK_MED_MS) {
            last_iso_blink_time = current_time;
            iso_blink_state = !iso_blink_state;
            
            // Update settings column based on blink state
            if (iso_blink_state) {
                // Turn on ISO LEDs
                int index = find_nearest_iso_index(iso_setting);
                set_led(index, 0, true);
            } else {
                // Turn off only the ISO column during blink
                for (int i = 0; i < LEDS_PER_COLUMN; i++) {
                    set_led(i, 0, false);
                }
            }
            
            // Update the shift registers
            update_shift_registers();
        }
    }
    
    // Handle whole seconds blinking (separate from ISO blinking)
    if (showing_seconds && !in_brightness_mode) {
        static uint32_t last_seconds_blink_time = 0;
        
        if (current_time - last_seconds_blink_time >= BLINK_FAST_MS) {
            last_seconds_blink_time = current_time;
            seconds_blink_state = !seconds_blink_state;
            
            // Update shutter column based on blink state
            if (seconds_blink_state) {
                // Recalculate and show the shutter speed
                if (last_lux_reading > 0) {
                    float shutter_speed = calculate_shutter_speed(last_lux_reading, iso_setting, aperture_setting);
                    
                    // We need to restore the LEDs but can't call update_shutter_display as that would
                    // reset the blink state. Instead, manually restore the LED states.
                    float display_speed = shutter_speed >= 1.0 ? shutter_speed : 1.0 / shutter_speed;
                    int index = find_nearest_shutter_index(display_speed);
                    set_led(index, 1, true);
                    
                    // If needed, also show the next LED for intermediate values
                    float percent_to_next = 0.0;
                    if (index < (SHUTTER_COUNT - 1)) {
                        float range = shutter_speed_values[index+1] - shutter_speed_values[index];
                        percent_to_next = (display_speed - shutter_speed_values[index]) / range;
                        
                        if (percent_to_next > 0.25 && index < (SHUTTER_COUNT - 1)) {
                            set_led_brightness(index + 1, 1, current_brightness / 2);
                        }
                    }
                }
            } else {
                // Turn off only the shutter column LEDs during blink
                for (int i = 0; i < LEDS_PER_COLUMN; i++) {
                    set_led(i, 1, false);
                }
            }
            
            // Update the shift registers
            update_shift_registers();
        }
    }
    
    // Clear the interrupt flag
    TCB0.INTFLAGS = TCB_CAPT_bm;
}

/**
 * Button interrupt service routine with debouncing
 */
ISR(PORTA_PORT_vect) {
    /* Check which button triggered the interrupt */
    
    // Mode button handling for both press and release
    if (PORTA.INTFLAGS & MODE_BUTTON_PIN) {
        // Check if it's a press (pin went low) or release (pin went high)
        if (!(PORTA.IN & MODE_BUTTON_PIN)) {
            // Button is pressed (pin is low)
            if (debounce_check(&last_mode_press_time)) {
                mode_button_down = true;
                mode_button_press_time = current_time;
                last_activity_time = current_time;
            }
        } else {
            // Button is released (pin is high)
            mode_button_down = false;
            
            if (mode_button_held) {
                // This was a release after holding, exit brightness mode
                mode_button_held = false;
                in_brightness_mode = false;
                
                // Save the brightness setting to EEPROM
                save_settings();
                
                // Restore normal display
                update_settings_display();
                if (last_lux_reading > 0) {
                    update_shutter_display(calculate_shutter_speed(last_lux_reading, iso_setting, aperture_setting));
                }
            } else {
                // This was a regular click (not held)
                if (debounce_check(&last_mode_press_time)) {
                    mode_button_flag = true;
                    last_activity_time = current_time;
                }
            }
        }
        PORTA.INTFLAGS = MODE_BUTTON_PIN;  // Clear the interrupt flag
    }
    
    // Read button with debouncing
    if (PORTA.INTFLAGS & READ_BUTTON_PIN) {
        if (debounce_check(&last_read_press_time)) {
            wake_flag = true;
            last_activity_time = current_time;
        }
        PORTA.INTFLAGS = READ_BUTTON_PIN;
    }
    
    // Up button with debouncing
    if (PORTA.INTFLAGS & UP_BUTTON_PIN) {
        if (debounce_check(&last_up_press_time)) {
            up_button_flag = true;
            last_activity_time = current_time;
        }
        PORTA.INTFLAGS = UP_BUTTON_PIN;
    }
    
    // Down button with debouncing
    if (PORTA.INTFLAGS & DOWN_BUTTON_PIN) {
        if (debounce_check(&last_down_press_time)) {
            down_button_flag = true;
            last_activity_time = current_time;
        }
        PORTA.INTFLAGS = DOWN_BUTTON_PIN;
    }
}

/**
 * EEPROM functions for saving and loading settings
 */

/**
 * Save current settings to EEPROM
 */
void save_settings(void) {
    /* Save ISO setting (16-bit value) */
    eeprom_write_word(EEPROM_ISO_ADDR, iso_setting);
    
    /* Save aperture setting (32-bit float) */
    eeprom_write_float(EEPROM_APERTURE_ADDR, aperture_setting);
    
    /* Save LED brightness setting (8-bit value) */
    eeprom_write_byte(EEPROM_BRIGHTNESS_ADDR, current_brightness);
    
    /* Mark that settings have been saved */
    eeprom_write_byte(EEPROM_FIRST_RUN_ADDR, 0x42);
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
        current_brightness = DEFAULT_BRIGHTNESS;
        
        /* Save defaults to EEPROM */
        save_settings();
        return;
    }
    
    /* Load and validate ISO setting */
    uint16_t loaded_iso = eeprom_read_word(EEPROM_ISO_ADDR);
    if (loaded_iso >= 25 && loaded_iso <= 3200) {
        iso_setting = loaded_iso;
    } else {
        iso_setting = 400;  // Default if invalid
    }
    
    /* Load and validate aperture setting */
    float loaded_aperture = eeprom_read_float(EEPROM_APERTURE_ADDR);
    if (loaded_aperture >= 1.0 && loaded_aperture <= 45.0) {
        aperture_setting = loaded_aperture;
    } else {
        aperture_setting = 11.0;  // Default if invalid
    }
    
    /* Load and validate brightness setting */
    uint8_t loaded_brightness = eeprom_read_byte(EEPROM_BRIGHTNESS_ADDR);
    if (loaded_brightness >= MIN_BRIGHTNESS && loaded_brightness <= MAX_BRIGHTNESS) {
        current_brightness = loaded_brightness;
    } else {
        current_brightness = DEFAULT_BRIGHTNESS;  // Default if invalid
    }
    
    /* If any setting was invalid, save the corrected values */
    if (loaded_iso != iso_setting || 
        loaded_aperture != aperture_setting || 
        loaded_brightness != current_brightness) {
        save_settings();
    }
}

/**
 * Power management functions
 */

/**
 * Enter sleep mode to save power
 */
void enter_sleep_mode(void) {
    /* Set all LEDs off to save power */
    for (int i = 0; i < LEDS_PER_COLUMN; i++) {
        set_led(i, 0, false);
        set_led(i, 1, false);
    }
    update_shift_registers();
    
    /* Put the VEML7700 into power saving mode */
    veml7700_power_save_enable(3); // Maximum power saving
    
    /* Configure sleep mode (Power-Down for minimal power consumption) */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    
    /* Enter sleep mode */
    sleep_mode();
    
    /* Code resumes here after wake-up */
    wake_from_sleep();
}

/**
 * Wake up from sleep mode
 */
void wake_from_sleep(void) {
    /* Disable sleep mode */
    sleep_disable();
    
    /* Take VEML7700 out of power saving mode */
    veml7700_power_save_disable();
    
    /* Reset the activity timer */
    last_activity_time = current_time;
    
    /* Allow time for devices to wake up */
    _delay_ms(10);
}

/**
 * Main program entry point
 */
int main(void) {
    // Initialize all system components
    system_init();
    
    // Display initial settings (aperture or ISO)
    update_settings_display();
    
    // Main loop
    while (1) {
        // Handle brightness adjustment mode
        if (in_brightness_mode) {
            // Check for up button press in brightness mode
            if (up_button_flag) {
                up_button_flag = false;
                
                // Increase brightness by BRIGHTNESS_STEP
                if (current_brightness <= MAX_BRIGHTNESS - BRIGHTNESS_STEP) {
                    current_brightness += BRIGHTNESS_STEP;
                } else {
                    current_brightness = MAX_BRIGHTNESS;
                }
                
                // Update the brightness display
                display_brightness();
            }
            
            // Check for down button press in brightness mode
            if (down_button_flag) {
                down_button_flag = false;
                
                // Decrease brightness by BRIGHTNESS_STEP
                if (current_brightness >= MIN_BRIGHTNESS + BRIGHTNESS_STEP) {
                    current_brightness -= BRIGHTNESS_STEP;
                } else {
                    current_brightness = MIN_BRIGHTNESS;
                }
                
                // Update the brightness display
                display_brightness();
            }
            
            // Skip the rest of the loop while in brightness mode
            continue;
        }
        
        // Check if we need to take a light reading
        if (wake_flag) {
            wake_flag = false;
            
            // Take a light reading
            float lux = measure_light();
            
            // Calculate shutter speed
            float shutter_speed = calculate_shutter_speed(lux, iso_setting, aperture_setting);
            
            // Update display
            update_shutter_display(shutter_speed);
        }
        
        // Check for mode button press (toggle between ISO and aperture)
        if (mode_button_flag) {
            mode_button_flag = false;
            
            // Toggle between aperture and ISO mode
            if (display_mode == DISPLAY_APERTURE) {
                display_mode = DISPLAY_ISO;
            } else {
                display_mode = DISPLAY_APERTURE;
            }
            
            // Update the display to show the current mode
            update_settings_display();
        }
        
        // Check for up button press
        if (up_button_flag) {
            up_button_flag = false;
            
            // Up button pressed - increment current setting
            if (display_mode == DISPLAY_APERTURE) {
                aperture_setting = increment_aperture(aperture_setting);
            } else {
                iso_setting = increment_iso(iso_setting);
            }
            
            // Update display to show new setting
            update_settings_display();
            
            // Update shutter speed if we have a reading
            if (last_lux_reading > 0) {
                float shutter_speed = calculate_shutter_speed(last_lux_reading, iso_setting, aperture_setting);
                update_shutter_display(shutter_speed);
            }
        }
        
        // Check for down button press
        if (down_button_flag) {
            down_button_flag = false;
            
            // Decrement current setting based on mode
            if (display_mode == DISPLAY_APERTURE) {
                aperture_setting = decrement_aperture(aperture_setting);
            } else {
                iso_setting = decrement_iso(iso_setting);
            }
            
            // Update display to show new setting
            update_settings_display();
            
            // Update shutter speed if we have a reading
            if (last_lux_reading > 0) {
                float shutter_speed = calculate_shutter_speed(last_lux_reading, iso_setting, aperture_setting);
                update_shutter_display(shutter_speed);
            }
        }
        
        // Check for sleep timeout
        if (!wake_flag && !mode_button_flag && !up_button_flag && !down_button_flag && !in_brightness_mode) {
            if (current_time - last_activity_time > SLEEP_TIMEOUT_MS) {
                // Save settings before sleeping
                save_settings();
                
                // Enter low power sleep mode
                enter_sleep_mode();
            }
        }
    }
    
    return 0;
}