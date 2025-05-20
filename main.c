/**
 * Light Meter Main Program - LED Matrix Implementation
 * 
 * This file contains the main program for a light meter using direct LED matrix
 * control with 24 LEDs arranged in a 6×4 matrix. This implementation replaces
 * the previous shift register approach with direct GPIO control for reduced
 * component count and cost.
 * 
 * Overview of how the program works:
 * 1. The system initializes all components (LED matrix, light sensor, buttons, battery monitor)
 * 2. It loads saved settings from EEPROM (or uses defaults if first run)
 * 3. It enters a main loop that checks for button presses
 * 4. When the read button is pressed, it takes a light reading and displays the appropriate shutter speed
 * 5. When up/down buttons are pressed, it changes settings and updates the display
 * 6. After 20 seconds of inactivity, it saves settings and enters low-power sleep mode
 * 7. Battery level is monitored periodically and warnings are displayed when low
 */

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <avr/sleep.h>
 #include <util/delay.h>
 #include <stdbool.h>
 #include <string.h>  // For memset
 #include <math.h>    // For fabsf
 
 /* Include our custom header files */
 #include "veml7700.h"
 #include "light_meter.h"
 #include "power_management.h"
 #include "eeprom.h"
 #include "i2c.h"
 #include "led_matrix.h"
 
 /**
  * GLOBAL VARIABLES
  */
 
 /* LED state variables */
 volatile uint8_t pwm_phase = 0;             // Phase for simplified PWM (0-1, for full/half brightness)
 
 /* Button flags */
 volatile bool wake_flag = false;            // Flag for wake button press
 volatile bool mode_button_flag = false;     // Flag for mode button press
 volatile bool up_button_flag = false;       // Flag for up button press
 volatile bool down_button_flag = false;     // Flag for down button press
 volatile bool mode_button_held = false;     // Flag for mode button being held
 
 /* Button state tracking (for hold detection) */
 volatile uint32_t mode_button_press_time = 0;  // When mode button was pressed
 volatile bool mode_button_down = false;        // Whether mode button is currently down
 
 /* Time constants for button hold detection */
 #define BUTTON_HOLD_THRESHOLD 500  // Time in ms to count as a "hold" (500ms = 0.5 seconds)
 
 /* Timing variables for sleep timeout and blink control */
 volatile uint32_t current_time = 0;       // Current system time in milliseconds
 volatile uint32_t last_activity_time = 0; // Time of last button activity
 volatile uint32_t last_blink_time = 0;    // Time of last blink toggle
 volatile uint32_t last_battery_check = 0; // Time of last battery level check
 
 /* Battery check interval (every 30 seconds) */
 #define BATTERY_CHECK_INTERVAL 30000
 
 /* Battery state tracking */
 volatile bool critical_battery_shutdown = false;  // Flag to trigger immediate shutdown
 volatile bool displaying_battery_warning = false; // Currently showing battery warning
 volatile uint32_t battery_warning_start_time = 0; // When we started showing warning
 volatile bool led_state_saved = false;           // Whether we've saved the LED state
 
 /* Arrays to store LED state during battery warning */
 uint8_t saved_led_matrix_data[TOTAL_LEDS];
 uint8_t saved_led_brightness[TOTAL_LEDS];
 
 /* Separate blink state tracking for different functions */
 volatile bool iso_blink_state = false;      // Blink state for ISO mode
 volatile bool seconds_blink_state = false;  // Blink state for whole seconds display
 
 /* Default settings (will be overridden by loaded settings if available) */
 uint16_t iso_setting = 400;               // Default ISO: 400
 float aperture_setting = 11.0;            // Default f-stop: f/11
 uint8_t display_mode = DISPLAY_APERTURE;  // Default display mode: aperture
 bool showing_seconds = false;             // Whether we're showing shutter speeds in seconds
 
 /* Global brightness level - fixed value since user adjustment is removed */
 uint8_t current_brightness = FULL_BRIGHTNESS;
 
 /* Sleep timeout in milliseconds (20 seconds) */
 #define SLEEP_TIMEOUT_MS 20000
 
 /* Blink intervals in milliseconds (for software-based blinking) */
 #define BLINK_SLOW_MS 1000  // 0.5Hz - once per 2 seconds
 #define BLINK_MED_MS  500   // 1Hz - once per second (ISO mode indicator)
 #define BLINK_FAST_MS 250   // 2Hz - twice per second (whole seconds indicator)
 
 /* Button pins - updated for new pinout */
 #define MODE_BUTTON_PIN  PIN1_bm   // Mode button on PA1
 #define READ_BUTTON_PIN  PIN2_bm   // Read button on PA2
 #define UP_BUTTON_PIN    PIN3_bm   // Up button on PA3
 #define DOWN_BUTTON_PIN  PIN4_bm   // Down button on PA4
 
 /* Button debouncing constants and variables */
 #define DEBOUNCE_TIME_MS  20    // 20ms debounce time
 
 /* Variables to track the last time each button was pressed */
 volatile uint32_t last_read_press_time = 0;
 volatile uint32_t last_mode_press_time = 0;
 volatile uint32_t last_up_press_time = 0;
 volatile uint32_t last_down_press_time = 0;
 
 /**
  * Save the current LED state before displaying battery warning
  */
 void save_led_state(void) {
     // Only save if not already saved
     if (!led_state_saved) {
         // Copy the LED state arrays
         memcpy(saved_led_matrix_data, led_matrix_data, sizeof(led_matrix_data));
         memcpy(saved_led_brightness, led_brightness, sizeof(led_brightness));
         led_state_saved = true;
     }
 }
 
 /**
  * Restore the LED state after battery warning
  */
 void restore_led_state(void) {
     // Only restore if we saved state
     if (led_state_saved) {
         // Copy back the saved LED states
         memcpy(led_matrix_data, saved_led_matrix_data, sizeof(led_matrix_data));
         memcpy(led_brightness, saved_led_brightness, sizeof(led_brightness));
         led_state_saved = false;
     }
 }
 
 /**
  * LED Matrix Scanning PWM Interrupt Service Routine
  * 
  * This interrupt handler is called by Timer/Counter B1 at a high frequency
  * to implement matrix scanning and simplified 2-level brightness control.
  * 
  * For each interrupt:
  * 1. It turns off the current row to prevent ghosting
  * 2. Updates the PWM phase and advances to the next row when needed
  * 3. Sets column outputs based on which LEDs in the new row should be lit
  * 4. Turns on the new row
  * 
  * The simplified PWM uses just 2 phases:
  * - Full brightness LEDs are on during both phases (100% duty cycle)
  * - Half brightness LEDs are only on during phase 0 (50% duty cycle)
  */
 ISR(TCB1_INT_vect) {
     // Turn off all rows first to prevent ghosting during transition
     ROWS_PORT_A.OUTCLR = ROW1_PIN | ROW2_PIN | ROW3_PIN;
     ROWS_PORT_C.OUTCLR = ROW4_PIN | ROW5_PIN | ROW6_PIN;
     
     // Update row and PWM phase
     // We complete a full PWM cycle (both phases) before moving to the next row
     pwm_phase++;
     if (pwm_phase >= 2) {
         pwm_phase = 0;
         current_row = (current_row + 1) % 6;
     }
     
     // Set all columns high initially (all LEDs off)
     COLS_PORT.OUTSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
     
     // For each column, check if the corresponding LED should be lit
     for (uint8_t col = 0; col < 4; col++) {
         // For columns 0-1 (ISO/Aperture LEDs)
         if (col < 2) {
             // Calculate the logical LED index
             uint8_t logical_row = (col == 0) ? current_row : (current_row + 6);
             uint8_t led_idx = logical_row;  // For left side (col 0, ISO/Aperture)
             
             // Check if LED should be on in current PWM phase
             bool should_be_on = false;
             
             if (led_matrix_data[led_idx]) {
                 if (led_brightness[led_idx] == FULL_BRIGHTNESS) {
                     // Full brightness - always on
                     should_be_on = true;
                 } else if (led_brightness[led_idx] == HALF_BRIGHTNESS) {
                     // Half brightness - only on during first phase
                     should_be_on = (pwm_phase == 0);
                 }
             }
             
             if (should_be_on) {
                 // Turn on this LED by setting its column LOW (active low)
                 COLS_PORT.OUTCLR = column_pins[col];
             }
         }
         // For columns 2-3 (Shutter Speed LEDs)
         else {
             // Calculate the logical LED index
             uint8_t logical_row = (col == 2) ? current_row : (current_row + 6);
             uint8_t led_idx = logical_row + LEDS_PER_COLUMN;  // For right side (col 1, Shutter)
             
             // Check if LED should be on in current PWM phase
             bool should_be_on = false;
             
             if (led_matrix_data[led_idx]) {
                 if (led_brightness[led_idx] == FULL_BRIGHTNESS) {
                     // Full brightness - always on
                     should_be_on = true;
                 } else if (led_brightness[led_idx] == HALF_BRIGHTNESS) {
                     // Half brightness - only on during first phase
                     should_be_on = (pwm_phase == 0);
                 }
             }
             
             if (should_be_on) {
                 // Turn on this LED by setting its column LOW (active low)
                 COLS_PORT.OUTCLR = column_pins[col];
             }
         }
     }
     
     // Turn on the current row by setting it HIGH
     if (current_row < 3) {
         // Rows 0-2 on PORTA
         ROWS_PORT_A.OUTSET = row_pins[current_row];
     } else {
         // Rows 3-5 on PORTC
         ROWS_PORT_C.OUTSET = row_pins[current_row];
     }
     
     // Clear the interrupt flag
     TCB1.INTFLAGS = TCB_CAPT_bm;
 }
 
 /**
  * Initialize GPIO pins
  * 
  * This function configures all the GPIO pins for the application,
  * including buttons, LED matrix control, and I2C communication.
  */
 void init_gpio(void) {
     /* Configure button pins as inputs with pull-ups */
     PORTA.DIRCLR = MODE_BUTTON_PIN;  // Mode button (PA1)
     PORTA.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;  // Both edges for press and release
     
     PORTA.DIRCLR = READ_BUTTON_PIN;  // Read button (PA2)
     PORTA.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
     
     PORTA.DIRCLR = UP_BUTTON_PIN;    // Up button (PA3)
     PORTA.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
     
     PORTA.DIRCLR = DOWN_BUTTON_PIN;  // Down button (PA4)
     PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
     
     /* Configure I2C pins as inputs with pull-ups */
     PORTB.DIRCLR = PIN0_bm;  // SDA pin
     PORTB.DIRCLR = PIN1_bm;  // SCL pin
     
     /* Enable pull-ups on I2C pins */
     PORTB.PIN0CTRL = PORT_PULLUPEN_bm;
     PORTB.PIN1CTRL = PORT_PULLUPEN_bm;
     
     /* Configure UPDI programming pin as input */
     PORTA.DIRCLR = PIN0_bm;  // UPDI pin
     
     /* Configure battery voltage ADC pin as input */
     PORTC.DIRCLR = PIN2_bm;  // Battery ADC input
 }
 
 /**
  * Initialize millisecond timer
  * 
  * This function sets up Timer/Counter B0 to generate interrupts
  * every millisecond for time tracking and other timing functions.
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
  * 
  * This function helps with button debouncing by checking if enough time
  * has passed since the last button press to consider it a new press.
  * 
  * @param last_time Pointer to the time variable tracking the last press
  * @return True if enough time has passed, false otherwise
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
  * 
  * This function initializes all hardware components and peripherals
  * needed for the light meter to function.
  */
 void system_init(void) {
     // Configure GPIO pins
     init_gpio();
 
     // Disable digital input buffers on unused pins to save power
     // For pins that are used as outputs or special functions (not digital inputs)
     
     // PORTA pins that don't need digital input capability
     PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;  // UPDI programming pin
     
     // Row control pins (outputs)
     PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Row 1
     PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Row 2
     PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Row 3
     
     // Column control pins (outputs)
     PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Column 1
     PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Column 2
     PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Column 3
     PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Column 4
     
     // Additional PORTC pins (outputs or unused)
     PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Row 4
     PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Row 5
     PORTC.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;  // Row 6

    // 4 unused pins on the ATTiny3217 package:
    // Set direction to input for all unused pins (before setting PIN*CTRL registers)
    PORTB.DIRCLR = PIN6_bm | PIN7_bm;
    PORTC.DIRCLR = PIN4_bm | PIN5_bm;
    // Then configure with pull-ups and disable digital input
    PORTB.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
     
     // PC2 is handled in init_battery_monitoring()
     
     // Initialize I2C communication
     init_i2c();
     
     // Initialize the LED matrix for display
     init_led_matrix();
     
     // Initialize the VEML7700 light sensor
     veml7700_init();
     
     // Initialize battery monitoring
     init_battery_monitoring();
     
     // Initialize the millisecond timer
     init_timer();
     
     // Load settings from EEPROM
     load_settings();
     
     // Enable global interrupts
     sei();
 }
 
 /**
  * Update the display to show current settings
  * 
  * This function updates the LED display to show the current
  * ISO or aperture setting based on the active display mode.
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
         set_led_brightness(index + 1, 1, HALF_BRIGHTNESS);
     }
 }
 
 /**
  * Timer interrupt service routine
  * 
  * This function is called every millisecond by the Timer/Counter B0 interrupt.
  * It handles time tracking, button hold detection, and blinking effects.
  */
 ISR(TCB0_INT_vect) {
     // Increment millisecond counter
     current_time++;
     
     // Check if mode button is being held
     if (mode_button_down && !mode_button_held) {
         if (current_time - mode_button_press_time >= BUTTON_HOLD_THRESHOLD) {
             // Mode button has been held down long enough
             mode_button_held = true;
         }
     }
     
     // Check if it's time to verify battery status
     if (current_time - last_battery_check >= BATTERY_CHECK_INTERVAL) {
         last_battery_check = current_time;
         
         // Check battery level
         if (is_battery_critical()) {
             // Critical battery - we need to sleep immediately
             critical_battery_shutdown = true;
         }
         else if (is_battery_low() && !displaying_battery_warning) {
             // Low battery - show temporary warning
             displaying_battery_warning = true;
             battery_warning_start_time = current_time;
             
             // Save current LED state to restore after warning
             save_led_state();
         }
     }
     
     // Handle ISO mode blinking (separate from seconds blinking)
     if (display_mode == DISPLAY_ISO) {
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
         }
     }
     
     // Handle whole seconds blinking (separate from ISO blinking)
     if (showing_seconds) {
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
                             set_led_brightness(index + 1, 1, HALF_BRIGHTNESS);
                         }
                     }
                 }
             } else {
                 // Turn off only the shutter column LEDs during blink
                 for (int i = 0; i < LEDS_PER_COLUMN; i++) {
                     set_led(i, 1, false);
                 }
             }
         }
     }
     
     // Clear the interrupt flag
     TCB0.INTFLAGS = TCB_CAPT_bm;
 }
 
 /**
  * Button interrupt service routine with debouncing
  * 
  * This function is called when a button press is detected.
  * It includes debouncing to prevent false button detections.
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
                 // This was a release after holding, just clear the flag
                 mode_button_held = false;
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
  * Main program entry point
  */
 int main(void) {
     // Initialize all system components
     system_init();
     
     // Display initial settings (aperture or ISO)
     update_settings_display();
     
     // Main loop
     while (1) {
         // Check if battery is critically low (both at startup and when flagged by timer)
         if (is_battery_critical() || critical_battery_shutdown) {
             // Reset the flag
             critical_battery_shutdown = false;
             
             // If battery is critically low, save settings and go straight to sleep
             save_settings();
             enter_sleep_mode();
             continue;  // After waking, restart the loop to check battery again
         }
         
         // Check if we need to show a temporary battery warning
         if (displaying_battery_warning) {
             // Flash the warning
             flash_low_battery_warning();
             
             // Clear the flag
             displaying_battery_warning = false;
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
                 float shutter_speed = calculate_shutter_speed(
                     last_lux_reading, iso_setting, aperture_setting);
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
                 float shutter_speed = calculate_shutter_speed(
                     last_lux_reading, iso_setting, aperture_setting);
                 update_shutter_display(shutter_speed);
             }
         }
         
         // Check for sleep timeout
         if (current_time - last_activity_time > SLEEP_TIMEOUT_MS) {
             // Save settings before sleeping
             save_settings();
             
             // Enter low power sleep mode
             enter_sleep_mode();
         }
     }
     
     return 0;
 }