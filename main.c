/**
 * Light Meter Main Program - Refactored
 *
 * ATtiny3217 based light meter with VEML7700 and 6x4 LED matrix.
 * Incorporates refactoring for stability and race condition avoidance.
 */

 #define F_CPU 4000000UL // 4MHz CPU Frequency

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <avr/sleep.h>
 #include <util/delay.h>
 #include <util/atomic.h> // For ATOMIC_BLOCK
 #include <stdbool.h>
 #include <string.h>      // For memset
 #include <math.h>        // For fabsf
 
 #include "veml7700.h"
 #include "light_meter.h"
 #include "power_management.h"
 #include "eeprom.h"
 #include "i2c.h"
 #include "led_matrix.h"
 
 // --- Global Variables ---
 // Settings
 uint16_t iso_setting = 400;
 float aperture_setting = 11.0f;
 uint8_t display_mode = 0; // 0 for Aperture, 1 for ISO
 #define DISPLAY_APERTURE 0
 #define DISPLAY_ISO 1
 uint8_t current_led_matrix_brightness_level = FULL_BRIGHTNESS; // Fixed, not user-adjustable
 
 // Button Flags (volatile as set by ISR)
 volatile bool wake_button_flag = false;
 volatile bool mode_button_flag = false;
 volatile bool up_button_flag = false;
 volatile bool down_button_flag = false;
 
 // Timing (volatile as current_time_ms is modified by ISR)
 volatile uint32_t current_time_ms = 0;
 volatile uint32_t last_activity_time_ms = 0;
 volatile uint32_t last_blink_time_ms = 0; 
 volatile uint32_t last_battery_check_trigger_time_ms = 0;
 
 // State Flags
 bool showing_seconds_shutter = false; 
 volatile bool critical_battery_shutdown_flag = false;
 volatile bool display_low_battery_warning_flag = false;
 volatile bool request_battery_check_flag = false; 
 
 // Blink states for display
 bool iso_blink_led_on = false;
 bool seconds_blink_led_on = false;
 
 // Debounce and Timeout Constants
 #define DEBOUNCE_TIME_MS 50     
 #define SLEEP_TIMEOUT_MS 20000  
 #define BATTERY_CHECK_INTERVAL_MS 30000 
 
 // Blink intervals for display
 #define ISO_BLINK_INTERVAL_MS 500    
 #define SECONDS_BLINK_INTERVAL_MS 250 
 
// Button Pins (UPDATED - Read/Wake moved to PA2 for power-down wake capability)
// PA1: Mode Button
// PA2: Read/Wake Button (MOVED from PA3 - PA2 can wake from power-down sleep)
// PA3: Up Button (MOVED from PA2)  
// PA4: Down Button

#define MODE_BUTTON_PORT PORTA
#define READ_BUTTON_PORT PORTA  // Now PA2
#define UP_BUTTON_PORT   PORTA  // Now PA3
#define DOWN_BUTTON_PORT PORTA

#define MODE_BUTTON_PIN_CTRL PIN1CTRL 
#define READ_BUTTON_PIN_CTRL PIN2CTRL  // Changed from PIN3CTRL
#define UP_BUTTON_PIN_CTRL   PIN3CTRL  // Changed from PIN2CTRL
#define DOWN_BUTTON_PIN_CTRL PIN4CTRL

#define MODE_BUTTON_PIN_bm PIN1_bm
#define READ_BUTTON_PIN_bm PIN2_bm     // Changed from PIN3_bm
#define UP_BUTTON_PIN_bm   PIN3_bm     // Changed from PIN2_bm
#define DOWN_BUTTON_PIN_bm PIN4_bm

 
 
 // --- Function Prototypes ---
 static void init_gpio_pins(void);
 static void init_system_timer_tcb0(void);
 static void system_init_all(void);
 static void handle_button_actions(void);
 static void update_settings_on_display(void);
 static void update_shutter_speed_on_display(float calculated_shutter_seconds);
 static void process_periodic_tasks(void);
 
 
 // --- Initialization Functions ---
 static void init_gpio_pins(void) {
    // Button Pins as inputs with pull-ups and falling edge interrupt
    MODE_BUTTON_PORT.DIRCLR = MODE_BUTTON_PIN_bm;
    READ_BUTTON_PORT.DIRCLR = READ_BUTTON_PIN_bm;  // PA2 now
    UP_BUTTON_PORT.DIRCLR = UP_BUTTON_PIN_bm;      // PA3 now
    DOWN_BUTTON_PORT.DIRCLR = DOWN_BUTTON_PIN_bm;

    // Update PINnCTRL registers with new assignments
    PORTA.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // Mode Button (PA1) - unchanged
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // Read/Wake Button (PA2) - CHANGED
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // Up Button (PA3) - CHANGED  
    PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // Down Button (PA4) - unchanged
    
    // UPDI Pin (PA0) - unchanged
    PORTA.DIRCLR = PIN0_bm; 
    PORTA.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
}
 
 static void init_system_timer_tcb0(void) {
     TCB0.CTRLB = TCB_CNTMODE_INT_gc;    
     TCB0.CCMP = 4000;                   
     TCB0.INTCTRL = TCB_CAPT_bm;         
     TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; 
 }
 
 static void system_init_all(void) {
     init_gpio_pins();
     init_i2c();
     
     if (veml7700_init() != 0) {
         // Handle VEML7700 init error 
     }
     
     init_led_matrix(); 
     init_battery_monitoring();
     load_settings_from_eeprom();
 
     init_system_timer_tcb0(); 
     init_matrix_timer();      
 
     last_activity_time_ms = 0; 
     current_time_ms = 0;       
 }
 
 
 // --- ISRs ---
 ISR(TCB0_INT_vect) { 
     TCB0.INTFLAGS = TCB_CAPT_bm; 
     current_time_ms++;
 
     if ((current_time_ms - last_battery_check_trigger_time_ms) >= BATTERY_CHECK_INTERVAL_MS) {
         last_battery_check_trigger_time_ms = current_time_ms;
         request_battery_check_flag = true;
     }
 
     uint32_t current_blink_cycle_time = current_time_ms - last_blink_time_ms;
 
     if (display_mode == DISPLAY_ISO) {
         if (current_blink_cycle_time >= ISO_BLINK_INTERVAL_MS) {
             iso_blink_led_on = !iso_blink_led_on;
         }
     } else {
          iso_blink_led_on = true; 
     }
 
     if (showing_seconds_shutter) {
         if (current_blink_cycle_time >= SECONDS_BLINK_INTERVAL_MS) {
             seconds_blink_led_on = !seconds_blink_led_on;
         }
     } else {
         seconds_blink_led_on = true; 
     }
     
     uint32_t longest_interval = (ISO_BLINK_INTERVAL_MS > SECONDS_BLINK_INTERVAL_MS) ? ISO_BLINK_INTERVAL_MS : SECONDS_BLINK_INTERVAL_MS;
     if(current_blink_cycle_time >= longest_interval ){
          last_blink_time_ms = current_time_ms; 
     }
 }
 
 ISR(TCB1_INT_vect) { 
     TCB1.INTFLAGS = TCB_CAPT_bm; 
 
     ROWS_PORT_A.OUTCLR = ROW1_PIN | ROW2_PIN | ROW3_PIN;
     ROWS_PORT_C.OUTCLR = ROW4_PIN | ROW5_PIN | ROW6_PIN;
 
     pwm_phase++;
     if (pwm_phase >= 2) { 
         pwm_phase = 0;
         current_row = (current_row + 1) % ROWS_PER_COLUMN;
     }
 
     COLS_PORT.OUTSET = COL1_PIN | COL2_PIN | COL3_PIN | COL4_PIN;
 
     for (uint8_t phys_col = 0; phys_col < 4; phys_col++) {
         uint8_t led_idx = led_index_lookup[current_row][phys_col];
         if (led_idx >= TOTAL_LEDS) continue;
 
         if (led_matrix_data[led_idx]) {
             bool illuminate = false;
             if (led_brightness[led_idx] == FULL_BRIGHTNESS) {
                 illuminate = true;
             } else if (led_brightness[led_idx] == HALF_BRIGHTNESS && pwm_phase == 0) {
                 illuminate = true;
             }
             
             uint8_t logical_led_col = (led_idx < LEDS_PER_COLUMN) ? 0 : 1;
 
             if (logical_led_col == 0 && display_mode == DISPLAY_ISO) { 
                 if (!iso_blink_led_on) illuminate = false;
             }
             if (logical_led_col == 1 && showing_seconds_shutter) { 
                 if (!seconds_blink_led_on) illuminate = false;
             }
 
             if (illuminate) {
                 if (phys_col == 0) COLS_PORT.OUTCLR = COL1_PIN;
                 else if (phys_col == 1) COLS_PORT.OUTCLR = COL2_PIN;
                 else if (phys_col == 2) COLS_PORT.OUTCLR = COL3_PIN;
                 else if (phys_col == 3) COLS_PORT.OUTCLR = COL4_PIN;
             }
         }
     }
 
     if (current_row < 3) { 
         if (current_row == 0) ROWS_PORT_A.OUTSET = ROW1_PIN;
         else if (current_row == 1) ROWS_PORT_A.OUTSET = ROW2_PIN;
         else if (current_row == 2) ROWS_PORT_A.OUTSET = ROW3_PIN;
     } else { 
         if (current_row == 3) ROWS_PORT_C.OUTSET = ROW4_PIN;
         else if (current_row == 4) ROWS_PORT_C.OUTSET = ROW5_PIN;
         else if (current_row == 5) ROWS_PORT_C.OUTSET = ROW6_PIN;
     }
 }
 
 ISR(PORTA_PORT_vect) { 
    uint32_t time_now;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        time_now = current_time_ms;
    }
    
    static volatile uint32_t last_mode_press_time = 0;
    static volatile uint32_t last_read_press_time = 0;  // Now for PA2
    static volatile uint32_t last_up_press_time = 0;    // Now for PA3
    static volatile uint32_t last_down_press_time = 0;

    if (PORTA.INTFLAGS & MODE_BUTTON_PIN_bm) {
        PORTA.INTFLAGS = MODE_BUTTON_PIN_bm; 
        if ((time_now - last_mode_press_time) >= DEBOUNCE_TIME_MS) {
            mode_button_flag = true;
            last_activity_time_ms = time_now;
            last_mode_press_time = time_now;
        }
    }
    
    // READ BUTTON NOW ON PA2 (was PA3)
    if (PORTA.INTFLAGS & READ_BUTTON_PIN_bm) {  // Now PIN2_bm
        PORTA.INTFLAGS = READ_BUTTON_PIN_bm;    // Now PIN2_bm
        if ((time_now - last_read_press_time) >= DEBOUNCE_TIME_MS) {
            wake_button_flag = true; 
            last_activity_time_ms = time_now;
            last_read_press_time = time_now;
        }
    }
    
    // UP BUTTON NOW ON PA3 (was PA2)  
    if (PORTA.INTFLAGS & UP_BUTTON_PIN_bm) {    // Now PIN3_bm
        PORTA.INTFLAGS = UP_BUTTON_PIN_bm;      // Now PIN3_bm
        if ((time_now - last_up_press_time) >= DEBOUNCE_TIME_MS) {
            up_button_flag = true;
            last_activity_time_ms = time_now;
            last_up_press_time = time_now;
        }
    }
    
    if (PORTA.INTFLAGS & DOWN_BUTTON_PIN_bm) {
        PORTA.INTFLAGS = DOWN_BUTTON_PIN_bm;
        if ((time_now - last_down_press_time) >= DEBOUNCE_TIME_MS) {
            down_button_flag = true;
            last_activity_time_ms = time_now;
            last_down_press_time = time_now;
        }
    }
}
 
 
 // --- Main Application Logic ---
 static void update_settings_on_display(void) {
     for (uint8_t i = 0; i < LEDS_PER_COLUMN; ++i) {
         set_led(i, 0, false); 
     }
 
     if (display_mode == DISPLAY_APERTURE) {
         int base_idx = find_nearest_aperture_index(aperture_setting);
         set_led(base_idx, 0, true);
         set_led_brightness(base_idx, 0, FULL_BRIGHTNESS);
 
         bool is_standard = false;
         for(int i=0; i < APERTURE_COUNT; ++i) {
             if (fabsf(aperture_setting - aperture_values[i]) < 0.001f * aperture_values[i]) {
                 is_standard = true;
                 break;
             }
         }
         if (!is_standard) { 
             if (aperture_setting > aperture_values[base_idx] && base_idx < APERTURE_COUNT - 1) {
                 set_led(base_idx + 1, 0, true); 
                 set_led_brightness(base_idx + 1, 0, FULL_BRIGHTNESS);
             } else if (aperture_setting < aperture_values[base_idx] && base_idx > 0) {
                  set_led(base_idx -1, 0, true); 
                  set_led_brightness(base_idx -1, 0, FULL_BRIGHTNESS);
             } else if (base_idx == 0 && aperture_setting < aperture_values[0] && APERTURE_COUNT > 1) {
                  set_led(0,0,true); set_led_brightness(0,0,FULL_BRIGHTNESS);
                  if (APERTURE_COUNT > 1) {set_led(1,0,true); set_led_brightness(1,0,FULL_BRIGHTNESS);}
             } else if (base_idx == APERTURE_COUNT -1 && aperture_setting > aperture_values[APERTURE_COUNT-1] && APERTURE_COUNT > 1) {
                  set_led(APERTURE_COUNT-1,0,true); set_led_brightness(APERTURE_COUNT-1,0,FULL_BRIGHTNESS);
                  if (APERTURE_COUNT > 1) {set_led(APERTURE_COUNT-2,0,true); set_led_brightness(APERTURE_COUNT-2,0,FULL_BRIGHTNESS);}
             }
         }
     } else { 
         int idx = find_nearest_iso_index(iso_setting);
         set_led(idx, 0, true);
         set_led_brightness(idx, 0, FULL_BRIGHTNESS);
     }
 }
 
 static void update_shutter_speed_on_display(float calculated_shutter_seconds) {
     for (uint8_t i = 0; i < LEDS_PER_COLUMN; ++i) {
         set_led(i, 1, false); 
     }
 
     showing_seconds_shutter = (calculated_shutter_seconds >= 0.95f); 
                                                                 
     int base_idx = find_nearest_shutter_index(calculated_shutter_seconds);
     set_led(base_idx, 1, true);
     set_led_brightness(base_idx, 1, FULL_BRIGHTNESS);
 
     float interpolation = get_shutter_interpolation(calculated_shutter_seconds, base_idx);
     if (interpolation > 0.25f && base_idx < SHUTTER_COUNT - 1) {
         set_led(base_idx + 1, 1, true);
         set_led_brightness(base_idx + 1, 1, HALF_BRIGHTNESS);
     }
 }
 
 
 static void handle_button_actions(void) {
     uint32_t time_now;
     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
         time_now = current_time_ms;
     }
 
     if (wake_button_flag) {
         wake_button_flag = false;
         last_activity_time_ms = time_now;
 
         bool tcb1_was_enabled = (TCB1.CTRLA & TCB_ENABLE_bm);
         if (tcb1_was_enabled) TCB1.CTRLA &= ~TCB_ENABLE_bm;
         led_matrix_all_off_immediate(); 
         _delay_ms(10); 
 
         float lux = measure_light_auto_adjust();
         
         if (tcb1_was_enabled) TCB1.CTRLA |= TCB_ENABLE_bm;
 
         if (lux >= 0.0f) { 
             float shutter = calculate_shutter_speed(lux, iso_setting, aperture_setting);
             update_settings_on_display(); 
             update_shutter_speed_on_display(shutter);
         } else {
             update_settings_on_display(); 
             for (uint8_t i = 0; i < LEDS_PER_COLUMN; ++i) set_led(i, 1, false); 
         }
     }
 
     if (mode_button_flag) {
         mode_button_flag = false;
         last_activity_time_ms = time_now;
         display_mode = (display_mode == DISPLAY_APERTURE) ? DISPLAY_ISO : DISPLAY_APERTURE;
         iso_blink_led_on = (display_mode == DISPLAY_ISO); 
         seconds_blink_led_on = true; 
         update_settings_on_display();
         if (last_lux_reading > 0.0f) { 
              float shutter = calculate_shutter_speed(last_lux_reading, iso_setting, aperture_setting);
              update_shutter_speed_on_display(shutter);
         }
     }
 
     bool settings_changed = false;
     if (up_button_flag) {
         up_button_flag = false;
         last_activity_time_ms = time_now;
         if (display_mode == DISPLAY_APERTURE) {
             aperture_setting = increment_aperture(aperture_setting);
         } else {
             iso_setting = increment_iso(iso_setting);
         }
         settings_changed = true;
     }
 
     if (down_button_flag) {
         down_button_flag = false;
         last_activity_time_ms = time_now;
         if (display_mode == DISPLAY_APERTURE) {
             aperture_setting = decrement_aperture(aperture_setting);
         } else {
             iso_setting = decrement_iso(iso_setting);
         }
         settings_changed = true;
     }
 
     if (settings_changed) {
         update_settings_on_display();
         if (last_lux_reading > 0.0f) {
             float shutter = calculate_shutter_speed(last_lux_reading, iso_setting, aperture_setting);
             update_shutter_speed_on_display(shutter);
         }
     }
 }
 
 static void process_periodic_tasks(void) {
     uint32_t time_now;
     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
         time_now = current_time_ms;
     }
 
     if (request_battery_check_flag) {
         request_battery_check_flag = false;
         if (!critical_battery_shutdown_flag) { 
             if (is_battery_critical()) {
                 critical_battery_shutdown_flag = true;
             } else if (is_battery_low()) {
                 display_low_battery_warning_flag = true;
             }
         }
     }
     
     if (critical_battery_shutdown_flag) {
         save_settings_to_eeprom();
         enter_power_down_sleep(); 
         // After enter_power_down_sleep returns, the device has woken up
         // and internal_wake_from_sleep_sequence (in power_management.c) has run.
         // This includes re-checking battery.
         // No explicit return here, allow main loop to continue to re-evaluate state.
     }
 
     if (display_low_battery_warning_flag) {
         bool tcb1_was_enabled = (TCB1.CTRLA & TCB_ENABLE_bm);
         if (tcb1_was_enabled) TCB1.CTRLA &= ~TCB_ENABLE_bm; 
         
         flash_low_battery_warning(); 
         
         if (tcb1_was_enabled) TCB1.CTRLA |= TCB_ENABLE_bm; 
         
         display_low_battery_warning_flag = false;
         last_activity_time_ms = time_now; 
         update_settings_on_display(); 
         if(last_lux_reading > 0.0f) {
             update_shutter_speed_on_display(calculate_shutter_speed(last_lux_reading, iso_setting, aperture_setting));
         }
     }
 
     // Check sleep timeout only if not handling a critical shutdown or battery warning display
     // And also ensure that if we just woke from critical shutdown, we don't immediately sleep.
     // The last_activity_time_ms is updated by internal_wake_from_sleep_sequence.
     if (!critical_battery_shutdown_flag && !display_low_battery_warning_flag) {
         if ((time_now - last_activity_time_ms) >= SLEEP_TIMEOUT_MS) {
             save_settings_to_eeprom();
             enter_power_down_sleep();
             // Device has woken up and re-initialized by the time enter_power_down_sleep returns.
         }
     }
      critical_battery_shutdown_flag = false; // Clear flag after handling, it will be re-evaluated if still critical
 }
 
 
 // --- Main Function ---
 int main(void) {
     cli(); 
     system_init_all();
     update_settings_on_display(); 
     sei(); 
 
     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
         last_activity_time_ms = current_time_ms; 
     }
 
     while (1) {
         handle_button_actions();
         process_periodic_tasks(); 
         
         // The main loop is event-driven by ISR flags and periodic checks.
         // If the CPU has nothing to do, it can be put into a light sleep mode here
         // to save a bit more power, waking on any interrupt.
         // For example:
         // if (!wake_button_flag && !mode_button_flag && !up_button_flag && !down_button_flag && !request_battery_check_flag) {
         //    set_sleep_mode(SLEEP_MODE_IDLE);
         //    sleep_cpu();
         // }
     }
     return 0; 
 }
 