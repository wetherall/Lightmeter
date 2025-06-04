/**
 * Power Management Implementation
 * * Handles sleep modes, wake-up, and battery monitoring for the light meter.
 * * Corrected sleep/wake interrupt handling.
 * * MODIFIED: Uses IDLE sleep mode instead of POWER_DOWN for compatibility.
 */

 #include <avr/io.h>
 #include <avr/sleep.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include <util/atomic.h> 
 #include <stdbool.h>     
 
 #include "power_management.h"
 #include "led_matrix.h"       
 #include "veml7700.h"         
 #include "i2c.h"              
 
 // External global variables from main.c needed for wake handling
 extern volatile uint32_t current_time_ms; 
 extern volatile uint32_t last_activity_time_ms;
 extern volatile bool wake_button_flag; // Read button is also the wake button
 extern volatile bool mode_button_flag;
 extern volatile bool up_button_flag;
 extern volatile bool down_button_flag;
 extern volatile bool critical_battery_shutdown_flag; 
 extern volatile bool display_low_battery_warning_flag; 
 
 /**
  * @brief Initializes battery monitoring (ADC setup).
  */
 void init_battery_monitoring(void) {
     PORTC.DIRCLR = PIN2_bm; 
     PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; 
 
     ADC0.CTRLA = ADC_ENABLE_bm; 
     ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc; 
     ADC0.MUXPOS = BATTERY_ADC_CHANNEL; 
     ADC0.CTRLB = ADC_SAMPNUM_ACC16_gc; 
 }
 
 /**
  * @brief Reads the current battery voltage.
  */
 uint16_t get_battery_voltage_mv(void) {
     ADC0.COMMAND = ADC_STCONV_bm; 
     while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)); 
 
     uint16_t adc_result_sum = ADC0.RES;
     uint16_t adc_result_avg = adc_result_sum >> 4; 
 
     float voltage_at_adc_pin_mv = ((float)adc_result_avg / 1023.0f) * (float)VDD_VOLTAGE_MV;
     uint16_t battery_voltage = (uint16_t)(voltage_at_adc_pin_mv * VOLTAGE_DIVIDER_FACTOR);
     
     return battery_voltage;
 }
 
 /**
  * @brief Checks if the battery is critically low.
  */
 bool is_battery_critical(void) {
     return get_battery_voltage_mv() < BATTERY_VOLTAGE_CRITICAL_MV;
 }
 
 /**
  * @brief Checks if the battery is low (but not critical).
  */
 bool is_battery_low(void) {
     uint16_t voltage = get_battery_voltage_mv();
     return (voltage < BATTERY_VOLTAGE_LOW_MV && voltage >= BATTERY_VOLTAGE_CRITICAL_MV);
 }
 
 /**
  * @brief Flashes LEDs to indicate low battery.
  */
 void flash_low_battery_warning(void) {
     led_matrix_all_off_immediate(); 
 
     for (int i = 0; i < 3; i++) {
         ROWS_PORT_A.OUTSET = ROW1_PIN; 
         COLS_PORT.OUTCLR = COL1_PIN | COL3_PIN;
         _delay_ms(200);
         
         COLS_PORT.OUTSET = COL1_PIN | COL3_PIN; 
         ROWS_PORT_A.OUTCLR = ROW1_PIN; 
         _delay_ms(200);
     }
 }
 
 /**
  * @brief Actions to take upon waking from sleep.
  * This function is now called internally by enter_power_down_sleep after waking.
  */
 static void internal_wake_from_sleep_sequence(void) {
     // Interrupts are typically re-enabled by the ISR that caused the wake.
     // If not, or for safety, ensure they are enabled here.
     sei(); 
 
     // Re-enable system tick timer (TCB0)
     // Full re-init of timer control A to ensure clock source and enable bit are set
     TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; 
     
     // Re-enable ADC
     ADC0.CTRLA |= ADC_ENABLE_bm;
     
     // Re-initialize I2C
     init_i2c(); 
 
     // Power on VEML7700 and allow stabilization
     veml7700_power_on(); 
     _delay_ms(50); // Short delay for VEML7700 to stabilize
 
     // DON'T clear button flags - this was preventing wake actions from working
     // wake_button_flag = false;     // REMOVED
     // mode_button_flag = false;     // REMOVED
     // up_button_flag = false;       // REMOVED
     // down_button_flag = false;     // REMOVED

     // Reset activity timer using the current time
     uint32_t time_now;
     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
         time_now = current_time_ms;
     }
     last_activity_time_ms = time_now;
     
     // Re-enable LED matrix timer (TCB1)
     TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; 

     // Re-check battery status immediately after fully waking peripherals
     if (is_battery_critical()) {
         critical_battery_shutdown_flag = true; 
     } else if (is_battery_low()) {
         display_low_battery_warning_flag = true;
     }
 }
 
 
 /**
  * @brief Puts the device into Power-Down sleep mode.
  * This function now handles the full sleep and wake sequence internally.
  * MODIFIED: Uses IDLE sleep mode for better pin change interrupt compatibility.
  */
 void enter_power_down_sleep(void) {
     // 1. Disable Timers and Peripherals
     TCB1.CTRLA &= ~TCB_ENABLE_bm; // Disable LED matrix timer
     led_matrix_all_off_immediate(); // Ensure LEDs are physically off
 
     veml7700_shutdown(); // Put light sensor to sleep
 
     // Configure I2C pins for low power (outputs driven low)
     PORTB.DIRSET = PIN0_bm | PIN1_bm;  
     PORTB.OUTCLR = PIN0_bm | PIN1_bm;  
     TWI0.MCTRLA = 0; // Disable TWI peripheral
 
     ADC0.CTRLA &= ~ADC_ENABLE_bm; // Disable ADC
     TCB0.CTRLA &= ~TCB_ENABLE_bm; // Disable system tick timer
 
     // 2. Configure Sleep Mode and Enable Interrupts for Wake-up
     set_sleep_mode(SLEEP_MODE_PWR_DOWN); // 
     
     cli(); // Disable interrupts briefly
     sleep_enable(); // Set Sleep Enable (SE) bit
     sei();          // IMPORTANT: Enable global interrupts *before* calling sleep_cpu()
     sleep_cpu();    // Enter sleep mode. Execution stops here until wake-up.
     
     // --- CPU WAKES UP HERE (due to an enabled interrupt) ---
     
     sleep_disable(); // Clear Sleep Enable (SE) bit after waking
     
     // 3. Call the internal wake sequence to restore peripherals and state
     internal_wake_from_sleep_sequence();
 }