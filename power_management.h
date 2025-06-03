#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include <stdint.h>
#include <stdbool.h>

// Battery ADC Pin (PC2 as per README)
#define BATTERY_ADC_CHANNEL  ADC_MUXPOS_AIN2_gc // ADC Channel for PC2

// Battery voltage thresholds for LIR2450 (in millivolts)
#define BATTERY_VOLTAGE_FULL_MV      4200 // Fully charged LIR2450
#define BATTERY_VOLTAGE_NOMINAL_MV   3700 // Nominal voltage
#define BATTERY_VOLTAGE_LOW_MV       3300 // Low battery warning threshold
#define BATTERY_VOLTAGE_CRITICAL_MV  3000 // Critical - protect battery

// VDD voltage provided by STNS01 regulator (e.g., 3.1V = 3100mV)
#define VDD_VOLTAGE_MV 3100 
// Voltage divider resistors (example: R1=470k from VBAT, R2=220k to GND, ADC measures across R2)
// V_adc = V_bat * (R2 / (R1 + R2))
// V_bat = V_adc * ((R1 + R2) / R2)
#define VOLTAGE_DIVIDER_R1   470000.0f // Ohms
#define VOLTAGE_DIVIDER_R2   220000.0f // Ohms
#define VOLTAGE_DIVIDER_FACTOR ((VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2)


/**
 * @brief Initializes battery monitoring (ADC setup).
 */
void init_battery_monitoring(void);

/**
 * @brief Reads the current battery voltage.
 * @return uint16_t Battery voltage in millivolts.
 */
uint16_t get_battery_voltage_mv(void);

/**
 * @brief Checks if the battery is critically low.
 * @return true If battery voltage is below BATTERY_VOLTAGE_CRITICAL_MV.
 * @return false Otherwise.
 */
bool is_battery_critical(void);

/**
 * @brief Checks if the battery is low (but not critical).
 * @return true If battery voltage is below BATTERY_VOLTAGE_LOW_MV and not critical.
 * @return false Otherwise.
 */
bool is_battery_low(void);

/**
 * @brief Flashes LEDs to indicate low battery.
 * This is a blocking function.
 */
void flash_low_battery_warning(void);

/**
 * @brief Puts the device into Power-Down sleep mode.
 * Handles disabling peripherals and configuring wake-up sources.
 */
void enter_power_down_sleep(void);

/**
 * @brief Actions to take upon waking from sleep.
 * Re-enables peripherals.
 */
void wake_from_power_down_sleep(void);

#endif // POWER_MANAGEMENT_H
