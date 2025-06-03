#ifndef LIGHT_METER_H
#define LIGHT_METER_H

#include <stdint.h>

// Define counts for settings arrays
#define ISO_COUNT 12
#define APERTURE_COUNT 12
#define SHUTTER_COUNT 12

// Arrays of supported values (declarations)
extern const uint16_t iso_values[ISO_COUNT];
extern const float aperture_values[APERTURE_COUNT];
extern const uint16_t shutter_speed_display_values[SHUTTER_COUNT]; // For display purposes
extern const float shutter_time_seconds[SHUTTER_COUNT];      // Actual time in seconds

/**
 * @brief Calculate Exposure Value (EV) from lux.
 * @param lux Measured light level in lux.
 * @return float Calculated EV.
 */
float calculate_ev(float lux);

/**
 * @brief Calculate appropriate shutter speed.
 * @param lux Measured light level in lux.
 * @param iso Current ISO setting.
 * @param aperture Current aperture (f-number) setting.
 * @return float Calculated shutter speed in seconds.
 */
float calculate_shutter_speed(float lux, uint16_t iso, float aperture);

/**
 * @brief Find the index of the nearest standard ISO value.
 * @param iso The ISO value to match.
 * @return int Index in the iso_values array.
 */
int find_nearest_iso_index(uint16_t iso);

/**
 * @brief Find the index of the nearest standard aperture value.
 * @param aperture The aperture value (f-number) to match.
 * @return int Index in the aperture_values array.
 */
int find_nearest_aperture_index(float aperture);

/**
 * @brief Find the index of the nearest standard shutter speed.
 * @param shutter_speed_seconds The calculated shutter time in seconds.
 * @return int Index in the shutter_time_seconds array.
 */
int find_nearest_shutter_index(float shutter_speed_seconds);

/**
 * @brief Calculate the percentage distance to the next shutter speed for interpolation.
 * @param shutter_speed_seconds Calculated shutter time in seconds.
 * @param current_index Index of the nearest standard shutter speed.
 * @return float Percentage (0.0 to 1.0) towards the next faster speed.
 */
float get_shutter_interpolation(float shutter_speed_seconds, int current_index);

/**
 * @brief Increment the ISO setting to the next standard value.
 * @param current_iso The current ISO value.
 * @return uint16_t The new ISO value.
 */
uint16_t increment_iso(uint16_t current_iso);

/**
 * @brief Decrement the ISO setting to the previous standard value.
 * @param current_iso The current ISO value.
 * @return uint16_t The new ISO value.
 */
uint16_t decrement_iso(uint16_t current_iso);

/**
 * @brief Increment the aperture setting (supports half-stops).
 * @param current_aperture The current aperture value (f-number).
 * @return float The new aperture value.
 */
float increment_aperture(float current_aperture);

/**
 * @brief Decrement the aperture setting (supports half-stops).
 * @param current_aperture The current aperture value (f-number).
 * @return float The new aperture value.
 */
float decrement_aperture(float current_aperture);

#endif // LIGHT_METER_H
