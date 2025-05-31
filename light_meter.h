/**
 * Light Meter Calculations
 * 
 * This header file declares functions for performing the core light metering
 * calculations that convert light measurements (lux) into camera exposure 
 * settings (shutter speed, aperture, and ISO).
 * 
 * These calculations are based on established photography principles like:
 * - The relationship between lux and Exposure Value (EV)
 * - The Exposure Triangle (ISO, aperture, and shutter speed)
 * - The "Sunny 16" rule and related exposure formulas
 * 
 * IMPROVED: Added clearer time-based arrays and interpolation functions
 * for more understandable shutter speed calculations.
 */

#ifndef LIGHT_METER_H
#define LIGHT_METER_H

#include <stdint.h>

/**
 * CONSTANTS
 */

/* Number of available ISO and aperture values */
#define ISO_COUNT        12      // Reduced from 13 to 12 (removed ISO 6400)
#define APERTURE_COUNT   12      // Reduced from 13 to 12 (removed f/64)
#define SHUTTER_COUNT    12      // Reduced from 13 to 12 (removed 1/4000)

/**
 * FUNCTION DECLARATIONS
 * 
 * These functions handle all the light meter calculations and
 * conversions between different photography settings.
 */

/**
 * Calculate Exposure Value (EV) from lux
 * 
 * Exposure Value is a standardized way to represent light levels
 * in photography. It combines aperture and shutter speed into a
 * single number that represents the amount of light reaching the sensor.
 * 
 * Parameters:
 *   lux - The measured light level in lux
 * 
 * Returns:
 *   The calculated EV (Exposure Value)
 */
float calculate_ev(float lux);

/**
 * Calculate the appropriate shutter speed based on light level
 * 
 * This is the core light meter function that determines the correct
 * shutter speed for proper exposure given the current lighting conditions
 * and camera settings.
 * 
 * Parameters:
 *   lux - The measured light level in lux
 *   iso - The current ISO setting
 *   aperture - The current aperture (f-stop) setting
 * 
 * Returns:
 *   The calculated shutter speed in seconds
 */
float calculate_shutter_speed(float lux, uint16_t iso, float aperture);

/**
 * Increment the ISO value
 * 
 * This function increases the ISO to the next standard value.
 * 
 * Parameters:
 *   current_iso - The current ISO setting
 * 
 * Returns:
 *   The next higher standard ISO value
 */
uint16_t increment_iso(uint16_t current_iso);

/**
 * Decrement the ISO value
 * 
 * This function decreases the ISO to the next lower standard value.
 * 
 * Parameters:
 *   current_iso - The current ISO setting
 * 
 * Returns:
 *   The next lower standard ISO value
 */
uint16_t decrement_iso(uint16_t current_iso);

/**
 * Increment the aperture value
 * 
 * This function increases the aperture to the next standard or half-stop value.
 * Larger aperture values (f-stops) mean smaller physical aperture openings.
 * 
 * Parameters:
 *   current_aperture - The current aperture (f-stop) setting
 * 
 * Returns:
 *   The next higher aperture value
 */
float increment_aperture(float current_aperture);

/**
 * Decrement the aperture value
 * 
 * This function decreases the aperture to the next lower standard or half-stop value.
 * Smaller aperture values (f-stops) mean larger physical aperture openings.
 * 
 * Parameters:
 *   current_aperture - The current aperture (f-stop) setting
 * 
 * Returns:
 *   The next lower aperture value
 */
float decrement_aperture(float current_aperture);

/**
 * Find the nearest standard ISO index
 * 
 * This function finds which standard ISO value is closest to
 * the given ISO setting.
 * 
 * Parameters:
 *   iso - The ISO value to find
 * 
 * Returns:
 *   The index of the nearest standard ISO value in the iso_values array
 */
int find_nearest_iso_index(uint16_t iso);

/**
 * Find the nearest standard aperture index
 * 
 * This function finds which standard aperture value is closest to
 * the given aperture setting.
 * 
 * Parameters:
 *   aperture - The aperture value to find
 * 
 * Returns:
 *   The index of the nearest standard aperture value in the aperture_values array
 */
int find_nearest_aperture_index(float aperture);

/**
 * Find the nearest standard shutter speed index
 * 
 * This function finds which standard shutter speed value is closest to
 * the given shutter speed.
 * 
 * Parameters:
 *   shutter_speed - The shutter speed value to find (in seconds)
 * 
 * Returns:
 *   The index of the nearest standard shutter speed in the shutter_speed_values array
 */
int find_nearest_shutter_index(float shutter_speed);

/**
 * Calculate the percentage distance to the next shutter speed
 * 
 * This helper function determines how far the calculated shutter speed
 * is between two standard values, used for the half-brightness LED indication.
 * 
 * Parameters:
 *   shutter_speed - The calculated shutter time in seconds
 *   current_index - The index of the nearest standard speed
 * 
 * Returns:
 *   Percentage (0.0 to 1.0) of how far we are to the next speed
 */
float get_shutter_interpolation(float shutter_speed, int current_index);

/* External array declarations */
extern const uint16_t iso_values[ISO_COUNT];
extern const float aperture_values[APERTURE_COUNT];
extern const uint16_t shutter_speed_values[SHUTTER_COUNT];
extern const float shutter_time_seconds[SHUTTER_COUNT];

#endif /* LIGHT_METER_H */