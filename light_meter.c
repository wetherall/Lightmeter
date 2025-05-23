/**
 * Light Meter Calculations Implementation
 * 
 * This file contains the implementation of functions for
 * performing light meter calculations.
 * 
 * Updated to fix the shutter speed index calculation logic.
 */

#include <math.h>
#include "light_meter.h"

// Arrays of supported values - now defined only in light_meter.c
// ISO values (light sensitivity) from lowest to highest - removed 6400
const uint16_t iso_values[ISO_COUNT] = {25, 50, 100, 125, 160, 200, 250, 400, 500, 800, 1600, 3200};

// Aperture values (f-stops) from lowest to highest - removed f/64
const float aperture_values[APERTURE_COUNT] = {1.0, 1.4, 2.0, 2.8, 4.0, 5.6, 8.0, 11.0, 16.0, 22.0, 32.0, 45.0};

// Shutter speed values in fractions of a second (displayed as 1/n) - removed 4000
const uint16_t shutter_speed_values[SHUTTER_COUNT] = {1, 2, 4, 8, 15, 30, 60, 125, 250, 500, 1000, 2000};

/**
 * Calculate the Exposure Value (EV) from lux
 * 
 * The formula is: EV = log2(lux * 0.125)
 * This is based on the standard relationship between lux and EV
 */
float calculate_ev(float lux) {
    // Apply the conversion factor (calibration factor)
    // The 0.125 factor is based on standard conversion and may need calibration
    float calibrated_lux = lux * 0.125;
    
    // Calculate EV
    // log2(x) = log(x) / log(2)
    return log10f(calibrated_lux) / log10f(2.0);
}

/**
 * Calculate the appropriate shutter speed based on lux, ISO, and aperture
 * 
 * Using the Sunny 16 rule and related calculations:
 * EV = log2(N² / t) - log2(ISO / 100)
 * where N is the aperture, t is the shutter time in seconds
 * 
 * Rearranging to find t (shutter time):
 * t = (N² * 100) / (ISO * 2^EV)
 */
float calculate_shutter_speed(float lux, uint16_t iso, float aperture) {
    // Calculate the Exposure Value
    float ev = calculate_ev(lux);
    
    // Calculate the appropriate shutter speed
    float shutter_time = (aperture * aperture * 100.0) / (iso * powf(2.0, ev));
    
    // Return the shutter time in seconds
    return shutter_time;
}

/**
 * Find the nearest ISO index
 */
int find_nearest_iso_index(uint16_t iso) {
    int nearest_index = 0;
    int min_difference = abs((int)iso_values[0] - (int)iso);
    
    for (int i = 1; i < ISO_COUNT; i++) {
        int difference = abs((int)iso_values[i] - (int)iso);
        if (difference < min_difference) {
            min_difference = difference;
            nearest_index = i;
        }
    }
    
    return nearest_index;
}

/**
 * Find the nearest aperture index
 */
int find_nearest_aperture_index(float aperture) {
    int nearest_index = 0;
    float min_difference = fabsf(aperture_values[0] - aperture);
    
    for (int i = 1; i < APERTURE_COUNT; i++) {
        float difference = fabsf(aperture_values[i] - aperture);
        if (difference < min_difference) {
            min_difference = difference;
            nearest_index = i;
        }
    }
    
    return nearest_index;
}

/**
 * Find the nearest shutter speed index
 * 
 * This function has been fixed to properly handle the comparison between
 * the calculated shutter speed (in seconds) and our standard values.
 * 
 * Our shutter_speed_values array contains denominators for fractions:
 * - For speeds < 1 second: the value represents 1/n seconds (e.g., 125 = 1/125s)
 * - For speeds ≥ 1 second: we treat the same values as whole seconds
 * 
 * The logic has been corrected to properly compare the actual time values.
 */
int find_nearest_shutter_index(float shutter_speed) {
    int nearest_index = 0;
    float min_difference = 1000000.0;
    
    // For each standard shutter speed value
    for (int i = 0; i < SHUTTER_COUNT; i++) {
        float standard_time;
        float difference;
        
        if (shutter_speed < 1.0) {
            // For fractional seconds, our array values represent denominators
            // So we need to convert them to actual time values (1/n)
            standard_time = 1.0 / (float)shutter_speed_values[i];
            difference = fabsf(standard_time - shutter_speed);
        } else {
            // For whole seconds, treat the array values as whole seconds
            standard_time = (float)shutter_speed_values[i];
            difference = fabsf(standard_time - shutter_speed);
        }
        
        // Check if this is the closest match so far
        if (difference < min_difference) {
            min_difference = difference;
            nearest_index = i;
        }
    }
    
    return nearest_index;
}

/**
 * Increment the ISO value
 */
uint16_t increment_iso(uint16_t current_iso) {
    // Find the current index
    int current_index = find_nearest_iso_index(current_iso);
    
    // Increment index, with bounds checking
    if (current_index < ISO_COUNT - 1) {
        current_index++;
    }
    
    // Return the new ISO value
    return iso_values[current_index];
}

/**
 * Decrement the ISO value
 */
uint16_t decrement_iso(uint16_t current_iso) {
    // Find the current index
    int current_index = find_nearest_iso_index(current_iso);
    
    // Decrement index, with bounds checking
    if (current_index > 0) {
        current_index--;
    }
    
    // Return the new ISO value
    return iso_values[current_index];
}

/**
 * Increment the aperture value
 * This implementation supports half-stops between standard aperture values
 */
float increment_aperture(float current_aperture) {
    // Find the current index
    int current_index = find_nearest_aperture_index(current_aperture);
    
    // Check if we're exactly at a standard aperture value
    float difference = fabsf(current_aperture - aperture_values[current_index]);
    
    if (difference < 0.01) {
        // We're at a standard value, go to the half-stop
        if (current_index < APERTURE_COUNT - 1) {
            // Calculate half-stop value (geometric mean between adjacent stops)
            float half_stop = sqrtf(aperture_values[current_index] * aperture_values[current_index + 1]);
            return half_stop;
        } else {
            // At the maximum, can't increment
            return current_aperture;
        }
    } else {
        // We're at a half-stop, go to the next standard value
        if (current_index < APERTURE_COUNT - 1) {
            return aperture_values[current_index + 1];
        } else {
            // At the maximum, can't increment
            return current_aperture;
        }
    }
}

/**
 * Decrement the aperture value
 * This implementation supports half-stops between standard aperture values
 */
float decrement_aperture(float current_aperture) {
    // Find the current index
    int current_index = find_nearest_aperture_index(current_aperture);
    
    // Check if we're exactly at a standard aperture value
    float difference = fabsf(current_aperture - aperture_values[current_index]);
    
    if (difference < 0.01) {
        // We're at a standard value, go to the half-stop below
        if (current_index > 0) {
            // Calculate half-stop value (geometric mean between adjacent stops)
            float half_stop = sqrtf(aperture_values[current_index] * aperture_values[current_index - 1]);
            return half_stop;
        } else {
            // At the minimum, can't decrement
            return current_aperture;
        }
    } else {
        // We're at a half-stop, go to the lower standard value
        return aperture_values[current_index];
    }
}