/**
 * Light Meter Calculations Implementation
 * 
 * This file contains the implementation of functions for
 * performing light meter calculations.
 * 
 * Updated to fix the shutter speed index calculation logic.
 * 
 * IMPROVED: Made shutter speed calculations clearer by using explicit
 * time values in seconds throughout, with better documentation of the
 * dual interpretation of the display values.
 */

 #include <math.h>
 #include "light_meter.h"
 
 // Arrays of supported values - now defined only in light_meter.c
 // ISO values (light sensitivity) from lowest to highest - removed 6400
 const uint16_t iso_values[ISO_COUNT] = {25, 50, 100, 125, 160, 200, 250, 400, 500, 800, 1600, 3200};
 
 // Aperture values (f-stops) from lowest to highest - removed f/64
 const float aperture_values[APERTURE_COUNT] = {1.0, 1.4, 2.0, 2.8, 4.0, 5.6, 8.0, 11.0, 16.0, 22.0, 32.0, 45.0};
 
 // Shutter speed display values
 // These numbers serve dual purpose:
 // - For speeds < 1 second: they represent the DENOMINATOR of 1/n seconds
 // - For speeds ≥ 1 second: they represent WHOLE seconds
 // This dual interpretation is handled by the display logic
 const uint16_t shutter_speed_values[SHUTTER_COUNT] = {1, 2, 4, 8, 15, 30, 60, 125, 250, 500, 1000, 2000};
 
 // IMPROVED: Array of actual shutter times in seconds for clearer calculations
 // This array contains the actual time values that correspond to the display values
 const float shutter_time_seconds[SHUTTER_COUNT] = {
     1.0,      // "1"    = 1 second (or 1/1 second)
     0.5,      // "2"    = 1/2 second
     0.25,     // "4"    = 1/4 second
     0.125,    // "8"    = 1/8 second
     0.0667,   // "15"   = 1/15 second (approximately)
     0.0333,   // "30"   = 1/30 second (approximately)
     0.0167,   // "60"   = 1/60 second (approximately)
     0.008,    // "125"  = 1/125 second
     0.004,    // "250"  = 1/250 second
     0.002,    // "500"  = 1/500 second
     0.001,    // "1000" = 1/1000 second
     0.0005    // "2000" = 1/2000 second
 };
 
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
  * IMPROVED: This function now uses the explicit time array for clearer logic.
  * It always works with actual time values in seconds, making the comparison
  * straightforward and less error-prone.
  * 
  * @param shutter_speed The calculated shutter time in seconds
  * @return The index of the closest standard shutter speed
  */
 int find_nearest_shutter_index(float shutter_speed) {
     int nearest_index = 0;
     float min_difference = fabsf(shutter_time_seconds[0] - shutter_speed);
     
     // Compare with each standard shutter time
     for (int i = 1; i < SHUTTER_COUNT; i++) {
         float difference = fabsf(shutter_time_seconds[i] - shutter_speed);
         
         if (difference < min_difference) {
             min_difference = difference;
             nearest_index = i;
         } else {
             // Since the array is sorted from slowest to fastest,
             // once differences start increasing, we've passed the minimum
             break;
         }
     }
     
     return nearest_index;
 }
 
 /**
  * Calculate the percentage distance to the next shutter speed
  * 
  * This helper function determines how far the calculated shutter speed
  * is between two standard values, used for the half-brightness LED indication.
  * 
  * @param shutter_speed The calculated shutter time in seconds
  * @param current_index The index of the nearest standard speed
  * @return Percentage (0.0 to 1.0) of how far we are to the next speed
  */
 float get_shutter_interpolation(float shutter_speed, int current_index) {
     // Can't interpolate if we're at the last value
     if (current_index >= SHUTTER_COUNT - 1) {
         return 0.0;
     }
     
     // Get the current and next standard times
     float current_time = shutter_time_seconds[current_index];
     float next_time = shutter_time_seconds[current_index + 1];
     
     // Calculate the percentage
     // Note: next_time < current_time because array goes from slow to fast
     float range = current_time - next_time;
     float distance = current_time - shutter_speed;
     
     // Ensure we don't go negative or over 1.0
     float percentage = distance / range;
     if (percentage < 0.0) percentage = 0.0;
     if (percentage > 1.0) percentage = 1.0;
     
     return percentage;
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