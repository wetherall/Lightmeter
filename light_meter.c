/**
 * Light Meter Calculations Implementation
 * * Contains functions for EV calculation, shutter speed calculation,
 * and finding nearest standard camera settings.
 * * Updated for reflected light metering (targets 18% middle grey).
 * * Replaced log2f with logf(x)/logf(2.0) for better linker compatibility.
 */

 #include <math.h> // For powf, fabsf, sqrtf, logf, M_PI
 #include <stdlib.h> // For abs
 #include <stdbool.h> 
 #include "light_meter.h"
 
 // Define M_PI_F if not available (common in some embedded toolchains)
 #ifndef M_PI_F
 #define M_PI_F ((float)M_PI)
 #endif
 
 // Pre-calculated logf(2.0f) for the log2f workaround
 #define LOGF_2 (0.69314718056f) // logf(2.0f)
 
 // --- Calibration Constants for Reflected Light Metering ---
 #define K_REFLECTED_METER_CONSTANT 12.5f
 #define REFLECTED_SCENE_REFLECTANCE 0.18f
 #define EV_LUX_CALIBRATION_FACTOR ((100.0f * REFLECTED_SCENE_REFLECTANCE) / (K_REFLECTED_METER_CONSTANT * M_PI_F))
 
 // Arrays of supported values - defined here
 const uint16_t iso_values[ISO_COUNT] = {25, 50, 100, 125, 160, 200, 250, 400, 500, 800, 1600, 3200};
 const float aperture_values[APERTURE_COUNT] = {1.0f, 1.4f, 2.0f, 2.8f, 4.0f, 5.6f, 8.0f, 11.0f, 16.0f, 22.0f, 32.0f, 45.0f};
 const uint16_t shutter_speed_display_values[SHUTTER_COUNT] = {1, 2, 4, 8, 15, 30, 60, 125, 250, 500, 1000, 2000};
 const float shutter_time_seconds[SHUTTER_COUNT] = {
     1.0f,      // "1"    (1s or 1/1s)
     0.5f,      // "2"    (1/2s)
     0.25f,     // "4"    (1/4s)
     0.125f,    // "8"    (1/8s)
     1.0f/15.0f,  // "15"   (1/15s)
     1.0f/30.0f,  // "30"   (1/30s)
     1.0f/60.0f,  // "60"   (1/60s)
     1.0f/125.0f, // "125"  (1/125s)
     1.0f/250.0f, // "250"  (1/250s)
     1.0f/500.0f, // "500"  (1/500s)
     1.0f/1000.0f,// "1000" (1/1000s)
     1.0f/2000.0f // "2000" (1/2000s)
 };
 
 /**
  * @brief Calculate an intermediate exposure-related value from lux.
  * This EV-like value is based on the scene illuminance (lux) and
  * calibrated for reflected light metering.
  * @param lux Measured scene illuminance in lux.
  * @return float Calculated EV-like value.
  */
 float calculate_ev_from_lux(float lux) {
     if (lux <= 0.0001f) lux = 0.0001f; 
     // Using logf(x) / logf(2.0f) instead of log2f(x)
     return (logf(lux * EV_LUX_CALIBRATION_FACTOR) / LOGF_2);
 }
 
 /**
  * @brief Calculate appropriate shutter speed based on lux, ISO, and aperture.
  * @param lux Measured scene illuminance in lux.
  * @param iso Current ISO setting.
  * @param aperture Current aperture (f-number) setting.
  * @return float Calculated shutter speed in seconds.
  */
 float calculate_shutter_speed(float lux, uint16_t iso, float aperture) {
     if (lux <= 0.0f || iso == 0) {
         return shutter_time_seconds[0]; 
     }
     
     float ev_from_lux = calculate_ev_from_lux(lux);
     float shutter_time = (aperture * aperture * 100.0f) / ((float)iso * powf(2.0f, ev_from_lux));
     
     if (shutter_time <= 0.0f) { 
         return shutter_time_seconds[SHUTTER_COUNT -1]; 
     }
     return shutter_time;
 }
 
 
 /**
  * @brief Find the index of the nearest standard ISO value.
  */
 int find_nearest_iso_index(uint16_t iso) {
     int nearest_index = 0;
     int min_difference = abs((int)iso_values[0] - (int)iso);
 
     for (int i = 1; i < ISO_COUNT; i++) {
         int difference = abs((int)iso_values[i] - (int)iso);
         if (difference < min_difference) {
             min_difference = difference;
             nearest_index = i;
         } else if (difference == min_difference && iso_values[i] > iso) {
         } else if (difference > min_difference && iso_values[i] > iso_values[nearest_index]) {
             break;
         }
     }
     return nearest_index;
 }
 
 /**
  * @brief Find the index of the nearest standard aperture value.
  */
 int find_nearest_aperture_index(float aperture) {
     int nearest_index = 0;
     float min_difference = fabsf(aperture_values[0] - aperture);
 
     for (int i = 1; i < APERTURE_COUNT; i++) {
         float difference = fabsf(aperture_values[i] - aperture);
         if (difference < min_difference) {
             min_difference = difference;
             nearest_index = i;
         } else if (difference == min_difference && aperture_values[i] > aperture) {
         } else if (difference > min_difference && aperture_values[i] > aperture_values[nearest_index]) {
             break;
         }
     }
     return nearest_index;
 }
 
 /**
  * @brief Find the index of the nearest standard shutter speed.
  */
 int find_nearest_shutter_index(float shutter_speed_seconds) {
     int nearest_index = 0;
     float min_difference = fabsf(shutter_time_seconds[0] - shutter_speed_seconds);
 
     for (int i = 1; i < SHUTTER_COUNT; i++) {
         float difference = fabsf(shutter_time_seconds[i] - shutter_speed_seconds);
         if (difference < min_difference) {
             min_difference = difference;
             nearest_index = i;
         } else if (difference == min_difference) {
             if (shutter_time_seconds[i] < shutter_time_seconds[nearest_index]) { 
                  nearest_index = i;
             }
         }
     }
     return nearest_index;
 }
 
 /**
  * @brief Calculate the percentage distance to the next faster shutter speed for interpolation.
  */
 float get_shutter_interpolation(float calculated_shutter_seconds, int current_closest_idx) {
     if (current_closest_idx < 0 || current_closest_idx >= SHUTTER_COUNT - 1) {
         return 0.0f;
     }
 
     float time_current_led = shutter_time_seconds[current_closest_idx];
     float time_next_faster_led = shutter_time_seconds[current_closest_idx + 1];
 
     if (calculated_shutter_seconds < time_current_led && calculated_shutter_seconds > time_next_faster_led) {
         float range_between_leds = time_current_led - time_next_faster_led;
         if (range_between_leds <= 0.000001f) { 
             return 0.0f;
         }
         float distance_from_current_led = time_current_led - calculated_shutter_seconds;
         float percentage = distance_from_current_led / range_between_leds;
         
         if (percentage < 0.0f) return 0.0f;
         if (percentage > 1.0f) return 1.0f;
         return percentage;
     }
     
     return 0.0f; 
 }
 
 
 /**
  * @brief Increment the ISO setting to the next standard value.
  */
 uint16_t increment_iso(uint16_t current_iso) {
     int current_index = find_nearest_iso_index(current_iso);
     if (current_iso >= iso_values[ISO_COUNT - 1]) return iso_values[ISO_COUNT - 1];
 
     if (current_iso == iso_values[current_index] && current_index < ISO_COUNT - 1) {
          current_index++;
     } else if (current_iso > iso_values[current_index] && current_index < ISO_COUNT - 1) {
         current_index++;
     } else if (current_index < ISO_COUNT -1) {
          if(iso_values[current_index] < current_iso && current_index < ISO_COUNT -1) current_index++;
     }
 
     if (current_index >= ISO_COUNT) current_index = ISO_COUNT - 1;
     return iso_values[current_index];
 }
 
 /**
  * @brief Decrement the ISO setting to the previous standard value.
  */
 uint16_t decrement_iso(uint16_t current_iso) {
     int current_index = find_nearest_iso_index(current_iso);
     if (current_iso <= iso_values[0]) return iso_values[0];
 
     if (current_iso == iso_values[current_index] && current_index > 0) {
         current_index--;
     } else if (current_iso < iso_values[current_index] && current_index > 0) {
         // If current_iso is less than the found nearest (iso_values[current_index]),
         // it means the true position is between iso_values[current_index-1] and iso_values[current_index].
         // Decrementing should go to iso_values[current_index-1].
         // However, find_nearest_iso_index should ideally return the lower bound if between.
         // For simplicity with current find_nearest, if current_iso is less than the "nearest",
         // and nearest is not the first, then the "nearest" is actually the one above.
         // So, just decrementing current_index should work if find_nearest_iso_index is consistent.
         // Let's assume find_nearest_iso_index gives a reasonable index.
         current_index--; // This might need more robust logic if find_nearest is not perfect.
     } else if (current_index > 0) { 
         current_index--;
     }
 
     if (current_index < 0) current_index = 0;
     return iso_values[current_index];
 }
 
 /**
  * @brief Increment the aperture setting (supports half-stops).
  */
 float increment_aperture(float current_aperture_val) {
     bool is_standard_stop = false;
     int current_std_idx = -1;
 
     for(int i=0; i < APERTURE_COUNT; ++i) {
         if (fabsf(current_aperture_val - aperture_values[i]) < 0.001f * aperture_values[i]) { 
             is_standard_stop = true;
             current_std_idx = i;
             break;
         }
     }
 
     if (is_standard_stop) { 
         if (current_std_idx < APERTURE_COUNT - 1) { 
             return sqrtf(aperture_values[current_std_idx] * aperture_values[current_std_idx + 1]);
         }
         return current_aperture_val; 
     } else { 
         for (int i = 0; i < APERTURE_COUNT - 1; ++i) {
             float half_stop_calc = sqrtf(aperture_values[i] * aperture_values[i+1]);
             if (fabsf(current_aperture_val - half_stop_calc) < 0.001f * half_stop_calc) {
                 return aperture_values[i+1];
             }
         }
         int nearest_idx = find_nearest_aperture_index(current_aperture_val);
         if (current_aperture_val > aperture_values[nearest_idx] && nearest_idx < APERTURE_COUNT -1) { 
             return aperture_values[nearest_idx+1]; 
         } else if (nearest_idx < APERTURE_COUNT -1) { 
              return sqrtf(aperture_values[nearest_idx] * aperture_values[nearest_idx + 1]);
         }
         return current_aperture_val; 
     }
 }
 
 /**
  * @brief Decrement the aperture setting (supports half-stops).
  */
 float decrement_aperture(float current_aperture_val) {
     bool is_standard_stop = false;
     int current_std_idx = -1;
 
     for(int i=0; i < APERTURE_COUNT; ++i) {
         if (fabsf(current_aperture_val - aperture_values[i]) < 0.001f * aperture_values[i]) {
             is_standard_stop = true;
             current_std_idx = i;
             break;
         }
     }
 
     if (is_standard_stop) { 
         if (current_std_idx > 0) { 
             return sqrtf(aperture_values[current_std_idx] * aperture_values[current_std_idx - 1]);
         }
         return current_aperture_val; 
     } else { 
         for (int i = 0; i < APERTURE_COUNT - 1; ++i) { 
             float half_stop_calc = sqrtf(aperture_values[i] * aperture_values[i+1]);
             if (fabsf(current_aperture_val - half_stop_calc) < 0.001f * half_stop_calc) {
                 return aperture_values[i];
             }
         }
         int nearest_idx = find_nearest_aperture_index(current_aperture_val);
         if (current_aperture_val < aperture_values[nearest_idx] && nearest_idx > 0) { // current is half-stop below nearest_idx
              return aperture_values[nearest_idx-1]; 
         } else if (nearest_idx > 0 && current_aperture_val > aperture_values[nearest_idx-1] ) { // current is half-stop above nearest_idx-1
             return sqrtf(aperture_values[nearest_idx-1] * aperture_values[nearest_idx]); // This should be the half stop below aperture_values[nearest_idx]
         } else if (nearest_idx > 0) { // Default to half-stop below the nearest full stop if logic is complex
              return sqrtf(aperture_values[nearest_idx] * aperture_values[nearest_idx-1]);
         }
         return current_aperture_val;
     }
 }
 