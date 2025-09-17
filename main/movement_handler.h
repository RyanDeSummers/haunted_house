#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Forward declarations
typedef struct {
    float x;
    float y;
    float z;
    float magnitude;
} accelerometer_data_t;

// Callback function types
typedef void (*movement_event_callback_t)(bool significant_movement, float movement_magnitude, uint8_t reduction_amount);
typedef void (*accelerometer_data_callback_t)(const accelerometer_data_t* accel_data);

// Public API functions
bool movement_handler_init(void);
bool movement_handler_start(void);
void movement_handler_stop(void);
void movement_handler_shutdown(void);

// Callback management
void movement_handler_set_movement_callback(movement_event_callback_t callback);
void movement_handler_set_accelerometer_callback(accelerometer_data_callback_t callback);

// Configuration
void movement_handler_set_threshold(float threshold);
float movement_handler_get_threshold(void);
void movement_handler_set_sample_rate(uint32_t sample_rate_ms);
uint32_t movement_handler_get_sample_rate(void);

// Status
bool movement_handler_is_accelerometer_working(void);
bool movement_handler_is_running(void);

// Data access
bool movement_handler_get_latest_data(accelerometer_data_t* data);

#ifdef __cplusplus
}
#endif

