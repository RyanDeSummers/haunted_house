#pragma once

#include <stdbool.h>
#include <stdint.h>

// Initializes the vibration motor driver. Safe to call multiple times.
bool motor_handler_init(void);

// Stops any active pattern and shuts down resources.
void motor_handler_shutdown(void);

// Generic single-shot pulse helper (in milliseconds).
void motor_handler_pulse(uint32_t duration_ms);

// Pattern hooks tuned for gameplay events.
void motor_handler_trigger_guest_hit(void);
void motor_handler_trigger_actor_confirm(void);

