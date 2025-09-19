#include "actor_guest_test.h"
#include "movement_handler.h"
#include "IR_handler.h"
#include "SoundManager.hpp"
#include "espnow_handler.h"
#include "esp_log.h"
#include "esp_random.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "M5GFX.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include <string>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char* TAG = "ACTOR_GUEST_TEST";


// --- Soft status tone scheduler ---
static uint32_t tone_next_ms = 0;
static uint8_t  tone_phase   = 0; // for multi-beat patterns
static uint8_t  last_zone    = 0; // 0=green,1=yellow,2=red

static inline uint8_t radiation_zone(uint8_t r){
    return (r < 33) ? 0 : (r < 66) ? 1 : 2;
}

// External objects from main.cpp
extern M5GFX display;
extern led_strip_handle_t led_strip;
extern SoundManager speaker;

// ---- Non-blocking LED hit effect (red pulsing overlay) ----
static uint32_t ledfx_hit_next_ms = 0;
static uint8_t  ledfx_hit_phase   = 0; // 0=idle, 1=active

// Forward declarations
static inline bool ledfx_is_active();
static inline bool actorfx_is_active();

static void ledfx_trigger_hit() {
    ledfx_hit_phase   = 1;
    ledfx_hit_next_ms = (esp_timer_get_time() / 1000); // start now
}

// Helper: is any hit overlay active right now?
static inline bool ledfx_is_active() {
    if (ledfx_hit_phase == 0) return false;
    const uint32_t now = esp_timer_get_time() / 1000;
    // Guard the same window ledfx_tick() uses (~700 ms)
    return (int32_t)(now - ledfx_hit_next_ms) < 700;
}

static void ledfx_tick() {
    if (!led_strip || ledfx_hit_phase == 0) return;
    const uint32_t now = esp_timer_get_time() / 1000;
    
    // Create pulsing red effect during hit animation
    if (ledfx_hit_phase > 0) {
        // Fast pulsing red (2x the rate of red radiation zone for urgency)
        uint8_t pulse = (sin(now * 0.012) + 1) * 127; // Fast pulse: 0-255 (2x red zone rate)
        for (int i = 0; i < 10; ++i) {
            led_strip_set_pixel(led_strip, i, pulse, 0, 0);
        }
        led_strip_refresh(led_strip);
        
        // Check if hit animation duration is complete (~700ms to match alarm sound)
        if ((int32_t)(now - ledfx_hit_next_ms) >= 700) {
            ledfx_hit_phase = 0; // End hit effect
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
        }
    }
}

// Status tone scheduler - non-blocking modern sounds based on radiation level
static void status_tone_scheduler(uint8_t rad)
{
    const uint32_t now = esp_timer_get_time() / 1000;
    const uint8_t z = radiation_zone(rad);

    // Reset pattern when zone changes
    if (z != last_zone) { tone_phase = 0; tone_next_ms = now; last_zone = z; }

    if ((int32_t)(now - tone_next_ms) < 0) return; // wait

    switch (z) {
        case 0: { // GREEN: single soft thump every ~2s
            speaker.playGreenTone();
            tone_next_ms = now + 2000;
            tone_phase = 0;
        } break;

        case 1: { // YELLOW: double blip, then rest
            if (tone_phase == 0) {
                speaker.playYellowWarning();   // queues the pair inside SM
                tone_next_ms = now + 1200;    // rest until next pair
                tone_phase = 0;
            }
        } break;

        case 2: { // RED: low siren sweep, repeat faster
            speaker.playRedSiren();            // one 700ms sweep
            tone_next_ms = now + 900;         // short rest to loop
            tone_phase = 0;
        } break;
    }
}

// Button configuration
#define BUTTON_A_PIN 39
#define BUTTON_C_PIN 37
#define BUTTON_MIDDLE_PIN 38  // Middle button for start screen
#define BUTTON_HOLD_TIME_MS 5000

// Test configuration
#define RADIATION_PER_SIGNAL 10
#define GUEST_COOLDOWN_MS 5000
#define MIN_RADIATION 1
#define MAX_RADIATION 20

// Device modes
typedef enum {
    MODE_START_SCREEN = 0,
    MODE_GUEST = 1,
    MODE_ACTOR = 2
} device_mode_t;

// Test state
static device_mode_t current_mode = MODE_START_SCREEN;
static uint8_t current_radiation = 0;
static uint32_t movement_count = 0;
static uint32_t last_movement_time = 0;
static uint32_t last_ir_received_time = 0;
static uint32_t ir_signals_received = 0;
static uint32_t ir_signals_sent = 0;
static bool test_running = false;
static bool buttons_held = false;
static uint32_t button_hold_start_time = 0;
static uint8_t self_mac[6] = {0};  // Device's own MAC address
static uint8_t actor_radiation_amount = RADIATION_PER_SIGNAL;  // Current radiation amount for actor
static bool actor_signal_active = false;  // Track if actor is actively sending IR signals
static uint16_t actor_hit_count = 0;  // Track successful hits on guests


// Visual indicator functions
static void show_radiation_indicator(uint8_t radiation) {
    if (!led_strip) return;
    // If a hit overlay is active, skip baseline radiation LEDs this frame.
    if (ledfx_is_active() || actorfx_is_active()) return;
    
    uint32_t time_ms = esp_timer_get_time() / 1000;
    
    if (radiation < 33) {
        // Green - low radiation (solid)
        for (int i = 0; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, 0, 255, 0);
        }
        led_strip_refresh(led_strip);
        
    } else if (radiation < 66) {
        // Yellow - medium radiation (slow pulsing like menu)
        uint8_t pulse = (sin(time_ms * 0.003) + 1) * 127; // Slow pulse: 0-255
        for (int i = 0; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, pulse, pulse, 0);
        }
        led_strip_refresh(led_strip);
        
    } else {
        // Red - high radiation (fast pulsing, twice the rate of yellow)
        uint8_t pulse = (sin(time_ms * 0.006) + 1) * 127; // Fast pulse: 0-255 (2x yellow rate)
        for (int i = 0; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, pulse, 0, 0);
        }
        led_strip_refresh(led_strip);
    }
}

static void show_movement_indicator() {
    // No-op here; consider adding a non-blocking blue pulse similar to ledfx_* if desired.
}

static void show_mode_indicator() {
    if (!led_strip) return;
    // If a hit overlay is active, skip mode LED for this frame.
    if (ledfx_is_active() || actorfx_is_active()) return;
    
    if (current_mode == MODE_START_SCREEN) {
        // Slow pulsing yellow for start screen - light up all LEDs
        uint32_t time_ms = esp_timer_get_time() / 1000;
        uint8_t pulse = (sin(time_ms * 0.003) + 1) * 127; // Slow pulse: 0-255
        for (int i = 0; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, pulse, pulse, 0);
        }
    } else if (current_mode == MODE_ACTOR) {
        // Purple strobe for actor mode - fast pulsing effect
        uint32_t time_ms = esp_timer_get_time() / 1000;
        uint8_t strobe = (sin(time_ms * 0.02) + 1) * 127; // Fast strobe: 0-255
        for (int i = 0; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, strobe, 0, strobe);
        }
    }
    // Guest mode: no constant LED, only radiation-based colors
    led_strip_refresh(led_strip);
}


// -------- Non-blocking ACTOR hit-confirm overlay (green flashes) --------
static uint8_t  actorfx_phase      = 0;     // 0=idle, 1=on#1, 2=off#1, 3=on#2, 4=off#2
static uint32_t actorfx_next_ms    = 0;
static bool     actorfx_sound_arm  = false; // play sound at start from main loop

static inline bool actorfx_is_active() {
    return actorfx_phase != 0;
}

static void actorfx_trigger_hit_confirm() {
    actorfx_phase     = 1;
    actorfx_next_ms   = (esp_timer_get_time() / 1000);
    actorfx_sound_arm = true;  // let the main loop play the sound safely
    ESP_LOGI(TAG, "✅ Actor FX armed");
}

static void actorfx_tick() {
    if (!led_strip || actorfx_phase == 0) return;
    const uint32_t now = esp_timer_get_time() / 1000;
    if ((int32_t)(now - actorfx_next_ms) < 0) return;

    // Fire sound once, at overlay start (safe context)
    if (actorfx_sound_arm) {
        ESP_LOGI(TAG, "🔊 Actor confirm sound");
        actorfx_sound_arm = false;
        // Bright two-note confirm (queued to sound task; non-blocking)
        speaker.playActorHitConfirm();
    }

    switch (actorfx_phase) {
        case 1: // ON #1 (80 ms)
            for (int i = 0; i < 10; ++i) led_strip_set_pixel(led_strip, i, 0, 255, 0);
            led_strip_refresh(led_strip);
            actorfx_next_ms = now + 80;
            actorfx_phase   = 2;
            break;
        case 2: // OFF #1 (40 ms)
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            actorfx_next_ms = now + 40;
            actorfx_phase   = 3;
            break;
        case 3: // ON #2 (80 ms)
            for (int i = 0; i < 10; ++i) led_strip_set_pixel(led_strip, i, 0, 255, 0);
            led_strip_refresh(led_strip);
            actorfx_next_ms = now + 80;
            actorfx_phase   = 4;
            break;
        case 4: // OFF #2 and done
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            actorfx_phase = 0;
            break;
    }
}

// ESP-NOW callback for received packets
static void espnow_rx_callback(const espnow_packet_t* packet) {
    ESP_LOGI(TAG, "📡 ESP-NOW RX: type=0x%02X, from=%02X:%02X:%02X:%02X:%02X:%02X",
             packet->type,
             packet->sender_mac[0], packet->sender_mac[1], packet->sender_mac[2],
             packet->sender_mac[3], packet->sender_mac[4], packet->sender_mac[5]);
    
    // Filter out packets sent by self
    if (espnow_mac_equal(packet->sender_mac, self_mac)) {
        ESP_LOGI(TAG, "Ignoring ESP-NOW packet from self");
        return;
    }
    
    // Handle different packet types
    switch (packet->type) {
        case ESPNOW_IR_HIT_ACK:
            // Guest acknowledged our IR hit - show feedback in actor mode
            if (current_mode == MODE_ACTOR) {
                actor_hit_count++;  // Increment hit counter
                ESP_LOGI(TAG, "🎯 Received IR hit acknowledgment from guest! Total hits: %d", actor_hit_count);
                // DO NOT block / draw LEDs here; defer to main loop
                actorfx_trigger_hit_confirm();
            }
            break;
            
        case ESPNOW_HEARTBEAT:
            ESP_LOGD(TAG, "💓 Received heartbeat");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown ESP-NOW packet type: 0x%02X", packet->type);
            break;
    }
}


static void update_display() {
    if (!display.getPanel()) return;
    
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextDatum(textdatum_t::middle_center);
    
    // Start Screen
    if (current_mode == MODE_START_SCREEN) {
        // Black background
        display.fillScreen(TFT_BLACK);
        
        // Title
        display.setTextSize(3);
        display.setTextColor(TFT_RED, TFT_BLACK);
        display.drawString("BIOHAZARD", display.width()/2, 100);
        
        // Subtitle
        display.setTextSize(2);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        display.drawString("Press Middle Button", display.width()/2, 140);
        display.drawString("to Start", display.width()/2, 165);
        
        return;
    }
    
    // Game Screen
    if (current_mode == MODE_ACTOR) {
        // Actor Mode - Clean Design
        // Mode indicator
        display.setTextSize(2);
        display.setTextColor(TFT_MAGENTA, TFT_BLACK);
        display.drawString("ACTOR MODE", display.width()/2, 30);
        
        // Signal status indicator
        display.setTextSize(1);
        if (actor_signal_active) {
            display.setTextColor(TFT_GREEN, TFT_BLACK);
            display.drawString("SIGNAL ACTIVE", display.width()/2, 55);
        } else {
            display.setTextColor(TFT_RED, TFT_BLACK);
            display.drawString("SIGNAL INACTIVE", display.width()/2, 55);
        }
        
        // Large radiation amount in center
        display.setTextSize(4);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        char radiation_str[32];
        sprintf(radiation_str, "%d", actor_radiation_amount);
        display.drawString(radiation_str, display.width()/2, 120);
        
        // Hit counter below radiation
        display.setTextSize(2);
        display.setTextColor(TFT_CYAN, TFT_BLACK);
        char hit_str[32];
        sprintf(hit_str, "HITS: %d", actor_hit_count);
        display.drawString(hit_str, display.width()/2, 160);
        
        // Instructions
        display.setTextSize(1);
        display.setTextColor(TFT_YELLOW, TFT_BLACK);
        display.drawString("Point this device at guest device", display.width()/2, 180);
        display.drawString("A: decrease radiation, C: increase radiation", display.width()/2, 200);
        
        // Button instructions
        display.setTextColor(TFT_ORANGE, TFT_BLACK);
        display.drawString("Hold A 2s: Reset Hits", display.width()/2, 220);
        display.drawString("Hold A+C 5s: Switch Modes", display.width()/2, 235);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        
    } else {
        // Guest Mode - Clean Design
        // Title with color-coded radiation level
        display.setTextSize(2);
        if (current_radiation < 33) {
            display.setTextColor(TFT_GREEN, TFT_BLACK);
        } else if (current_radiation < 66) {
            display.setTextColor(TFT_YELLOW, TFT_BLACK);
        } else {
            display.setTextColor(TFT_RED, TFT_BLACK);
        }
        display.drawString("RADIATION LEVEL", display.width()/2, 50);
        
        // Radiation bar
        int bar_width = (current_radiation * (display.width() - 40)) / 100;
        if (current_radiation < 33) {
            display.fillRect(20, 80, bar_width, 20, TFT_GREEN);
        } else if (current_radiation < 66) {
            display.fillRect(20, 80, bar_width, 20, TFT_YELLOW);
        } else {
            display.fillRect(20, 80, bar_width, 20, TFT_RED);
        }
        display.drawRect(20, 80, display.width() - 40, 20, TFT_WHITE);
        
        display.setTextColor(TFT_WHITE, TFT_BLACK);
    }
}

// Button handling
static void check_buttons() {
    static uint32_t last_button_press = 0;
    static bool button_a_was_pressed = false;
    static bool button_c_was_pressed = false;
    static bool button_middle_was_pressed = false;
    
    bool button_a_pressed = (gpio_get_level((gpio_num_t)BUTTON_A_PIN) == 0);
    bool button_c_pressed = (gpio_get_level((gpio_num_t)BUTTON_C_PIN) == 0);
    bool button_middle_pressed = (gpio_get_level((gpio_num_t)BUTTON_MIDDLE_PIN) == 0);
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Handle middle button press (start screen)
    if (button_middle_pressed && !button_middle_was_pressed && current_time - last_button_press > 200) {
        if (current_mode == MODE_START_SCREEN) {
            // Start the game as Guest
            current_mode = MODE_GUEST;
            ESP_LOGI(TAG, "Game started! Mode: GUEST");
            update_display();
            show_mode_indicator();
        }
        last_button_press = current_time;
    }
    
    // Handle individual button presses (with debouncing)
    if (button_a_pressed && !button_a_was_pressed && current_time - last_button_press > 200) {
        if (current_mode == MODE_ACTOR) {
            // Decrease radiation amount
            if (actor_radiation_amount > MIN_RADIATION) {
                actor_radiation_amount--;
                ESP_LOGI(TAG, "Actor radiation decreased to %d%%", actor_radiation_amount);
                update_display(); // Only update when radiation changes
            }
        }
        last_button_press = current_time;
    }
    
    // Handle button A hold (reset hit counter) - only in actor mode
    static bool button_a_held = false;
    static uint32_t button_a_hold_start = 0;
    if (button_a_pressed && current_mode == MODE_ACTOR) {
        if (!button_a_held) {
            button_a_held = true;
            button_a_hold_start = current_time;
        } else if (current_time - button_a_hold_start > 2000) { // 2 second hold
            // Reset hit counter
            actor_hit_count = 0;
            ESP_LOGI(TAG, "Hit counter reset to 0");
            update_display();
            button_a_hold_start = current_time; // Reset timer to prevent spam
        }
    } else {
        button_a_held = false;
    }
    
    if (button_c_pressed && !button_c_was_pressed && current_time - last_button_press > 200) {
        if (current_mode == MODE_ACTOR) {
            // Increase radiation amount
            if (actor_radiation_amount < MAX_RADIATION) {
                actor_radiation_amount++;
                ESP_LOGI(TAG, "Actor radiation increased to %d%%", actor_radiation_amount);
                update_display(); // Only update when radiation changes
            }
        }
        last_button_press = current_time;
    }
    
    // Handle mode switching (both buttons held) - only in game modes
    if (button_a_pressed && button_c_pressed && current_mode != MODE_START_SCREEN) {
        if (!buttons_held) {
            buttons_held = true;
            button_hold_start_time = current_time;
            ESP_LOGI(TAG, "Buttons A+C pressed, starting hold timer...");
        } else {
            uint32_t hold_duration = current_time - button_hold_start_time;
            if (hold_duration >= BUTTON_HOLD_TIME_MS) {
                // Switch modes
                current_mode = (current_mode == MODE_GUEST) ? MODE_ACTOR : MODE_GUEST;
                ESP_LOGI(TAG, "Mode switched to: %s", (current_mode == MODE_ACTOR) ? "ACTOR" : "GUEST");
                
                // Reset button state
                buttons_held = false;
                button_hold_start_time = 0;
                
                // Update display and LED
                update_display();
                show_mode_indicator();
            }
        }
    } else {
        buttons_held = false;
        button_hold_start_time = 0;
    }
    
    // Update button state for next iteration
    button_a_was_pressed = button_a_pressed;
    button_c_was_pressed = button_c_pressed;
    button_middle_was_pressed = button_middle_pressed;
}

// Movement callback
static void movement_callback(bool significant_movement, float movement_magnitude, uint8_t reduction_amount) {
    // Only process movement in guest mode (not start screen or actor mode)
    if (significant_movement && current_mode == MODE_GUEST) {
        movement_count++;
        last_movement_time = esp_timer_get_time() / 1000;
        
        // Reduce radiation based on movement
        if (current_radiation >= reduction_amount) {
            current_radiation -= reduction_amount;
        } else {
            current_radiation = 0;
        }
        
        ESP_LOGI(TAG, "🏃 Movement detected! Magnitude=%.3f, Reduction=%d%%, New radiation=%d%%", 
                 movement_magnitude, reduction_amount, current_radiation);
        
        // Visual feedback
        show_movement_indicator();
        update_display();
    }
}

// IR callback for guest mode
static void ir_rx_callback(const ir_packet_t* packet, uint8_t rssi) {
    ESP_LOGI(TAG, "📡 IR callback triggered! Mode=%s, RSSI=%d", 
             (current_mode == MODE_GUEST) ? "GUEST" : "ACTOR", rssi);
    
    // Log sender MAC for debugging
    ESP_LOGI(TAG, "📡 Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             packet->sender_mac[0], packet->sender_mac[1], packet->sender_mac[2],
             packet->sender_mac[3], packet->sender_mac[4], packet->sender_mac[5]);
    
    // Filter out packets sent by self
    if (memcmp(packet->sender_mac, self_mac, 6) == 0) {
        ESP_LOGI(TAG, "Ignoring packet from self");
        return;
    }
    
    if (current_mode != MODE_GUEST) {
        ESP_LOGI(TAG, "Not in guest mode, ignoring IR signal");
        return;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Check cooldown
    if (current_time - last_ir_received_time < GUEST_COOLDOWN_MS) {
        ESP_LOGI(TAG, "IR signal received but in cooldown period (%lu ms remaining)", 
                 GUEST_COOLDOWN_MS - (current_time - last_ir_received_time));
        return;
    }
    
    // Extract radiation value from packet payload
    uint8_t radiation_to_add = RADIATION_PER_SIGNAL; // Default value
    
    // Try to extract from packet payload if available
    if (packet->payload_len > 0) {
        radiation_to_add = packet->payload[0];
        ESP_LOGI(TAG, "📦 IR packet payload: len=%d, data[0]=%d", packet->payload_len, radiation_to_add);
        
        // Only validate payload length (keep this safety check)
        if (packet->payload_len > 10) {
            ESP_LOGW(TAG, "⚠️ Suspicious payload length: %d, using default %d%%", packet->payload_len, RADIATION_PER_SIGNAL);
            radiation_to_add = RADIATION_PER_SIGNAL;
        }
    } else {
        ESP_LOGI(TAG, "📦 IR packet has no payload, using default %d%%", radiation_to_add);
    }
    
    // Apply radiation
    if (current_radiation + radiation_to_add <= 100) {
        current_radiation += radiation_to_add;
    } else {
        current_radiation = 100;
    }
    
    ir_signals_received++;
    last_ir_received_time = current_time;
    
    ESP_LOGI(TAG, "📡 IR signal received! Adding %d%% radiation, new total: %d%%", 
             radiation_to_add, current_radiation);
    
    // IR Hit Effect: play short alarm whoop (own task) + trigger LED flashes
    speaker.hit_alarm_whoop();
    ledfx_trigger_hit();
    
    // Send ESP-NOW acknowledgment back to actor
    if (espnow_is_initialized()) {
        ESP_LOGI(TAG, "ACK target(actor)=%02X:%02X:%02X:%02X:%02X:%02X  self(guest)=%02X:%02X:%02X:%02X:%02X:%02X",
                 packet->sender_mac[0], packet->sender_mac[1], packet->sender_mac[2],
                 packet->sender_mac[3], packet->sender_mac[4], packet->sender_mac[5],
                 self_mac[0], self_mac[1], self_mac[2], self_mac[3], self_mac[4], self_mac[5]);
        
        esp_err_t ack_ret = espnow_send_ir_hit_ack(packet->sender_mac, self_mac);
        if (ack_ret == ESP_OK) {
            ESP_LOGI(TAG, "📡 ESP-NOW acknowledgment sent to actor");
        } else {
            ESP_LOGW(TAG, "📡 Failed to send ESP-NOW acknowledgment: %s", esp_err_to_name(ack_ret));
        }
    } else {
        ESP_LOGW(TAG, "📡 ESP-NOW not initialized, cannot send acknowledgment");
    }
    
    // Visual feedback
    update_display();
    show_radiation_indicator(current_radiation);
}

// Actor mode task
static void actor_task_func(void* pvParameters) {
    ESP_LOGI(TAG, "Actor task started");
    
    while (test_running && current_mode == MODE_ACTOR) {
        // Set signal active status
        actor_signal_active = true;
        
        // Create IR packet with radiation value using proper function
        ir_packet_t packet;
        uint8_t radiation_data[] = {actor_radiation_amount};
        ir_create_packet(&packet, ir_signals_sent, radiation_data, sizeof(radiation_data));
        
        // Send IR signal
        if (ir_transmit_packet(&packet)) {
            ir_signals_sent++;
            ESP_LOGI(TAG, "📡 IR signal sent with %d%% radiation", actor_radiation_amount);
        } else {
            ESP_LOGW(TAG, "Failed to send IR signal");
            actor_signal_active = false;
        }
        
        // Update display to show signal status
        update_display();
        
        // Wait before next transmission
        vTaskDelay(pdMS_TO_TICKS(1000)); // Send every second
    }
    
    // Clear signal status when task ends
    actor_signal_active = false;
    ESP_LOGI(TAG, "Actor task ended");
    vTaskDelete(nullptr);
}

extern "C" void actor_guest_test_main(void) {
    ESP_LOGI(TAG, "🧪 ACTOR/GUEST TEST");
    ESP_LOGI(TAG, "==================");
    
    // Initialize display
    if (!display.begin()) {
        ESP_LOGE(TAG, "Display initialization failed!");
        return;
    } else {
        ESP_LOGI(TAG, "Display initialized");
        display.setRotation(1);
        
    }
    
    // Initialize buttons
    gpio_config_t button_config = {};
    button_config.pin_bit_mask = (1ULL << BUTTON_A_PIN) | (1ULL << BUTTON_C_PIN) | (1ULL << BUTTON_MIDDLE_PIN);
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Button GPIO config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize movement handler
    ESP_LOGI(TAG, "Initializing Movement Handler...");
    if (!movement_handler_init()) {
        ESP_LOGE(TAG, "❌ Failed to initialize movement handler!");
        return;
    }
    
    // Set up movement callback
    movement_handler_set_movement_callback(movement_callback);
    
    // Start movement detection
    ESP_LOGI(TAG, "Starting movement detection...");
    if (!movement_handler_start()) {
        ESP_LOGE(TAG, "❌ Failed to start movement detection!");
        return;
    }
    
    // Get device MAC address
    esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Self MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             self_mac[0], self_mac[1], self_mac[2],
             self_mac[3], self_mac[4], self_mac[5]);
    
    // Initialize IR handler
    ESP_LOGI(TAG, "Initializing IR Handler...");
    if (!ir_handler_init()) {
        ESP_LOGE(TAG, "❌ Failed to initialize IR handler!");
        return;
    }
    
    // Set up IR callbacks
    ir_set_rx_callback(ir_rx_callback);
    ir_set_reception_enabled(true);  // Enable IR reception!
    
    // Initialize ESP-NOW handler
    ESP_LOGI(TAG, "Initializing ESP-NOW Handler...");
    esp_err_t espnow_ret = espnow_handler_init();
    if (espnow_ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize ESP-NOW handler: %s", esp_err_to_name(espnow_ret));
        return;
    }
    
    // Set up ESP-NOW callback
    espnow_set_rx_callback(espnow_rx_callback);
    ESP_LOGI(TAG, "✅ ESP-NOW handler initialized and callback set");
    
    test_running = true;
    current_radiation = 0;
    movement_count = 0;
    last_movement_time = esp_timer_get_time() / 1000;
    last_ir_received_time = 0;
    ir_signals_received = 0;
    ir_signals_sent = 0;
    
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🎮 ACTOR/GUEST TEST INSTRUCTIONS:");
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "1. Device starts in GUEST mode");
    ESP_LOGI(TAG, "2. Hold A+C buttons for 5 seconds to switch to ACTOR mode");
    ESP_LOGI(TAG, "3. GUEST: Radiation increases by 2%% per second automatically");
    ESP_LOGI(TAG, "4. GUEST: Receives IR signals and adds radiation");
    ESP_LOGI(TAG, "5. GUEST: Move device to reduce radiation");
    ESP_LOGI(TAG, "6. ACTOR: Sends IR signals continuously");
    ESP_LOGI(TAG, "7. ACTOR: Press A to decrease radiation -1%%, C to increase +1%%");
    ESP_LOGI(TAG, "8. Guest has 2-second cooldown between IR signals");
    ESP_LOGI(TAG, "");
    
    // Initial display update
    update_display();
    show_mode_indicator();
    
    ESP_LOGI(TAG, "✅ Test started! Device is in GUEST mode.");
    
    uint32_t last_update = 0;
    TaskHandle_t actor_task = nullptr;
    
    for (;;) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // Check buttons
        check_buttons();
        
        // Increase radiation for guest mode (1 point per second) - only in game modes
        if (current_mode == MODE_GUEST) {
            static uint32_t last_radiation_tick = 0;
            if (current_time - last_radiation_tick > 1000) { // Every 1 second
                if (current_radiation < 100) {
                    current_radiation += 1; // Increase by 1% every second
                    ESP_LOGI(TAG, "📈 Guest radiation increased to %d%%", current_radiation);
                    last_radiation_tick = current_time;
                    
                    // Update display immediately
                    update_display();
                    show_radiation_indicator(current_radiation);
                }
            }
            
            // --- Drive the status tones without blocking (green/yellow/red) ---
            // status_tone_scheduler(current_radiation); // Disabled for now
        }
        
        // Update display every 500ms (skip for start screen and actor mode since they're static)
        if (current_time - last_update > 500 && current_mode == MODE_GUEST) {
            update_display();
            show_radiation_indicator(current_radiation);
            last_update = current_time;
        }
        
        // Update LED indicator for all modes (pulsing/strobe effects)
        if (current_mode == MODE_START_SCREEN || current_mode == MODE_ACTOR) {
            show_mode_indicator();
        } else if (current_mode == MODE_GUEST) {
            // Continuous LED updates for smooth pulsing in guest mode
            show_radiation_indicator(current_radiation);
        }
        
        // Draw overlays LAST so they can't be overwritten this frame.
        ledfx_tick();
        actorfx_tick();
        
        // Manage actor task
        if (current_mode == MODE_ACTOR && actor_task == nullptr) {
            // Start actor task
            BaseType_t task_ret = xTaskCreatePinnedToCore(
                actor_task_func,
                "actor_task",
                4096,
                nullptr,
                4,
                &actor_task,
                tskNO_AFFINITY
            );
            if (task_ret != pdPASS) {
                ESP_LOGE(TAG, "Failed to create actor task");
            }
        } else if (current_mode == MODE_GUEST && actor_task != nullptr) {
            // Stop actor task
            vTaskDelete(actor_task);
            actor_task = nullptr;
        }
        
        // Log status every 10 seconds
        if (current_time % 10000 < 500) {
            ESP_LOGI(TAG, "📊 Status: Mode=%s, Radiation=%d%%, Movements=%lu, IR_RX=%lu, IR_TX=%lu", 
                     (current_mode == MODE_ACTOR) ? "ACTOR" : "GUEST",
                     current_radiation, movement_count, ir_signals_received, ir_signals_sent);
        }
        
        vTaskDelay(pdMS_TO_TICKS(16));  // ~60 Hz UI/LED cadence
    }
}

