#include "actor_guest_test.h"
#include "movement_handler.h"
#include "IR_handler.h"
#include "esp_log.h"
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

static const char* TAG = "ACTOR_GUEST_TEST";

// External objects from main.cpp
extern M5GFX display;
extern led_strip_handle_t led_strip;

// Button configuration
#define BUTTON_A_PIN 39
#define BUTTON_C_PIN 37
#define BUTTON_MIDDLE_PIN 38  // Middle button for start screen
#define BUTTON_HOLD_TIME_MS 5000

// Test configuration
#define RADIATION_PER_SIGNAL 10
#define GUEST_COOLDOWN_MS 2000
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


// Visual indicator functions
static void show_radiation_indicator(uint8_t radiation) {
    if (!led_strip) return;
    
    // Color based on radiation level
    uint8_t red = 0, green = 0, blue = 0;
    
    if (radiation < 33) {
        // Green - low radiation
        green = 255;
    } else if (radiation < 66) {
        // Yellow - medium radiation
        red = 255;
        green = 255;
    } else {
        // Red - high radiation
        red = 255;
    }
    
    // Light up all LEDs in the strip (10 total: 5 on each side)
    for (int i = 0; i < 10; i++) {
        led_strip_set_pixel(led_strip, i, red, green, blue);
    }
    led_strip_refresh(led_strip);
}

static void show_movement_indicator() {
    if (!led_strip) return;
    
    // Blue flash for movement - light up all LEDs (10 total: 5 on each side)
    for (int i = 0; i < 10; i++) {
        led_strip_set_pixel(led_strip, i, 0, 0, 255);
    }
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
}

static void show_mode_indicator() {
    if (!led_strip) return;
    
    if (current_mode == MODE_START_SCREEN) {
        // Slow pulsing purple for start screen - light up all LEDs
        uint32_t time_ms = esp_timer_get_time() / 1000;
        uint8_t pulse = (sin(time_ms * 0.003) + 1) * 127; // Slow pulse: 0-255
        for (int i = 0; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, pulse, 0, pulse);
        }
    } else if (current_mode == MODE_ACTOR) {
        // Purple for actor mode - light up all LEDs
        for (int i = 0; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, 255, 0, 255);
        }
    }
    // Guest mode: no constant LED, only radiation-based colors
    led_strip_refresh(led_strip);
}


static void update_display() {
    if (!display.getPanel()) return;
    
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextDatum(textdatum_t::middle_center);
    
    // Start Screen
    if (current_mode == MODE_START_SCREEN) {
        // Title
        display.setTextSize(3);
        display.setTextColor(TFT_RED, TFT_BLACK);
        display.drawString("HAUNTED HOUSE", display.width()/2, 80);
        
        // Subtitle
        display.setTextSize(2);
        display.setTextColor(TFT_ORANGE, TFT_BLACK);
        display.drawString("Press Middle Button", display.width()/2, 120);
        display.drawString("to Start", display.width()/2, 145);
        
        // Instructions
        display.setTextSize(1);
        display.setTextColor(TFT_YELLOW, TFT_BLACK);
        display.drawString("Radiation levels will start to increase", display.width()/2, 180);
        display.drawString("Move and shake the device to reduce radiation", display.width()/2, 200);
        
        return;
    }
    
    // Game Screen
    if (current_mode == MODE_ACTOR) {
        // Actor Mode - Clean Design
        // Mode indicator
        display.setTextSize(2);
        display.setTextColor(TFT_MAGENTA, TFT_BLACK);
        display.drawString("ACTOR MODE", display.width()/2, 30);
        
        // Large radiation amount in center
        display.setTextSize(4);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        char radiation_str[32];
        sprintf(radiation_str, "%d", actor_radiation_amount);
        display.drawString(radiation_str, display.width()/2, 120);
        
        // Instructions
        display.setTextSize(1);
        display.setTextColor(TFT_YELLOW, TFT_BLACK);
        display.drawString("Point this device at guest device", display.width()/2, 180);
        display.drawString("A: decrease radiation, C: increase radiation", display.width()/2, 200);
        
        // Button instructions
        display.setTextColor(TFT_ORANGE, TFT_BLACK);
        display.drawString("Hold A+C for 5s to switch modes", display.width()/2, 220);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        
    } else {
        // Guest Mode - Clean Design
        // Title
        display.setTextSize(2);
        display.setTextColor(TFT_CYAN, TFT_BLACK);
        display.drawString("RADIATION LEVEL", display.width()/2, 30);
        
        // Radiation level
        display.setTextSize(3);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        char radiation_str[32];
        sprintf(radiation_str, "%d", current_radiation);
        display.drawString(radiation_str, display.width()/2, 75);
        
        // Radiation bar
        int bar_width = (current_radiation * (display.width() - 40)) / 100;
        if (current_radiation < 33) {
            display.fillRect(20, 105, bar_width, 20, TFT_GREEN);
        } else if (current_radiation < 66) {
            display.fillRect(20, 105, bar_width, 20, TFT_YELLOW);
        } else {
            display.fillRect(20, 105, bar_width, 20, TFT_RED);
        }
        display.drawRect(20, 105, display.width() - 40, 20, TFT_WHITE);
        
        // Instructions
        display.setTextSize(1);
        display.setTextColor(TFT_YELLOW, TFT_BLACK);
        display.drawString("Move device to reduce radiation", display.width()/2, 180);
        display.drawString("Point actor device at this device", display.width()/2, 200);
        
        // Test status
        if (test_running) {
            display.setTextColor(TFT_GREEN, TFT_BLACK);
            display.drawString("TEST RUNNING", display.width()/2, 220);
        } else {
            display.setTextColor(TFT_RED, TFT_BLACK);
            display.drawString("TEST STOPPED", display.width()/2, 220);
        }
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
    // Only process movement in game modes (not start screen)
    if (significant_movement && current_mode != MODE_START_SCREEN) {
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
    
    // Visual feedback
    update_display();
    show_radiation_indicator(current_radiation);
}

// Actor mode task
static void actor_task_func(void* pvParameters) {
    ESP_LOGI(TAG, "Actor task started");
    
    while (test_running && current_mode == MODE_ACTOR) {
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
        }
        
        // Wait before next transmission
        vTaskDelay(pdMS_TO_TICKS(1000)); // Send every second
    }
    
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
        }
        
        // Update display every 500ms (skip for start screen and actor mode since they're static)
        if (current_time - last_update > 500 && current_mode == MODE_GUEST) {
            update_display();
            show_radiation_indicator(current_radiation);
            last_update = current_time;
        }
        
        // Update LED indicator for start screen (pulsing effect)
        if (current_mode == MODE_START_SCREEN) {
            show_mode_indicator();
        }
        
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
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
