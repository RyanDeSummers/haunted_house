#include "movement_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "math.h"

static const char* TAG = "MOVEMENT_HANDLER";

// MPU6886 I2C configuration
#define MPU6886_I2C_ADDR 0x68
#define MPU6886_I2C_PORT I2C_NUM_0
#define MPU6886_SDA_PIN 21
#define MPU6886_SCL_PIN 22
#define MPU6886_FREQ_HZ 400000

// MPU6886 register addresses
#define MPU6886_WHO_AM_I 0x75
#define MPU6886_PWR_MGMT_1 0x6B
#define MPU6886_ACCEL_XOUT_H 0x3B
#define MPU6886_ACCEL_XOUT_L 0x3C
#define MPU6886_ACCEL_YOUT_H 0x3D
#define MPU6886_ACCEL_YOUT_L 0x3E
#define MPU6886_ACCEL_ZOUT_H 0x3F
#define MPU6886_ACCEL_ZOUT_L 0x40

// Default configuration
#define DEFAULT_THRESHOLD 0.3f  // Lowered for natural movement detection
#define DEFAULT_SAMPLE_RATE_MS 50
#define MOVEMENT_COOLDOWN_MS 100  // Reduced cooldown for more responsive detection

// Natural movement detection parameters
#define MOVEMENT_HISTORY_SIZE 20  // Keep last 20 samples for pattern analysis
#define WALKING_THRESHOLD 0.15f   // Threshold for walking-like movement
#define CUMULATIVE_THRESHOLD 2.0f // Total movement needed for radiation reduction
#define MOVEMENT_DECAY_RATE 0.95f // How quickly cumulative movement decays

// Global state
static bool g_initialized = false;
static bool g_running = false;
static bool g_accelerometer_working = false;
static float g_threshold = DEFAULT_THRESHOLD;
static uint32_t g_sample_rate_ms = DEFAULT_SAMPLE_RATE_MS;
static TaskHandle_t g_detection_task = nullptr;

// Natural movement detection state
static float g_movement_history[MOVEMENT_HISTORY_SIZE] = {0};
static int g_history_index = 0;
static float g_cumulative_movement = 0.0f;
static uint32_t g_last_cumulative_reduction = 0;

// Callbacks
static movement_event_callback_t g_movement_callback = nullptr;
static accelerometer_data_callback_t g_accelerometer_callback = nullptr;

// Data storage
static accelerometer_data_t g_current_data = {0};
static accelerometer_data_t g_previous_data = {0};
static uint32_t g_last_movement_time = 0;

// Private function declarations
static bool init_i2c(void);
static bool init_mpu6886(void);
static bool read_accelerometer_data(accelerometer_data_t* data);
static void detection_task_func(void* pvParameters);
static float calculate_magnitude(float x, float y, float z);
static uint8_t calculate_reduction_amount(float magnitude);

// I2C helper functions
static esp_err_t mpu6886_write_reg(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6886_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MPU6886_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6886_read_reg(uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6886_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6886_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MPU6886_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static bool init_i2c(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = MPU6886_SDA_PIN;
    conf.scl_io_num = MPU6886_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = MPU6886_FREQ_HZ;
    
    esp_err_t ret = i2c_param_config(MPU6886_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(MPU6886_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    return true;
}

static bool init_mpu6886(void) {
    // Check WHO_AM_I register
    uint8_t who_am_i;
    if (mpu6886_read_reg(MPU6886_WHO_AM_I, &who_am_i, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return false;
    }
    
    ESP_LOGI(TAG, "MPU6886 WHO_AM_I: 0x%02X", who_am_i);
    
    // Wake up the device
    if (mpu6886_write_reg(MPU6886_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6886");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for device to wake up
    
    return true;
}

static bool read_accelerometer_data(accelerometer_data_t* data) {
    uint8_t raw_data[6];
    
    if (mpu6886_read_reg(MPU6886_ACCEL_XOUT_H, raw_data, 6) != ESP_OK) {
        return false;
    }
    
    // Convert raw data to acceleration values (assuming ±2g range)
    int16_t raw_x = (raw_data[0] << 8) | raw_data[1];
    int16_t raw_y = (raw_data[2] << 8) | raw_data[3];
    int16_t raw_z = (raw_data[4] << 8) | raw_data[5];
    
    // Convert to g values (assuming ±2g range, so divide by 16384)
    data->x = (float)raw_x / 16384.0f;
    data->y = (float)raw_y / 16384.0f;
    data->z = (float)raw_z / 16384.0f;
    
    // Calculate magnitude
    data->magnitude = calculate_magnitude(data->x, data->y, data->z);
    
    return true;
}

static float calculate_magnitude(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

static uint8_t calculate_reduction_amount(float magnitude) {
    // For now, all movements reduce radiation by 1%
    // TODO: Can be modified later to give different reductions based on movement type
    return 1;
    
    // Future implementation could be:
    // if (magnitude < 0.5f) return 1;
    // if (magnitude < 1.0f) return 2;
    // if (magnitude < 1.5f) return 3;
    // if (magnitude < 2.0f) return 4;
    // return 5;
}

// Detect walking-like rhythmic movement patterns
static bool detect_walking_pattern() {
    // Look for alternating high/low values in movement history
    int peaks = 0;
    int valleys = 0;
    
    for (int i = 1; i < MOVEMENT_HISTORY_SIZE - 1; i++) {
        float prev = g_movement_history[i - 1];
        float curr = g_movement_history[i];
        float next = g_movement_history[i + 1];
        
        // Detect peaks (local maxima)
        if (curr > prev && curr > next && curr > WALKING_THRESHOLD) {
            peaks++;
        }
        // Detect valleys (local minima)
        if (curr < prev && curr < next && curr < WALKING_THRESHOLD) {
            valleys++;
        }
    }
    
    // Walking pattern: alternating peaks and valleys
    return (peaks >= 2 && valleys >= 2 && abs(peaks - valleys) <= 1);
}

// Add movement to history and update cumulative tracking
static void update_movement_tracking(float movement_delta) {
    // Add to circular buffer
    g_movement_history[g_history_index] = movement_delta;
    g_history_index = (g_history_index + 1) % MOVEMENT_HISTORY_SIZE;
    
    // Update cumulative movement (with decay)
    g_cumulative_movement *= MOVEMENT_DECAY_RATE;
    g_cumulative_movement += movement_delta;
    
    // Cap cumulative movement to prevent overflow
    if (g_cumulative_movement > 10.0f) {
        g_cumulative_movement = 10.0f;
    }
}

static void detection_task_func(void* pvParameters) {
    ESP_LOGI(TAG, "Movement detection task started");
    
    while (g_running) {
        if (read_accelerometer_data(&g_current_data)) {
            // Call accelerometer callback if set
            if (g_accelerometer_callback) {
                g_accelerometer_callback(&g_current_data);
            }
            
            // Check for movement
            float movement_delta = fabsf(g_current_data.magnitude - g_previous_data.magnitude);
            
            // Update movement tracking for natural movement detection
            update_movement_tracking(movement_delta);
            
            uint32_t current_time = esp_timer_get_time() / 1000;
            bool should_trigger_reduction = false;
            uint8_t reduction = 1;
            
            // Check for immediate strong movement (original behavior)
            bool significant_movement = (movement_delta > g_threshold);
            if (significant_movement && (current_time - g_last_movement_time > MOVEMENT_COOLDOWN_MS)) {
                should_trigger_reduction = true;
                reduction = calculate_reduction_amount(movement_delta);
                ESP_LOGI(TAG, "Strong movement detected: delta=%.3f, reduction=%d%%", movement_delta, reduction);
            }
            
            // Check for natural movement patterns (walking, arm swinging)
            else if (g_cumulative_movement > CUMULATIVE_THRESHOLD && 
                     detect_walking_pattern() && 
                     (current_time - g_last_cumulative_reduction > MOVEMENT_COOLDOWN_MS)) {
                should_trigger_reduction = true;
                reduction = 2; // Natural movement gives slightly more reduction
                ESP_LOGI(TAG, "Natural movement detected: cumulative=%.3f, reduction=%d%%", g_cumulative_movement, reduction);
                g_last_cumulative_reduction = current_time;
                g_cumulative_movement = 0.0f; // Reset after triggering
            }
            
            // Trigger reduction if any condition met
            if (should_trigger_reduction) {
                // Call movement callback if set
                if (g_movement_callback) {
                    g_movement_callback(true, movement_delta, reduction);
                }
                g_last_movement_time = current_time;
            }
            
            // Update previous data
            g_previous_data = g_current_data;
        } else {
            ESP_LOGW(TAG, "Failed to read accelerometer data");
            g_accelerometer_working = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(g_sample_rate_ms));
    }
    
    ESP_LOGI(TAG, "Movement detection task ended");
    g_detection_task = nullptr;
    vTaskDelete(nullptr);
}

// Public API implementation
bool movement_handler_init(void) {
    if (g_initialized) {
        ESP_LOGW(TAG, "Movement handler already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing movement handler...");
    
    // Initialize I2C
    if (!init_i2c()) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return false;
    }
    
    // Initialize MPU6886
    if (!init_mpu6886()) {
        ESP_LOGE(TAG, "MPU6886 initialization failed");
        return false;
    }
    
    // Read initial data to verify accelerometer is working
    if (!read_accelerometer_data(&g_current_data)) {
        ESP_LOGE(TAG, "Failed to read initial accelerometer data");
        return false;
    }
    
    g_previous_data = g_current_data;
    g_accelerometer_working = true;
    g_initialized = true;
    
    ESP_LOGI(TAG, "Movement handler initialized successfully");
    return true;
}

bool movement_handler_start(void) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Movement handler not initialized");
        return false;
    }
    
    if (g_running) {
        ESP_LOGW(TAG, "Movement detection already running");
        return true;
    }
    
    ESP_LOGI(TAG, "Starting movement detection...");
    
    g_running = true;
    BaseType_t ret = xTaskCreatePinnedToCore(
        detection_task_func,
        "movement_detection",
        4096,
        nullptr,
        5,
        &g_detection_task,
        tskNO_AFFINITY
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create detection task");
        g_running = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Movement detection started");
    return true;
}

void movement_handler_stop(void) {
    if (!g_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping movement detection...");
    g_running = false;
    
    // Wait for task to finish
    if (g_detection_task) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
    }
}

void movement_handler_shutdown(void) {
    ESP_LOGI(TAG, "Shutting down movement handler...");
    
    movement_handler_stop();
    
    if (g_initialized) {
        i2c_driver_delete(MPU6886_I2C_PORT);
        g_initialized = false;
    }
    
    g_accelerometer_working = false;
    g_movement_callback = nullptr;
    g_accelerometer_callback = nullptr;
    
    ESP_LOGI(TAG, "Movement handler shutdown complete");
}

void movement_handler_set_movement_callback(movement_event_callback_t callback) {
    g_movement_callback = callback;
}

void movement_handler_set_accelerometer_callback(accelerometer_data_callback_t callback) {
    g_accelerometer_callback = callback;
}

void movement_handler_set_threshold(float threshold) {
    if (threshold > 0.0f && threshold <= 10.0f) {
        g_threshold = threshold;
        ESP_LOGI(TAG, "Movement threshold set to %.3f", g_threshold);
    } else {
        ESP_LOGW(TAG, "Invalid threshold value: %.3f", threshold);
    }
}

float movement_handler_get_threshold(void) {
    return g_threshold;
}

void movement_handler_set_sample_rate(uint32_t sample_rate_ms) {
    if (sample_rate_ms >= 10 && sample_rate_ms <= 1000) {
        g_sample_rate_ms = sample_rate_ms;
        ESP_LOGI(TAG, "Sample rate set to %lu ms", (unsigned long)g_sample_rate_ms);
    } else {
        ESP_LOGW(TAG, "Invalid sample rate: %lu ms", (unsigned long)sample_rate_ms);
    }
}

uint32_t movement_handler_get_sample_rate(void) {
    return g_sample_rate_ms;
}

bool movement_handler_is_accelerometer_working(void) {
    return g_accelerometer_working;
}

bool movement_handler_is_running(void) {
    return g_running;
}

bool movement_handler_get_latest_data(accelerometer_data_t* data) {
    if (!data || !g_initialized) {
        return false;
    }
    
    *data = g_current_data;
    return true;
}
