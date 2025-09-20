#include "motor_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <cstddef>

namespace {
static constexpr gpio_num_t kMotorGpio = GPIO_NUM_13;
static constexpr const char* TAG = "MOTOR_HANDLER";

struct MotorPatternStep {
    bool on;
    uint16_t duration_ms;
};

struct MotorCommand {
    const MotorPatternStep* steps;
    size_t step_count;
    uint32_t pulse_ms;
    bool is_pulse;
};

static QueueHandle_t s_command_queue = nullptr;
static TaskHandle_t s_worker_task = nullptr;
static bool s_initialized = false;

static const MotorPatternStep kGuestHitPattern[] = {
    {true, 180},
    {false, 70},
    {true, 150},
};

static const MotorPatternStep kActorConfirmPattern[] = {
    {true, 120},
    {false, 60},
    {true, 120},
};

void apply_motor(bool on) {
    gpio_set_level(kMotorGpio, on ? 1 : 0);
}

bool should_abort_current_pattern() {
    return s_command_queue && uxQueueMessagesWaiting(s_command_queue) > 0;
}

bool wait_with_abort(uint32_t duration_ms) {
    uint32_t remaining = duration_ms;
    while (remaining > 0) {
        if (should_abort_current_pattern()) {
            return false;
        }
        uint32_t slice = remaining > 20 ? 20 : remaining;
        vTaskDelay(pdMS_TO_TICKS(slice));
        if (remaining <= slice) {
            break;
        }
        remaining -= slice;
    }
    return true;
}

void run_pattern(const MotorPatternStep* steps, size_t count) {
    if (!steps || count == 0) {
        apply_motor(false);
        return;
    }

    bool keep_running = true;
    for (size_t i = 0; i < count && keep_running; ++i) {
        apply_motor(steps[i].on);
        if (steps[i].duration_ms == 0) {
            continue;
        }
        keep_running = wait_with_abort(steps[i].duration_ms);
    }

    apply_motor(false);
}

void run_pulse(uint32_t duration_ms) {
    if (duration_ms == 0) {
        return;
    }
    MotorPatternStep single{true, static_cast<uint16_t>(duration_ms)};
    run_pattern(&single, 1);
}

void enqueue_command(const MotorCommand& cmd) {
    if (!s_command_queue) {
        return;
    }
    if (xQueueSend(s_command_queue, &cmd, 0) != pdTRUE) {
        // Queue full; drop oldest command to favor most recent haptics.
        MotorCommand discarded{};
        if (xQueueReceive(s_command_queue, &discarded, 0) == pdTRUE) {
            xQueueSend(s_command_queue, &cmd, 0);
        }
    }
}

void motor_task(void* /*arg*/) {
    MotorCommand cmd{};
    while (xQueueReceive(s_command_queue, &cmd, portMAX_DELAY) == pdTRUE) {
        if (!cmd.is_pulse && cmd.steps == nullptr && cmd.step_count == 0) {
            break; // shutdown signal
        }

        if (cmd.is_pulse) {
            run_pulse(cmd.pulse_ms);
        } else {
            run_pattern(cmd.steps, cmd.step_count);
        }
    }

    apply_motor(false);
    s_worker_task = nullptr;
    vTaskDelete(nullptr);
}

} // namespace

bool motor_handler_init(void) {
    if (s_initialized) {
        return true;
    }

    gpio_config_t io_conf{};
    io_conf.pin_bit_mask = (1ULL << kMotorGpio);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure motor GPIO: %s", esp_err_to_name(err));
        return false;
    }

    apply_motor(false);

    s_command_queue = xQueueCreate(4, sizeof(MotorCommand));
    if (!s_command_queue) {
        ESP_LOGE(TAG, "Failed to create motor command queue");
        return false;
    }

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        motor_task,
        "motor_fx",
        2048,
        nullptr,
        3,
        &s_worker_task,
        tskNO_AFFINITY);

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to start motor worker task");
        vQueueDelete(s_command_queue);
        s_command_queue = nullptr;
        return false;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Motor handler initialized on GPIO %d", static_cast<int>(kMotorGpio));
    return true;
}

void motor_handler_shutdown(void) {
    if (!s_initialized) {
        return;
    }

    s_initialized = false;

    if (s_command_queue) {
        MotorCommand stop_cmd{};
        xQueueSend(s_command_queue, &stop_cmd, portMAX_DELAY);
    }

    // Wait for worker to exit gracefully
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (s_worker_task != nullptr && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (s_command_queue) {
        vQueueDelete(s_command_queue);
        s_command_queue = nullptr;
    }

    apply_motor(false);
    gpio_reset_pin(kMotorGpio);
}

void motor_handler_pulse(uint32_t duration_ms) {
    if (!s_initialized || duration_ms == 0) {
        return;
    }

    MotorCommand cmd{};
    cmd.is_pulse = true;
    cmd.pulse_ms = duration_ms;
    enqueue_command(cmd);
}

void motor_handler_trigger_guest_hit(void) {
    if (!s_initialized) {
        return;
    }

    MotorCommand cmd{};
    cmd.steps = kGuestHitPattern;
    cmd.step_count = sizeof(kGuestHitPattern) / sizeof(kGuestHitPattern[0]);
    enqueue_command(cmd);
}

void motor_handler_trigger_actor_confirm(void) {
    if (!s_initialized) {
        return;
    }

    MotorCommand cmd{};
    cmd.steps = kActorConfirmPattern;
    cmd.step_count = sizeof(kActorConfirmPattern) / sizeof(kActorConfirmPattern[0]);
    enqueue_command(cmd);
}

