extern "C" {
    #include "led_strip.h"
}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "M5GFX.h"
#include "SoundManager.hpp"
#include "actor_guest_test.h"
#include "health_ui_test.h"

// Uncomment the line below to run Health UI Test instead of main game
// #define RUN_HEALTH_UI_TEST

M5GFX display;
led_strip_handle_t led_strip = NULL;
SoundManager speaker;

extern "C" void app_main(void)
{
    printf("Haunted House Project - Actor/Guest IR Test\n");
    printf("Testing IR radiation communication between devices\n");
    
    // Initialize LED strip
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = 15;
    strip_config.max_leds = 10; // M5Stack Fire has 10 LEDs total (5 on each side)
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    if (led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip) != ESP_OK) {
        printf("LED strip init failed\n");
        led_strip = NULL;
    }

    // Initialize sound (but keep it quiet)
    if (!speaker.init(25)) {
        printf("Sound init failed\n");
    }

    // Initialize display
    if (display.begin()) {
        display.setRotation(1);
        printf("Display initialized\n");
    } else {
        printf("Display initialization failed\n");
    }
    
#ifdef RUN_HEALTH_UI_TEST
    // Run the health UI test
    printf("Starting Health UI Test...\n");
    health_ui_test_main();
#else
    // Run the actor/guest test
    printf("Starting Actor/Guest Test...\n");
    actor_guest_test_main();
#endif
}