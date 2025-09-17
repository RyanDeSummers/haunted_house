idf_component_register(
    SRCS
        "main.cpp"
        "SoundManager.cpp"
        "IR_handler.cpp"        # IR communication handler
        "movement_handler.cpp"   # Movement detection handler
        "actor_guest_test.cpp"   # Actor/Guest IR radiation test

    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        M5GFX
        espressif__led_strip
        esp_timer
        esp_driver_dac
        esp_driver_rmt
        esp_driver_uart
        esp_driver_i2c
        esp_driver_gpio
        freertos
        driver
        esp_wifi
    )












