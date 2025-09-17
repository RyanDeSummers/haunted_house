# 🏚️ Haunted House Project - Function Summary

## 📋 **Project Overview**
A multiplayer radiation-themed game for M5Stack Fire devices where players can be "Guests" (radiation victims) or "Actors" (radiation sources). Guests accumulate radiation over time and can reduce it by shaking their device. Actors send radiation signals to guests via IR communication.

---

## 🎮 **Main Game Functions** (`actor_guest_test.cpp`)

### **Core Game Loop**
- **`actor_guest_test_main()`** - Main entry point that initializes the game, sets up buttons, IR communication, and runs the main game loop
- **`check_buttons()`** - Handles all button input (A, C, Middle) for mode switching and radiation adjustment
- **`update_display()`** - Updates the screen based on current mode (Start Screen, Guest, Actor)

### **Device Modes**
- **`MODE_START_SCREEN`** - Welcome screen with "Haunted House" title and start button
- **`MODE_GUEST`** - Player receives radiation from actors and shakes to reduce it
- **`MODE_ACTOR`** - Player sends radiation signals to nearby guests

### **Radiation Management**
- **Guest Mode**: Radiation increases by 1% per second automatically
- **Movement Detection**: Shaking device reduces radiation by 1%
- **IR Reception**: Receiving signals from actors adds radiation (with 2-second cooldown)

### **Visual Feedback**
- **`show_radiation_indicator()`** - Sets LED colors based on radiation level (Green→Yellow→Red)
- **`show_movement_indicator()`** - Blue flash when movement is detected
- **`show_mode_indicator()`** - Purple pulsing for start screen, solid purple for actor mode

---

## 🏃 **Movement Detection** (`movement_handler.cpp`)

### **Core Functions**
- **`movement_handler_init()`** - Initializes MPU6886 accelerometer via I2C
- **`movement_handler_start()`** - Begins continuous movement monitoring
- **`movement_handler_stop()`** - Stops movement monitoring
- **`movement_handler_shutdown()`** - Cleans up accelerometer resources

### **Configuration**
- **`movement_handler_set_threshold()`** - Sets sensitivity threshold (default: 0.8)
- **`movement_handler_set_sample_rate()`** - Sets monitoring frequency (default: 50ms)
- **`movement_handler_get_threshold()`** - Gets current sensitivity setting
- **`movement_handler_get_sample_rate()`** - Gets current monitoring frequency

### **Callbacks**
- **`movement_handler_set_movement_callback()`** - Sets function to call when movement detected
- **`movement_handler_set_accelerometer_callback()`** - Sets function to receive raw accelerometer data

### **Status & Data**
- **`movement_handler_is_accelerometer_working()`** - Checks if accelerometer is functioning
- **`movement_handler_is_running()`** - Checks if monitoring is active
- **`movement_handler_get_latest_data()`** - Gets most recent accelerometer readings

---

## 📡 **IR Communication** (`IR_handler.cpp`)

### **Initialization & Control**
- **`ir_handler_init()`** - Initializes IR transmission (RMT) and reception (UART) systems
- **`ir_handler_shutdown()`** - Cleans up all IR resources
- **`ir_set_reception_enabled()`** - Enables/disables IR listening
- **`ir_set_rx_callback()`** - Sets function to call when packets received
- **`ir_set_tx_callback()`** - Sets function to call when transmission completes

### **Packet Management**
- **`ir_transmit_packet()`** - Queues IR packet for transmission
- **`ir_create_packet()`** - Creates properly formatted IR packet
- **`ir_validate_packet()`** - Validates packet structure
- **`ir_get_current_sequence()`** - Gets auto-incrementing sequence number
- **`ir_get_self_mac()`** - Gets device's MAC address

### **Protocol Details**
- **Preamble**: "ZT" (0x5A, 0x54) to identify valid packets
- **Carrier**: 38kHz IR frequency with 33% duty cycle
- **Data Rate**: 2400 bps
- **Packet Structure**: Preamble + Length + Sequence + MAC + Payload + CRC
- **Hardware**: TX on GPIO 26, RX on GPIO 36

---

## 🔊 **Sound System** (`SoundManager.cpp`)

### **Core Functions**
- **`init()`** - Initializes DAC on GPIO 25 for audio output
- **`beep()`** - Plays a simple beep at specified frequency and duration
- **`death_jingle()`** - Plays a short musical sequence
- **`shutdown()`** - Cleans up audio resources

### **Technical Details**
- **Hardware**: Uses DAC1 (GPIO 25) on M5Stack Fire
- **Method**: Square wave generation via timer interrupts
- **Queue System**: Commands queued and processed by dedicated task
- **Blocking**: Audio commands block until completion

---

## 🎯 **Hardware Configuration**

### **M5Stack Fire Setup**
- **Display**: M5GFX library for screen rendering
- **LEDs**: 10 WS2812B LEDs (5 per side) on GPIO 15
- **Buttons**: A (GPIO 39), C (GPIO 37), Middle (GPIO 38)
- **Accelerometer**: MPU6886 via I2C
- **IR**: TX on GPIO 26, RX on GPIO 36
- **Audio**: DAC on GPIO 25

### **Game Controls**
- **Start Screen**: Press Middle button to begin as Guest
- **Guest Mode**: Shake device to reduce radiation
- **Actor Mode**: Hold A+C for 5 seconds to switch from Guest
- **Actor Controls**: A button decreases radiation, C button increases radiation

---

## 🔧 **Technical Architecture**

### **Task Structure**
- **Main Task**: Game loop, button handling, display updates
- **Movement Task**: Continuous accelerometer monitoring
- **IR TX Task**: Handles packet transmission
- **IR RX Task**: Processes incoming IR signals
- **Sound Task**: Manages audio playback queue

### **Memory Management**
- **Static Allocation**: All buffers and structures pre-allocated
- **No Dynamic Memory**: Avoids heap fragmentation
- **Queue Systems**: Inter-task communication via FreeRTOS queues

### **Error Handling**
- **Hardware Checks**: Validates accelerometer, IR, and LED initialization
- **Graceful Degradation**: Game continues even if some features fail
- **Debug Logging**: Comprehensive logging for troubleshooting

---

## 🚀 **Getting Started**

1. **Build**: `idf.py build`
2. **Flash**: `idf.py flash`
3. **Monitor**: `idf.py monitor`
4. **Play**: Press middle button to start as Guest, shake to reduce radiation!

---

*This project demonstrates advanced embedded programming concepts including multi-task coordination, hardware abstraction, real-time communication protocols, and user interface design on resource-constrained hardware.*
