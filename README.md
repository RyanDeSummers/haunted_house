# 🏚️ Haunted House ESP32 Game

A multiplayer radiation survival game for ESP32 devices featuring IR communication, movement detection, and dynamic gameplay mechanics.

## 🎮 Game Overview

This ESP32-based haunted house game creates an immersive multiplayer experience where players must manage radiation levels through movement and strategic IR communication.

### 🎯 Core Features

- **Dual Mode System**: Switch between Guest and Actor roles
- **IR Communication**: Wireless signal transmission between devices
- **Movement Detection**: Accelerometer-based radiation reduction
- **Dynamic Radiation**: Automatic increase with manual reduction options
- **Visual Feedback**: LED strips and display updates
- **Audio Support**: Sound effects and alerts

## 🎲 Game Modes

### 👻 Guest Mode
- **Automatic Radiation Increase**: +2% every second
- **IR Signal Reception**: Receive radiation from actor devices
- **Movement Detection**: Shake device to reduce radiation by 1%
- **Cooldown Protection**: 2-second cooldown between IR signals

### 🎭 Actor Mode
- **IR Signal Transmission**: Continuously sends IR signals
- **Adjustable Radiation**: Press A (-1%) or C (+1%) to adjust radiation amount (1-20%)
- **Mode Switching**: Hold A+C for 5 seconds to switch between modes

## 🎮 Controls

| Button | Action |
|--------|--------|
| **A** | Decrease radiation (Actor mode) |
| **C** | Increase radiation (Actor mode) |
| **A + C** (Hold 5s) | Switch between Guest/Actor modes |
| **Device Shake** | Reduce radiation by 1% (Guest mode) |

## 🔧 Technical Features

### 🛡️ Robust Communication
- **CRC-8 Validation**: Ensures packet integrity
- **MAC Address Filtering**: Prevents self-reception
- **Payload Length Validation**: Prevents buffer overruns
- **Error Handling**: Comprehensive logging and recovery

### 📡 IR Protocol
- **Preamble**: "ZT" synchronization
- **Packet Structure**: Length, Sequence, MAC, Payload, CRC
- **Transmission**: RMT-based IR LED control
- **Reception**: UART-based IR sensor input

### 🎯 Hardware Integration
- **M5Stack Core**: ESP32-based development board
- **MPU6886**: Accelerometer for movement detection
- **IR LED/Sensor**: Wireless communication
- **LED Strip**: Visual feedback
- **Speaker**: Audio output

## 🚀 Getting Started

### Prerequisites
- ESP-IDF v5.3.3 or later
- M5Stack Core device
- IR LED and sensor components

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/RyanDeSummers/haunted_house.git
   cd haunted_house
   ```

2. Build the project:
   ```bash
   idf.py build
   ```

3. Flash to device:
   ```bash
   idf.py flash
   ```

### Configuration
- Modify `sdkconfig` for hardware-specific settings
- Adjust movement sensitivity in `movement_handler.cpp`
- Configure IR transmission power in `IR_handler.cpp`

## 📁 Project Structure

```
haunted_house/
├── main/
│   ├── actor_guest_test.cpp    # Main game logic
│   ├── actor_guest_test.h      # Game header
│   ├── IR_handler.cpp          # IR communication
│   ├── IR_handler.h            # IR protocol definitions
│   ├── movement_handler.cpp    # Accelerometer handling
│   ├── movement_handler.h      # Movement detection API
│   ├── SoundManager.cpp        # Audio system
│   ├── SoundManager.hpp        # Sound header
│   └── main.cpp               # Application entry point
├── components/
│   └── M5GFX/                  # Display graphics library
├── managed_components/
│   └── espressif__led_strip/   # LED control
└── CMakeLists.txt              # Build configuration
```

## 🎯 Game Flow

1. **Device starts as Guest** - radiation increases automatically
2. **Guest receives IR signals** - adds radiation from actors
3. **Guest shakes device** - reduces radiation through movement
4. **Hold A+C for 5 seconds** - switches to Actor mode
5. **Actor sends IR signals** - with adjustable radiation amount
6. **Actor adjusts radiation** - using A/C buttons (1-20% range)

## 🔧 Development

### Building
```bash
idf.py build
```

### Flashing
```bash
idf.py flash
```

### Monitoring
```bash
idf.py monitor
```

### Clean Build
```bash
idf.py fullclean
idf.py build
```

## 📊 Performance

- **Build Size**: ~360KB (64% free space)
- **Memory Usage**: Optimized for ESP32 constraints
- **IR Range**: ~2-3 meters (depending on environment)
- **Battery Life**: Hours of continuous gameplay

## 🐛 Troubleshooting

### Common Issues
- **IR signals not received**: Check CRC validation settings
- **Movement not detected**: Adjust sensitivity threshold
- **Build errors**: Ensure ESP-IDF v5.3.3+ is installed
- **Flash failures**: Check USB connection and drivers

### Debug Mode
Enable debug logging by modifying log levels in `sdkconfig`:
```
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is open source. See individual component licenses for details.

## 🎉 Acknowledgments

- **M5Stack** for the development platform
- **ESP-IDF** for the framework
- **M5GFX** for display graphics
- **Espressif** for LED strip components

---

**Happy Haunting! 👻🎮**