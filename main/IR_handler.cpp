#include "IR_handler.h"
#include <string.h>
#include <stdio.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Defensive assertion for packet header size
_Static_assert(PACKET_HEADER_SIZE == 3, "Expecting 2-byte ZT + 1-byte LEN");

static const char* TAG = "IR_HANDLER";

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// Hardware handles
static rmt_channel_handle_t rmt_tx = nullptr;
static rmt_encoder_handle_t tx_copy = nullptr;
static TaskHandle_t beacon_task = nullptr;
static TaskHandle_t uart_rx_task = nullptr;
static esp_timer_handle_t beacon_timer = nullptr;

// Callback functions
static ir_rx_callback_t rx_callback = nullptr;
static ir_tx_callback_t tx_callback = nullptr;

// State variables
static volatile bool ir_reception_enabled = false;
static volatile bool ir_system_initialized = false;
static uint16_t current_sequence = 0;
static uint8_t self_mac[6] = {0};

// ============================================================================
// RMT TRANSMISSION FUNCTIONS
// ============================================================================

/**
 * @brief Converts a UART byte into IR transmission symbols
 * 
 * @param b The UART byte to convert (0-255)
 * @param out Array of RMT symbols to fill with IR representation
 * @return Number of RMT symbols generated
 * 
 * This function converts a single UART byte into IR transmission symbols:
 * - LSB first transmission order
 * - Mark = 0 (IR LED on), Space = 1 (IR LED off)
 * - Each bit gets IR_BIT_US microseconds duration
 * - Adds start and stop bits around the 8 data bits
 */
static size_t build_uart_byte(uint8_t b, rmt_symbol_word_t* out) {
    if (out == nullptr) {
        ESP_LOGE(TAG, "build_uart_byte: out parameter is null");
        return 0;
    }
    
    // Lambda functions for creating mark/space symbols
    auto create_mark = [](rmt_symbol_word_t* s, uint32_t us) {
        s->level0 = 1;      // IR LED on
        s->duration0 = us;  // Duration in microseconds
        s->level1 = 0;      // IR LED off
        s->duration1 = 1;   // Minimum space
    };
    
    auto create_space = [](rmt_symbol_word_t* s, uint32_t us) {
        s->level0 = 0;      // IR LED off
        s->duration0 = us;  // Duration in microseconds
        s->level1 = 0;      // IR LED off
        s->duration1 = 1;   // Minimum space
    };
    
    size_t symbol_index = 0;
    
    // Start bit (mark)
    create_mark(&out[symbol_index++], IR_BIT_US);
    
    // 8 data bits, LSB first
    for (int bit = 0; bit < 8; bit++) {
        if ((b >> bit) & 1) {
            // Bit is 1: create space
            create_space(&out[symbol_index++], IR_BIT_US);
        } else {
            // Bit is 0: create mark
            create_mark(&out[symbol_index++], IR_BIT_US);
        }
    }
    
    // Stop bit (space)
    create_space(&out[symbol_index++], IR_BIT_US);
    
    return symbol_index;
}

/**
 * @brief Calculates CRC-8 checksum for packet data
 * 
 * @param data Pointer to data array
 * @param len Length of data array
 * @return 8-bit CRC value
 * 
 * This function implements CRC-8 with polynomial 0x31 (x^8 + x^5 + x^4 + 1).
 * Used to verify packet integrity during transmission and reception.
 */
static uint8_t calculate_crc8(const uint8_t* data, size_t len) {
    if (data == nullptr) {
        ESP_LOGE(TAG, "calculate_crc8: data parameter is null");
        return 0;
    }
    if (len == 0) {
        ESP_LOGW(TAG, "calculate_crc8: length is 0");
        return 0;
    }
    
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Builds complete IR packet from components
 * 
 * @param packet Pointer to IR packet structure
 * @param symbols Array to fill with IR transmission symbols
 * @return Number of symbols generated
 * 
 * This function constructs the complete IR packet:
 * - Preamble (ZT bytes)
 * - Length field
 * - Sequence number (2 bytes, little-endian)
 * - MAC address (6 bytes)
 * - Payload data (if any)
 * - CRC-8 checksum
 */
static size_t build_ir_packet(const ir_packet_t* packet, rmt_symbol_word_t* symbols) {
    if (packet == nullptr || symbols == nullptr) {
        ESP_LOGE(TAG, "build_ir_packet: null parameters");
        return 0;
    }
    
    size_t n = 0;
    
    // Preamble
    n += build_uart_byte(IR_PREAMBLE0, &symbols[n]);
    n += build_uart_byte(IR_PREAMBLE1, &symbols[n]);
    
    // Length field (sequence + MAC + payload)
    const uint8_t len = PACKET_SEQ_SIZE + IR_MAC_LEN + packet->payload_len;
    n += build_uart_byte(len, &symbols[n]);
    
    // Sequence number (little-endian)
    n += build_uart_byte((uint8_t)(packet->sequence & 0xFF), &symbols[n]);      // LSB
    n += build_uart_byte((uint8_t)(packet->sequence >> 8), &symbols[n]);        // MSB
    
    // MAC address
    for (int i = 0; i < IR_MAC_LEN; i++) {
        n += build_uart_byte(packet->sender_mac[i], &symbols[n]);
    }
    
    // Payload data (if any)
    for (int i = 0; i < packet->payload_len; i++) {
        n += build_uart_byte(packet->payload[i], &symbols[n]);
    }
    
    // CRC-8 checksum - build exactly [LEN][SEQ_LO][SEQ_HI][MAC][PAYLOAD]
    uint8_t crc_src[1 + PACKET_SEQ_SIZE + IR_MAC_LEN + IR_MAX_PAYLOAD];
    size_t crc_idx = 0;
    crc_src[crc_idx++] = len;                                    // Length
    crc_src[crc_idx++] = (uint8_t)(packet->sequence & 0xFF);   // Sequence low
    crc_src[crc_idx++] = (uint8_t)(packet->sequence >> 8);      // Sequence high
    memcpy(&crc_src[crc_idx], packet->sender_mac, IR_MAC_LEN);  // MAC address
    crc_idx += IR_MAC_LEN;                                       // Advance index after MAC
    if (packet->payload_len > 0) {
        memcpy(&crc_src[crc_idx], packet->payload, packet->payload_len); // Payload
        crc_idx += packet->payload_len;                          // Advance index after payload
    }
    
    uint8_t crc = calculate_crc8(crc_src, crc_idx);
    
    // Debug logging for TX CRC
    ESP_LOGI(TAG, "TX CRC over %u bytes:", (unsigned)crc_idx);
    for (size_t i = 0; i < crc_idx; ++i) {
        ESP_LOGI(TAG, "  %02X", crc_src[i]);
    }
    ESP_LOGI(TAG, "TX CRC = %02X", crc);
    
    n += build_uart_byte(crc, &symbols[n]);
    
    return n;
}

/**
 * @brief Configures RMT peripheral for IR LED transmission
 * 
 * @return true if setup successful, false otherwise
 * 
 * This function sets up the RMT hardware for transmitting IR signals:
 * - Configures RMT channel on GPIO 26
 * - Sets up 38kHz carrier frequency with 33% duty cycle
 * - Creates copy encoder for IR symbol generation
 * - Enables the RMT channel for transmission
 */
static bool setup_rmt_tx() {
    rmt_tx_channel_config_t txc = { 
        .gpio_num = IR_TX_GPIO, 
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ, 
        .mem_block_symbols = RMT_MEM_BLOCK_SYMBOLS, 
        .trans_queue_depth = RMT_TX_QUEUE_DEPTH,
        .intr_priority = RMT_TX_INTR_PRIORITY,
        .flags = {0}
    };
    
    ESP_LOGI(TAG, "Setting up RMT TX for IR on GPIO %d", IR_TX_GPIO);
    
    // Create RMT TX channel
    esp_err_t err = rmt_new_tx_channel(&txc, &rmt_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(err));
        return false;
    }
    
    // Configure carrier frequency
    rmt_carrier_config_t c = { 
        .frequency_hz = IR_CARRIER_FREQ_HZ, 
        .duty_cycle = IR_CARRIER_DUTY_CYCLE, 
        .flags = { .polarity_active_low = 0 } 
    };
    err = rmt_apply_carrier(rmt_tx, &c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply RMT carrier: %s", esp_err_to_name(err));
        rmt_del_channel(rmt_tx);
        rmt_tx = nullptr;
        return false;
    }
    
    // Create copy encoder
    rmt_copy_encoder_config_t cc = {};
    err = rmt_new_copy_encoder(&cc, &tx_copy);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT copy encoder: %s", esp_err_to_name(err));
        rmt_del_channel(rmt_tx);
        rmt_tx = nullptr;
        return false;
    }
    
    // Enable RMT channel
    err = rmt_enable(rmt_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(err));
        rmt_del_channel(rmt_tx);
        rmt_tx = nullptr;
        return false;
    }
    
    ESP_LOGI(TAG, "RMT TX setup completed successfully");
    return true;
}

// ============================================================================
// UART RECEPTION FUNCTIONS
// ============================================================================

/**
 * @brief Configures UART peripheral for receiving demodulated IR signals
 * 
 * @return true if setup successful, false otherwise
 * 
 * This function sets up UART2 for IR reception:
 * - Configures UART parameters (2400 baud, 8N1 format)
 * - Installs UART driver with receive buffer
 * - Binds GPIO 36 as RX pin for IR receiver input
 */
static bool setup_uart_rx() {
    uart_config_t cfg = { 
        .baud_rate = IR_BAUD, 
        .data_bits = UART_DATA_8_BITS, 
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .source_clk = UART_SCLK_DEFAULT
    };
    
    ESP_LOGI(TAG, "Setting up UART RX for IR on GPIO %d", IR_RX_GPIO);
    
    // Configure UART parameters
    esp_err_t err = uart_param_config(IR_UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(err));
        return false;
    }
    
    // Install UART driver
    err = uart_driver_install(IR_UART_PORT, UART_RING_BUFFER_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
        return false;
    }
    
    // Set UART pins
    err = uart_set_pin(IR_UART_PORT, UART_PIN_NO_CHANGE, IR_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
        uart_driver_delete(IR_UART_PORT);
        return false;
    }
    
    ESP_LOGI(TAG, "UART RX setup completed successfully on GPIO%d @ %d-8N1", (int)IR_RX_GPIO, IR_BAUD);
    return true;
}

/**
 * @brief Validates CRC-8 checksum for received packet
 * 
 * @param pkt Pointer to packet data starting at length field (after ZT preamble)
 * @param len Length of packet data (from length field)
 * @param received_crc CRC value received in packet
 * @return true if CRC matches, false otherwise
 */
static bool validate_packet_crc(const uint8_t* pkt, uint8_t len, uint8_t received_crc) {
    if (pkt == nullptr) {
        ESP_LOGE(TAG, "validate_packet_crc: pkt parameter is null");
        return false;
    }
    
    // Hash exactly [LEN][SEQ_LO][SEQ_HI][MAC][PAYLOAD] - same as TX
    // pkt points to LEN (right after 'Z','T')
    // layout: [LEN][SEQ_LO][SEQ_HI][MAC(6)][PAYLOAD...][CRC]
    size_t crc_input_len = 1u + len;  // [LEN..last payload]
    
    uint8_t calculated_crc = calculate_crc8(pkt, crc_input_len);
    
    // Debug logging for RX CRC
    ESP_LOGI(TAG, "RX CRC over %u bytes:", (unsigned)crc_input_len);
    for (size_t i = 0; i < crc_input_len; ++i) {
        ESP_LOGI(TAG, "  %02X", pkt[i]);
    }
    ESP_LOGI(TAG, "RX calc=%02X recv=%02X match=%s", 
             calculated_crc, received_crc, (calculated_crc == received_crc) ? "YES" : "NO");
    
    return calculated_crc == received_crc;
}

/**
 * @brief Processes a valid IR packet and calls the RX callback
 * 
 * @param packet_data Pointer to packet data
 * @param packet_len Length of packet data
 * @param rssi Signal strength indicator
 */
static void process_valid_packet(const uint8_t* packet_data, uint8_t packet_len, uint8_t rssi) {
    if (rx_callback == nullptr) return;
    
    // Extract packet components
    if (packet_len < PACKET_SEQ_SIZE + IR_MAC_LEN) {
        ESP_LOGW(TAG, "Packet too short: %d bytes", packet_len);
        return;
    }
    
    ir_packet_t packet = {0};
    
    // Sequence number (little-endian)
    packet.sequence = (uint16_t)packet_data[0] | ((uint16_t)packet_data[1] << 8);
    
    // MAC address
    memcpy(packet.sender_mac, &packet_data[2], IR_MAC_LEN);
    
    // Payload data (if any)
    if (packet_len > PACKET_SEQ_SIZE + IR_MAC_LEN) {
        packet.payload_len = packet_len - PACKET_SEQ_SIZE - IR_MAC_LEN;
        if (packet.payload_len > sizeof(packet.payload)) {
            packet.payload_len = sizeof(packet.payload);
        }
        memcpy(packet.payload, &packet_data[2 + IR_MAC_LEN], packet.payload_len);
    }
    
    // Call the RX callback
    rx_callback(&packet, rssi);
}

/**
 * @brief Scans ring buffer for valid IR packets
 * 
 * @param ring_buffer Ring buffer containing UART data
 * @param buffer_length Current length of data in ring buffer
 * @return Number of bytes consumed (packets processed)
 */
static int scan_for_packets(uint8_t* ring_buffer, int buffer_length) {
    int scan_index = 0;
    
    // Scan through the ring buffer looking for valid packets
    while (scan_index + 4 < buffer_length) {
        // Look for IR packet preamble
        if (ring_buffer[scan_index] == IR_PREAMBLE0 && 
            ring_buffer[scan_index + 1] == IR_PREAMBLE1) {
            
            ESP_LOGI(TAG, "Found preamble at index %d", scan_index);
            
            // Extract the packet length
            uint8_t len = ring_buffer[scan_index + 2];
            ESP_LOGI(TAG, "Packet length: %d", len);
            
            // Check if we have a complete packet in the buffer
            if (len >= PACKET_SEQ_SIZE + IR_MAC_LEN && 
                (scan_index + PACKET_HEADER_SIZE + len + 1) <= buffer_length) {
                
                ESP_LOGI(TAG, "Packet size check passed");
                
                // Extract packet data and CRC
                const uint8_t* packet_data = &ring_buffer[scan_index + 3]; // SEQ_LO (for parsing)
                const uint8_t* len_ptr = &ring_buffer[scan_index + 2];     // LEN byte (for CRC)
                uint8_t received_crc = ring_buffer[scan_index + 3 + len];
                
                ESP_LOGI(TAG, "Received CRC: 0x%02X", received_crc);
                ESP_LOG_BUFFER_HEX(TAG, packet_data, len);
                
                // Debug: Log what we're hashing for CRC validation
                ESP_LOGI(TAG, "RX will CRC over %u bytes starting at LEN:", (unsigned)(1 + len));
                ESP_LOG_BUFFER_HEX(TAG, len_ptr, 1 + len);
                
                // Validate packet integrity with fixed CRC validation
                if (validate_packet_crc(len_ptr, len, received_crc)) {
                    ESP_LOGI(TAG, "✅ CRC validation passed - processing packet");
                    process_valid_packet(packet_data, len, 128); // TODO: Implement RSSI measurement
                } else {
                    ESP_LOGW(TAG, "❌ CRC validation failed - discarding corrupted packet");
                    // Move past this packet without processing
                }
                
                // Move past this packet (include CRC byte)
                scan_index += PACKET_HEADER_SIZE + len + 1;
                continue;
            } else {
                ESP_LOGW(TAG, "Packet size check failed: len=%d, needed=%d, buffer_len=%d", 
                         len, PACKET_SEQ_SIZE + IR_MAC_LEN, buffer_length - scan_index);
            }
        }
        
        // No valid frame found at this position, advance to next byte
        scan_index++;
    }
    
    return scan_index;
}

// ============================================================================
// TASK FUNCTIONS
// ============================================================================

/**
 * @brief Timer callback for beacon transmission
 */
static void beacon_timer_cb(void* arg) {
    if (beacon_task) {
        xTaskNotifyGive(beacon_task);
    }
}

/**
 * @brief Background task for IR packet transmission
 */
static void beacon_task_func(void* arg) {
    beacon_task = xTaskGetCurrentTaskHandle();
    
    // Create periodic timer if it doesn't exist
    if (!beacon_timer) {
        const esp_timer_create_args_t timer_args = {
            .callback = &beacon_timer_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ir_beacon"
        };
        
        esp_err_t err = esp_timer_create(&timer_args, &beacon_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create beacon timer: %s", esp_err_to_name(err));
            return;
        }
    }
    
    for (;;) {
        // Wait for timer notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // This task is currently used for periodic beacons in the original game
        // For the haunted house, we'll use it for periodic transmission if needed
        // For now, it just waits for notifications
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Background task for processing incoming IR data
 */
static void uart_rx_task_func(void* arg) {
    uint8_t ring_buffer[UART_RING_BUFFER_SIZE];
    int buffer_length = 0;
    uint8_t read_buffer[UART_READ_BUFFER_SIZE];
    
    for (;;) {
        // Wait if IR reception is disabled
        if (!ir_reception_enabled) {
            vTaskDelay(pdMS_TO_TICKS(UART_TASK_DELAY_MS));
            continue;
        }
        
        // Read UART data with timeout
        int bytes_read = uart_read_bytes(IR_UART_PORT, read_buffer, sizeof(read_buffer), pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
        if (bytes_read < 0) {
            ESP_LOGW(TAG, "UART read error: %d", bytes_read);
            vTaskDelay(pdMS_TO_TICKS(UART_TASK_DELAY_MS));
            continue;
        }
        if (bytes_read == 0) continue;
        
        // Debug: Log raw received data
        ESP_LOGI(TAG, "UART RX: %d bytes received", bytes_read);
        if (bytes_read > 0) {
            ESP_LOG_BUFFER_HEX(TAG, read_buffer, bytes_read);
        }
        
        // Append new data to ring buffer with overflow protection
        if (bytes_read > (int)sizeof(ring_buffer) - buffer_length) {
            ESP_LOGW(TAG, "Ring buffer overflow, resetting (had %d bytes, got %d)", buffer_length, bytes_read);
            buffer_length = 0;
        }
        memcpy(&ring_buffer[buffer_length], read_buffer, bytes_read);
        buffer_length += bytes_read;
        
        // Scan for and process complete packets
        int consumed = scan_for_packets(ring_buffer, buffer_length);
        
        // Compact the ring buffer
        if (consumed > 0 && consumed < buffer_length) {
            memmove(ring_buffer, &ring_buffer[consumed], buffer_length - consumed);
            buffer_length -= consumed;
        } else if (consumed >= buffer_length) {
            buffer_length = 0;
        }
    }
}

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

bool ir_handler_init(void) {
    if (ir_system_initialized) {
        ESP_LOGW(TAG, "IR handler already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing IR handler system...");
    
    // Read this device's MAC address
    esp_err_t err = esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MAC address: %s", esp_err_to_name(err));
        return false;
    }
    
    // Initialize RMT for transmission
    if (!setup_rmt_tx()) {
        ESP_LOGE(TAG, "RMT TX setup failed");
        return false;
    }
    
    // Initialize UART for reception
    if (!setup_uart_rx()) {
        ESP_LOGE(TAG, "UART RX setup failed");
        return false;
    }
    
    // Create background tasks
    xTaskCreatePinnedToCore(beacon_task_func, "ir_beacon", 4096, nullptr, 4, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(uart_rx_task_func, "ir_rx", 4096, nullptr, 4, nullptr, tskNO_AFFINITY);
    
    ir_system_initialized = true;
    ESP_LOGI(TAG, "IR handler system initialized successfully");
    return true;
}

void ir_handler_shutdown(void) {
    if (!ir_system_initialized) return;
    
    ESP_LOGI(TAG, "Shutting down IR handler system...");
    
    // Disable reception
    ir_reception_enabled = false;
    
    // Stop and delete timer
    if (beacon_timer) {
        esp_timer_stop(beacon_timer);
        esp_timer_delete(beacon_timer);
        beacon_timer = nullptr;
    }
    
    // Delete tasks
    if (beacon_task) {
        vTaskDelete(beacon_task);
        beacon_task = nullptr;
    }
    if (uart_rx_task) {
        vTaskDelete(uart_rx_task);
        uart_rx_task = nullptr;
    }
    
    // Clean up hardware
    if (rmt_tx) {
        rmt_disable(rmt_tx);
        rmt_del_channel(rmt_tx);
        rmt_tx = nullptr;
    }
    if (tx_copy) {
        rmt_del_encoder(tx_copy);
        tx_copy = nullptr;
    }
    
    // Delete UART driver
    uart_driver_delete(IR_UART_PORT);
    
    ir_system_initialized = false;
    ESP_LOGI(TAG, "IR handler system shut down");
}

void ir_set_reception_enabled(bool enabled) {
    ir_reception_enabled = enabled;
    ESP_LOGI(TAG, "IR reception %s", enabled ? "enabled" : "disabled");
}

void ir_set_rx_callback(ir_rx_callback_t callback) {
    rx_callback = callback;
}

void ir_set_tx_callback(ir_tx_callback_t callback) {
    tx_callback = callback;
}

bool ir_transmit_packet(const ir_packet_t* packet) {
    if (!ir_system_initialized || !packet) {
        return false;
    }
    
    // Validate packet
    if (!ir_validate_packet(packet)) {
        ESP_LOGE(TAG, "Invalid packet for transmission");
        return false;
    }
    
    // Build IR symbols
    rmt_symbol_word_t symbols[IR_SYMBOL_BUFFER_SIZE];
    size_t symbol_count = build_ir_packet(packet, symbols);
    if (symbol_count == 0) {
        ESP_LOGE(TAG, "Failed to build IR packet");
        return false;
    }
    
    // Transmit via RMT
    rmt_transmit_config_t cfg = {
        .loop_count = 0,
        .flags = {0}
    };
    
    esp_err_t err = rmt_transmit(rmt_tx, tx_copy, symbols, symbol_count * sizeof(symbols[0]), &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
        if (tx_callback) {
            tx_callback(packet->sequence, false);
        }
        return false;
    }
    
    // Wait for transmission to complete
    err = rmt_tx_wait_all_done(rmt_tx, pdMS_TO_TICKS(RMT_TX_WAIT_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT wait failed: %s", esp_err_to_name(err));
        if (tx_callback) {
            tx_callback(packet->sequence, false);
        }
        return false;
    }
    
    ESP_LOGI(TAG, "IR packet transmitted (SEQ=%u)", (unsigned)packet->sequence);
    
    if (tx_callback) {
        tx_callback(packet->sequence, true);
    }
    
    return true;
}

uint16_t ir_get_current_sequence(void) {
    return current_sequence;
}

void ir_get_self_mac(uint8_t mac_out[6]) {
    if (mac_out) {
        memcpy(mac_out, self_mac, 6);
    }
}

// ============================================================================
// PACKET UTILITIES
// ============================================================================

void ir_create_packet(ir_packet_t* packet, uint16_t sequence, const uint8_t* payload, uint8_t payload_len) {
    if (!packet) return;
    
    packet->sequence = sequence;
    memcpy(packet->sender_mac, self_mac, 6);
    
    if (payload && payload_len > 0) {
        packet->payload_len = (payload_len > sizeof(packet->payload)) ? sizeof(packet->payload) : payload_len;
        memcpy(packet->payload, payload, packet->payload_len);
    } else {
        packet->payload_len = 0;
    }
}

bool ir_validate_packet(const ir_packet_t* packet) {
    if (!packet) return false;
    
    // Check payload length
    if (packet->payload_len > sizeof(packet->payload)) return false;
    
    // Check MAC address (basic validation)
    bool mac_valid = false;
    for (int i = 0; i < 6; i++) {
        if (packet->sender_mac[i] != 0) {
            mac_valid = true;
            break;
        }
    }
    
    return mac_valid;
}
