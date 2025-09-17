#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/rmt_tx.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// IR HANDLER CONFIGURATION
// ============================================================================

// Hardware pin assignments
#define IR_TX_GPIO   GPIO_NUM_26   // IR LED
#define IR_RX_GPIO   GPIO_NUM_36   // IR Receiver  
#define IR_UART_PORT UART_NUM_2    // UART Peripheral (IR Receiver)

// IR Protocol Configuration
#define IR_BAUD         2400
#define IR_BIT_US       (1000000 / IR_BAUD)   // 416 microseconds at 2400 bps
#define IR_PREAMBLE0    0x5A
#define IR_PREAMBLE1    0x54
#define IR_MAC_LEN      6

// RMT Configuration
#define RMT_RESOLUTION_HZ 1000000            // 1MHz = 1us per tick
#define RMT_TX_WAIT_TIMEOUT_MS 1000         // Timeout for RMT transmission
#define RMT_TX_QUEUE_DEPTH 4                 // RMT transmit queue depth
#define RMT_TX_INTR_PRIORITY 0              // RMT interrupt priority

// IR Carrier Configuration
#define IR_CARRIER_FREQ_HZ 38000            // IR carrier frequency (38kHz)
#define IR_CARRIER_DUTY_CYCLE 0.33          // IR carrier duty cycle (33%)

// Buffer and Memory Configuration
#define RMT_MEM_BLOCK_SYMBOLS 128           // RMT memory block size
#define IR_SYMBOL_BUFFER_SIZE 200           // Maximum IR symbols per packet
#define UART_RING_BUFFER_SIZE 256           // Ring buffer size for UART reception
#define UART_READ_BUFFER_SIZE 128           // Single read buffer size
#define UART_READ_TIMEOUT_MS 20             // Timeout for UART read operations
#define UART_TASK_DELAY_MS 10               // Delay between UART task iterations

// Packet Structure
#define PACKET_HEADER_SIZE 3                // Preamble(2) + Length(1)
#define PACKET_SEQ_SIZE 2                   // Sequence number (2 bytes)
#define PACKET_CRC_SIZE 1                   // CRC-8 checksum (1 byte)
#define PACKET_TOTAL_OVERHEAD (PACKET_HEADER_SIZE + PACKET_SEQ_SIZE + PACKET_CRC_SIZE)
#define EXPECTED_PACKET_LEN (IR_MAC_LEN + PACKET_SEQ_SIZE)  // MAC(6) + SEQ(2)
#define IR_MAX_PAYLOAD 8                    // Maximum payload size (matches struct)

// ============================================================================
// IR PACKET STRUCTURE
// ============================================================================

typedef struct {
    uint16_t sequence;        // Sequence number
    uint8_t sender_mac[6];    // MAC address of sender
    uint8_t payload[8];       // Optional payload data (for future expansion)
    uint8_t payload_len;      // Length of payload data
} ir_packet_t;

// ============================================================================
// IR EVENT CALLBACKS
// ============================================================================

/**
 * @brief Callback function for when a valid IR packet is received
 * 
 * @param packet Pointer to the received IR packet
 * @param rssi Signal strength indicator (0-255, higher = stronger)
 * 
 * This callback is called when a valid IR packet is received and parsed.
 * The packet contains the sequence number, sender MAC, and any payload data.
 */
typedef void (*ir_rx_callback_t)(const ir_packet_t* packet, uint8_t rssi);

/**
 * @brief Callback function for IR transmission completion
 * 
 * @param sequence Sequence number of the transmitted packet
 * @param success true if transmission was successful, false if failed
 * 
 * This callback is called when an IR transmission attempt completes.
 */
typedef void (*ir_tx_callback_t)(uint16_t sequence, bool success);

// ============================================================================
// IR HANDLER API
// ============================================================================

/**
 * @brief Initializes the IR handler system
 * 
 * @return true if initialization successful, false otherwise
 * 
 * This function initializes both IR transmission (RMT) and reception (UART)
 * subsystems. Must be called before any other IR handler functions.
 */
bool ir_handler_init(void);

/**
 * @brief Shuts down the IR handler system
 * 
 * Cleans up all IR handler resources including tasks, timers, and hardware.
 */
void ir_handler_shutdown(void);

/**
 * @brief Enables or disables IR reception
 * 
 * @param enabled true to enable IR reception, false to disable
 * 
 * When enabled, the device will listen for IR packets and call the RX callback.
 * When disabled, IR reception is stopped to save power.
 */
void ir_set_reception_enabled(bool enabled);

/**
 * @brief Sets the IR reception callback function
 * 
 * @param callback Function to call when valid IR packets are received
 * 
 * The callback will be called from the IR reception task context.
 * Set to NULL to disable RX callbacks.
 */
void ir_set_rx_callback(ir_rx_callback_t callback);

/**
 * @brief Sets the IR transmission callback function
 * 
 * @param callback Function to call when IR transmissions complete
 * 
 * The callback will be called from the IR transmission context.
 * Set to NULL to disable TX callbacks.
 */
void ir_set_tx_callback(ir_tx_callback_t callback);

/**
 * @brief Transmits an IR packet
 * 
 * @param packet Pointer to the packet to transmit
 * @return true if packet queued for transmission, false if queue full
 * 
 * This function queues an IR packet for transmission. The actual transmission
 * happens asynchronously. The TX callback will be called when transmission completes.
 */
bool ir_transmit_packet(const ir_packet_t* packet);

/**
 * @brief Gets the current sequence number for transmissions
 * 
 * @return Current sequence number (auto-increments with each transmission)
 */
uint16_t ir_get_current_sequence(void);

/**
 * @brief Gets this device's MAC address
 * 
 * @param mac_out Output buffer for MAC address (6 bytes)
 */
void ir_get_self_mac(uint8_t mac_out[6]);

// ============================================================================
// IR PACKET UTILITIES
// ============================================================================

/**
 * @brief Creates an IR packet with basic information
 * 
 * @param packet Output packet structure to fill
 * @param sequence Sequence number for this packet
 * @param payload Optional payload data (can be NULL)
 * @param payload_len Length of payload data (0 if no payload)
 */
void ir_create_packet(ir_packet_t* packet, uint16_t sequence, const uint8_t* payload, uint8_t payload_len);

/**
 * @brief Validates an IR packet structure
 * 
 * @param packet Pointer to packet to validate
 * @return true if packet is valid, false otherwise
 */
bool ir_validate_packet(const ir_packet_t* packet);

#ifdef __cplusplus
}
#endif

