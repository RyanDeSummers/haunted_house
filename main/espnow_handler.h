#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// ESP-NOW packet types
typedef enum {
    ESPNOW_IR_HIT_ACK = 0x01,    // Guest acknowledges IR hit
    ESPNOW_HEARTBEAT = 0x02,     // Keep-alive (future use)
    ESPNOW_CUSTOM = 0xFF         // Custom data (future use)
} espnow_packet_type_t;

// ESP-NOW packet structure
typedef struct {
    uint8_t type;                // Packet type
    uint8_t sender_mac[6];       // MAC of sender (for routing)
    uint8_t target_mac[6];       // MAC of intended recipient
    uint8_t sequence;            // Sequence number for deduplication
    uint8_t payload_len;         // Length of payload data
    uint8_t payload[200];        // Actual payload data (max ESP-NOW payload is 250 bytes)
    uint8_t rssi;                // Received signal strength (filled by receiver)
    uint32_t timestamp;          // Timestamp when sent
} espnow_packet_t;

// Callback function type for received ESP-NOW packets
typedef void (*espnow_rx_callback_t)(const espnow_packet_t* packet);

// ESP-NOW handler functions
esp_err_t espnow_handler_init(void);
esp_err_t espnow_handler_deinit(void);
esp_err_t espnow_send_packet(const uint8_t* target_mac, espnow_packet_type_t type, 
                           const uint8_t* payload, uint8_t payload_len);
esp_err_t espnow_send_ir_hit_ack(const uint8_t* target_mac, const uint8_t* guest_mac);
void espnow_set_rx_callback(espnow_rx_callback_t callback);
bool espnow_is_initialized(void);
void espnow_get_self_mac(uint8_t* mac);

// Utility functions
void espnow_print_mac(const char* label, const uint8_t* mac);
bool espnow_mac_equal(const uint8_t* mac1, const uint8_t* mac2);
void espnow_copy_mac(uint8_t* dest, const uint8_t* src);

#ifdef __cplusplus
}
#endif

#endif // ESPNOW_HANDLER_H

