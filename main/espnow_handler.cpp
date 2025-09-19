#include "espnow_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <cstring>

static const char* TAG = "ESPNOW_HANDLER";

// ESP-NOW state
static bool s_espnow_initialized = false;
static uint8_t s_self_mac[6] = {0};
static espnow_rx_callback_t s_rx_callback = nullptr;
static uint8_t s_sequence_counter = 0;
static const uint8_t ESPNOW_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Add (or auto-add) the peer before sending
static esp_err_t ensure_peer_added(const uint8_t* mac) {
    if (!mac) return ESP_ERR_INVALID_ARG;
    if (esp_now_is_peer_exist(mac)) return ESP_OK;

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, mac, 6);
    peer.ifidx    = WIFI_IF_STA;        // you start Wi-Fi in STA mode
    peer.channel  = 1;                  // must match on all devices (see step 2)
    peer.encrypt  = false;

    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "📡 Added ESP-NOW peer: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        ESP_LOGW(TAG, "📡 Failed to add ESP-NOW peer: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// ESP-NOW receive callback
static void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
    if (len < (int)sizeof(espnow_packet_t)) {
        ESP_LOGW(TAG, "Received packet too short: %d bytes", len);
        return;
    }
    
    // Copy packet to avoid mutating the RX buffer
    espnow_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    pkt.rssi = recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0;
    
    ESP_LOGI(TAG, "📡 ESP-NOW RX: type=0x%02X, from=%02X:%02X:%02X:%02X:%02X:%02X, RSSI=%d, seq=%d",
             pkt.type,
             pkt.sender_mac[0], pkt.sender_mac[1], pkt.sender_mac[2],
             pkt.sender_mac[3], pkt.sender_mac[4], pkt.sender_mac[5],
             pkt.rssi, pkt.sequence);
    
    // Call user callback if set
    if (s_rx_callback) {
        s_rx_callback(&pkt);
    }
}

// ESP-NOW send callback
static void espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "📡 ESP-NOW TX: SUCCESS to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGW(TAG, "📡 ESP-NOW TX: FAILED to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

esp_err_t espnow_handler_init(void) {
    if (s_espnow_initialized) {
        ESP_LOGW(TAG, "ESP-NOW already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "🚀 Initializing ESP-NOW handler...");
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize WiFi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Pick a channel for your show; 1 is fine if venue Wi-Fi allows
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    // Get our own MAC address
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_self_mac));
    ESP_LOGI(TAG, "📱 Self MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_self_mac[0], s_self_mac[1], s_self_mac[2],
             s_self_mac[3], s_self_mac[4], s_self_mac[5]);
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    
    // Set primary master key (optional, for encryption)
    // ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"pmk1234567890123"));
    
    s_espnow_initialized = true;
    ESP_LOGI(TAG, "✅ ESP-NOW handler initialized successfully");
    
    return ESP_OK;
}

esp_err_t espnow_handler_deinit(void) {
    if (!s_espnow_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "🛑 Deinitializing ESP-NOW handler...");
    
    esp_err_t ret = esp_now_deinit();
    if (ret == ESP_OK) {
        s_espnow_initialized = false;
        s_rx_callback = nullptr;
        ESP_LOGI(TAG, "✅ ESP-NOW handler deinitialized");
    } else {
        ESP_LOGE(TAG, "❌ Failed to deinitialize ESP-NOW: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t espnow_send_packet(const uint8_t* target_mac, espnow_packet_type_t type, 
                           const uint8_t* payload, uint8_t payload_len) {
    if (!s_espnow_initialized) {
        ESP_LOGE(TAG, "ESP-NOW not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (payload_len > 200) {
        ESP_LOGE(TAG, "Payload too large: %d bytes (max 200)", payload_len);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Build packet
    espnow_packet_t packet = {};
    packet.type = (uint8_t)type;
    espnow_copy_mac(packet.sender_mac, s_self_mac);
    espnow_copy_mac(packet.target_mac, target_mac);
    packet.sequence = s_sequence_counter++;
    packet.payload_len = payload_len;
    packet.timestamp = (uint32_t)(esp_timer_get_time() / 1000); // ms
    
    if (payload && payload_len > 0) {
        memcpy(packet.payload, payload, payload_len);
    }
    
    ESP_LOGI(TAG, "📡 ESP-NOW TX: type=0x%02X, to=%02X:%02X:%02X:%02X:%02X:%02X, payload_len=%d, seq=%d",
             packet.type,
             packet.target_mac[0], packet.target_mac[1], packet.target_mac[2],
             packet.target_mac[3], packet.target_mac[4], packet.target_mac[5],
             packet.payload_len, packet.sequence);
    
    // Ensure peer is added before sending
    esp_err_t add_ret = ensure_peer_added(target_mac);
    if (add_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ESP-NOW peer %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 target_mac[0], target_mac[1], target_mac[2],
                 target_mac[3], target_mac[4], target_mac[5],
                 esp_err_to_name(add_ret));
        return add_ret;
    }
    
    // Send packet
    esp_err_t ret = esp_now_send(target_mac, (uint8_t*)&packet, sizeof(packet));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send ESP-NOW packet: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t espnow_send_ir_hit_ack(const uint8_t* target_mac, const uint8_t* guest_mac) {
    if (!target_mac || !guest_mac) {
        ESP_LOGE(TAG, "Invalid MAC addresses");
        return ESP_ERR_INVALID_ARG;
    }

    // If target MAC looks wrong (all zeros) or equals my own, fallback to broadcast
    bool invalid = true;
    for (int i=0;i<6;i++) if (target_mac[i] != 0x00) { invalid = false; break; }
    bool to_self = espnow_mac_equal(target_mac, s_self_mac);
    const uint8_t* dest = (invalid || to_self) ? ESPNOW_BROADCAST : target_mac;

    ESP_LOGI(TAG, "🎯 Sending IR hit ACK to %s: %02X:%02X:%02X:%02X:%02X:%02X",
             (dest==ESPNOW_BROADCAST) ? "BROADCAST" : "ACTOR",
             dest[0],dest[1],dest[2],dest[3],dest[4],dest[5]);

    // Create payload with guest MAC for actor to know who hit them
    uint8_t payload[6];
    espnow_copy_mac(payload, guest_mac);
    
    return espnow_send_packet(dest, ESPNOW_IR_HIT_ACK, payload, sizeof(payload));
}

void espnow_set_rx_callback(espnow_rx_callback_t callback) {
    s_rx_callback = callback;
    ESP_LOGI(TAG, "📡 ESP-NOW RX callback %s", callback ? "set" : "cleared");
}

bool espnow_is_initialized(void) {
    return s_espnow_initialized;
}

void espnow_get_self_mac(uint8_t* mac) {
    if (mac) {
        espnow_copy_mac(mac, s_self_mac);
    }
}

// Utility functions
void espnow_print_mac(const char* label, const uint8_t* mac) {
    if (label && mac) {
        ESP_LOGI(TAG, "%s: %02X:%02X:%02X:%02X:%02X:%02X",
                 label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

bool espnow_mac_equal(const uint8_t* mac1, const uint8_t* mac2) {
    if (!mac1 || !mac2) return false;
    return memcmp(mac1, mac2, 6) == 0;
}

void espnow_copy_mac(uint8_t* dest, const uint8_t* src) {
    if (dest && src) {
        memcpy(dest, src, 6);
    }
}
