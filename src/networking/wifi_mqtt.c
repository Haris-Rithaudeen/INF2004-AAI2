#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "networking/wifi_mqtt.h"

// MQTT client instance
static mqtt_client_t *mqtt_client = NULL;
static bool mqtt_connected = false;
static int mqtt_reconnect_failures = 0;

// ============================================================================
// MQTT Callbacks
// ============================================================================

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, 
                               mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] ✓ Connected to broker\n");
        mqtt_connected = true;
        mqtt_reconnect_failures = 0;
        
        // Subscribe to command topic
        err_t err = mqtt_subscribe(client, MQTT_SUB_TOPIC, 0, NULL, NULL);
        if (err == ERR_OK) {
            printf("[MQTT] ✓ Subscribed to %s\n", MQTT_SUB_TOPIC);
        } else {
            printf("[MQTT] × Subscribe failed: %d\n", err);
        }
    } else {
        printf("[MQTT] × Connection failed, status: %d\n", status);
        mqtt_connected = false;
        mqtt_reconnect_failures++;
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("[MQTT] << Message on topic: %s (length: %lu)\n", topic, tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("[MQTT] << Data received: ");
    for (int i = 0; i < len; i++) {
        putchar(data[i]);
    }
    printf("\n");
}

static void mqtt_pub_request_cb(void *arg, err_t result) {
    if (result != ERR_OK) {
        printf("[MQTT] × Publish failed: %d\n", result);
    }
}

// ============================================================================
// Public Functions
// ============================================================================

bool wifi_mqtt_init(void) {
    printf("[WiFi] Initializing CYW43 chip...\n");
    
    if (cyw43_arch_init()) {
        printf("[WiFi] × CYW43 init failed\n");
        return false;
    }
    
    cyw43_arch_enable_sta_mode();
    printf("[WiFi] ✓ CYW43 chip initialized\n");
    return true;
}

bool wifi_mqtt_connect(void) {
    printf("[WiFi] → Connecting to SSID: %s\n", WIFI_SSID);
    
    int result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, 
        WIFI_PASSWORD, 
        CYW43_AUTH_WPA2_AES_PSK, 
        30000
    );
    
    if (result != 0) {
        printf("[WiFi] × Connection failed (code: %d)\n", result);
        printf("[WiFi] Check: SSID, password, 2.4GHz network, WPA2\n");
        return false;
    }
    
    printf("[WiFi] ✓ Connected successfully\n");
    return true;
}

bool mqtt_connect_broker(void) {
    if (mqtt_client != NULL) {
        mqtt_disconnect(mqtt_client);
        mqtt_client_free(mqtt_client);
    }
    
    mqtt_client = mqtt_client_new();
    if (!mqtt_client) {
        printf("[MQTT] × Failed to create MQTT client\n");
        return false;
    }
    
    // Setup MQTT client info
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;
    ci.keep_alive = 60;
    
    // Set callbacks
    mqtt_set_inpub_callback(mqtt_client, 
                           mqtt_incoming_publish_cb, 
                           mqtt_incoming_data_cb, 
                           NULL);
    
    // Parse broker IP
    ip_addr_t broker_ip;
    if (!ip4addr_aton(MQTT_BROKER_IP, &broker_ip)) {
        printf("[MQTT] × Invalid broker IP: %s\n", MQTT_BROKER_IP);
        return false;
    }
    
    printf("[MQTT] → Connecting to broker %s:%d\n", 
           MQTT_BROKER_IP, MQTT_BROKER_PORT);
    
    // Connect to broker
    err_t err = mqtt_client_connect(mqtt_client, 
                                    &broker_ip, 
                                    MQTT_BROKER_PORT, 
                                    mqtt_connection_cb, 
                                    NULL, 
                                    &ci);
    
    if (err != ERR_OK) {
        printf("[MQTT] × mqtt_client_connect error: %d\n", err);
        return false;
    }
    
    // Wait for connection (with timeout)
    int timeout = 100; // ~10 seconds
    while (!mqtt_connected && timeout-- > 0) {
        cyw43_arch_poll();
        sleep_ms(100);
    }
    
    return mqtt_connected;
}

bool mqtt_is_connected(void) {
    return mqtt_connected;
}

bool mqtt_publish_telemetry(float left_speed, float right_speed, 
                            float left_dist, float right_dist,
                            float heading, int16_t accel_x, 
                            int16_t accel_y, int16_t accel_z) {
    if (!mqtt_connected) {
        return false;
    }
    
    // Build compact JSON telemetry
    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer),
        "{\"ls\":%.1f,\"rs\":%.1f,\"ld\":%.1f,\"rd\":%.1f,"
        "\"hd\":%.1f,\"ax\":%d,\"ay\":%d,\"az\":%d}",
        left_speed, right_speed, left_dist, right_dist,
        heading, accel_x, accel_y, accel_z);
    
    if (len < 0 || len >= sizeof(buffer)) {
        printf("[MQTT] × Telemetry format error\n");
        return false;
    }
    
    err_t err = mqtt_publish(mqtt_client, 
                            MQTT_PUB_TOPIC, 
                            buffer, 
                            (u16_t)len, 
                            0,  // QoS 0
                            0,  // Not retained
                            mqtt_pub_request_cb, 
                            NULL);
    
    return (err == ERR_OK);
}

bool mqtt_publish_ping(int count) {
    if (!mqtt_connected) {
        printf("[MQTT] × Not connected, cannot publish ping\n");
        return false;
    }
    
    char message[64];
    snprintf(message, sizeof(message), "PING #%d from Pico W", count);
    
    printf("[MQTT] >> Publishing ping #%d\n", count);
    
    err_t err = mqtt_publish(mqtt_client, 
                            "robot/ping", 
                            message, 
                            strlen(message), 
                            0, 
                            0, 
                            mqtt_pub_request_cb, 
                            NULL);
    
    return (err == ERR_OK);
}

void wifi_mqtt_poll(void) {
    cyw43_arch_poll();
    
    // Handle reconnection if disconnected
    if (!mqtt_connected && mqtt_reconnect_failures < MQTT_MAX_RECONNECT) {
        printf("[MQTT] ! Attempting reconnect...\n");
        mqtt_connect_broker();
    }
}

void wifi_mqtt_deinit(void) {
    if (mqtt_client) {
        mqtt_disconnect(mqtt_client);
        mqtt_client_free(mqtt_client);
        mqtt_client = NULL;
    }
    
    cyw43_arch_deinit();
    mqtt_connected = false;
}

bool mqtt_publish_sensors(float ultrasonic_cm, uint16_t ir_line_raw,bool on_line) {
    if (!mqtt_connected) return false;
    char buffer[128];
    int len = snprintf(buffer, sizeof(buffer),
        "{\"ultrasonic\":%.2f,\"ir_line\":%u,\"on_line\":%d}",
        ultrasonic_cm, ir_line_raw, on_line ? 1 : 0);
    
    if (len < 0 || len >= sizeof(buffer)) return false;
    
    err_t err = mqtt_publish(mqtt_client, 
                            "robot/sensors", 
                            buffer, (u16_t)len, 
                            0, 0, mqtt_pub_request_cb, NULL);
    return (err == ERR_OK);
}
bool mqtt_publish_barcode(const char* barcode_value, 
                          uint8_t bar_count) {
    if (!mqtt_connected) return false;
    
    char buffer[128];
    int len = snprintf(buffer, sizeof(buffer),
        "{\"barcode\":\"%s\",\"bars\":%u}",
        barcode_value, bar_count);
    
    if (len < 0 || len >= sizeof(buffer)) return false;
    
    err_t err = mqtt_publish(mqtt_client, 
                            "robot/barcode", 
                            buffer, (u16_t)len, 
                            0, 0, mqtt_pub_request_cb, NULL);
    return (err == ERR_OK);
}