#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "networking/wifi_mqtt.h"

// MQTT client instance
static mqtt_client_t *mqtt_client = NULL;
static bool mqtt_connected = false;
static int  mqtt_reconnect_failures = 0;

// User callback (optional)
static mqtt_msg_cb_t user_msg_cb = NULL;

// Buffer to accumulate chunked inbound payloads
#define INBUF_MAX 512
static char     in_topic[128];
static uint8_t  in_buf[INBUF_MAX];
static size_t   in_len = 0;

// ============================================================================
// Internal helpers
// ============================================================================

static void inbuf_reset(void) {
    in_topic[0] = '\0';
    in_len = 0;
}

static void safe_copy_topic(const char *topic) {
    if (!topic) { in_topic[0] = '\0'; return; }
    strncpy(in_topic, topic, sizeof(in_topic) - 1);
    in_topic[sizeof(in_topic) - 1] = '\0';
}

// ============================================================================
// MQTT Callbacks
// ============================================================================

static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] ✓ Connected to broker\n");
        mqtt_connected = true;
        mqtt_reconnect_failures = 0;

        // Subscribe to default command topic (backward-compatible)
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

// Called once at start of an incoming publish; topic is valid here.
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("[MQTT] << topic=%s total_len=%lu\n", topic ? topic : "(null)", (unsigned long)tot_len);
    inbuf_reset();
    safe_copy_topic(topic);
}

// Called for each chunk of data. When flags has MQTT_DATA_FLAG_LAST, payload is complete.
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    if (!data || len == 0) return;

    size_t copy_len = len;
    if (in_len + copy_len > INBUF_MAX) {
        copy_len = (INBUF_MAX > in_len) ? (INBUF_MAX - in_len) : 0;
    }
    if (copy_len) {
        memcpy(in_buf + in_len, data, copy_len);
        in_len += copy_len;
    }

    if (flags & MQTT_DATA_FLAG_LAST) {
        // Ensure NUL-termination if user treats as string (safe)
        if (in_len < INBUF_MAX) {
            in_buf[in_len] = 0;
        } else {
            in_buf[INBUF_MAX - 1] = 0;
        }

        // Print a preview for debugging
        printf("[MQTT] << payload(%zu): ", in_len);
        size_t preview = (in_len < 80) ? in_len : 80;
        for (size_t i = 0; i < preview; i++) putchar((char)in_buf[i]);
        if (in_len > preview) printf("...(+%zu)", in_len - preview);
        printf("\n");

        // Invoke user callback if set
        if (user_msg_cb) {
            user_msg_cb(in_topic, in_buf, in_len);
        }

        // Reset for next message
        inbuf_reset();
    }
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
                            float left_dist,  float right_dist,
                            float heading,    int16_t accel_x,
                            int16_t accel_y,  int16_t accel_z) {
    if (!mqtt_connected) return false;

    // Build compact JSON telemetry
    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer),
        "{\"ls\":%.1f,\"rs\":%.1f,\"ld\":%.1f,\"rd\":%.1f,"
        "\"hd\":%.1f,\"ax\":%d,\"ay\":%d,\"az\":%d}",
        left_speed, right_speed, left_dist, right_dist,
        heading, accel_x, accel_y, accel_z);

    if (len < 0 || len >= (int)sizeof(buffer)) {
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

bool mqtt_publish_text(const char *topic, const char *text, int qos, bool retain) {
    if (!mqtt_connected || !topic || !text) return false;
    err_t err = mqtt_publish(mqtt_client,
                             topic,
                             text,
                             (u16_t)strlen(text),
                             (u8_t)(qos & 0x3),
                             retain ? 1 : 0,
                             mqtt_pub_request_cb,
                             NULL);
    return (err == ERR_OK);
}

bool mqtt_publish_raw(const char *topic, const uint8_t *bytes, size_t len, int qos, bool retain) {
    if (!mqtt_connected || !topic || !bytes || len == 0) return false;
    if (len > 0xFFFF) len = 0xFFFF; // lwIP expects u16_t length
    err_t err = mqtt_publish(mqtt_client,
                             topic,
                             bytes,
                             (u16_t)len,
                             (u8_t)(qos & 0x3),
                             retain ? 1 : 0,
                             mqtt_pub_request_cb,
                             NULL);
    return (err == ERR_OK);
}

bool mqtt_subscribe_topic(const char *topic, int qos) {
    if (!mqtt_connected || !topic) return false;
    err_t err = mqtt_subscribe(mqtt_client, topic, (u8_t)(qos & 0x3), NULL, NULL);
    if (err == ERR_OK) {
        printf("[MQTT] ✓ Subscribed to %s (QoS %d)\n", topic, qos);
        return true;
    } else {
        printf("[MQTT] × Subscribe failed (%s): %d\n", topic, err);
        return false;
    }
}

void mqtt_set_message_cb(mqtt_msg_cb_t cb) {
    user_msg_cb = cb;
}

void wifi_mqtt_poll(void) {
    cyw43_arch_poll();

    // Handle reconnection if disconnected
    if (!mqtt_connected && mqtt_reconnect_failures < MQTT_MAX_RECONNECT) {
        static uint32_t last_try = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_try > 2000) { // retry every ~2s
            printf("[MQTT] ! Attempting reconnect...\n");
            (void)mqtt_connect_broker();
            last_try = now;
        }
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
