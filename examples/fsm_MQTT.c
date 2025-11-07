// fsm.c — MQTT-only FSM for Section 8 (publish + subscribe proof)
// Requires your updated wifi_mqtt.h/.c wrapper.

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "networking/wifi_mqtt.h"

// ---------- Topics (edit if you like) ----------
#ifndef CMD_TOPIC
#define CMD_TOPIC      "cmd/robot1/#"
#endif
#ifndef HEARTBEAT_TOPIC
#define HEARTBEAT_TOPIC "tele/robot1/heartbeat"
#endif
#ifndef REPLY_TOPIC
#define REPLY_TOPIC     "tele/robot1/reply"
#endif

// ---------- Timings ----------
#define HEARTBEAT_MS     1000u     // 1 Hz
#define PRINT_ON_CONNECT 1

// ---------- State ----------
static uint32_t last_hb_ms = 0;

// Utility
static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

// ---------- Inbound message handler ----------
static void on_mqtt_msg(const char *topic, const uint8_t *payload, size_t len) {
    // Print exactly what the report needs to show
    printf("[MQTT<-] topic=%s payload=%.*s\n", topic, (int)len, (const char*)payload);

    // Minimal command demo: ping -> pong
    if (len == 4 && memcmp(payload, "ping", 4) == 0) {
        mqtt_publish_text(REPLY_TOPIC, "pong", 0, false);
        printf("[MQTT->] %s pong\n", REPLY_TOPIC);
    }
}

// ---------- FSM lifecycle ----------
void fsm_init(void) {
    stdio_init_all();
    sleep_ms(300); // allow USB CDC to come up

    printf("\n=== MQTT Demo (Publish 1 Hz heartbeat, Subscribe echo) ===\n");

    if (!wifi_mqtt_init()) {
        printf("[WiFi] init failed\n");
        return;
    }
    if (!wifi_mqtt_connect()) {
        printf("[WiFi] connect failed (check SSID/password)\n");
        return;
    }

    mqtt_set_message_cb(on_mqtt_msg);

    if (!mqtt_connect_broker()) {
        printf("[MQTT] broker connect failed (check broker IP/port)\n");
        // wifi_mqtt_poll() will attempt reconnects; continue
    }

    // Subscribe to commands
    if (mqtt_is_connected()) {
        mqtt_subscribe_topic(CMD_TOPIC, 0);
    }

#if PRINT_ON_CONNECT
    if (mqtt_is_connected()) {
        printf("[MQTT] ✓ Connected & subscribed to %s\n", CMD_TOPIC);
    }
#endif

    last_hb_ms = now_ms();
}

void fsm_step(void) {
    // keep lwIP/CYW43 and MQTT client alive + handle reconnects
    wifi_mqtt_poll();

    // (Re)subscribe once connected after a reconnect
    static bool sub_done = false;
    if (mqtt_is_connected() && !sub_done) {
        if (mqtt_subscribe_topic(CMD_TOPIC, 0)) {
            printf("[MQTT] ✓ Subscribed to %s\n", CMD_TOPIC);
            sub_done = true;
        }
    }
    if (!mqtt_is_connected()) {
        sub_done = false; // force resubscribe after reconnection
        return;
    }

    // 1 Hz heartbeat
    uint32_t t = now_ms();
    if (t - last_hb_ms >= HEARTBEAT_MS) {
        char payload[96];
        // Keep JSON compact but extensible for your report
        snprintf(payload, sizeof(payload),
                 "{\"uptime_ms\":%u,\"status\":\"ok\"}", t);
        mqtt_publish_text(HEARTBEAT_TOPIC, payload, 0, false);
        printf("[MQTT->] %s %s\n", HEARTBEAT_TOPIC, payload);
        last_hb_ms = t;
    }
}
