// fsm.c — MQTT-only FSM for Section 8 (publish + subscribe proof)
// Requires your updated wifi_mqtt.h/.c wrapper.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "networking/wifi_mqtt.h"
#include "drivers/ultrasonic.h"
#include <math.h>
#include "fsm.h"
#include "config.h"

// ---------- Topics (edit if you like) ----------
#ifndef CMD_TOPIC
#define CMD_TOPIC      "cmd/robot1/#"
#endif


#define PRINT_ON_CONNECT 1
static uint32_t last_hb_ms = 0;
static uint32_t last_print_ms = 0;
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ---------- Inbound message handler ----------
static void on_mqtt_msg(const char *topic, const uint8_t *payload, size_t len) {
    // Print exactly what the report needs to show
    printf("[MQTT<-] topic=%s payload=%.*s\n", topic, (int)len, (const char*)payload);

}

// ---------- FSM lifecycle ----------
void fsm_init(void) {
    stdio_init_all();
    sleep_ms(300); // allow USB CDC to come up
    ultrasonic_init();

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
    uint32_t t = now_ms();
    float cm; 
    if (t - last_print_ms >= (uint32_t)ULTRASONIC_PRINT_INTERVAL_MS) {
        cm = ultrasonic_measure_cm();
        if (isnan(cm)) {
            printf("[ULTRASONIC] Out of range / timeout\n");
        } else {
            printf("[ULTRASONIC] Distance: %.2f cm\n", cm);
        }

        const char *UStopic   = "robot/sensors";
        char USmessage[64];
        snprintf(USmessage, 64, "{\"ultrasonic\":%d,\"ir_line\":%d,\"on_line\":%d}", (int)cm, (int)cm, true);
        mqtt_publish_text(UStopic, USmessage, 0, false);
        //printf("[MQTT->] %s %s\n", UStopic, USmessage);

        const char *tm_topic = "robot/telemetry";
        char tm_message[64];
        //Replace cm with actual telemetry data when available
        snprintf(tm_message, 64, "{\"ls\":%d,\"rs\":%d,\"ld\":%d,\"rd\":%d}", (int)cm,(int)cm,(int)cm,(int)cm);
        mqtt_publish_text(tm_topic, tm_message, 0, false);

        const char *com_topic = "robot/compass";
        char com_message[64];
        //Replace cm with actual compass data when available
        snprintf(com_message, 64, "{\"heading\":%d,\"mx\":%d,\"my\":%d,\"mz\":%d}", (int)cm, (int)cm, (int)cm, (int)cm);
        mqtt_publish_text(com_topic, com_message, 0, false);

        const char *bc_topic = "robot/barcode";
        char bc_message[64];
        //Replace cm with actual barcode data when available
        snprintf(bc_message, 64, "{\"barcode\":\"%c\",\"bars\":%d}", 'A', (int)cm);
        mqtt_publish_text(bc_topic, bc_message, 0, false);

        last_print_ms = t;
    }

}
