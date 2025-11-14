#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "config.h"
#include "fsm.h"
#include "networking/wifi_mqtt.h"
#include <stdio.h>
#include <math.h>
#include "drivers/encoder.h"

#ifndef CMD_TOPIC
#define CMD_TOPIC "cmd/robot1/#"
#endif

static inline uint32_t now_ms(void){ return to_ms_since_boot(get_absolute_time()); }

// Forward for MQTT callback
static void on_mqtt_msg(const char *topic, const uint8_t *payload, size_t len) {
    // Pass straight into FSM (don’t parse here if you don’t want to)
    fsm_on_cmd(topic, payload, len);
}

int main(void) {
    stdio_init_all();
    sleep_ms(300);

    adc_init();
    fsm_init();

    if (!wifi_mqtt_init())    { printf("[WiFi] init failed\n"); }
    if (!wifi_mqtt_connect()) { printf("[WiFi] connect failed\n"); }

    mqtt_set_message_cb(on_mqtt_msg);
    if (!mqtt_connect_broker()) {
        printf("[MQTT] broker connect failed\n");
        // continue; poller will retry
    }

    bool sub_done = false;
    uint32_t last_pub_ms = 0;
    const uint32_t PUB_PERIOD_MS = 100;

    while (true) {
        // Keep lwIP/CYW43 and MQTT alive
        wifi_mqtt_poll();

        if (mqtt_is_connected() && !sub_done) {
            if (mqtt_subscribe_topic(CMD_TOPIC, 0)) {
                printf("[MQTT] ✓ Subscribed to %s\n", CMD_TOPIC);
                sub_done = true;
            }
        }
        if (!mqtt_is_connected()) sub_done = false;

        uint32_t t = now_ms();
        int leftspd, rightspd, leftdist, rightdist;
        char last_barcode,current_barcode;
        fsm_step(t);

        //Publish to MQTT at fixed rate
        if (mqtt_is_connected() && (t - last_pub_ms >= PUB_PERIOD_MS)) {
            last_pub_ms = t;

            fsm_telemetry_t tm;
            fsm_get_telemetry(&tm);

             //Sensors
            {
                const char *UStopic = "robot/sensors";
                char USmessage[64];
                snprintf(USmessage, sizeof USmessage,
                         "{\"ultrasonic\":%d,\"ir_line\":%d,\"on_line\":%d}",
                         tm.ultra_cm, tm.ir_line_raw, tm.ir_on_line ? 1 : 0);
                mqtt_publish_text(UStopic, USmessage, 0, false);
                wifi_mqtt_poll();
            }

            //Telemetry
            {
                const char *tm_topic = "robot/telemetry";
                char tm_message[64];
                leftspd = tm.v_l_mm_s * 0.1f;
                rightspd = tm.v_r_mm_s * 0.1f;
                leftdist = tm.left_ticks * encoder_mm_per_tick() * 0.1f;
                rightdist = tm.right_ticks * encoder_mm_per_tick() * 0.1f;
                snprintf(tm_message, sizeof tm_message,
                         "{\"ls\":%d,\"rs\":%d,\"ld\":%d,\"rd\":%d}", 
                         leftspd, rightspd, leftdist, rightdist);
                mqtt_publish_text(tm_topic, tm_message, 0, false);
                wifi_mqtt_poll();
            }

            //Compass
            {
                const char *com_topic = "robot/compass";
                char com_message[64];
                int hdg = (isnan(tm.heading_deg) ? -1000 : (int)tm.heading_deg);
                snprintf(com_message, sizeof com_message,
                         "{\"heading\":%d,\"mx\":%d,\"my\":%d,\"mz\":%d}",
                         hdg, (int)tm.mx, (int)tm.my, (int)tm.mz);
                mqtt_publish_text(com_topic, com_message, 0, false);
                wifi_mqtt_poll();
            }
            //Barcode
            {
                const char *bc_topic = "robot/barcode";
                char bc_message[96];
                //last_barcode_char = tm.barcode_char;
                //theres a bug where it keeps sending the same barcode multiple times
                if (last_barcode != tm.barcode_char) {
                    last_barcode = tm.barcode_char;
                    current_barcode = tm.barcode_char;
                }
                else {
                    current_barcode = '\0';
                }
                //char code = (tm.barcode_char ? tm.barcode_char : ' ');
                snprintf(bc_message, sizeof bc_message, "{\"barcode\":\"%c\",\"bars\":%d}", current_barcode, tm.bars_count);
                mqtt_publish_text(bc_topic, bc_message, 0, false);
                wifi_mqtt_poll();
            }
        }
        tight_loop_contents();
    }
}
