#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdbool.h>
#include <stdint.h>

// Try to include credentials file, otherwise use defaults
#ifdef USE_WIFI_CREDENTIALS_FILE
    #include "../../wifi_credentials.h"
    #define WIFI_SSID       WIFI_SSID_ACTUAL
    #define WIFI_PASSWORD   WIFI_PASSWORD_ACTUAL
    #define MQTT_BROKER_IP  MQTT_BROKER_IP_ACTUAL
#else
    // Fallback defaults (safe placeholders)
    #ifndef WIFI_SSID
    #define WIFI_SSID               "YOUR_SSID"
    #endif

    #ifndef WIFI_PASSWORD
    #define WIFI_PASSWORD           "YOUR_PASSWORD"
    #endif

    #ifndef MQTT_BROKER_IP
    #define MQTT_BROKER_IP          "192.168.1.100"
    #endif
#endif

#define MQTT_BROKER_PORT        1883
#define MQTT_CLIENT_ID          "pico_w_robot"
#define MQTT_PUB_TOPIC          "robot/telemetry"
#define MQTT_SUB_TOPIC          "robot/commands"
#define MQTT_PING_INTERVAL_MS   3000
#define MQTT_MAX_RECONNECT      3

// Function declarations...
bool wifi_mqtt_init(void);
bool wifi_mqtt_connect(void);
bool mqtt_connect_broker(void);
bool mqtt_is_connected(void);
bool mqtt_publish_telemetry(float left_speed, float right_speed, 
                            float left_dist, float right_dist,
                            float heading, int16_t accel_x, 
                            int16_t accel_y, int16_t accel_z);
bool mqtt_publish_ping(int count);
bool mqtt_publish_sensors(float ultrasonic_cm, uint16_t ir_line_raw, bool on_line);
bool mqtt_publish_barcode(const char* barcode_value, uint8_t bar_count);
void wifi_mqtt_poll(void);
void wifi_mqtt_deinit(void);

#endif