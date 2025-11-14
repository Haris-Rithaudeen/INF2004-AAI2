#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

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
#define MQTT_PUB_TOPIC          "robot/sensors"
#define MQTT_SUB_TOPIC          "robot/sensors"
#define MQTT_PING_INTERVAL_MS   3000
#define MQTT_MAX_RECONNECT      3


// -----------------------------
// Public API
// -----------------------------

// Init Wi-Fi station mode and lwIP/CYW43
bool wifi_mqtt_init(void);

// Connect to Wi-Fi (blocking with timeout)
bool wifi_mqtt_connect(void);

// Connect (or reconnect) to MQTT broker; returns true when connected
bool mqtt_connect_broker(void);
bool mqtt_is_connected(void);
bool mqtt_publish_telemetry(float left_speed, float right_speed, 
                            float left_dist, float right_dist,
                            float heading, int16_t accel_x, 
                            int16_t accel_y, int16_t accel_z);
bool mqtt_publish_ping(int count);
bool mqtt_publish_sensors(float ultrasonic_cm, uint16_t ir_line_raw, bool on_line);
bool mqtt_publish_barcode(const char* barcode_value, uint8_t bar_count);

// Poll lwIP/CYW43 + reconnect logic (call frequently in main loop)
void wifi_mqtt_poll(void);

// Graceful cleanup
void wifi_mqtt_deinit(void);

// Connection state
bool mqtt_is_connected(void);

// --------- Publish helpers ---------
bool mqtt_publish_telemetry(float left_speed, float right_speed,
                            float left_dist,  float right_dist,
                            float heading,    int16_t accel_x,
                            int16_t accel_y,  int16_t accel_z);

// Simple “PING #n” demo publisher
bool mqtt_publish_ping(int count);

// General text/bytes publishers
bool mqtt_publish_text(const char *topic, const char *text, int qos, bool retain);
bool mqtt_publish_raw (const char *topic, const uint8_t *bytes, size_t len, int qos, bool retain);

// --------- Subscribe helpers ---------
bool mqtt_subscribe_topic(const char *topic, int qos);

// User message callback: called after a full MQTT payload is received
typedef void (*mqtt_msg_cb_t)(const char *topic, const uint8_t *payload, size_t len);
void mqtt_set_message_cb(mqtt_msg_cb_t cb);

// Add these new publish functions
bool mqtt_publish_imu(float heading, int16_t mx, int16_t my, int16_t mz, 
                     int16_t ax, int16_t ay, int16_t az);
bool mqtt_publish_motors(float left_speed, float right_speed, 
                        float left_dist, float right_dist);
bool mqtt_publish_pid(float kp, float ki, float kd, float error, float output);
bool mqtt_publish_state(const char* state, const char* command);
bool mqtt_publish_obstacle(float distance, int servo_angle, 
                          const char* chosen_path, const char* status);

#endif
