#pragma once
// ==== Motor pins ====
#define M1_A 8
#define M1_B 9
#define M2_A 10
#define M2_B 11

// ==== Encoder pins ====
#define LEFT_ENCODER_PIN   16
#define RIGHT_ENCODER_PIN  26

// ==== Button pins ====
#define BUTTON_DIR    21
#define BUTTON_SPD    20

// ==== IR Sensor 1: Line Follower ====
#define IR_LINE_ADC_GPIO       28
#define IR_LINE_ADC_CHANNEL    2
#define IR_LINE_DIGITAL_GPIO    7

// ==== IR Sensor 2: Barcode Scanner ====
#define IR_BARCODE_ADC_GPIO    27
#define IR_BARCODE_ADC_CHANNEL  1
#define IR_BARCODE_DIGITAL_GPIO 6

// ==== Ultrasonic Sensor ====
#define ULTRASONIC_TRIG_GPIO   5
#define ULTRASONIC_ECHO_GPIO   4

// ==== Servo ====
#define SERVO_GPIO            15

// ==== I2C (LSM303) ====
#define I2C_PORT      i2c1
#define I2C_SDA_PIN   2
#define I2C_SCL_PIN   3
#define I2C_BAUDRATE  100000
