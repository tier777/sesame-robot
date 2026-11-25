// Board settings: the only file to change when the robot moves to another ESP32 board.
// PCA9685 (0x40) and the OLED (0x3C) share one I2C bus on these two pins.
#pragma once

// Classic ESP32 DevKit (30 or 38 pins)
#define QUAD_SDA 21
#define QUAD_SCL 22

// ESP32-C3 SuperMini:          #define QUAD_SDA 8   #define QUAD_SCL 9
// ESP32-S3 (DevKit, Zero):     #define QUAD_SDA 8   #define QUAD_SCL 9
// Cardputer ADV, Grove port:   #define QUAD_SDA 2   #define QUAD_SCL 1

// Wi-Fi access point the phone connects to; the page is at http://192.168.4.1
#define QUAD_AP_SSID "spider-quad"
#define QUAD_AP_PASS "spiderquad"  // at least 8 characters
