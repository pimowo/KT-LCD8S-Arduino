#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// Konfiguracja wyświetlacza
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Piny UART do kontrolera
#define CONTROLLER_RX 16
#define CONTROLLER_TX 17

// Komendy protokołu
#define CMD_START 0x11
#define CMD_STATUS 0x01
#define CMD_SPEED 0x02
#define CMD_PAS 0x03
#define CMD_CONFIG 0x51

// Struktura konfiguracji KT-LCD8S
struct LCD8SConfig {
    // Parametry główne
    struct {
        uint8_t limitSpeed;    // LIM: Ograniczenie prędkości (km/h)
        uint8_t wheelSize;     // DIM: 5,6,8,10,12,14,16,18,20,22,24,26,27(700c),28,29
        uint8_t units;         // UNT: 0=Km/h Km °C, 1=MPH Mile °C, 2=Km/h Km °F, 3=MPH Mile °F
    } main;

    // Parametry P
    struct {
        uint8_t p1;           // P1: Ilość magnesów * przełożenie
        uint8_t p2;           // P2: 0=bez przekładni, 1=przekładnia, 2-6=multi-magnet
        uint8_t p3;           // P3: 0=speed control, 1=torque sim
        uint8_t p4;           // P4: 0=throttle from 0, 1=PAS start
        uint8_t p5;           // P5: 1=voltage based, 4-11=24V, 5-15=36V, 6-20=48V, 7-30=60V
    } p;

    // Parametry C
    struct {
        uint8_t c1;           // C1: 0-4=right PAS, 5-7=left PAS
        uint8_t c2;           // C2: Phase sampling 0-7
        uint8_t c3;           // C3: Start PAS level 0-8
        uint8_t c4;           // C4: Throttle/PAS behavior 0-4
        uint8_t c5;           // C5: Power limit 0-10
        uint8_t c6;           // C6: Display brightness 1-5
        uint8_t c7;           // C7: Cruise control 0/1
        uint8_t c8;           // C8: Motor temp display 0/1
        uint8_t c9;           // C9: Startup password
        uint8_t c10;          // C10: Factory reset 0/1
        uint8_t c11;          // C11: Service setting
        uint8_t c12;          // C12: Controller LVC adj 0-7
        uint8_t c13;          // C13: Regen brake 0-5
        uint8_t c14;          // C14: PAS sensitivity 1-3
        uint8_t c15;          // C15: Walk assist speed
    } c;

    // Parametry L
    struct {
        uint8_t l1;           // L1: Controller LVC 0-3
        uint8_t l2;           // L2: High speed motor mode 0/1
        uint8_t l3;           // L3: Dual hall mode 0/1
        uint8_t l4;           // L4: Auto power off time (minutes)
    } l;
} config;
