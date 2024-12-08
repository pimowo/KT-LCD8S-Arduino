#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Konfiguracja wyświetlacza
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Piny UART do kontrolera
#define CONTROLLER_RX 16
#define CONTROLLER_TX 17

// Twoje stałe ustawienia - zmień według potrzeb
const uint8_t WHEEL_SIZE = 26;      // rozmiar koła w calach
const uint8_t MAX_SPEED = 25;       // limit prędkości km/h
const uint8_t PAS_LEVELS = 5;       // liczba poziomów wspomagania
const uint8_t VOLTAGE = 36;         // napięcie systemu (V)
const bool USE_KMH = true;          // true = km/h, false = mph

// Struktura danych wyświetlacza
struct DisplayData {
    float speed;
    float batteryVoltage;
    uint8_t batteryPercent;
    uint8_t pasLevel;
    float power;
    float distance;
    uint8_t error;
} data;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    // Inicjalizacja UART do kontrolera
    Serial2.begin(1200, SERIAL_8N1, CONTROLLER_RX, CONTROLLER_TX);
    
    // Inicjalizacja OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    display.clearDisplay();
    
    // Wyślij początkową konfigurację do kontrolera
    sendInitialConfig();
}

void loop() {
    static unsigned long lastDisplay = 0;
    
    // Odbieranie danych z kontrolera
    if(Serial2.available()) {
        processControllerData();
    }
    
    // Aktualizacja wyświetlacza co 100ms
    if(millis() - lastDisplay > 100) {
        updateDisplay();
        lastDisplay = millis();
    }
}

void processControllerData() {
    static uint8_t buffer[10];
    static uint8_t bufIndex = 0;
    
    while(Serial2.available()) {
        uint8_t incoming = Serial2.read();
        
        if(bufIndex == 0 && incoming != 0x11) {
            continue;
        }
        
        buffer[bufIndex++] = incoming;
        
        if(bufIndex == 6) {  // Standardowa długość ramki
            if(validateChecksum(buffer)) {
                parseFrame(buffer);
            }
            bufIndex = 0;
        }
    }
}

bool validateChecksum(uint8_t* frame) {
    uint8_t checksum = 0;
    for(int i = 0; i < 5; i++) {
        checksum ^= frame[i];
    }
    return checksum == frame[5];
}

void parseFrame(uint8_t* frame) {
    switch(frame[1]) {
        case 0x01: // Status systemu
            data.batteryVoltage = frame[2] * 0.1;
            data.batteryPercent = frame[3];
            data.error = frame[4];
            break;
            
        case 0x02: // Dane prędkości
            data.speed = frame[2] * 0.1;
            break;
            
        case 0x03: // Dane PAS i mocy
            data.pasLevel = frame[2];
            data.power = frame[3] * 10.0;
            break;
    }
}

void sendInitialConfig() {
    // Ramka konfiguracyjna
    uint8_t config[] = {
        0x11,           // Start byte
        0x51,           // Command: Set config
        0x04,           // Length
        WHEEL_SIZE,     // Wheel size
        MAX_SPEED,      // Speed limit
        PAS_LEVELS,     // PAS levels
        0x00            // Checksum (obliczone poniżej)
    };
    
    // Oblicz sumę kontrolną
    config[6] = config[0] ^ config[1] ^ config[2] ^ config[3] ^ config[4] ^ config[5];
    
    // Wyślij konfigurację
    Serial2.write(config, 7);
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    
    // Prędkość (duże cyfry na górze)
    display.setTextSize(3);
    display.setCursor(0,0);
    display.print(data.speed, 1);
    display.setTextSize(1);
    display.print(USE_KMH ? " KM/H" : " MPH");
    
    // Bateria i PAS (średni rozmiar)
    display.setTextSize(2);
    display.setCursor(0, 32);
    display.print(data.batteryVoltage, 1);
    display.print("V");
    
    // PAS poziom
    display.setCursor(70, 32);
    display.print("P");
    display.print(data.pasLevel);
    
    // Moc na dole
    display.setCursor(0, 48);
    display.print(data.power, 0);
    display.print("W");
    
    // Wyświetl błędy jeśli są
    if(data.error) {
        display.setTextSize(1);
        display.setCursor(70, 54);
        display.print("E:");
        display.print(data.error);
    }
    
    display.display();
}
