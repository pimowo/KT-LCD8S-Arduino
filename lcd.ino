#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// Tryb debug
#define DEBUG

// Konfiguracja wyświetlacza
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Piny UART do kontrolera
#define CONTROLLER_RX 16
#define CONTROLLER_TX 17

// Parametry protokołu
#define FRAME_MAX_LENGTH 12
#define FRAME_MIN_LENGTH 4   // start + cmd + len + checksum
#define FRAME_TIMEOUT 100    // ms

// Komendy protokołu
#define CMD_START 0x11
#define CMD_STATUS 0x01
#define CMD_SPEED 0x02
#define CMD_PAS 0x03
#define CMD_CONFIG 0x51

// Struktura ramki danych
struct ControllerFrame {
    uint8_t start;      // Zawsze 0x11
    uint8_t command;    // Typ komendy
    uint8_t data[8];    // Dane (max 8 bajtów)
    uint8_t checksum;   // XOR wszystkich bajtów
};

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

// Struktura danych wyświetlacza
struct DisplayData {
    float speed;
    float batteryVoltage;
    uint8_t batteryPercent;
    uint8_t pasLevel;
    float power;
    float distance;
    float temperature;
    uint8_t error;
    bool cruise;
    bool walk;
    unsigned long lastUpdate;  // Timestamp ostatniej aktualizacji
} data;

// Inicjalizacja wyświetlacza
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Funkcje debugowania
#ifdef DEBUG
void debugPrint(const char* msg) {
    Serial.println(msg);
}

void debugFrame(uint8_t* frame, uint8_t length) {
    Serial.print("Frame: ");
    for (uint8_t i = 0; i < length; i++) {
        if (frame[i] < 0x10) Serial.print("0");
        Serial.print(frame[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void debugData() {
    Serial.print("Speed: "); Serial.print(data.speed);
    Serial.print(" Battery: "); Serial.print(data.batteryVoltage);
    Serial.print("V "); Serial.print(data.batteryPercent);
    Serial.print("% PAS: "); Serial.print(data.pasLevel);
    Serial.print(" Power: "); Serial.print(data.power);
    Serial.print("W Temp: "); Serial.print(data.temperature);
    Serial.println("°C");
}
#else
#define debugPrint(msg)
#define debugFrame(frame, length)
#define debugData()
#endif

// Funkcje konfiguracji
void loadDefaultConfig() {
    // Główne parametry
    config.main.limitSpeed = 25;    // Ograniczenie do 25 km/h (tryb legal)
    config.main.wheelSize = 27;     // 700C
    config.main.units = 0;          // Km/h

    // Parametry P
    config.p.p1 = 96;              // Mxus FX15/XF08/XP03/XP04
    config.p.p2 = 1;               // Silnik przekładniowy
    config.p.p3 = 1;               // Symulacja momentu
    config.p.p4 = 0;               // Start od 0
    config.p.p5 = 1;               // Pomiar napięcia na bieżąco

    // Parametry C
    config.c.c1 = 2;               // Prawy PAS
    config.c.c2 = 0;               // Domyślne próbkowanie
    config.c.c3 = 8;               // Ostatni używany poziom
    config.c.c4 = 3;               // Pełna kontrola manetki
    config.c.c5 = 10;              // 100% mocy
    config.c.c6 = 5;               // Max jasność
    config.c.c7 = 1;               // Tempomat włączony
    config.c.c8 = 0;               // Bez pomiaru temperatury
    config.c.c9 = 0;               // Bez hasła
    config.c.c10 = 0;              // Bez resetu
    config.c.c11 = 0;              // Domyślne serwisowe
    config.c.c12 = 4;              // Domyślne LVC
    config.c.c13 = 0;              // Bez rekuperacji
    config.c.c14 = 1;              // Niskie wspomaganie
    config.c.c15 = 6;              // 6 km/h walk

    // Parametry L
    config.l.l1 = 0;               // Fabryczne LVC
    config.l.l2 = 0;               // Normalny silnik
    config.l.l3 = 1;               // Zabezpieczenie hall
    config.l.l4 = 5;               // 5 min auto-off
    
    debugPrint("Default config loaded");
}

void saveConfig() {
    EEPROM.put(0, config);
    debugPrint("Config saved to EEPROM");
}

void loadConfig() {
    EEPROM.get(0, config);
    debugPrint("Config loaded from EEPROM");
}

// Funkcje komunikacji z kontrolerem
void sendConfigFrame(uint8_t cmd, ...) {
    uint8_t buffer[FRAME_MAX_LENGTH];
    va_list args;
    va_start(args, cmd);
    
    buffer[0] = CMD_START;    // Start byte
    buffer[1] = cmd;          // Command
    
    uint8_t len = 0;
    uint8_t checksum = buffer[0] ^ buffer[1];
    
    // Dodaj parametry do ramki
    while(len < 8) {  // Max 8 bajtów danych
        uint8_t param = va_arg(args, int);  // va_arg używa int dla uint8_t
        buffer[2 + len] = param;
        checksum ^= param;
        len++;
        
        if(va_arg(args, void*) == NULL) break;
    }
    
    buffer[2 + len] = checksum;
    Serial2.write(buffer, 3 + len);
    
    debugFrame(buffer, 3 + len);
    va_end(args);
}

void sendFullConfig() {
    debugPrint("Sending full config to controller...");
    
    // Główne parametry
    sendConfigFrame(0x51, config.main.limitSpeed, config.main.wheelSize, config.main.units);
    delay(50);  // Daj czas kontrolerowi na przetworzenie
    
    // Parametry P
    sendConfigFrame(0x52, config.p.p1, config.p.p2, config.p.p3, config.p.p4, config.p.p5);
    delay(50);
    
    // Parametry C
    for(uint8_t i = 0; i < 15; i++) {
        uint8_t* cParams = (uint8_t*)&config.c;
        sendConfigFrame(0x53 + i, cParams[i]);
        delay(50);
    }
    
    // Parametry L
    sendConfigFrame(0x54, config.l.l1, config.l.l2, config.l.l3, config.l.l4);
    
    debugPrint("Config sent successfully");
}

bool parseFrame(uint8_t* frame, uint8_t length) {
    if (length < FRAME_MIN_LENGTH || length > FRAME_MAX_LENGTH) {
        debugPrint("Invalid frame length");
        return false;
    }
    
    // Obliczamy sumę kontrolną
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length - 1; i++) {
        checksum ^= frame[i];
    }
    
    // Sprawdzamy czy suma kontrolna się zgadza
    if (checksum != frame[length - 1]) {
        debugPrint("Checksum error");
        return false;
    }
    
    return true;
}

void updateDataFromFrame(uint8_t* frame) {
    switch (frame[1]) {  // Sprawdzamy typ komendy
        case CMD_STATUS:
            data.batteryVoltage = frame[2] / 10.0;  // Napięcie w V
            data.batteryPercent = frame[3];         // Procent naładowania
            data.temperature = frame[4];            // Temperatura
            data.error = frame[5];                  // Kod błędu
            data.cruise = frame[6] & 0x01;          // Status tempomatu
            data.walk = frame[6] & 0x02;            // Status wspomagania
            break;
            
        case CMD_SPEED:
            data.speed = (frame[2] | (frame[3] << 8)) / 10.0;  // Prędkość w km/h
            data.power = frame[4] | (frame[5] << 8);           // Moc w W
            break;
            
        case CMD_PAS:
            data.pasLevel = frame[2];  // Poziom PAS
            data.distance = (frame[3] | (frame[4] << 8) | (frame[5] << 16)) / 1000.0;  // Dystans w km
            break;
    }
    
    data.lastUpdate = millis();
    debugData();
}

void processControllerData() {
    static uint8_t buffer[FRAME_MAX_LENGTH];
    static uint8_t bufferIndex = 0;
    static unsigned long lastByteTime = 0;
    
    // Reset bufora jeśli minął timeout
    if (bufferIndex > 0 && millis() - lastByteTime > FRAME_TIMEOUT) {
        debugPrint("Frame timeout - resetting buffer");
        bufferIndex = 0;
    }
    
    while (Serial2.available()) {
        uint8_t byte = Serial2.read();
        lastByteTime = millis();
        
        // Szukamy początku nowej ramki
        if (bufferIndex == 0 && byte == CMD_START) {
            buffer[bufferIndex++] = byte;
            continue;
        }
        
        // Zapisujemy kolejne bajty
        if (bufferIndex > 0) {
            buffer[bufferIndex++] = byte;
            
            // Sprawdzamy czy mamy kompletną ramkę
            if (bufferIndex > 2 && bufferIndex == buffer[2] + 4) {
                debugFrame(buffer, bufferIndex);
                
                if (parseFrame(buffer, bufferIndex)) {
                    updateDataFromFrame(buffer);
                }
                bufferIndex = 0;  // Reset dla następnej ramki
            }
            
            // Zabezpieczenie przed przepełnieniem bufora
            if (bufferIndex >= FRAME_MAX_LENGTH) {
                debugPrint("Buffer overflow - resetting");
                bufferIndex = 0;
            }
        }
    }
}

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);  // Port debug
    debugPrint("KT-LCD8S Arduino Controller starting...");
    #endif
    
    // Inicjalizacja UART do kontrolera
    Serial2.begin(1200, SERIAL_8N1, CONTROLLER_RX, CONTROLLER_TX);
    debugPrint("Controller UART initialized");
    
    // Inicjalizacja OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        debugPrint("SSD1306 allocation failed");
        for(;;);  // Nie kontynuuj jeśli wyświetlacz nie działa
    }
    display.clearDisplay();
    debugPrint("Display initialized");
    
    // Wczytaj konfigurację lub ustaw domyślną
    if(EEPROM.read(0) == 0xFF) {
        debugPrint("No config found in EEPROM");
        loadDefaultConfig();
        saveConfig();
    } else {
        loadConfig();
    }
    
    // Wyślij konfigurację do kontrolera
    sendFullConfig();
    
    // Inicjalizacja struktury danych
    memset(&data, 0, sizeof(data));
    data.lastUpdate = millis();
    
    debugPrint("Setup completed");
}

void loop() {
    static unsigned long lastDisplay = 0;
    static unsigned long lastDataCheck = 0;
    
    // Odbieranie danych z kontrolera
    if (Serial2.available()) {
        processControllerData();
    }
    
    // Sprawdzanie czy dane są aktualne
    if (millis() - lastDataCheck > 1000) {  // Co sekundę
        if (millis() - data.lastUpdate > 2000) {
            debugPrint("No data from controller!");
            // Można tu dodać obsługę braku komunikacji
        }
        lastDataCheck = millis();
    }
    
    // Aktualizacja wyświetlacza co 100ms
    if (millis() - lastDisplay > 100) {
        updateDisplay();
        lastDisplay = millis();
    }
}
