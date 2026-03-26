/**
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║   SISTEMA DE CONTROL DE ACCESO — RYMCU ESP32-S3 DevKitC-1           ║
 * ║                    FreeRTOS AVANZADO                                 ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <SD.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <time.h>

// ============================================
// CONFIGURACIÓN (CAMBIAR ANTES DE COMPILAR)
// ============================================
#define WIFI_SSID         "TU_WIFI_SSID"
#define WIFI_PASS         "TU_WIFI_PASSWORD"
#define TELEGRAM_TOKEN    "TU_BOT_TOKEN"
#define TELEGRAM_CHAT_ID  "TU_CHAT_ID"

// ============================================
// PINES PARA RYMCU ESP32-S3 DevKitC-1
// ============================================
#define PIN_SPI_MOSI     11
#define PIN_SPI_MISO     13
#define PIN_SPI_SCK      12
#define PIN_RFID_SS      10
#define PIN_RFID_RST     14
#define PIN_SD_CS        9
#define PIN_RELAY        8
#define PIN_I2C_SDA      4
#define PIN_I2C_SCL      5
#define PIN_LED_BUILTIN  38

// ============================================
// PARÁMETROS DE TIEMPO
// ============================================
#define RELAY_OPEN_MS        3000
#define RFID_POLL_MS         200
#define LCD_UPDATE_MS        300
#define LCD_IDLE_MS         5000
#define DOOR_SAFETY_TIMEOUT_MS    10000
#define WATCHDOG_DEADLINE_MS      15000

// NTP nativo
#define GMT_OFFSET_SEC   3600
#define DAYLIGHT_OFFSET  0

// ============================================
// TAMAÑOS DE COLAS
// ============================================
#define QUEUE_LOG_SIZE      8
#define QUEUE_TELEGRAM_SIZE 8
#define QUEUE_LCD_SIZE      4

// ============================================
// LCD I2C (implementación directa)
// ============================================
#define LCD_ADDRESS 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x04
#define LCD_DISPLAY_ON 0x08
#define LCD_CURSOR_OFF 0x0C
#define LCD_FUNCTION_4BIT_2LINE 0x28

class SimpleLCD {
private:
    uint8_t _addr;
    
    void sendCommand(uint8_t cmd) {
        Wire.beginTransmission(_addr);
        Wire.write(0x00);
        Wire.write(cmd);
        Wire.endTransmission();
        delayMicroseconds(50);
    }
    
    void sendData(uint8_t data) {
        Wire.beginTransmission(_addr);
        Wire.write(0x01);
        Wire.write(data);
        Wire.endTransmission();
        delayMicroseconds(50);
    }
    
public:
    SimpleLCD(uint8_t addr) : _addr(addr) {}
    
    void init() {
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        delay(100);
        sendCommand(0x30);
        delay(5);
        sendCommand(0x30);
        delay(1);
        sendCommand(0x30);
        delay(1);
        sendCommand(LCD_FUNCTION_4BIT_2LINE);
        delay(1);
        sendCommand(LCD_DISPLAY_ON | LCD_CURSOR_OFF);
        delay(1);
        sendCommand(LCD_CLEAR);
        delay(2);
        sendCommand(LCD_ENTRY_MODE);
        delay(1);
    }
    
    void clear() {
        sendCommand(LCD_CLEAR);
        delay(2);
    }
    
    void setCursor(uint8_t col, uint8_t row) {
        uint8_t address = 0x80;
        if (row == 0) address += col;
        else if (row == 1) address += 0x40 + col;
        sendCommand(address);
    }
    
    void print(const char* text) {
        while (*text) {
            sendData(*text++);
        }
    }
    
    void print(int number) {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%d", number);
        print(buffer);
    }
};

// ============================================
// USUARIOS AUTORIZADOS
// ============================================
struct User {
    const char* uid;
    const char* name;
};

static const User AUTHORIZED_USERS[] = {
    { "A1:B2:C3:D4", "Alice" },
    { "11:22:33:44", "Bob" },
    { "DE:AD:BE:EF", "Carlos" },
    { "CA:FE:BA:BE", "Diana" },
};
static const uint8_t NUM_USERS = sizeof(AUTHORIZED_USERS) / sizeof(User);

// ============================================
// ESTRUCTURAS DE DATOS
// ============================================
struct AccessEvent {
    char uid[20];
    char userName[32];
    bool granted;
    char timestamp[32];
};

struct LcdMessage {
    char line1[17];
    char line2[17];
    uint32_t showMs;
};

struct TelegramMsg {
    char text[256];
};

// ============================================
// HANDLES FREERTOS
// ============================================
QueueHandle_t queueLog;
QueueHandle_t queueTelegram;
QueueHandle_t queueLCD;

SemaphoreHandle_t mutexSPI;
SemaphoreHandle_t mutexSerial;

EventGroupHandle_t xEventGroupAcceso;
#define BIT_ACCESO_COMPLETADO   (1 << 0)
#define BIT_CERRADURA_CERRADA   (1 << 1)

TimerHandle_t xDoorSafetyTimer;
TaskHandle_t hTask_Cerradura_Notify = nullptr;

struct TaskWatchdog {
    TaskHandle_t handle;
    const char* name;
    TickType_t lastHeartbeat;
    bool enabled;
};

static TaskWatchdog watchdogs[] = {
    { nullptr, "RFID",      0, true },
    { nullptr, "Cerradura", 0, true },
    { nullptr, "LCD",       0, true },
    { nullptr, "Logger",    0, true },
    { nullptr, "Telegram",  0, true }
};
static const int NUM_WATCHDOGS = sizeof(watchdogs) / sizeof(TaskWatchdog);
TimerHandle_t xAppWatchdogTimer;

// ============================================
// OBJETOS GLOBALES
// ============================================
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
SimpleLCD lcd(LCD_ADDRESS);
bool ntpReady = false;
bool sdReady = false;

// ============================================
// FUNCIONES DE UTILIDAD
// ============================================
void serialLog(const char* tag, const char* msg) {
    if (xSemaphoreTake(mutexSerial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.printf("[%-12s] %s\n", tag, msg);
        xSemaphoreGive(mutexSerial);
    }
}

void vTaskHeartbeat(const char* taskName) {
    for (int i = 0; i < NUM_WATCHDOGS; i++) {
        if (watchdogs[i].enabled && strcmp(watchdogs[i].name, taskName) == 0) {
            watchdogs[i].lastHeartbeat = xTaskGetTickCount();
            return;
        }
    }
}

void uidToString(MFRC522::Uid& uid, char* out, size_t len) {
    out[0] = '\0';
    for (uint8_t i = 0; i < uid.size; i++) {
        char byte[5];
        snprintf(byte, sizeof(byte), (i < uid.size - 1) ? "%02X:" : "%02X", uid.uidByte[i]);
        strncat(out, byte, len - strlen(out) - 1);
    }
}

const User* lookupUser(const char* uid) {
    for (uint8_t i = 0; i < NUM_USERS; i++) {
        if (strcmp(AUTHORIZED_USERS[i].uid, uid) == 0) return &AUTHORIZED_USERS[i];
    }
    return nullptr;
}

void getTimestamp(char* out, size_t len) {
    if (ntpReady) {
        time_t now = time(nullptr);
        struct tm* timeinfo = localtime(&now);
        strftime(out, len, "%Y-%m-%d %H:%M:%S", timeinfo);
    } else {
        snprintf(out, len, "NO-NTP-%lu", millis());
    }
}

void initNTP() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, "pool.ntp.org", "time.nist.gov");
    int attempts = 0;
    while (time(nullptr) < 100000 && attempts < 20) {
        delay(500);
        attempts++;
    }
    if (time(nullptr) > 100000) {
        ntpReady = true;
        char timestamp[32];
        getTimestamp(timestamp, sizeof(timestamp));
        Serial.printf("NTP sincronizado: %s\n", timestamp);
    } else {
        Serial.println("NTP no disponible");
    }
}

bool sendTelegramMsg(const char* text) {
    if (WiFi.status() != WL_CONNECTED) return false;
    
    WiFiClientSecure client;
    client.setInsecure();
    
    HTTPClient https;
    String url = String("https://api.telegram.org/bot") + TELEGRAM_TOKEN + "/sendMessage";
    
    if (!https.begin(client, url)) return false;
    
    JsonDocument doc;
    doc["chat_id"] = TELEGRAM_CHAT_ID;
    doc["text"] = text;
    
    String payload;
    serializeJson(doc, payload);
    
    https.addHeader("Content-Type", "application/json");
    int httpCode = https.POST(payload);
    https.end();
    
    return (httpCode == HTTP_CODE_OK);
}

void vDoorSafetyTimerCallback(TimerHandle_t xTimer) {
    serialLog("TIMER", "SEGURIDAD: Cerrando puerta por timeout");
    digitalWrite(PIN_RELAY, LOW);
}

void vAppWatchdogTimerCallback(TimerHandle_t xTimer) {
    TickType_t now = xTaskGetTickCount();
    for (int i = 0; i < NUM_WATCHDOGS; i++) {
        if (!watchdogs[i].enabled) continue;
        TickType_t timeSinceHeartbeat = now - watchdogs[i].lastHeartbeat;
        if (timeSinceHeartbeat > pdMS_TO_TICKS(WATCHDOG_DEADLINE_MS)) {
            Serial.printf("WATCHDOG: Tarea %s colgada! Reiniciando...\n", watchdogs[i].name);
            delay(2000);
            ESP.restart();
        }
    }
}

// ============================================
// TAREA 1: Task_RFID (prioridad 3, Core 1)
// ============================================
void Task_RFID(void* pvParameters) {
    serialLog("RFID", "Iniciada - Prioridad 3");
    char uidStr[20], timestamp[32];
    
    for (;;) {
        vTaskHeartbeat("RFID");
        
        if (xSemaphoreTake(mutexSPI, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
                uidToString(rfid.uid, uidStr, sizeof(uidStr));
                rfid.PICC_HaltA();
                rfid.PCD_StopCrypto1();
                
                const User* user = lookupUser(uidStr);
                bool granted = (user != nullptr);
                getTimestamp(timestamp, sizeof(timestamp));
                
                serialLog("RFID", granted ? "ACCESO OK" : "DENEGADO");
                
                if (granted) {
                    // Task Notification
                    xTaskNotify(hTask_Cerradura_Notify, RELAY_OPEN_MS, eSetValueWithOverwrite);
                    xTimerReset(xDoorSafetyTimer, pdMS_TO_TICKS(100));
                    xEventGroupSetBits(xEventGroupAcceso, BIT_ACCESO_COMPLETADO);
                    
                    // Telegram
                    TelegramMsg tmsg;
                    snprintf(tmsg.text, sizeof(tmsg.text),
                        "✅ Acceso CONCEDIDO\nUsuario: %s\nUID: %s\nHora: %s",
                        user->name, uidStr, timestamp);
                    xQueueSend(queueTelegram, &tmsg, 0);
                    
                    // LED feedback
                    digitalWrite(PIN_LED_BUILTIN, LOW);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    digitalWrite(PIN_LED_BUILTIN, HIGH);
                }
                
                // Enviar a Logger
                AccessEvent evt;
                strncpy(evt.uid, uidStr, sizeof(evt.uid));
                strncpy(evt.userName, granted ? user->name : "DESCONOCIDO", sizeof(evt.userName));
                strncpy(evt.timestamp, timestamp, sizeof(evt.timestamp));
                evt.granted = granted;
                xQueueSend(queueLog, &evt, 0);
                
                // Enviar a LCD
                LcdMessage lmsg;
                lmsg.showMs = LCD_IDLE_MS;
                if (granted) {
                    snprintf(lmsg.line1, 17, "ACCESO OK");
                    snprintf(lmsg.line2, 17, "%-16s", user->name);
                } else {
                    snprintf(lmsg.line1, 17, "ACCESO DENEGADO");
                    snprintf(lmsg.line2, 17, "%.16s", uidStr);
                }
                xQueueSend(queueLCD, &lmsg, 0);
            }
            xSemaphoreGive(mutexSPI);
        }
        
        vTaskDelay(pdMS_TO_TICKS(RFID_POLL_MS));
    }
}

// ============================================
// TAREA 2: Task_Cerradura (prioridad 3, Core 1)
// ============================================
void Task_Cerradura(void* pvParameters) {
    serialLog("Cerradura", "Iniciada - Prioridad 3");
    
    for (;;) {
        vTaskHeartbeat("Cerradura");
        
        uint32_t openTimeMs = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (openTimeMs > 0) {
            serialLog("Cerradura", "ABRIENDO cerradura");
            digitalWrite(PIN_RELAY, HIGH);
            vTaskDelay(pdMS_TO_TICKS(openTimeMs));
            digitalWrite(PIN_RELAY, LOW);
            serialLog("Cerradura", "CERRADA");
            
            xEventGroupSetBits(xEventGroupAcceso, BIT_CERRADURA_CERRADA);
            xTimerStop(xDoorSafetyTimer, pdMS_TO_TICKS(100));
        }
    }
}

// ============================================
// TAREA 3: Task_LCD (prioridad 2, Core 0)
// ============================================
void Task_LCD(void* pvParameters) {
    serialLog("LCD", "Iniciada - Prioridad 2");
    LcdMessage msg;
    TickType_t showUntil = 0;
    char timestamp[32];
    
    lcd.init();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  Listo para    ");
    lcd.setCursor(0, 1);
    lcd.print("  lectura RFID  ");
    
    for (;;) {
        vTaskHeartbeat("LCD");
        
        if (xQueueReceive(queueLCD, &msg, pdMS_TO_TICKS(LCD_UPDATE_MS)) == pdTRUE) {
            showUntil = xTaskGetTickCount() + pdMS_TO_TICKS(msg.showMs);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(msg.line1);
            lcd.setCursor(0, 1);
            lcd.print(msg.line2);
        } else if (showUntil != 0 && xTaskGetTickCount() >= showUntil) {
            showUntil = 0;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("  Listo para    ");
            lcd.setCursor(0, 1);
            lcd.print("  lectura RFID  ");
            
            getTimestamp(timestamp, sizeof(timestamp));
            lcd.setCursor(0, 0);
            lcd.print(timestamp);
        }
    }
}

// ============================================
// TAREA 4: Task_Logger (prioridad 1, Core 0)
// ============================================
void Task_Logger(void* pvParameters) {
    serialLog("Logger", "Iniciada - Prioridad 1");
    AccessEvent evt;
    
    for (;;) {
        vTaskHeartbeat("Logger");
        
        if (xQueueReceive(queueLog, &evt, portMAX_DELAY) == pdTRUE) {
            char logLine[128];
            snprintf(logLine, sizeof(logLine), "%s,%s,%s,%s",
                     evt.timestamp, evt.uid, evt.userName,
                     evt.granted ? "OK" : "DENIED");
            
            if (sdReady) {
                if (xSemaphoreTake(mutexSPI, pdMS_TO_TICKS(500)) == pdTRUE) {
                    File logFile = SD.open("/access_log.csv", FILE_APPEND);
                    if (logFile) {
                        logFile.println(logLine);
                        logFile.close();
                        serialLog("Logger", "Escrito en SD");
                    } else {
                        serialLog("Logger", "ERROR: No se pudo abrir archivo");
                    }
                    xSemaphoreGive(mutexSPI);
                }
            } else {
                serialLog("Logger", "SD no disponible, solo serial");
            }
            
            serialLog("Logger", logLine);
        }
    }
}

// ============================================
// TAREA 5: Task_Telegram (prioridad 1, Core 0)
// ============================================
void Task_Telegram(void* pvParameters) {
    serialLog("Telegram", "Iniciada - Prioridad 1");
    TelegramMsg tmsg;
    
    for (;;) {
        vTaskHeartbeat("Telegram");
        
        if (xQueueReceive(queueTelegram, &tmsg, portMAX_DELAY) == pdTRUE) {
            if (WiFi.status() == WL_CONNECTED) {
                bool ok = sendTelegramMsg(tmsg.text);
                serialLog("Telegram", ok ? "Enviado OK" : "Error");
            } else {
                serialLog("Telegram", "Sin WiFi - mensaje descartado");
            }
        }
    }
}

// ============================================
// SETUP
// ============================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n╔══════════════════════════════════════════════════════════╗");
    Serial.println("║   CONTROL DE ACCESO - RYMCU ESP32-S3 DevKitC-1         ║");
    Serial.println("║   FreeRTOS Avanzado                                     ║");
    Serial.println("╚══════════════════════════════════════════════════════════╝\n");
    
    // Configurar pines
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);
    digitalWrite(PIN_LED_BUILTIN, HIGH);
    
    // Inicializar SPI
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    Serial.println("[SETUP] SPI iniciado");
    
    // Inicializar RFID
    rfid.PCD_Init();
    Serial.println("[SETUP] RFID RC522 iniciado");
    
    // Inicializar SD (usando SD nativa de ESP32)
    Serial.printf("[SETUP] Inicializando SD en CS=%d...\n", PIN_SD_CS);
    if (SD.begin(PIN_SD_CS)) {
        sdReady = true;
        Serial.println("[SETUP] SD Card: OK");
        
        // Crear archivo si no existe
        if (!SD.exists("/access_log.csv")) {
            File f = SD.open("/access_log.csv", FILE_WRITE);
            if (f) {
                f.println("timestamp,uid,user,granted");
                f.close();
                Serial.println("[SETUP] Archivo CSV creado");
            }
        }
    } else {
        Serial.println("[SETUP] SD Card: NO detectada - Logging desactivado");
    }
    
    // Conectar WiFi
    Serial.printf("[SETUP] Conectando a %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int wifiAttempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
        delay(500);
        Serial.print(".");
        wifiAttempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[SETUP] WiFi OK - IP: %s\n", WiFi.localIP().toString().c_str());
        initNTP();
    } else {
        Serial.println("\n[SETUP] WiFi FALLIDO");
    }
    
    // Crear primitivas FreeRTOS
    queueLog = xQueueCreate(QUEUE_LOG_SIZE, sizeof(AccessEvent));
    queueTelegram = xQueueCreate(QUEUE_TELEGRAM_SIZE, sizeof(TelegramMsg));
    queueLCD = xQueueCreate(QUEUE_LCD_SIZE, sizeof(LcdMessage));
    mutexSPI = xSemaphoreCreateMutex();
    mutexSerial = xSemaphoreCreateMutex();
    
    // Event Group
    xEventGroupAcceso = xEventGroupCreate();
    
    // Software Timer (seguridad puerta)
    xDoorSafetyTimer = xTimerCreate("DoorSafety", 
                                    pdMS_TO_TICKS(DOOR_SAFETY_TIMEOUT_MS),
                                    pdFALSE, NULL, vDoorSafetyTimerCallback);
    
    // Application Watchdog
    xAppWatchdogTimer = xTimerCreate("AppWatchdog",
                                     pdMS_TO_TICKS(WATCHDOG_DEADLINE_MS),
                                     pdTRUE, NULL, vAppWatchdogTimerCallback);
    xTimerStart(xAppWatchdogTimer, 0);
    
    // Crear tareas con las prioridades especificadas
    xTaskCreatePinnedToCore(Task_RFID,      "RFID",      4096, NULL, 3, &watchdogs[0].handle, 1);
    xTaskCreatePinnedToCore(Task_Cerradura, "Cerradura", 2048, NULL, 3, &hTask_Cerradura_Notify, 1);
    xTaskCreatePinnedToCore(Task_LCD,       "LCD",       3072, NULL, 2, &watchdogs[2].handle, 0);
    xTaskCreatePinnedToCore(Task_Logger,    "Logger",    4096, NULL, 1, &watchdogs[3].handle, 0);
    xTaskCreatePinnedToCore(Task_Telegram,  "Telegram",  8192, NULL, 1, &watchdogs[4].handle, 0);
    
    // Inicializar heartbeats
    for (int i = 0; i < NUM_WATCHDOGS; i++) {
        watchdogs[i].lastHeartbeat = xTaskGetTickCount();
    }
    
    Serial.println("\n[SETUP] Sistema iniciado correctamente!\n");
    
    // Mensaje en LCD
    lcd.init();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  Sistema Listo ");
    lcd.setCursor(0, 1);
    lcd.print("  Pase tarjeta  ");
}

void loop() {
    vTaskDelete(NULL);
}