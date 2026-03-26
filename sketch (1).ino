// ╔══════════════════════════════════════════════════════════════════════╗
// ║   Control de Acceso + PID + IR + Telemetría                        ║
// ║   Archivo único para Wokwi Web                                     ║
// ║   Pega este código en la pestaña sketch.ino de wokwi.com           ║
// ╚══════════════════════════════════════════════════════════════════════╝

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUDP.h>
#include <ArduinoJson.h>
#include <MFRC522.h>
#include <time.h>
#include <math.h>

// ══════════════════════════════════════════
//  CONFIGURACIÓN — EDITA AQUÍ
// ══════════════════════════════════════════
#define WIFI_SSID         "Wokwi-GUEST"   // WiFi especial de Wokwi, no cambies
#define WIFI_PASS         ""              // Sin contraseña en Wokwi
#define TELEGRAM_TOKEN    "TU_BOT_TOKEN"
#define TELEGRAM_CHAT_ID  "TU_CHAT_ID"
#define TELEMETRY_HOST    "192.168.1.100"
#define TELEMETRY_PORT    5005

// ══════════════════════════════════════════
//  PINES
// ══════════════════════════════════════════
#define PIN_SPI_MOSI     11
#define PIN_SPI_MISO     13
#define PIN_SPI_SCK      12
#define PIN_RFID_SS      10
#define PIN_RFID_RST     14
#define PIN_RELAY         8
#define PIN_LED_BUILTIN  38
#define PIN_I2C_SDA       4
#define PIN_I2C_SCL       5
#define PIN_MOT_L_FWD    15
#define PIN_MOT_L_BWD    16
#define PIN_MOT_R_FWD    17
#define PIN_MOT_R_BWD    18

#define NUM_IR_SENSORS    4
static const uint8_t IR_PINS[NUM_IR_SENSORS] = { 36, 37, 39, 32 };

// ══════════════════════════════════════════
//  TIEMPOS
// ══════════════════════════════════════════
#define RELAY_OPEN_MS            3000
#define RFID_POLL_MS              200
#define LCD_UPDATE_MS             300
#define LCD_IDLE_MS              5000
#define DOOR_SAFETY_TIMEOUT_MS  10000
#define WATCHDOG_DEADLINE_MS    15000
#define PID_PERIOD_MS               10
#define IR_PERIOD_MS                10
#define DRIVE_PERIOD_MS            100
#define TELEMETRY_PERIOD_MS         50
#define GMT_OFFSET_SEC            3600
#define DAYLIGHT_OFFSET              0

// ══════════════════════════════════════════
//  COLAS
// ══════════════════════════════════════════
#define QUEUE_LOG_SIZE         8
#define QUEUE_TELEGRAM_SIZE    8
#define QUEUE_LCD_SIZE         4
#define QUEUE_PID_TELEM_SIZE  16

// ══════════════════════════════════════════
//  PID
// ══════════════════════════════════════════
#define PID_OUT_MIN       -255.0f
#define PID_OUT_MAX        255.0f
#define PID_INTEGRAL_MAX   100.0f
#define PID_KP_INIT          2.0f
#define PID_KI_INIT          0.5f
#define PID_KD_INIT          0.1f
#define AT_RELAY_AMPLITUDE    80.0f
#define AT_NOISE_BAND          0.5f
#define AT_CYCLES_NEEDED          5
#define AT_MAX_DURATION_MS    10000

// ══════════════════════════════════════════
//  LCD I2C
// ══════════════════════════════════════════
#define LCD_ADDRESS   0x27

#define LCD_CLEAR              0x01
#define LCD_DISPLAY_ON         0x08
#define LCD_CURSOR_OFF         0x0C
#define LCD_ENTRY_MODE         0x04
#define LCD_FUNCTION_4BIT_2LINE 0x28

class SimpleLCD {
private:
    uint8_t _addr;
    void sendCommand(uint8_t cmd) {
        Wire.beginTransmission(_addr);
        Wire.write((uint8_t)0x00);
        Wire.write(cmd);
        Wire.endTransmission();
        delayMicroseconds(50);
    }
    void sendData(uint8_t data) {
        Wire.beginTransmission(_addr);
        Wire.write((uint8_t)0x01);
        Wire.write(data);
        Wire.endTransmission();
        delayMicroseconds(50);
    }
public:
    explicit SimpleLCD(uint8_t addr = LCD_ADDRESS) : _addr(addr) {}
    void init() {
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        delay(100);
        sendCommand(0x30); delay(5);
        sendCommand(0x30); delay(1);
        sendCommand(0x30); delay(1);
        sendCommand(LCD_FUNCTION_4BIT_2LINE); delay(1);
        sendCommand(LCD_DISPLAY_ON | LCD_CURSOR_OFF); delay(1);
        sendCommand(LCD_CLEAR); delay(2);
        sendCommand(LCD_ENTRY_MODE); delay(1);
    }
    void clear() { sendCommand(LCD_CLEAR); delay(2); }
    void setCursor(uint8_t col, uint8_t row) {
        sendCommand((row == 0) ? (0x80 + col) : (0xC0 + col));
    }
    void print(const char* text) {
        while (*text) sendData((uint8_t)*text++);
    }
    void printPadded(const char* text, uint8_t width) {
        uint8_t i = 0;
        while (text[i] && i < width) { sendData((uint8_t)text[i]); i++; }
        while (i < width) { sendData(' '); i++; }
    }
};

// ══════════════════════════════════════════
//  MODOS DE CONDUCCIÓN
// ══════════════════════════════════════════
typedef enum {
    DRIVE_STOP    = 0,
    DRIVE_SLOW    = 1,
    DRIVE_NORMAL  = 2,
    DRIVE_FAST    = 3,
    DRIVE_REVERSE = 4
} DriveMode_t;

// ══════════════════════════════════════════
//  ESTRUCTURAS
// ══════════════════════════════════════════
struct User { const char* uid; const char* name; };

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

struct TelegramMsg { char text[256]; };

struct IR_Position {
    float   position;
    uint8_t rawValues[NUM_IR_SENSORS];
    bool    lineDetected;
};

struct PID_Telemetry {
    uint32_t    timestamp_ms;
    float       setpoint, measured, error, output;
    float       kp, ki, kd;
    float       p_term, i_term, d_term;
    DriveMode_t driveMode;
    bool        autotuning;
};

struct DriveCommand {
    DriveMode_t mode;
    float       setpoint;
    float       speedLimit;
};

// ══════════════════════════════════════════
//  USUARIOS AUTORIZADOS
//  Cambia los UIDs por los de tus tarjetas
// ══════════════════════════════════════════
static const User AUTHORIZED_USERS[] = {
    { "A1:B2:C3:D4", "Alice"  },
    { "11:22:33:44", "Bob"    },
    { "DE:AD:BE:EF", "Carlos" },
    { "CA:FE:BA:BE", "Diana"  },
};
static const uint8_t NUM_USERS = sizeof(AUTHORIZED_USERS) / sizeof(User);

// ══════════════════════════════════════════
//  HANDLES FREERTOS
// ══════════════════════════════════════════
QueueHandle_t     queueLog;
QueueHandle_t     queueTelegram;
QueueHandle_t     queueLCD;
QueueHandle_t     queuePID_Telem;
QueueHandle_t     queueIR_PID;
QueueHandle_t     queueDrive_PID;

SemaphoreHandle_t mutexSPI;
SemaphoreHandle_t mutexSerial;
SemaphoreHandle_t mutexMotores;

EventGroupHandle_t xEventGroupAcceso;
EventGroupHandle_t xEventGroupPID;

#define BIT_ACCESO_COMPLETADO     (1 << 0)
#define BIT_CERRADURA_CERRADA     (1 << 1)
#define BIT_PID_AUTOTUNE_DONE     (1 << 2)
#define BIT_PID_AUTOTUNE_REQUEST  (1 << 3)

TimerHandle_t xDoorSafetyTimer;
TimerHandle_t xAppWatchdogTimer;
TaskHandle_t  hTask_Cerradura_Notify = nullptr;
TaskHandle_t  hTask_PID              = nullptr;

// ══════════════════════════════════════════
//  OBJETOS GLOBALES
// ══════════════════════════════════════════
MFRC522   rfid(PIN_RFID_SS, PIN_RFID_RST);
SimpleLCD lcd(LCD_ADDRESS);
WiFiUDP   udpTelemetry;

bool     ntpReady = false;
bool     sdReady  = false;

volatile float g_pid_kp = PID_KP_INIT;
volatile float g_pid_ki = PID_KI_INIT;
volatile float g_pid_kd = PID_KD_INIT;

// ══════════════════════════════════════════
//  WATCHDOG
// ══════════════════════════════════════════
struct TaskWatchdog {
    TaskHandle_t handle;
    const char*  name;
    TickType_t   lastHeartbeat;
    bool         enabled;
};

static TaskWatchdog watchdogs[] = {
    { nullptr, "RFID",       0, true },
    { nullptr, "Cerradura",  0, true },
    { nullptr, "LCD",        0, true },
    { nullptr, "Logger",     0, true },
    { nullptr, "Telegram",   0, true },
    { nullptr, "PID",        0, true },
    { nullptr, "IR_Array",   0, true },
    { nullptr, "DriveModes", 0, true },
    { nullptr, "Telemetry",  0, true },
};
static const int NUM_WATCHDOGS = sizeof(watchdogs) / sizeof(TaskWatchdog);

// ══════════════════════════════════════════
//  UTILIDADES
// ══════════════════════════════════════════
void serialLog(const char* tag, const char* msg) {
    if (xSemaphoreTake(mutexSerial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial0.printf("[%-12s] %s\n", tag, msg);
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
        snprintf(byte, sizeof(byte),
                 (i < uid.size - 1) ? "%02X:" : "%02X",
                 uid.uidByte[i]);
        strncat(out, byte, len - strlen(out) - 1);
    }
}

const User* lookupUser(const char* uid) {
    for (uint8_t i = 0; i < NUM_USERS; i++)
        if (strcmp(AUTHORIZED_USERS[i].uid, uid) == 0)
            return &AUTHORIZED_USERS[i];
    return nullptr;
}

void getTimestamp(char* out, size_t len) {
    if (ntpReady) {
        time_t now = time(nullptr);
        struct tm* t = localtime(&now);
        strftime(out, len, "%Y-%m-%d %H:%M:%S", t);
    } else {
        snprintf(out, len, "NO-NTP-%lu", millis());
    }
}

void initNTP() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, "pool.ntp.org", "time.nist.gov");
    Serial0.print("[NTP] Sincronizando");
    int attempts = 0;
    while (time(nullptr) < 100000 && attempts < 20) {
        delay(500); Serial0.print("."); attempts++;
    }
    if (time(nullptr) > 100000) {
        ntpReady = true;
        char ts[32]; getTimestamp(ts, sizeof(ts));
        Serial0.printf("\n[NTP] OK: %s\n", ts);
    } else {
        Serial0.println("\n[NTP] No disponible");
    }
}

void setMotorPWM(float leftSpeed, float rightSpeed) {
    int l = (int)constrain(leftSpeed,  -255.0f, 255.0f);
    int r = (int)constrain(rightSpeed, -255.0f, 255.0f);
    analogWrite(PIN_MOT_L_FWD, (l >= 0) ? l : 0);
    analogWrite(PIN_MOT_L_BWD, (l <  0) ? -l : 0);
    analogWrite(PIN_MOT_R_FWD, (r >= 0) ? r : 0);
    analogWrite(PIN_MOT_R_BWD, (r <  0) ? -r : 0);
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
    doc["text"]    = text;
    String payload; serializeJson(doc, payload);
    https.addHeader("Content-Type", "application/json");
    int code = https.POST(payload);
    https.end();
    return (code == HTTP_CODE_OK);
}

void vDoorSafetyTimerCallback(TimerHandle_t) {
    serialLog("TIMER", "SEGURIDAD: cerrando puerta por timeout");
    digitalWrite(PIN_RELAY, LOW);
}

void vAppWatchdogTimerCallback(TimerHandle_t) {
    TickType_t now = xTaskGetTickCount();
    for (int i = 0; i < NUM_WATCHDOGS; i++) {
        if (!watchdogs[i].enabled) continue;
        if ((now - watchdogs[i].lastHeartbeat) > pdMS_TO_TICKS(WATCHDOG_DEADLINE_MS)) {
            Serial0.printf("[WATCHDOG] Tarea '%s' colgada. Reiniciando...\n", watchdogs[i].name);
            delay(2000); ESP.restart();
        }
    }
}

// ══════════════════════════════════════════
//  TAREA: IR_Array
//  Core 1 | Prioridad 3 | 100 Hz
// ══════════════════════════════════════════
void Task_IR_Array(void* pvParameters) {
    serialLog("IR_Array", "Iniciada");

    uint16_t calMin[NUM_IR_SENSORS];
    uint16_t calMax[NUM_IR_SENSORS];
    for (int i = 0; i < NUM_IR_SENSORS; i++) { calMin[i] = 4095; calMax[i] = 0; }

    serialLog("IR_Array", "Calibrando (2s)...");
    TickType_t calEnd = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < calEnd) {
        for (int i = 0; i < NUM_IR_SENSORS; i++) {
            uint16_t v = (uint16_t)analogRead(IR_PINS[i]);
            if (v < calMin[i]) calMin[i] = v;
            if (v > calMax[i]) calMax[i] = v;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    serialLog("IR_Array", "Calibracion OK");

    const float SENSOR_POS[NUM_IR_SENSORS] = { -1.5f, -0.5f, 0.5f, 1.5f };

    IR_Position irData;
    irData.position = 0.0f;
    irData.lineDetected = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskHeartbeat("IR_Array");

        float   normalized[NUM_IR_SENSORS];
        float   maxVal = 0.0f;
        int     maxIdx = 0;

        for (int i = 0; i < NUM_IR_SENSORS; i++) {
            uint16_t raw = (uint16_t)analogRead(IR_PINS[i]);
            irData.rawValues[i] = (uint8_t)(raw >> 4);
            uint16_t span = calMax[i] - calMin[i];
            if (span < 100) span = 100;
            float n = (float)(raw - calMin[i]) / (float)span;
            normalized[i] = constrain(n, 0.0f, 1.0f);
            if (normalized[i] > maxVal) { maxVal = normalized[i]; maxIdx = i; }
        }

        irData.lineDetected = (maxVal > 0.3f);

        if (irData.lineDetected) {
            float position;
            if (maxIdx > 0 && maxIdx < NUM_IR_SENSORS - 1) {
                float y0 = normalized[maxIdx - 1];
                float y1 = normalized[maxIdx];
                float y2 = normalized[maxIdx + 1];
                float denom = y0 - 2.0f * y1 + y2;
                float delta = 0.0f;
                if (fabsf(denom) > 0.001f)
                    delta = constrain((y0 - y2) / (2.0f * denom), -0.5f, 0.5f);
                position = SENSOR_POS[maxIdx] + delta;
            } else {
                position = SENSOR_POS[maxIdx];
            }
            irData.position = constrain(position / 1.5f, -1.0f, 1.0f);
        }

        xQueueOverwrite(queueIR_PID, &irData);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IR_PERIOD_MS));
    }
}

// ══════════════════════════════════════════
//  TAREA: PID
//  Core 1 | Prioridad 4 | 100 Hz
// ══════════════════════════════════════════
void Task_PID(void* pvParameters) {
    serialLog("PID", "Iniciada — Prioridad 4 CRITICA");

    float kp = PID_KP_INIT, ki = PID_KI_INIT, kd = PID_KD_INIT;
    float integral  = 0.0f, prevError = 0.0f;
    float setpoint  = 0.0f, speedLimit = 1.0f, baseSpeed = 180.0f;
    DriveMode_t currentMode = DRIVE_NORMAL;
    bool autotuning = false;

    float at_output = 0.0f, at_peakMax = -999.0f, at_peakMin = 999.0f;
    int   at_lastDir = 0, at_peakCount = 0;
    uint32_t at_startMs = 0;
    float at_peakTimes[10] = {}, at_peakAmplitudes[10] = {};

    IR_Position  irData;  irData.position = 0.0f; irData.lineDetected = false;
    DriveCommand driveCmd; driveCmd.mode = DRIVE_NORMAL; driveCmd.setpoint = 0.0f; driveCmd.speedLimit = 1.0f;
    PID_Telemetry telem;
    const float dt = PID_PERIOD_MS / 1000.0f;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskHeartbeat("PID");
        xQueueReceive(queueIR_PID,    &irData,   0);
        xQueueReceive(queueDrive_PID, &driveCmd, 0);

        if (driveCmd.mode != currentMode || driveCmd.speedLimit != speedLimit) {
            currentMode = driveCmd.mode;
            speedLimit  = driveCmd.speedLimit;
            setpoint    = driveCmd.setpoint;
            switch (currentMode) {
                case DRIVE_SLOW:    baseSpeed =  120.0f; break;
                case DRIVE_FAST:    baseSpeed =  240.0f; break;
                case DRIVE_REVERSE: baseSpeed = -100.0f; break;
                case DRIVE_STOP:    baseSpeed =    0.0f; break;
                default:            baseSpeed =  180.0f; break;
            }
        }

        EventBits_t bits = xEventGroupGetBits(xEventGroupPID);
        if ((bits & BIT_PID_AUTOTUNE_REQUEST) && !autotuning) {
            xEventGroupClearBits(xEventGroupPID, BIT_PID_AUTOTUNE_REQUEST);
            autotuning = true; at_peakMax = -999.0f; at_peakMin = 999.0f;
            at_lastDir = 0; at_peakCount = 0; at_startMs = millis(); integral = 0.0f;
            serialLog("PID", "AUTOTUNING iniciado");
        }

        float measured = irData.position;
        float error    = setpoint - measured;
        float output   = 0.0f;
        float p_term = 0, i_term = 0, d_term = 0;

        if (autotuning) {
            if (error > AT_NOISE_BAND)       at_output =  AT_RELAY_AMPLITUDE;
            else if (error < -AT_NOISE_BAND) at_output = -AT_RELAY_AMPLITUDE;
            output = at_output;

            int dir = (error >= 0) ? 1 : -1;
            if (dir != at_lastDir && at_lastDir != 0 && at_peakCount < 10) {
                at_peakTimes[at_peakCount]      = (float)(millis() - at_startMs);
                at_peakAmplitudes[at_peakCount] = fabsf((dir > 0) ? at_peakMin : at_peakMax);
                at_peakCount++;
                at_peakMax = -999.0f; at_peakMin = 999.0f;

                bool timeout = (millis() - at_startMs) > AT_MAX_DURATION_MS;
                if (at_peakCount >= AT_CYCLES_NEEDED * 2 || (timeout && at_peakCount >= 4)) {
                    float tuSum = 0.0f; int tuN = 0;
                    for (int i = 1; i < at_peakCount; i++) {
                        tuSum += (at_peakTimes[i] - at_peakTimes[i-1]) * 2.0f; tuN++;
                    }
                    float Tu = (tuN > 0) ? (tuSum / tuN / 1000.0f) : 1.0f;
                    float auSum = 0.0f;
                    for (int i = 0; i < at_peakCount; i++) auSum += at_peakAmplitudes[i];
                    float Au = (at_peakCount > 0) ? (auSum / at_peakCount) : 1.0f;
                    float Ku = (4.0f * AT_RELAY_AMPLITUDE) / (M_PI * Au);
                    kp = 0.60f * Ku;
                    ki = 2.0f  * kp / Tu;
                    kd = kp    * Tu / 8.0f;
                    g_pid_kp = kp; g_pid_ki = ki; g_pid_kd = kd;
                    char msg[128];
                    snprintf(msg, sizeof(msg), "OK Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
                    serialLog("PID-AT", msg);
                    autotuning = false; integral = 0.0f; prevError = 0.0f;
                    xEventGroupSetBits(xEventGroupPID, BIT_PID_AUTOTUNE_DONE);
                }
            }
            if (error > at_peakMax) at_peakMax = error;
            if (error < at_peakMin) at_peakMin = error;
            at_lastDir = dir;

        } else {
            p_term   = kp * error;
            integral = constrain(integral + error * dt, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
            i_term   = ki * integral;
            float derivative = (error - prevError) / dt;
            d_term   = kd * derivative;
            prevError = error;
            output = constrain(p_term + i_term + d_term, PID_OUT_MIN, PID_OUT_MAX);

            if (xSemaphoreTake(mutexMotores, pdMS_TO_TICKS(5)) == pdTRUE) {
                if (currentMode == DRIVE_STOP) setMotorPWM(0, 0);
                else {
                    float base = baseSpeed * speedLimit;
                    setMotorPWM(base - output, base + output);
                }
                xSemaphoreGive(mutexMotores);
            }

            telem.timestamp_ms = millis();
            telem.setpoint = setpoint; telem.measured = measured;
            telem.error = error; telem.output = output;
            telem.kp = kp; telem.ki = ki; telem.kd = kd;
            telem.p_term = p_term; telem.i_term = i_term; telem.d_term = d_term;
            telem.driveMode = currentMode; telem.autotuning = autotuning;
            xQueueSend(queuePID_Telem, &telem, 0);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PID_PERIOD_MS));
    }
}

// ══════════════════════════════════════════
//  TAREA: DriveModes
//  Core 0 | Prioridad 2 | 10 Hz
// ══════════════════════════════════════════
void Task_DriveModes(void* pvParameters) {
    serialLog("DriveModes", "Iniciada");
    float       errorHistory[10] = {};
    int         histIdx = 0;
    uint32_t    lowErrorSince = 0, highErrorSince = 0;
    DriveMode_t lastMode = DRIVE_NORMAL;
    IR_Position irPeek;

    DriveCommand cmd; cmd.setpoint = 0.0f; cmd.mode = DRIVE_NORMAL; cmd.speedLimit = 0.75f;
    xQueueOverwrite(queueDrive_PID, &cmd);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskHeartbeat("DriveModes");
        bool  lineDetected = true;
        float absError = 0.0f;
        if (xQueuePeek(queueIR_PID, &irPeek, 0) == pdTRUE) {
            lineDetected = irPeek.lineDetected;
            absError = fabsf(irPeek.position);
        }
        errorHistory[histIdx % 10] = absError; histIdx++;
        float avgError = 0.0f;
        for (int i = 0; i < 10; i++) avgError += errorHistory[i];
        avgError /= 10.0f;

        uint32_t    now = millis();
        DriveMode_t newMode = DRIVE_NORMAL;

        if (!lineDetected) {
            newMode = DRIVE_REVERSE; lowErrorSince = 0; highErrorSince = 0;
        } else if (avgError < 0.08f) {
            if (lowErrorSince == 0) lowErrorSince = now;
            newMode = ((now - lowErrorSince) > 500) ? DRIVE_FAST : DRIVE_NORMAL;
            highErrorSince = 0;
        } else if (avgError > 0.35f) {
            if (highErrorSince == 0) highErrorSince = now;
            newMode = ((now - highErrorSince) > 80) ? DRIVE_SLOW : DRIVE_NORMAL;
            lowErrorSince = 0;
        } else {
            lowErrorSince = 0; highErrorSince = 0;
        }

        if (newMode != lastMode) {
            lastMode = newMode; cmd.mode = newMode;
            switch (newMode) {
                case DRIVE_FAST:    cmd.speedLimit = 1.00f; serialLog("Drive","FAST-recta");   break;
                case DRIVE_SLOW:    cmd.speedLimit = 0.50f; serialLog("Drive","SLOW-curva");   break;
                case DRIVE_REVERSE: cmd.speedLimit = 0.40f; serialLog("Drive","REVERSE");      break;
                default:            cmd.speedLimit = 0.75f; serialLog("Drive","NORMAL");       break;
            }
            xQueueOverwrite(queueDrive_PID, &cmd);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DRIVE_PERIOD_MS));
    }
}

// ══════════════════════════════════════════
//  TAREA: Telemetry
//  Core 0 | Prioridad 1 | 20 Hz
// ══════════════════════════════════════════
void Task_Telemetry(void* pvParameters) {
    serialLog("Telemetry", "Iniciada");
    PID_Telemetry telem;
    char jsonBuf[256];
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskHeartbeat("Telemetry");
        if (xQueueReceive(queuePID_Telem, &telem, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS)) == pdTRUE) {
            snprintf(jsonBuf, sizeof(jsonBuf),
                "{\"t\":%lu,\"sp\":%.3f,\"pv\":%.3f,\"e\":%.3f,\"out\":%.1f,"
                "\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"mode\":%d,\"at\":%d}\n",
                (unsigned long)telem.timestamp_ms,
                telem.setpoint, telem.measured, telem.error, telem.output,
                telem.kp, telem.ki, telem.kd,
                (int)telem.driveMode, telem.autotuning ? 1 : 0);

            if (xSemaphoreTake(mutexSerial, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial0.print("TELEM:"); Serial0.print(jsonBuf);
                xSemaphoreGive(mutexSerial);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

// ══════════════════════════════════════════
//  TAREA: RFID
//  Core 1 | Prioridad 3
// ══════════════════════════════════════════
void Task_RFID(void* pvParameters) {
    serialLog("RFID", "Iniciada");
    char uidStr[20], timestamp[32];

    for (;;) {
        vTaskHeartbeat("RFID");
        if (xSemaphoreTake(mutexSPI, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
                uidToString(rfid.uid, uidStr, sizeof(uidStr));
                rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
                xSemaphoreGive(mutexSPI);

                const User* user = lookupUser(uidStr);
                bool granted = (user != nullptr);
                getTimestamp(timestamp, sizeof(timestamp));

                char logLine[128];
                snprintf(logLine, sizeof(logLine), "%s | %s | %s",
                         uidStr, granted ? user->name : "DESCONOCIDO",
                         granted ? "CONCEDIDO" : "DENEGADO");
                serialLog("RFID", logLine);

                if (granted) {
                    xTaskNotify(hTask_Cerradura_Notify, RELAY_OPEN_MS, eSetValueWithOverwrite);
                    xTimerReset(xDoorSafetyTimer, pdMS_TO_TICKS(100));
                    xEventGroupSetBits(xEventGroupAcceso, BIT_ACCESO_COMPLETADO);

                    TelegramMsg tmsg;
                    snprintf(tmsg.text, sizeof(tmsg.text),
                        "Acceso CONCEDIDO\nUsuario: %s\nUID: %s\nHora: %s",
                        user->name, uidStr, timestamp);
                    xQueueSend(queueTelegram, &tmsg, 0);

                    digitalWrite(PIN_LED_BUILTIN, LOW);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    digitalWrite(PIN_LED_BUILTIN, HIGH);
                }

                AccessEvent evt;
                strncpy(evt.uid,       uidStr,                     sizeof(evt.uid) - 1);
                strncpy(evt.userName,  granted ? user->name : "?", sizeof(evt.userName) - 1);
                strncpy(evt.timestamp, timestamp,                   sizeof(evt.timestamp) - 1);
                evt.granted = granted;
                xQueueSend(queueLog, &evt, 0);

                LcdMessage lmsg; lmsg.showMs = LCD_IDLE_MS;
                if (granted) {
                    snprintf(lmsg.line1, 17, "ACCESO OK       ");
                    snprintf(lmsg.line2, 17, "%-16s", user->name);
                } else {
                    snprintf(lmsg.line1, 17, "DENEGADO        ");
                    snprintf(lmsg.line2, 17, "%.16s", uidStr);
                }
                xQueueSend(queueLCD, &lmsg, 0);

            } else {
                xSemaphoreGive(mutexSPI);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(RFID_POLL_MS));
    }
}

// ══════════════════════════════════════════
//  TAREA: Cerradura
//  Core 1 | Prioridad 3
// ══════════════════════════════════════════
void Task_Cerradura(void* pvParameters) {
    serialLog("Cerradura", "Iniciada");
    for (;;) {
        vTaskHeartbeat("Cerradura");
        uint32_t openTimeMs = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (openTimeMs > 0) {
            serialLog("Cerradura", "ABRIENDO");
            digitalWrite(PIN_RELAY, HIGH);
            vTaskDelay(pdMS_TO_TICKS(openTimeMs));
            digitalWrite(PIN_RELAY, LOW);
            serialLog("Cerradura", "CERRADA");
            xEventGroupSetBits(xEventGroupAcceso, BIT_CERRADURA_CERRADA);
            xTimerStop(xDoorSafetyTimer, pdMS_TO_TICKS(100));
        }
    }
}

// ══════════════════════════════════════════
//  TAREA: LCD
//  Core 0 | Prioridad 2
// ══════════════════════════════════════════
void Task_LCD(void* pvParameters) {
    serialLog("LCD", "Iniciada");
    LcdMessage msg;
    TickType_t showUntil = 0;
    char timestamp[32];

    lcd.init(); lcd.clear();
    lcd.setCursor(0, 0); lcd.print("PID+RFID+IR v2.0");
    lcd.setCursor(0, 1); lcd.print("  Iniciando...  ");

    for (;;) {
        vTaskHeartbeat("LCD");
        if (xQueueReceive(queueLCD, &msg, pdMS_TO_TICKS(LCD_UPDATE_MS)) == pdTRUE) {
            showUntil = xTaskGetTickCount() + pdMS_TO_TICKS(msg.showMs);
            lcd.clear();
            lcd.setCursor(0, 0); lcd.print(msg.line1);
            lcd.setCursor(0, 1); lcd.print(msg.line2);
        } else if (showUntil != 0 && xTaskGetTickCount() >= showUntil) {
            showUntil = 0;
            getTimestamp(timestamp, sizeof(timestamp));
            lcd.clear();
            lcd.setCursor(0, 0); lcd.printPadded(timestamp, 16);
            lcd.setCursor(0, 1); lcd.print("  Pase tarjeta  ");
        }
    }
}

// ══════════════════════════════════════════
//  TAREA: Logger
//  Core 0 | Prioridad 1
// ══════════════════════════════════════════
void Task_Logger(void* pvParameters) {
    serialLog("Logger", "Iniciada — solo serial (sin SD en Wokwi)");
    AccessEvent evt;
    for (;;) {
        vTaskHeartbeat("Logger");
        if (xQueueReceive(queueLog, &evt, portMAX_DELAY) == pdTRUE) {
            char logLine[128];
            snprintf(logLine, sizeof(logLine), "%s,%s,%s,%s",
                     evt.timestamp, evt.uid, evt.userName,
                     evt.granted ? "OK" : "DENIED");
            serialLog("Logger", logLine);
        }
    }
}

// ══════════════════════════════════════════
//  TAREA: Telegram
//  Core 0 | Prioridad 1
// ══════════════════════════════════════════
void Task_Telegram(void* pvParameters) {
    serialLog("Telegram", "Iniciada");
    TelegramMsg tmsg;
    for (;;) {
        vTaskHeartbeat("Telegram");
        if (xQueueReceive(queueTelegram, &tmsg, portMAX_DELAY) == pdTRUE) {
            if (WiFi.status() == WL_CONNECTED) {
                bool ok = sendTelegramMsg(tmsg.text);
                serialLog("Telegram", ok ? "Enviado OK" : "Error");
            } else {
                serialLog("Telegram", "Sin WiFi");
            }
        }
    }
}

// ══════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════
void setup() {
    Serial0.begin(115200, SERIAL_8N1, 44, 43); // UART0 hardware: RX=GPIO44, TX=GPIO43
    delay(1000);
    Serial0.println("\n╔══════════════════════════════════════════╗");
    Serial0.println("║  Control Acceso + PID  |  Wokwi Web     ║");
    Serial0.println("╚══════════════════════════════════════════╝\n");

    pinMode(PIN_RELAY,       OUTPUT); digitalWrite(PIN_RELAY,       LOW);
    pinMode(PIN_LED_BUILTIN, OUTPUT); digitalWrite(PIN_LED_BUILTIN, HIGH);
    pinMode(PIN_MOT_L_FWD,   OUTPUT); pinMode(PIN_MOT_L_BWD, OUTPUT);
    pinMode(PIN_MOT_R_FWD,   OUTPUT); pinMode(PIN_MOT_R_BWD, OUTPUT);
    for (int i = 0; i < NUM_IR_SENSORS; i++) pinMode(IR_PINS[i], INPUT);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    rfid.PCD_Init();
    Serial0.println("[SETUP] RFID OK");

    // WiFi — en Wokwi usa "Wokwi-GUEST" sin contraseña
    Serial0.print("[SETUP] Conectando WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int wifiAtt = 0;
    while (WiFi.status() != WL_CONNECTED && wifiAtt < 20) {
        delay(500); Serial0.print("."); wifiAtt++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial0.printf("\n[SETUP] WiFi OK — IP: %s\n", WiFi.localIP().toString().c_str());
        initNTP();
    } else {
        Serial0.println("\n[SETUP] WiFi no disponible");
    }

    // Primitivas FreeRTOS
    queueLog       = xQueueCreate(QUEUE_LOG_SIZE,       sizeof(AccessEvent));
    queueTelegram  = xQueueCreate(QUEUE_TELEGRAM_SIZE,  sizeof(TelegramMsg));
    queueLCD       = xQueueCreate(QUEUE_LCD_SIZE,       sizeof(LcdMessage));
    queuePID_Telem = xQueueCreate(QUEUE_PID_TELEM_SIZE, sizeof(PID_Telemetry));
    queueIR_PID    = xQueueCreate(1,                    sizeof(IR_Position));
    queueDrive_PID = xQueueCreate(1,                    sizeof(DriveCommand));

    mutexSPI     = xSemaphoreCreateMutex();
    mutexSerial  = xSemaphoreCreateMutex();
    mutexMotores = xSemaphoreCreateMutex();

    xEventGroupAcceso = xEventGroupCreate();
    xEventGroupPID    = xEventGroupCreate();

    xDoorSafetyTimer = xTimerCreate("DoorSafety",
        pdMS_TO_TICKS(DOOR_SAFETY_TIMEOUT_MS), pdFALSE, nullptr, vDoorSafetyTimerCallback);
    xAppWatchdogTimer = xTimerCreate("AppWatchdog",
        pdMS_TO_TICKS(WATCHDOG_DEADLINE_MS), pdTRUE, nullptr, vAppWatchdogTimerCallback);
    xTimerStart(xAppWatchdogTimer, 0);

    for (int i = 0; i < NUM_WATCHDOGS; i++)
        watchdogs[i].lastHeartbeat = xTaskGetTickCount();

    // Crear tareas
    xTaskCreatePinnedToCore(Task_PID,       "PID",       3072, nullptr, 4, &hTask_PID,             1);
    xTaskCreatePinnedToCore(Task_IR_Array,  "IR_Array",  2048, nullptr, 3, &watchdogs[6].handle,   1);
    xTaskCreatePinnedToCore(Task_RFID,      "RFID",      4096, nullptr, 3, &watchdogs[0].handle,   1);
    xTaskCreatePinnedToCore(Task_Cerradura, "Cerradura", 2048, nullptr, 3, &hTask_Cerradura_Notify, 1);
    xTaskCreatePinnedToCore(Task_DriveModes,"DriveModes",2048, nullptr, 2, &watchdogs[7].handle,   0);
    xTaskCreatePinnedToCore(Task_LCD,       "LCD",       3072, nullptr, 2, &watchdogs[2].handle,   0);
    xTaskCreatePinnedToCore(Task_Logger,    "Logger",    4096, nullptr, 1, &watchdogs[3].handle,   0);
    xTaskCreatePinnedToCore(Task_Telegram,  "Telegram",  8192, nullptr, 1, &watchdogs[4].handle,   0);
    xTaskCreatePinnedToCore(Task_Telemetry, "Telemetry", 4096, nullptr, 1, &watchdogs[8].handle,   0);

    watchdogs[1].handle = hTask_Cerradura_Notify;
    watchdogs[5].handle = hTask_PID;

    // Autotuning inicial (comenta si ya tienes buenos valores de Kp/Ki/Kd)
    xEventGroupSetBits(xEventGroupPID, BIT_PID_AUTOTUNE_REQUEST);

    Serial0.println("[SETUP] Sistema listo!\n");
}

void loop() {
    vTaskDelete(nullptr);
}
