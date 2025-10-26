#include "VescBLE.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>

// -----------------------------------------------------
// Public globals
// -----------------------------------------------------
ComEVesc UART;

float speed      = 0.0f;   // km/h
float watts      = 0.0f;   // W
float total_km   = 0.0f;   // km total ODO
float trip_km    = 0.0f;   // km seit letztem Trip-Reset/Boot
bool  connected  = false;  // "valid data + BLE link up"

int32_t vesc_tachometer     = 0; // inkrementeller Tachozähler vom VESC
int32_t vesc_tachometerAbs  = 0; // absoluter Tachozähler vom VESC

unsigned long vesc_last_update_ms = 0; // last valid dataset timestamp

// -----------------------------------------------------
// VescBLE v1.3.4 (Zachway RC15)
// -----------------------------------------------------

// BLE device name of the VESC BLE module
static const char *BLE_TARGET_NAME = "VESC BLE UART";

// Nordic UART service UUIDs used by VESC BLE module
static BLEUUID UART_SERVICE_UUID   ("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID UART_CHAR_TX_UUID   ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // ESP32 -> VESC (write)
static BLEUUID UART_CHAR_RX_UUID   ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // VESC -> ESP32 (notify)

// Physical parameters for speed/odo calculation
static const float MOTOR_POLES         = 30.0f;   // magnet count, not pole pairs
static const float WHEEL_DIAMETER_MM   = 246.0f;  // mm
static const float GEAR_RATIO          = 1.0f;    // 1.0 = direct drive

// EEPROM layout
#define EEPROM_SIZE            100
#define EEPROM_MAGIC_ADDR      0
#define EEPROM_TOTAL_KM_ADDR   4
#define EEPROM_MAGIC_VALUE     0xAB

// Connection / timeout parameters
static const unsigned long DATA_TIMEOUT_MS     = 60000UL; // 60s ohne neue Daten -> stale
static const unsigned long RECONNECT_INTERVAL  = 5000UL;  // Scan/Connect-Versuch alle 5s

// Internal BLE state
static BLEScan               *pBLEScan      = nullptr;
static BLEAdvertisedDevice   *vescDevice    = nullptr;
static BLEClient             *pClient       = nullptr;
static BLERemoteCharacteristic  *pRemoteTX  = nullptr;
static BLERemoteCharacteristic  *pRemoteRX  = nullptr;
static bool          bleInitialized        = false;
static bool          deviceFound           = false;
static unsigned long lastReconnectAttempt  = 0;

// EEPROM / odometer state
static bool          eepromReady      = false;
static float         lastSaveKm       = 0.0f;
static float         startup_total_km = 0.0f; // km loaded from EEPROM at boot

// Trip logic (non-persistent)
static float         raw_km_now        = 0.0f; // km (aus tachometerAbs rekonstruiert)
static float         trip_base_km      = 0.0f; // km-Stand bei letztem Reset/Boot
static bool          trip_base_valid   = false;

// RX ring buffer
static uint8_t rxBuf[1024];
static size_t  rxHead = 0;
static size_t  rxTail = 0;
static uint8_t accumBuf[1024];
static size_t  accumLen = 0;

// -----------------------------------------------------
// CRC16 helper (VESC-Style)
// -----------------------------------------------------
static uint16_t crc16_calc(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// -----------------------------------------------------
// Ringbuffer helpers
// -----------------------------------------------------
static void rxPush(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        rxBuf[rxHead] = data[i];
        rxHead = (rxHead + 1) % sizeof(rxBuf);
        if (rxHead == rxTail) {
            rxTail = (rxTail + 1) % sizeof(rxBuf); // drop oldest on overflow
        }
    }
}

static void pullToAccum() {
    while (rxHead != rxTail && accumLen < sizeof(accumBuf)) {
        accumBuf[accumLen] = rxBuf[rxTail];
        rxTail = (rxTail + 1) % sizeof(rxBuf);
        accumLen++;
    }
}

// -----------------------------------------------------
// Payload unpack helpers
// -----------------------------------------------------
static int16_t read_i16(const uint8_t *buf, int32_t *idx) {
    int16_t r = ((int16_t)buf[*idx] << 8) | buf[*idx + 1];
    (*idx) += 2;
    return r;
}

static int32_t read_i32(const uint8_t *buf, int32_t *idx) {
    int32_t r =  ((int32_t)buf[*idx]     << 24) |
                 ((int32_t)buf[*idx + 1] << 16) |
                 ((int32_t)buf[*idx + 2] << 8 ) |
                  (int32_t)buf[*idx + 3];
    (*idx) += 4;
    return r;
}

static float read_f16(const uint8_t *buf, float scale, int32_t *idx) {
    return (float)read_i16(buf, idx) / scale;
}

static float read_f32(const uint8_t *buf, float scale, int32_t *idx) {
    return (float)read_i32(buf, idx) / scale;
}

// -----------------------------------------------------
// EEPROM init
// -----------------------------------------------------
static void initEEPROMIfNeeded() {
    if (eepromReady) return;

    EEPROM.begin(EEPROM_SIZE);

    uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
    if (magic != EEPROM_MAGIC_VALUE) {
        // erster Start / ungültig -> init auf 0 km
        EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
        float zero = 0.0f;
        EEPROM.put(EEPROM_TOTAL_KM_ADDR, zero);
        EEPROM.commit();

        startup_total_km = 0.0f;
    } else {
        // gespeicherten ODO laden
        EEPROM.get(EEPROM_TOTAL_KM_ADDR, startup_total_km);
        if (isnan(startup_total_km) ||
            startup_total_km < 0.0f ||
            startup_total_km > 100000.0f) {
            // falls Schrott drin: hart auf 0
            startup_total_km = 0.0f;
        }
    }

    total_km        = startup_total_km;
    trip_km         = 0.0f;      // Trip beginnt pro Boot frisch
    lastSaveKm      = total_km;
    eepromReady     = true;

    trip_base_valid = false;     // wird beim ersten gültigen Datensatz gesetzt
}

// -----------------------------------------------------
// resetODO() public
//  -> Odo im EEPROM auf 0 km
//  -> Trip neu auf 0 km
//  -> Baseline an aktuelle physische Position setzen
//  -> total_km sofort 0.000 km
// -----------------------------------------------------
void resetODO() {
    EEPROM.begin(EEPROM_SIZE);

    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);

    float zero = 0.0f;
    EEPROM.put(EEPROM_TOTAL_KM_ADDR, zero);
    EEPROM.commit();

    startup_total_km = 0.0f;
    lastSaveKm       = 0.0f;

    trip_base_km      = raw_km_now;
    trip_base_valid   = true;
    trip_km           = 0.0f;

    total_km          = 0.0f;

    eepromReady = true;

    Serial.println("ODO/TRIP RESET -> total_km=0, trip_km=0 set");
}

// -----------------------------------------------------
// timeout checker
// -----------------------------------------------------
static bool data_timed_out() {
    unsigned long now = millis();
    if (vesc_last_update_ms == 0) return true;
    if ((now - vesc_last_update_ms) > DATA_TIMEOUT_MS) return true;
    return false;
}

// -----------------------------------------------------
// Parse COMM_GET_VALUES payload from VESC
// -----------------------------------------------------
static bool parseVescPayload(const uint8_t* payload, int paylen) {
    if (paylen <= 0) return false;

    int32_t ind = 0;
    uint8_t cmd = payload[ind++];
    if (cmd != 4) { // 4 = COMM_GET_VALUES
        return false;
    }

    float tempMosfet       = read_f16(payload,   10.0f,   &ind);
    float tempMotor        = read_f16(payload,   10.0f,   &ind);
    float avgMotorCurrent  = read_f32(payload,  100.0f,   &ind);
    float avgInputCurrent  = read_f32(payload,  100.0f,   &ind);

    float avgId            = read_f32(payload,  100.0f,   &ind);
    float avgIq            = read_f32(payload,  100.0f,   &ind);

    float dutyCycleNow     = read_f16(payload, 1000.0f,   &ind);
    float rpm_erpm         = read_f32(payload,    1.0f,   &ind);
    float inpVoltage       = read_f16(payload,   10.0f,   &ind);

    float ampHours         = read_f32(payload,10000.0f,   &ind);
    float ampHoursCharged  = read_f32(payload,10000.0f,   &ind);
    float wattHours        = read_f32(payload,10000.0f,   &ind);
    float wattHoursCharged = read_f32(payload,10000.0f,   &ind);

    int32_t tachometer_val    = read_i32(payload,         &ind);
    int32_t tachometerAbs_val = read_i32(payload,         &ind);

    int faultCode          = payload[ind++];

    float pidPos           = read_f32(payload,1000000.0f, &ind);
    uint8_t controllerId   = (uint8_t)payload[ind++];

    UART.data.tempMosfet         = tempMosfet;
    UART.data.tempMotor          = tempMotor;
    UART.data.avgMotorCurrent    = avgMotorCurrent;
    UART.data.avgInputCurrent    = avgInputCurrent;
    UART.data.avgId              = avgId;
    UART.data.avgIq              = avgIq;
    UART.data.dutyCycleNow       = dutyCycleNow;
    UART.data.inpVoltage         = inpVoltage;
    UART.data.rpm                = rpm_erpm;
    UART.data.ampHours           = ampHours;
    UART.data.ampHoursCharged    = ampHoursCharged;
    UART.data.wattHours          = wattHours;
    UART.data.wattHoursCharged   = wattHoursCharged;
    UART.data.error              = faultCode;
    UART.data.pidPos             = pidPos;
    UART.data.controllerId       = controllerId;

    watts = avgInputCurrent * inpVoltage;

    float polePairs    =  MOTOR_POLES / 2.0f;
    float mechRPM      =  fabsf(rpm_erpm) / polePairs;
    float wheel_circ_m = (3.141592f * WHEEL_DIAMETER_MM) / 1000.0f;
    speed = ((mechRPM * wheel_circ_m * GEAR_RATIO) / 1000.0f) * 60.0f;

    vesc_tachometer    = tachometer_val;
    vesc_tachometerAbs = tachometerAbs_val;

    float tacho_calc = (float)tachometerAbs_val / (MOTOR_POLES * 3.0f);
    raw_km_now = tacho_calc / 1000.0f;

    if (!trip_base_valid) {
        trip_base_km    = raw_km_now;
        trip_base_valid = true;
    }

    trip_km  = raw_km_now - trip_base_km;
    if (trip_km < 0.0f) trip_km = 0.0f;

    total_km = startup_total_km + trip_km;

    if (eepromReady && (total_km - lastSaveKm) >= 0.1f) {
        EEPROM.put(EEPROM_TOTAL_KM_ADDR, total_km);
        EEPROM.commit();
        lastSaveKm = total_km;
    }

    vesc_last_update_ms = millis();
    connected = true;
    return true;
}

// -----------------------------------------------------
// Consume accumBuf and parse complete frames (CRC-checked)
// -----------------------------------------------------
static bool processAccum() {
    bool updated = false;
    size_t i = 0;

    while ((accumLen - i) >= 5) {
        if (accumBuf[i] != 2) {
            i++;
            continue;
        }

        if ((i + 1) >= accumLen) break;
        uint8_t paylen   = accumBuf[i + 1];
        size_t frameTot  = (size_t)paylen + 5; // 2 start+len + payload + 2 CRC + 1 stop
        if ((accumLen - i) < frameTot) {
            break;
        }

        const uint8_t *payload = &accumBuf[i + 2];
        uint16_t crc_rx = ((uint16_t)accumBuf[i + 2 + paylen] << 8) |
                           (uint16_t)accumBuf[i + 2 + paylen + 1];
        uint8_t stopByte = accumBuf[i + 2 + paylen + 2];

        uint16_t crc_calc = crc16_calc(payload, paylen);

        if (stopByte == 3 && crc_rx == crc_calc) {
            if (parseVescPayload(payload, paylen)) {
                updated = true;
            }
        }

        size_t consumed = i + frameTot;
        size_t remaining = accumLen - consumed;
        memmove(accumBuf, &accumBuf[consumed], remaining);
        accumLen = remaining;
        i = 0;
    }

    return updated;
}

// -----------------------------------------------------
// Notification callback vom BLE RX Characteristic
// -----------------------------------------------------
static void notifyCallback(BLERemoteCharacteristic*, uint8_t* pData, size_t length, bool) {
    rxPush(pData, length);
}

// -----------------------------------------------------
// Build COMM_GET_VALUES request packet
// -----------------------------------------------------
static int buildGetValuesPacket(uint8_t *out) {
    uint8_t payload[1];
    payload[0] = 4; // COMM_GET_VALUES

    uint16_t c = crc16_calc(payload, 1);

    out[0] = 2;
    out[1] = 1;
    out[2] = payload[0];
    out[3] = (c >> 8) & 0xFF;
    out[4] = (c     ) & 0xFF;
    out[5] = 3;

    return 6;
}

// -----------------------------------------------------
// BLE scan callback
// -----------------------------------------------------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (advertisedDevice.getName() == BLE_TARGET_NAME ||
            (advertisedDevice.haveServiceUUID() &&
             advertisedDevice.getServiceUUID().equals(UART_SERVICE_UUID))) {

            if (pBLEScan) {
                pBLEScan->stop();
            }
            if (vescDevice) {
                delete vescDevice;
                vescDevice = nullptr;
            }
            vescDevice = new BLEAdvertisedDevice(advertisedDevice);
            deviceFound = true;
        }
    }
};

// -----------------------------------------------------
// BLE client callbacks
// -----------------------------------------------------
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient*) override {
        Serial.println("BLE Connected to VESC");
    }
    void onDisconnect(BLEClient*) override {
        Serial.println("BLE Disconnected from VESC");
        connected = false;
    }
};

// -----------------------------------------------------
// Start a BLE scan
// -----------------------------------------------------
static void startBLEScan() {
    deviceFound = false;
    if (vescDevice) {
        delete vescDevice;
        vescDevice = nullptr;
    }
    Serial.println("Starting BLE scan...");
    if (pBLEScan) {
        pBLEScan->start(5 /*sec*/, false);
    }
}

// -----------------------------------------------------
// Connect to discovered advertised device
// -----------------------------------------------------
static bool connectToVESC() {
    if (!vescDevice) return false;

    if (pClient) {
        if (pClient->isConnected()) {
            pClient->disconnect();
        }
        delete pClient;
        pClient = nullptr;
    }

    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    if (!pClient->connect(vescDevice)) {
        Serial.println("Failed to connect BLEClient->connect()");
        delete pClient;
        pClient = nullptr;
        return false;
    }

    BLERemoteService* pService = pClient->getService(UART_SERVICE_UUID);
    if (!pService) {
        Serial.println("Failed to get UART service");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        return false;
    }

    pRemoteTX = pService->getCharacteristic(UART_CHAR_TX_UUID);
    pRemoteRX = pService->getCharacteristic(UART_CHAR_RX_UUID);
    if (!pRemoteTX || !pRemoteRX) {
        Serial.println("Failed to get UART characteristics");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        pRemoteTX = nullptr;
        pRemoteRX = nullptr;
        return false;
    }

    // enable notifications on RX characteristic
    if (pRemoteRX->canNotify()) {
        pRemoteRX->registerForNotify(notifyCallback);
    }

    Serial.println("Connected & characteristics ready");
    connected = true;
    return true;
}

// -----------------------------------------------------
// Graceful disconnect
// -----------------------------------------------------
static void disconnectVESC() {
    if (pClient) {
        if (pClient->isConnected()) {
            pClient->disconnect();
        }
        delete pClient;
        pClient = nullptr;
    }
    connected   = false;
    pRemoteTX   = nullptr;
    pRemoteRX   = nullptr;
}

// -----------------------------------------------------
// Public API: BLE connection / reconnect / timeout handling
// -----------------------------------------------------
bool connectble() {
    initEEPROMIfNeeded();

    if (!bleInitialized) {
        BLEDevice::init("VESC-Display");
        pBLEScan = BLEDevice::getScan();
        if (pBLEScan) {
            pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true /*duplicates*/);
            pBLEScan->setActiveScan(true);
            pBLEScan->setInterval(100);
            pBLEScan->setWindow(99);
        }
        bleInitialized = true;

        Serial.println("BLE init done, starting first scan...");
        startBLEScan();
    }

    unsigned long now = millis();

    // Falls wir denken wir sind verbunden, aber Link ist stale/tot -> rauswerfen
    if (connected) {
        if (!pClient || !pClient->isConnected() || data_timed_out()) {
            Serial.println("Link stale or dead, disconnecting...");
            disconnectVESC();
        }
    }

    // Wenn nicht verbunden: entweder connecten oder neu scannen
    if (!connected) {
        if (deviceFound && vescDevice) {
            if (!connectToVESC()) {
                connected = false;
            } else {
                connected = true;
                vesc_last_update_ms = millis(); // baseline timestamp
            }
            deviceFound = false;
        } else {
            if ((now - lastReconnectAttempt) > RECONNECT_INTERVAL) {
                lastReconnectAttempt = now;
                startBLEScan();
            }
        }
    }

    return connected;
}

// -----------------------------------------------------
// Public API: poll telemetry and process RX buffer
// -----------------------------------------------------
bool getvescdata() {
    if (!pClient || !pClient->isConnected() || !pRemoteTX) {
        if (data_timed_out()) connected = false;
        return false;
    }

    // send COMM_GET_VALUES request
    uint8_t req[6];
    int req_len = buildGetValuesPacket(req);
    pRemoteTX->writeValue(req, req_len, false); // write w/o response wait

    // pull notifications and parse
    pullToAccum();
    bool updated = processAccum();

    // Link-Status updaten
    if (updated) {
        connected = true;
    } else if (data_timed_out()) {
        connected = false;
    }

    return updated;
}
