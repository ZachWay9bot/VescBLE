#include <Arduino.h>
#include <math.h>
#include <VescBLE.h>

#include <TFT_eSPI.h>
#include <SPI.h>

// VescBLE v1.3.4 (Zachway RC15)
// Portrait Dashboard für 240x320 TFT_eSPI
//
// Features:
// - Flimmerfreier Speed-Block mit Font7 (7-Segment-Style) + MAX Speed Latch
// - Akku-% mit automatischer 10S..20S Erkennung oder Override
// - Battery-Icon mit Kappe (grün/gelb/rot)
// - Duty%, Ströme, Id/Iq, Power, Temps, Trip, Odo, Ah, Wh, BLE, CID/PID, Fault
// - tachometerAbs rechts unten in dunkelgrau zum Sync/Debug
// - Trip/Odo Reset per freshEEPROM oder per 'r' im Serial Monitor
//
// freshEEPROM = 1  -> Odo/Trip auf 0 km beim Boot
// freshEEPROM = 0  -> gespeicherten Odo laden
#define freshEEPROM 0

// ======================================
// Akku-Detektion / Override
// ======================================
// Wenn >0: feste Zellanzahl (z.B. 12 für 12S).
// Wenn 0: automatisch S aus Spannung bestimmen (10S..20S).
#define BAT_CELLS_OVERRIDE 0

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

unsigned long lastRedraw        = 0;
unsigned long lastConnectDebug  = 0;

// Cached values für flimmerfreies Redraw
static float    prevSpeed          = NAN;
static float    prevMaxSpeed       = NAN;
static float    prevVoltage        = NAN;
static float    prevBattCurrent    = NAN;
static float    prevMotorCurrent   = NAN;
static float    prevId             = NAN;
static float    prevIq             = NAN;
static float    prevPower          = NAN;
static float    prevDutyPct        = NAN;
static float    prevTempMosfet     = NAN;
static float    prevTempMotor      = NAN;
static float    prevTrip           = NAN;
static float    prevOdo            = NAN;
static float    prevAh             = NAN;
static float    prevWh             = NAN;
static int      prevBleConnected   = -1;
static int      prevBatteryPct     = -1;
static int      prevFaultCode      = -9999;
static int32_t  prevTachoAbs       = 123456789;
static uint8_t  prevCtrlId         = 255;
static float    prevPidPos         = NAN;

// Session-Max-Speed-Latch
static float    maxSpeedSession    = 0.0f;

// Layout
static const int SPEED_BLOCK_H = 90;                 // Höhe der Speed-Anzeige
static const int yBase         = SPEED_BLOCK_H + 10; // Start-Y für Telemetrie
static const int lineH         = 12;                 // Zeilenhöhe bei TextSize=1

// ---------------------------------------------------------
// Zellanzahl-Erkennung 10S..20S
// ---------------------------------------------------------
static int detectCellsFromVoltage(float packVoltage) {
  int bestS = 0;
  float bestErr = 999.0f;

  for (int s = 10; s <= 20; s++) {
    float cellV = packVoltage / (float)s;
    if (cellV < 3.0f || cellV > 4.25f) {
      continue; // unplausible Zellspannung
    }
    float err = fabsf(cellV - 3.7f); // 3.7V ~ nominale Zellspannung
    if (err < bestErr) {
      bestErr = err;
      bestS   = s;
    }
  }

  if (bestS == 0) {
    int guess = (int)roundf(packVoltage / 4.2f);
    if (guess < 10) guess = 10;
    if (guess > 20) guess = 20;
    bestS = guess;
  }

  return bestS;
}

// ---------------------------------------------------------
// Akkuprozent grob linear zwischen 4.20 V/Zelle (100%)
// und 3.30 V/Zelle (0%). Kein BMS-Ersatz, aber praxistauglich.
// ---------------------------------------------------------
static int estimateBatteryPct(float packVoltage) {
  int cells;
  if (BAT_CELLS_OVERRIDE > 0) {
    cells = BAT_CELLS_OVERRIDE;
  } else {
    cells = detectCellsFromVoltage(packVoltage);
  }

  if (cells < 10) cells = 10;
  if (cells > 20) cells = 20;

  float cellV = packVoltage / (float)cells;

  float pct = (cellV - 3.30f) / (4.20f - 3.30f) * 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;

  return (int)roundf(pct);
}

// ---------------------------------------------------------
// Akku-Icon mit Kappe
// ---------------------------------------------------------
static void drawBatteryIcon(int16_t x, int16_t y, int pct) {
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;

  const int bw   = 30;
  const int bh   = 12;
  const int capW = 3;
  const int capH = 6;

  // Rahmen
  tft.drawRect(x, y, bw, bh, TFT_WHITE);
  // Kappe
  tft.drawRect(x + bw, y + (bh/2 - capH/2), capW, capH, TFT_WHITE);

  int fillW = (bw - 2) * pct / 100;
  if (fillW < 0) fillW = 0;

  uint16_t fillColor = TFT_GREEN;
  if (pct < 20) {
    fillColor = TFT_RED;
  } else if (pct < 50) {
    fillColor = TFT_YELLOW;
  }

  if (fillW > 0) {
    tft.fillRect(x + 1, y + 1, fillW, bh - 2, fillColor);
  }
}

// ---------------------------------------------------------
// Speed-Header Sprite mit MaxSpeed-Latch
// ---------------------------------------------------------
static void updateBigSpeed(float spd) {
  // MAX Speed aktualisieren
  if (spd > maxSpeedSession) {
    maxSpeedSession = spd;
  }

  bool needRedrawMain = (isnan(prevSpeed)    || fabsf(spd - prevSpeed) >= 0.05f);
  bool needRedrawMax  = (isnan(prevMaxSpeed) || fabsf(maxSpeedSession - prevMaxSpeed) >= 0.05f);

  if (needRedrawMain || needRedrawMax) {
    tft.fillRect(0, 0, tft.width(), SPEED_BLOCK_H, TFT_BLACK);

    int spriteW = tft.width();
    int spriteH = 80;
    spr.setColorDepth(8);
    spr.createSprite(spriteW, spriteH);
    spr.fillSprite(TFT_BLACK);

    spr.setTextColor(TFT_CYAN, TFT_BLACK);

    // Label "SPD"
    spr.drawString("SPD", 4, 0, 2);

    // Hauptgeschwindigkeit mit Font 7 (7-Segment Optik)
    int numX = 10;
    int numY = 20;
    spr.drawFloat(spd, 1, numX, numY, 7);

    // Einheit "km/h"
    spr.setTextColor(TFT_CYAN, TFT_BLACK);
    spr.drawString("km/h", spriteW - 60, spriteH - 28, 4);

    // MAX Speed Anzeige (gelb) oben rechts
    {
      char buf[16];
      dtostrf(maxSpeedSession, 0, 1, buf);
      spr.setTextColor(TFT_YELLOW, TFT_BLACK);
      spr.drawString("MAX", spriteW - 60, 0, 2);
      spr.drawString(buf,  spriteW - 60, 16, 2);
    }

    spr.pushSprite(0, 0);
    spr.deleteSprite();

    prevSpeed     = spd;
    prevMaxSpeed  = maxSpeedSession;
  }
}

// ---------------------------------------------------------
// Generische Telemetrie-Zeile (Float)
// ---------------------------------------------------------
static void updateLineMetricSmall(const char *label,
                                  float value,
                                  const char *unit,
                                  int y,
                                  int dec,
                                  float &prevVal,
                                  float threshold = 0.05f,
                                  uint16_t color = TFT_GREEN) {
  if (isnan(prevVal) || fabsf(value - prevVal) >= threshold) {
    tft.fillRect(0, y, tft.width(), lineH, TFT_BLACK);

    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(2, y);
    tft.print(label);
    tft.print(": ");
    tft.print(value, dec);
    if (unit && unit[0]) {
      tft.print(" ");
      tft.print(unit);
    }

    prevVal = value;
  }
}

// Duty Cycle
static void updateDutySmall(float dutyPct, int y) {
  if (isnan(prevDutyPct) || fabsf(dutyPct - prevDutyPct) >= 0.2f) {
    tft.fillRect(0, y, tft.width(), lineH, TFT_BLACK);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(2, y);
    tft.print("Duty: ");
    tft.print(dutyPct, 1);
    tft.print(" %");

    prevDutyPct = dutyPct;
  }
}

// FOC Id/Iq
static void updateIdIqSmall(float idVal, float iqVal, int y) {
  bool need = false;
  if (isnan(prevId) || fabsf(idVal - prevId) >= 0.1f) need = true;
  if (isnan(prevIq) || fabsf(iqVal - prevIq) >= 0.1f) need = true;

  if (need) {
    tft.fillRect(0, y, tft.width(), lineH, TFT_BLACK);

    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(2, y);
    tft.print("Id/Iq: ");
    tft.print(idVal, 1);
    tft.print("/");
    tft.print(iqVal, 1);
    tft.print(" A");

    prevId = idVal;
    prevIq = iqVal;
  }
}

// BLE Status
static void updateBleSmall(bool isUp, int y) {
  int newState = isUp ? 1 : 0;
  if (prevBleConnected != newState) {
    tft.fillRect(0, y, tft.width(), lineH, TFT_BLACK);

    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(2, y);
    tft.print("BLE: ");
    tft.print(isUp ? "YES" : "NO");

    prevBleConnected = newState;
  }
}

// Battery-Zeile inkl. % und Icon mit Kappe
static void updateBatteryRowSmall(float packVoltage, int y) {
  int pct = estimateBatteryPct(packVoltage);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;

  if (prevBatteryPct != pct) {
    tft.fillRect(0, y, tft.width(), lineH + 8, TFT_BLACK);

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(2, y);
    tft.print("BAT: ");
    tft.print(pct);
    tft.print("%");

    drawBatteryIcon(60, y + 1, pct);

    prevBatteryPct = pct;
  }
}

// Controller ID / PIDPos
static void updateCtrlPidSmall(uint8_t ctrl, float pid, int y) {
  bool need = false;
  if (prevCtrlId != ctrl) need = true;
  if (isnan(prevPidPos) || fabsf(pid - prevPidPos) >= 0.0001f) need = true;

  if (need) {
    tft.fillRect(0, y, tft.width(), lineH, TFT_BLACK);

    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(2, y);
    tft.print("CID:");
    tft.print(ctrl);
    tft.print(" PID:");
    tft.print(pid, 4);

    prevCtrlId = ctrl;
    prevPidPos = pid;
  }
}

// Fault + tachometerAbs rechts grau
static void updateFaultAndTachoSmall(int faultCode, int32_t tachoAbs, int y) {
  bool need = false;
  if (prevFaultCode != faultCode) need = true;
  if (prevTachoAbs  != tachoAbs ) need = true;

  if (need) {
    tft.fillRect(0, y, tft.width(), lineH, TFT_BLACK);

    // Fault links
    tft.setTextSize(1);
    tft.setCursor(2, y);
    tft.setTextColor((faultCode == 0) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print("FAULT:");
    if (faultCode == 0) {
      tft.print("OK");
    } else {
      tft.print(faultCode);
    }

    // tachometerAbs rechts in dunkelgrau
    String tachoStr = String(tachoAbs);
    int16_t len   = tachoStr.length();
    int16_t charW = 6 * 1;
    int16_t startX = tft.width() - (len * charW) - 2;
    if (startX < 0) startX = 0;

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(startX, y);
    tft.print(tachoStr);

    prevFaultCode = faultCode;
    prevTachoAbs  = tachoAbs;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

#if freshEEPROM
  resetODO();
  Serial.println("EEPROM ODO HARD RESET via freshEEPROM macro.");
#endif

  maxSpeedSession = 0.0f;
  prevMaxSpeed    = NAN;

  tft.init();
  tft.setRotation(0); // Portrait
  tft.fillScreen(TFT_BLACK);
  tft.setTextWrap(false);

  // Boot-Text
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(2, 2);
  tft.println("VESC BLE Dash Portrait RC15");
  tft.setCursor(2, 14);
  tft.println("Connecting...");

  Serial.println("Portrait dashboard RC15");
  Serial.println("Serial 'r' -> resetODO() / Trip+Odo = 0 km");
  Serial.println("Use BAT_CELLS_OVERRIDE to force S-count if needed.");
}

void loop() {
  connectble();
  bool fresh = getvescdata();

  // Runtime Reset via Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      resetODO();
      Serial.println("ODO/TRIP RESET MANUAL -> total=0 km, trip=0 km");

      // Force redraw für Distanz-bezogene Werte
      prevTrip = NAN;
      prevOdo  = NAN;
      prevAh   = NAN;
      prevWh   = NAN;
      // maxSpeedSession bleibt erhalten
    }
  }

  unsigned long now = millis();

  if (now - lastRedraw > 20) {
    lastRedraw = now;

    // 1) Speed + MAX
    updateBigSpeed(speed);

    // 2) Telemetrie
    int y = yBase;

    updateLineMetricSmall("Volt", UART.data.inpVoltage, "V",
                          y, 2, prevVoltage, 0.02f, TFT_GREEN);
    y += lineH;

    updateLineMetricSmall("BattA", UART.data.avgInputCurrent, "A",
                          y, 2, prevBattCurrent, 0.05f, TFT_GREEN);
    y += lineH;

    updateLineMetricSmall("MotA", UART.data.avgMotorCurrent, "A",
                          y, 2, prevMotorCurrent, 0.05f, TFT_GREEN);
    y += lineH;

    updateIdIqSmall(UART.data.avgId, UART.data.avgIq, y);
    y += lineH;

    updateLineMetricSmall("Power", watts, "W",
                          y, 1, prevPower, 0.2f, TFT_GREEN);
    y += lineH;

    float dutyPct = UART.data.dutyCycleNow * 100.0f;
    updateDutySmall(dutyPct, y);
    y += lineH;

    updateLineMetricSmall("FET", UART.data.tempMosfet, "C",
                          y, 1, prevTempMosfet, 0.1f, TFT_GREEN);
    y += lineH;

    updateLineMetricSmall("Motor", UART.data.tempMotor, "C",
                          y, 1, prevTempMotor, 0.1f, TFT_GREEN);
    y += lineH;

    updateLineMetricSmall("Trip", trip_km, "km",
                          y, 3, prevTrip, 0.001f, TFT_GREEN);
    y += lineH;

    updateLineMetricSmall("Odo", total_km, "km",
                          y, 3, prevOdo, 0.001f, TFT_GREEN);
    y += lineH;

    updateLineMetricSmall("Ah", UART.data.ampHours, "Ah",
                          y, 3, prevAh, 0.001f, TFT_GREEN);
    y += lineH;

    updateLineMetricSmall("Wh", UART.data.wattHours, "Wh",
                          y, 3, prevWh, 0.001f, TFT_GREEN);
    y += lineH;

    updateBleSmall(connected, y);
    y += lineH;

    updateBatteryRowSmall(UART.data.inpVoltage, y);
    y += (lineH + 8);

    updateCtrlPidSmall(UART.data.controllerId, UART.data.pidPos, y);
    y += lineH;

    updateFaultAndTachoSmall(UART.data.error, vesc_tachometerAbs, y);
    y += lineH;
  }

  // Debug alle 500 ms auf Serial
  if (now - lastConnectDebug > 500) {
    lastConnectDebug = now;
    int debugCells = (BAT_CELLS_OVERRIDE > 0)
                   ? BAT_CELLS_OVERRIDE
                   : detectCellsFromVoltage(UART.data.inpVoltage);

    Serial.print("BLE ");
    Serial.print(connected ? "OK " : "NO ");
    Serial.print("| SPD ");
    Serial.print(speed, 1);
    Serial.print(" km/h | MAX ");
    Serial.print(maxSpeedSession, 1);
    Serial.print(" | V ");
    Serial.print(UART.data.inpVoltage, 2);
    Serial.print(" | Cells ");
    Serial.print(debugCells);
    Serial.print("S | BattA ");
    Serial.print(UART.data.avgInputCurrent, 2);
    Serial.print(" | MotA ");
    Serial.print(UART.data.avgMotorCurrent, 2);
    Serial.print(" | Duty ");
    Serial.print(UART.data.dutyCycleNow * 100.0f, 1);
    Serial.print("% | Trip ");
    Serial.print(trip_km, 3);
    Serial.print(" km | Odo ");
    Serial.print(total_km, 3);
    Serial.print(" km | TAbs ");
    Serial.print(vesc_tachometerAbs);
    Serial.print(" | Fault ");
    Serial.print(UART.data.error);
    Serial.print(" | CID ");
    Serial.print(UART.data.controllerId);
    Serial.print(" | PID ");
    Serial.println(UART.data.pidPos, 4);
  }

  delay(20);
}
