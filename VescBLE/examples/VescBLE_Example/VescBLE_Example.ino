#include <Arduino.h>
#include <VescBLE.h>

// VescBLE v1.3.4 (Zachway RC15)
// Serielles Dashboard mit voller Telemetrie + ODO/TRIP Reset.
//
// freshEEPROM = 1  -> persistenten Odometer (EEPROM) beim Boot löschen -> 0 km
// freshEEPROM = 0  -> gespeicherten Odometer laden/weiterführen
#define freshEEPROM 0

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

#if freshEEPROM
  resetODO();
  Serial.println("EEPROM ODO HARD RESET via freshEEPROM macro.");
#endif

  Serial.println("VESC BLE Dash Boot (v1.3.4 Zachway RC15)");
  Serial.println("Press 'r' + Enter in Serial Monitor to reset ODO/TRIP to 0 km");
}

void loop() {
  connectble();
  bool fresh = getvescdata();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      resetODO();
      Serial.println("ODO/TRIP RESET MANUAL -> total=0 km, trip=0 km");
    }
  }

  unsigned long now = millis();
  if (now - lastPrint > 500) {
    lastPrint = now;

    Serial.println("========== VESC DATA ==========");
    Serial.print("Speed km/h:         "); Serial.println(speed, 1);
    Serial.print("Voltage V:         "); Serial.println(UART.data.inpVoltage, 2);
    Serial.print("Temp Mosfet °C:    "); Serial.println(UART.data.tempMosfet, 2);
    Serial.print("Temp Motor  °C:    "); Serial.println(UART.data.tempMotor, 2);

    Serial.print("Batt Current A:    "); Serial.println(UART.data.avgInputCurrent, 2);
    Serial.print("Motor Current A:   "); Serial.println(UART.data.avgMotorCurrent, 2);
    Serial.print("FOC Id / Iq A:     "); Serial.print(UART.data.avgId, 2);
    Serial.print(" / ");
    Serial.println(UART.data.avgIq, 2);

    Serial.print("Duty Cycle %:      "); Serial.println(UART.data.dutyCycleNow * 100.0f, 2);

    Serial.print("Power W:           "); Serial.println(watts, 2);

    Serial.print("Ah used:           "); Serial.println(UART.data.ampHours, 4);
    Serial.print("Ah charged:        "); Serial.println(UART.data.ampHoursCharged, 4);
    Serial.print("Wh used:           "); Serial.println(UART.data.wattHours, 4);
    Serial.print("Wh charged:        "); Serial.println(UART.data.wattHoursCharged, 4);

    Serial.print("Trip km:           "); Serial.println(trip_km, 3);
    Serial.print("Odometer km:       "); Serial.println(total_km, 3);

    Serial.print("tachometer:        "); Serial.println(vesc_tachometer);
    Serial.print("tachometerAbs:     "); Serial.println(vesc_tachometerAbs);

    Serial.print("Fault Code:        "); Serial.println(UART.data.error);
    Serial.print("Controller ID:     "); Serial.println(UART.data.controllerId);
    Serial.print("PID Pos:           "); Serial.println(UART.data.pidPos, 6);

    Serial.print("Last Update ms:    "); Serial.println(vesc_last_update_ms);
    Serial.print("BLE Connected:     "); Serial.println(connected ? "YES" : "NO");
    Serial.println("===============================");
  }

  delay(50);
}
