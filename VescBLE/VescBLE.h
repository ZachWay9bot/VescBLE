#pragma once
#ifndef VESCBLE_H
#define VESCBLE_H

#include <Arduino.h>

// Data structure for telemetry from VESC (COMM_GET_VALUES)
class ComEVesc {
  public:
    struct Data {
      float inpVoltage;         // V (battery pack voltage)
      float tempMosfet;         // °C (MOSFET temperature)
      float tempMotor;          // °C (motor temperature)
      float avgInputCurrent;    // A (battery/input current)
      float avgMotorCurrent;    // A (motor/phase current)
      float avgId;              // A (FOC d-axis current)
      float avgIq;              // A (FOC q-axis current / torque-producing)
      float dutyCycleNow;       // -1..1 (PWM duty cycle, usually 0..1)
      float rpm;                // electrical RPM (eRPM)

      float ampHours;           // Ah drawn
      float ampHoursCharged;    // Ah charged back in
      float wattHours;          // Wh drawn
      float wattHoursCharged;   // Wh charged back in

      int   error;              // fault code
      float pidPos;             // PID position (if supported by FW)
      uint8_t controllerId;     // controller ID
    } data;
};

// Public globals populated after getvescdata()
extern ComEVesc UART;

extern float speed;                  // km/h (wheel speed)
extern float watts;                  // W (battery side power = V * batt current)
extern float total_km;               // km total (persistent ODO + session distance)
extern float trip_km;                // km since last reset/boot
extern bool  connected;              // true if BLE link is alive AND data fresh <60s

extern int32_t vesc_tachometer;      // incremental tachometer counter from VESC
extern int32_t vesc_tachometerAbs;   // absolute tachometer counter from VESC

extern unsigned long vesc_last_update_ms; // millis() of last valid dataset

// API
// connectble() maintains scan/connect/reconnect/timeout without NimBLE.
// getvescdata() polls COMM_GET_VALUES and parses all telemetry into globals.
bool connectble();
bool getvescdata();

// resetODO() sets the stored odometer in EEPROM back to 0 km and also resets
// trip_km to 0 km from the current physical wheel position.
void resetODO();

#endif // VESCBLE_H
