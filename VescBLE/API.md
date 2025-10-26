# VescBLE API

Version: v1.3.4 (RC15)

VescBLE ist eine ESP32-Bibliothek zur BLE-Auslesung eines VESC-Controllers.  
Sie stellt Verbindung (Scan, Connect, Reconnect, Timeout) ohne NimBLE her, fragt periodisch Telemetriedaten über `COMM_GET_VALUES` ab und bietet fertige Werte wie Geschwindigkeit, Akkustrom, Temperatur und Odometer.

## 1. Laufzeit

```cpp
connectble();    // verwaltet BLE-Scan, Verbindungsaufbau, Reconnect, Timeout
getvescdata();   // holt Telemetrie vom VESC und aktualisiert alle Werte
```

Nach diesen Aufrufen stehen die globalen Messwerte bereit.

## 2. Globale Messwerte

```cpp
extern bool  connected;
extern unsigned long vesc_last_update_ms;

extern float speed;     // km/h
extern float watts;     // W
extern float trip_km;   // km seit Reset/Boot
extern float total_km;  // km Gesamt-Odometer (persistent + Session)

extern int32_t vesc_tachometer;
extern int32_t vesc_tachometerAbs;
```

- `connected`: BLE aktiv + letzte gültige Messung <60 s alt.  
- `speed`: km/h am Rad (aus eRPM berechnet).  
- `watts`: Akku-Leistung = `avgInputCurrent * inpVoltage`.  
- `trip_km`: Session-Distanz.  
- `total_km`: Gesamtstrecke in km (EEPROM + aktuelle Fahrt).  
- `vesc_tachometerAbs`: absoluter Tachozähler vom VESC (Debug / Sync).  
- `vesc_last_update_ms`: Zeitstempel (millis) der letzten gültigen Daten.

## 3. Rohdatenstruktur

```cpp
class ComEVesc {
public:
    struct Data {
        float inpVoltage;         // V  Batteriespannung
        float tempMosfet;         // °C MOSFET-/Controller-Temp
        float tempMotor;          // °C Motor-Temp

        float avgInputCurrent;    // A  Akku-/Eingangsstrom ("BattA")
        float avgMotorCurrent;    // A  Motor-/Phasenstrom ("MotA")

        float avgId;              // A  FOC d-Achse
        float avgIq;              // A  FOC q-Achse (Drehmomentanteil)

        float dutyCycleNow;       // 0..1 PWM Duty Cycle
        float rpm;                // eRPM

        float ampHours;           // Ah entnommen
        float ampHoursCharged;    // Ah rekuperiert
        float wattHours;          // Wh entnommen
        float wattHoursCharged;   // Wh rekuperiert

        int   error;              // Fault Code (0=OK)
        float pidPos;             // PID Position
        uint8_t controllerId;     // Controller-/CAN-ID
    } data;
};

extern ComEVesc UART;
```

Wichtigste Felder:
- `inpVoltage`: Akkuspannung.  
- `avgInputCurrent` / `avgMotorCurrent`: Akku- vs. Motorstrom.  
- `avgIq`: q-Achsenstrom → direkt mit Drehmoment verknüpft.  
- `dutyCycleNow`: Aussteuerung (0..1).  
- `ampHours` / `wattHours`: Energiezähler.  
- `error`: Fault Code vom VESC.  
- `controllerId`: CAN-ID / Controller-ID.  
- `pidPos`: Positions-Info aus bestimmten VESC-Modi.

## 4. Odometer / Trip / EEPROM

Die Lib liest `tachometerAbs` vom VESC (absoluter Wegzähler), rekonstruiert daraus km und macht daraus:

- `trip_km`: km seit letztem Reset.  
- `total_km`: km gesamt (persistiert im EEPROM).

EEPROM wird nur alle ~0,1 km aktualisiert, um Flash-Schäden zu vermeiden.

### Reset

```cpp
void resetODO();
```

`resetODO()` setzt:
- Odometer im EEPROM auf 0 km,
- `trip_km` auf 0 km,
- `total_km` auf 0 km,
- und verschiebt die Baseline auf die aktuelle physische Radposition.

Kann beim Boot erzwungen werden (per `#define freshEEPROM 1`) oder zur Laufzeit (z. B. über `'r'` im Serial Monitor).

## 5. Akku-SoC-Schätzung (TFT Beispiel)

- Die Bibliothek schätzt die Zellanzahl (10S…20S) anhand der gemessenen Packspannung.  
- Alternativ kann per `#define BAT_CELLS_OVERRIDE` eine feste Zellanzahl gesetzt werden.  
- Aus Zellspannung wird linear auf 0…100 % gemappt:  
  4.20 V/Zelle → 100 %  
  3.30 V/Zelle →   0 %

Das ist ein Dashboard-Indikator, kein exakter BMS-SoC.

## 6. Öffentliche Funktionen

```cpp
bool connectble();
bool getvescdata();
void resetODO();
```

- `connectble()` initialisiert BLE, scannt nach `"VESC BLE UART"`, verbindet, reconnectet, schmeißt tote Verbindungen raus. Gibt `true` zurück, wenn verbunden.
- `getvescdata()` fragt `COMM_GET_VALUES` beim VESC ab, prüft CRC16, parsed alles in `UART.data`, aktualisiert `speed`, `trip_km`, `total_km`, usw. Gibt `true` zurück, wenn neue Daten angekommen sind.
- `resetODO()` setzt Odometer (EEPROM) und Trip auf 0 km.

## 7. Lizenz

MIT License. Siehe `LICENSE`.
