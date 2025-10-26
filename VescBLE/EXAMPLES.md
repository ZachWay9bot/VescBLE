# Beispiele / Integration

Enthaltene Beispiele:

- `examples/VescBLE_Example`
- `examples/VescBLE_TFT_Example_Portrait`

## Voraussetzungen

Hardware:
- ESP32
- VESC (Firmware mit `COMM_GET_VALUES` Support)
- Optional: 240x320 SPI-TFT für das Portrait-Dashboard

Libraries:
- `VescBLE`
- `TFT_eSPI`
- `SPI.h`

`TFT_eSPI` muss in `User_Setup.h` / `User_Setup_Select.h` korrekt konfiguriert sein.

Makros:
- `#define freshEEPROM 0`  
  `0`: Odometer aus EEPROM laden  
  `1`: Beim Boot `resetODO()` → total_km/trip_km starten bei 0.000 km

- `#define BAT_CELLS_OVERRIDE 0` (nur im TFT Beispiel)  
  `0`: automatische Schätzung 10S..20S aus Spannung  
  `>0`: feste Zellanzahl erzwingen (z. B. 12 für 12S)

Runtime-Reset:
- Beide Beispiele hören auf `'r'` (serieller Monitor).  
  `'r'` ruft `resetODO()` auf → total_km und trip_km springen live auf 0.

## 1. VescBLE_Example (seriell)

Zweck:
- Diagnose-Dashboard ohne Display.

Was es macht:
- `connectble()` + `getvescdata()` laufen zyklisch.
- Alle ~500 ms werden per Serial ausgegeben:
  - Speed (km/h)
  - Spannung (V)
  - Temperaturen (MOSFET/Motor)
  - Akku-Strom (BattA) / Motor-Strom (MotA)
  - FOC Id/Iq
  - Duty Cycle (%)
  - Leistung (W)
  - Ah/Wh (inkl. Reku)
  - Trip km / Odometer km
  - tachometer / tachometerAbs
  - Fault Code, Controller-ID, PID Pos
  - BLE Status

Reset:
- `freshEEPROM 1` → hartes Odo-Reset beim Boot
- `'r'` über Serial → Reset jederzeit

## 2. VescBLE_TFT_Example_Portrait (TFT Dashboard)

Zweck:
- Flimmerfreies Echtzeit-Dashboard für ein 240x320-Display im Hochformat.

Aufbau:
1. Speed-Block oben (~90px hoch)
   - Aktuelle Geschwindigkeit (Font7, 7-Segment Optik, bis zu 1 Nachkommastelle)
   - "km/h"
   - MAX Speed Latch (gelb): höchste Session-Geschwindigkeit

2. Telemetrie-Zeilen darunter (TextSize=1, kleine Zeilen, y-step 12px)
   - Volt (Batteriespannung)
   - BattA (avgInputCurrent, Akkustrom)
   - MotA  (avgMotorCurrent, Motorstrom)
   - Id/Iq (FOC d-/q-Achsenströme)
   - Power (W)
   - Duty (% PWM)
   - FET Temp (Controller)
   - Motor Temp
   - Trip (km)
   - Odo (km) = persistent + Session
   - Ah / Wh (Energiezähler)
   - BLE Status YES/NO
   - Batteriezeile: Prozent + Akku-Icon mit Kappe (grün/gelb/rot).
     - Prozent basiert auf S-Erkennung 10S..20S oder `BAT_CELLS_OVERRIDE`
   - CID/PID: Controller-ID und pidPos
   - Fault + tachometerAbs:
     - Fault 0 → grün "OK", sonst rot Fehlercode
     - rechts außen in dunkelgrau: `tachometerAbs` als Debug/Synchronwert

Flimmerfrei:
- Jede Zeile cached den letzten Wert (prevXYZ).
- Es wird **nur neu gezeichnet**, wenn sich etwas geändert hat
  oder die Änderung groß genug ist (Threshold).
- Der große Speed-Block wird in einen `TFT_eSprite` gezeichnet
  und dann als Ganzes gerendert.
- Ergebnis: keine Vollbild-Refreshs und kein hektisches Flackern bei ~20ms Loop.

Trip/Odo-Reset:
- `freshEEPROM 1` → Trip und Odo starten schon beim Boot bei 0.000 km.
- `'r'` im Serial Monitor → ruft `resetODO()` zur Laufzeit auf.
  Danach werden Trip/Odo-Variablen sofort neu gezeichnet.

MAX Speed:
- `maxSpeedSession` speichert die höchste gemessene Geschwindigkeit seit Boot.
- Trip-Reset ändert MAX nicht.
- MAX wird gelb oben im Speed-Block angezeigt.

## 3. Loop-Pattern beider Beispiele

```cpp
void loop() {
    connectble();     // BLE Scan / Connect / Reconnect / Timeout
    getvescdata();    // Telemetrie holen und globale Werte aktualisieren

    // optional: 'r' vom Serial einlesen -> resetODO()

    // Ausgabe:
    // - Serial.print(...) (Console Beispiel)
    // - updateBigSpeed()/updateLineMetricSmall(...) (TFT Beispiel)
}
```

Kernpunkt:
- `connectble()` + `getvescdata()` laufen kontinuierlich.
- Danach sind `speed`, `trip_km`, `total_km`, `connected`
  sowie alles in `UART.data` gültig.

## 4. Fault Handling / Debug

- `UART.data.error` wird als Fault Code angezeigt
  - 0 → OK (grün)
  - !=0 → Fehlercode (rot)

- `tachometerAbs` wird zusätzlich in dunkelgrau rechts unten gezeigt,
  nützlich um zu sehen, ob der Controller-Impulszähler sauber hochzählt.

## 5. TL;DR

- `VescBLE_Example`  
  CLI/Serial-Dashboard für Entwickler und Mess-Nerds.

- `VescBLE_TFT_Example_Portrait`  
  Fahr-Display: Speed groß, Rest klein, flimmerfrei gezeichnet,
  Akku mit Zellerkennung (10S..20S), Duty, Ströme, Temps,
  Trip/Odo mit EEPROM, Fault-Anzeige, MAX Speed Latch.

- Beide Beispiele unterstützen:
  - `freshEEPROM` Odo-Reset beim Boot
  - `'r'` zum Reset während der Fahrt
  - BLE-Verbindung ohne NimBLE
