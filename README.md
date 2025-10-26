ESP32 BLE Library für VESC (Firmware `COMM_GET_VALUES`).  
Version: v1.3.4 (RC15) • Autor: Zachway • Lizenz: MIT

Highlights:
- BLE Connect/Reconnect/Timeout-Logik ohne NimBLE
- Geschwindigkeit (km/h), Leistung (W), Ströme (BattA/MotA), Duty, FOC Id/Iq
- Temperaturen, Ah/Wh Counter, Controller-ID, pidPos
- Odometer im EEPROM + Trip Reset
- 240x320 Portrait-TFT Dashboard mit flimmerfreier Anzeige, MAX Speed Latch,
  Akku-Prozent mit automatischer 10S..20S-Erkennung oder Override

Doku:
- Siehe `API.md` für alle verfügbaren Felder, Globale Variablen, Funktionen.
- Siehe `EXAMPLES.md` für Serial- und TFT-Dashboards, flimmerfreie Updates,
  Trip/Odo Reset, Akku-Anzeige, MAX Speed.

Beispiele liegen unter `examples/`.
Lizenz: MIT (siehe `LICENSE`).
