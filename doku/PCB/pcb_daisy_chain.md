# R0192 "Daisy Chain" Driverlink-PCB — Konzept & Pre-Fab-Doku

## Kontext & Ziel

Das **Daisy-Chain-Board** ist die Bus-Verteilplatine, die pro Roboterachse einmal verbaut wird. Sie schleift die zentralen Versorgungen vom Rack-Tether (48 V, 5 V) sowie den CAN-Bus von Node zu Node durch und zweigt an jeder Achse die Motorversorgung ab. Zusätzlich wertet sie lokal den Homing-Hall-Sensor (TLE4905L) aus und bietet Erweiterungsanschlüsse für spätere Sensorik/LEDs.

Damit ist das Board die Hardware-Entsprechung zum Software-Homing-Node (`microcontroller/r0192_homing.ino`): Der bisherige Arduino-Uno-Prototyp mit separatem MCP2515 wird durch den **XIAO-ESP32-S3** mit integriertem CAN-Controller (TWAI) ersetzt.

**Designprinzip:** Identische Platine an jeder Achsposition einsetzbar — alle positionsabhängigen Eigenschaften (v. a. Bus-Termination) sind per Jumper konfigurierbar.

---

## Funktionsblöcke

| Block | Funktion |
| --- | --- |
| **48 V-Bus-Durchschleife** | Eingang → Ausgang (Daisy Chain) + Abzweig zum Motor |
| **5 V-Bus-Durchschleife** | Eingang → Ausgang (Daisy Chain), versorgt Logik |
| **CAN-Bus-Durchschleife** | CAN_H / CAN_L von Node zu Node, Termination jumperbar |
| **3,3 V-Erzeugung** | AP2112K-3.3 LDO aus 5 V, versorgt ESP32-Peripherie & Logik |
| **MCU** | XIAO-ESP32-S3 (gesockelt, tauschbar), CAN via TWAI |
| **CAN-Transceiver** | TJA1051T, 3,3 V-seitig, S-Pin auf GND (Normalbetrieb) |
| **Homing-Sensor** | TLE4905L Hall (Open-Collector), Pegelwandlung via Pull-up |
| **Erweiterung** | Freie GPIOs (D0/D1/D3) + 3,3 V + 5 V auf Stiftleiste |

---

## Schlüssel-Designentscheidungen

### MCP2515 eliminiert
Der ESP32-S3 hat einen integrierten CAN-Controller (TWAI). Dadurch entfallen MCP2515, Quarz, 2× 22 pF und der SPI-Pull-up gegenüber dem Arduino-Prototyp. Der TJA1051T bleibt als physischer Transceiver zwingend erforderlich. Die TX/RX-Pins des ESP32 (D6_TX / D7_RX) werden direkt mit dem TJA1051T verbunden.

> **Firmware-Konsequenz:** Umstieg von `autowp/mcp2515` auf `ESP32-TWAI-CAN` bzw. natives `driver/twai.h`. Das CAN-Protokoll (achsenspezifische CAN-ID, `CMD_ARM` / `RSP_DETECTED` / `RSP_ERROR`) bleibt unverändert. TX/RX-Pin-Zuordnung muss zwischen PCB und Firmware konsistent dokumentiert sein.

### Logik auf 3,3 V — TJA1051 als automatische Pegelanpassung
Der TJA1051T wird 3,3 V-seitig versorgt. Damit liegen alle Logikpegel (CAN-TX/RX, Sensor) auf 3,3 V → kein Logik-Level-Mismatch (war ein Befund aus dem früheren PCB-Review). Die 3,3 V werden lokal per AP2112K-3.3 aus der durchgeschleiften 5 V-Schiene erzeugt; es ist **kein** 48 V→3,3 V-Wandler nötig, da 5 V ohnehin über den Tether mitgeführt wird (Quelle: MeanWell LRS-50).

### TLE4905L — Open-Collector, Versorgung 5 V, Pull-up auf 3,3 V
Der TLE4905L benötigt mindestens 3,8 V Versorgung und wird daher aus 5 V betrieben. Sein Open-Collector-Ausgang wird über einen Pull-up (R1, 10 kΩ) auf **3,3 V** gezogen — der High-Pegel wird vom Pull-up bestimmt, nicht von der Sensorversorgung. Das ergibt einen für den ESP32-GPIO sicheren 3,3 V-Pegel und vermeidet, den GPIO mit 5 V zu beschädigen. Entkopplung: 100 nF (C1) direkt am Sensor.

> Pull-up bleibt bei 10 kΩ, da die Sensorleitung kurz ist (≤ 10 cm). Bei längeren Leitungen wäre 4,7 kΩ störungsärmer.

### Bus-Termination jumperbar
CAN-Termination (120 Ω) darf **nur an den beiden physischen Busenden** sitzen, nicht auf jedem Node (Impedanzproblem — Befund aus früherem Review). Realisierung: 120 Ω fest bestückt, über Jumper (JP1) parallel zu CAN_H/CAN_L zuschaltbar. So ist dieselbe Platine an jeder Position einsetzbar; Jumper nur an den Endknoten gesteckt.

### ESP32 gesockelt
Der XIAO-ESP32-S3 wird über Buchsenleisten (J12–J15) aufgesteckt, nicht verlötet → einfacher Austausch bei Defekt. Footprints müssen exakt zum XIAO-Rastermaß passen.

### Bremse extern
Die Failsafe-Bremsen sind in den Treibern (GDS68/RS05) integriert und benötigen keine Versorgung/Ansteuerung über dieses Board. Damit entfallen Brake-Spule und Freewheeling-Diode auf dem PCB.

---

## Spannungsdomänen

| Domäne | Quelle | Versorgt | Stecker |
| --- | --- | --- | --- |
| **48 V** | Tether-Bus (durchgeschleift) | Motorabzweig | XT60PW (Bus), XT30PB(2+2)-M (Motor) |
| **5 V** | Tether-Bus (durchgeschleift) | XIAO (5 V-Pin), TLE4905L, AP2112K-Eingang | XT30PW |
| **3,3 V** | AP2112K-3.3 (aus 5 V) | TJA1051, Pull-up, Erweiterung | — (on-board) |
| **GND** | gemeinsam (ein Netz) | alle | über alle Power-Stecker |

---

## Steckerkonzept

Bewusst unterschiedliche Steckertypen pro Domäne → mechanische Fehlsteck-Sicherheit (kein Stecker passt in den falschen Port), Polung gibt zugleich Verpolschutz.

| Funktion | Steckertyp | Begründung |
| --- | --- | --- |
| 48 V Power-Bus (durchschleifen) | **XT60PW** | hoher Strom, robust, gepolt |
| 48 V Motor-Abgriff | **XT30PB(2+2)-M** | 48 V + 2 Signalpins, eigene Geometrie |
| 5 V (Ein-/Ausgang) | **XT30PW** | Strom über JST-XH hinaus, gepolt |
| CAN-Bus (H/L) | **XH-2A (JST-XH)** | Signalpegel, ausreichend |
| Hall-Sensor TLE4905L | JST (J4/J5, 3-polig) | 5 V / Signal / GND |
| Erweiterung | Stiftleiste (J10, 6-polig) | D0/D1/D3 + 3,3 V + 5 V + GND |

> **CAN-GND:** Die XH-2A-CAN-Stecker führen nur CAN_H/CAN_L. Eine gemeinsame GND-Referenz zwischen allen Nodes ist für CAN zwingend — sie wird über die durchgehende GND der Power-Schienen (48 V/5 V) sichergestellt. Verifizieren, dass GND wirklich durchgängig über alle Boards verbunden ist.

---

## Masseführung (GND-Konzept)

**Ein einziges GND-Netz** (`GND`) für das gesamte Board — eine gemeinsame Referenz ist Pflicht (sonst kein CAN). Entscheidend ist die *Layout-Führung*, nicht eine Netztrennung:

- Elektrisch ein Netz, layout-seitig gedankliche Trennung in **Power-GND** (48 V-Rückpfad: XT60-Bus, XT30-Motor, 48 V-Kondensatorbank) und **Signal-GND** (ESP32, TJA1051, TLE4905L, AP2112K + deren Cs).
- Beide treffen sich an **einem Sternpunkt**, idealerweise am 5 V/GND-Eingang bzw. am 48 V-Bulk-Elko.
- Durchgehende **GND-Massefläche** (Pour), bei 2-Layer typischerweise Bottom.
- Motor-/Bus-GND mit breitem Kupfer **direkt** zwischen XT-Steckern und Kondensatorbank — nicht quer durch die Logik-Zone.
- Logik räumlich von den 48 V-Leistungsbahnen entfernt platzieren, sodass kein Motorstrom geometrisch durch die Logik-Masse fließt.

---

## Stützkondensatoren & Schutz

| Bauteil | Wert | Position | Zweck |
| --- | --- | --- | --- |
| C2, C3 | 100 µF / **≥ 63 V** | 48 V-Bus | Bulk-Puffer, Regen-Spikes |
| C4 | 100 nF | 48 V-Bus | HF-Entkopplung |
| C8 | 10 µF | AP2112K VIN (5 V) | Bulk gegen Lasteinbrüche/Spikes |
| C5 | 1 µF | AP2112K VIN | HF-Stabilität LDO-Eingang |
| C6 | 1 µF | AP2112K VOUT | LDO-Stabilität Ausgang |
| C7 | 100 nF | TJA1051 VCC (3,3 V) | IC-Entkopplung |
| C1 | 100 nF | TLE4905L VCC | Sensor-Entkopplung |

> **AP2112K-Hinweis:** Absolute-Max VIN = 6,0 V. Bei 5 V-Nominal ist das knapp; C8 (10 µF) dämpft Überschwinger. Optional als zusätzlicher Schutz eine TVS-Diode (z. B. SMAJ5.0A) zwischen 5 V und GND am Eingang.

---

## Strom-Dimensionierung (Kupfer)

- **48 V-Bus-Durchschleife (XT60):** auf das Netzteil-Limit auslegen — LRS-600N2-48 liefert max. ~12,5 A. Bus-Bahnen auf ~13 A dimensionieren.
- **48 V-Motorabzweig (XT30):** auf den **Spitzenstrom eines einzelnen Motors** (GIM8108) auslegen, nicht auf die Summe.
- **Theoretische Summe** aller Motor-Spitzenströme = 83,6 A — wird vom Netzteil ohnehin nie geliefert und tritt real nicht gleichzeitig auf; für die Bus-Dimensionierung **nicht** maßgeblich.
- **5 V-Schiene:** LRS-50 liefert max. ~10 A bei 5 V → Obergrenze. XT30PW gewählt, da JST-XH stromseitig zu knapp.

---

## Status & ERC

ERC: **0 Errors / 0 Warnings** (Stand letzter Review).

Behobene ERC-Punkte:
- PWR_FLAG auf +48 V, +5 V (Eingänge) und +3,3 V (AP2112K-Ausgang) gesetzt.
- NC-Flags auf ungenutzte ESP32-Sockel-Pins.
- AP2112K VIN + EN sauber an +5 V verbunden.
- TJA1051 Pin 8 (S) auf GND (Normalbetrieb) — **nicht von ERC prüfbar, manuell verifiziert**.

Manuell verifiziert (ERC-blind):
- TX/RX-Zuordnung ESP32 ↔ TJA1051 korrekt.
- TJA1051 S-Pin auf GND.

---

## Pre-Fab-Checkliste

### Schaltplan (erledigt)
- [x] MCP2515 entfernt, ESP32-TWAI genutzt
- [x] TJA1051 3,3 V-seitig, S-Pin auf GND
- [x] TX/RX korrekt zugeordnet
- [x] TLE4905L: Vcc 5 V, Pull-up (10 k) auf 3,3 V, 100 nF Entkopplung
- [x] Termination jumperbar (JP1, 120 Ω)
- [x] AP2112K mit C5/C6/C8, VIN+EN an 5 V
- [x] Decoupling: C7 (TJA), C1 (Sensor), 48 V-Bank (C2/C3/C4)
- [x] PWR_FLAGs gesetzt, ERC sauber
- [ ] Optional: TVS (SMAJ5.0A) am 5 V-Eingang

### Layout (nächster Schritt)
- [ ] Footprints allen Bauteilen zugewiesen
- [ ] XT60PW / XT30PW / XT30PB(2+2)-M / XH-2A Footprints beschafft/importiert (teils nicht in Standard-Lib)
- [ ] ESP32-Sockel-Footprint (J12–J15) mechanisch gegen XIAO geprüft
- [ ] GND-Pour mit Sternpunkt-Führung (Power-GND vs. Signal-GND)
- [ ] 48 V-Bus-Kupfer auf ~13 A, Motorabzweig auf Einzelmotor-Peak
- [ ] Logik räumlich von 48 V-Leistungsbahnen getrennt platziert
- [ ] Entkopplung-Cs nah an den jeweiligen IC-Pins platziert
- [ ] DRC sauber

### Bring-up (nach Fertigung)
- [ ] Sichtprüfung Lötung, Kurzschlusstest 48 V/5 V/3,3 V gegen GND
- [ ] 3,3 V-Rail messen (AP2112K-Ausgang)
- [ ] ESP32 ohne Bus flashen, CAN-Loopback-Test
- [ ] CAN-Kommunikation mit Pi (`/homing`-Protokoll) end-to-end
- [ ] TLE4905L mit Magnet: sauberer 3,3 V-Pegelwechsel am GPIO

---

## Offene Punkte / spätere Optionen
- TVS am 5 V-Eingang final entscheiden.
- XT30PB-Signalpins: Verwendung definieren oder einfachen XT30 nutzen.
- Sicherung/PTC im 48 V-Pfad pro Board erwägen (Steckergeometrie ersetzt keine Überstromsicherung).
- Erweiterungsstecker-Last bei AP2112K-Budget (600 mA) berücksichtigen, falls dort Sensorik versorgt wird.
