# R0192 „Daisy Chain" Driverlink-PCB

Bus-Verteilplatine, **einmal pro Achse**. Schleift 48 V, 5 V und CAN von Node zu Node durch, zweigt die Motorversorgung ab, wertet den Homing-Hallsensor aus und steuert die Aktuator-Haltebremse. Ersetzt den Arduino-Uno-Prototyp (`microcontroller/r0192_homing.ino`) durch einen **XIAO-ESP32-S3** mit integriertem CAN-Controller (TWAI).

Identische Platine an jeder Achsposition: Bus-Termination per Schalter, positionsabhängige Verbraucher (Bremse) per Bestückungsvariante.

KiCad-Projekt: `PCB/KiCad/DriverDasyChain.*` (4 Lagen: F.Cu / In1 GND / In2 PWR / B.Cu)

---

## Funktionsblöcke

| Block | Umsetzung |
| --- | --- |
| Power-Durchschleife | J12 / J13 (MR30PW 1×03): GND, +48 V, +5 V. Bulk C2/C3 = 2× 100 µF, C4 100 nF |
| 3,3 V-Erzeugung | U1 AP2112K-3.3 aus 5 V (C8 10 µ / C5 1 µ / C6 1 µ), EN an VIN, D2 SMAJ5.0A am Eingang |
| MCU | U2 XIAO-ESP32-S3, gesockelt (J9/J11), Versorgung über **3V3-Pin**, VBUS unbenutzt |
| CAN-Transceiver | U4 SN65HVD230 (nativ 3,3 V), Rs auf GND (High-Speed), Vref NC, C7 100 nF |
| CAN-Bus | J1/J2 (Micro-Fit 3.0 1×03: CANH/CANL/SHLD), Schirm über R6 1 M ‖ C11 100 n, R10 als DNP-Brücke auf GND |
| Terminierung | R5 120 Ω über SW2 zuschaltbar — **nur an den beiden Busenden** stecken |
| Motorabzweig | J4 (XT30PW 2+2): CANH, CANL, GND, +48 V |
| Homing-Sensor | J5 (JST PH 1×03): 5 V / OUT / GND. R1 4,7 k Pull-up auf **3,3 V**, C10 1 nF, R9 1 k Serienschutz |
| Bremsen-Endstufe | J16 + TH1 + Q2 + D1, Gate über U3 74AHCT1G125 (3,3 V → 5 V) |
| Status-LED | D4 + R11 an D5 |
| UART-Debug | J6 1×04: GND / D6 / D7 / +3,3 V |

---

## GPIO-Belegung (XIAO-ESP32-S3)

| Pin | GPIO | Funktion |
| --- | --- | --- |
| D0 | GPIO1 | **Bremse PWM** → U3 → R2 22 Ω → Q2 Gate |
| D1 | GPIO2 | **Hallsensor** (TLE4905L, über R9 1 k) |
| D3 | GPIO4 (ADC1_CH3) | **Bremsen-Schaltknoten-Sense** (Teiler R7 150 k / R8 10 k, C12 100 n) |
| D5 | GPIO6 | Status-LED |
| D6 | GPIO43 | UART TX (Debug, J6) |
| D7 | GPIO44 | UART RX (Debug, J6) |
| D9 | GPIO8 | **CAN RX** → U4 Pin 4 (R) |
| D10 | GPIO9 | **CAN TX** → U4 Pin 1 (D) |

> **Firmware:** `ESP32-TWAI-CAN` bzw. `driver/twai.h` statt `autowp/mcp2515`. CAN liegt auf D10/D9 (**nicht** D6/D7 — die sind UART0/Debug). CAN-Protokoll unverändert: achsenspezifische ID, `CMD_ARM` / `RSP_DETECTED` / `RSP_ERROR`.

> **XIAO-Block ist bewusst eine „Insel":** J7/J8 (Board-Signale) sind im Schaltplan nicht mit J9/J11+U2 verdrahtet — dort kommen noch Pins dazu, die mitzählen sollen. Die Zuordnung oben ist positionsbezogen (J7.n ↔ J9.n, J8.n ↔ J11.n).

---

## Bremsen-Endstufe

**Bremse SteadyWin STW-S035** (gleich in GIM6010 und GIM8108), **stromlos geschlossen** (fail-safe):

| Parameter | Wert |
| --- | --- |
| Nennspannung / -strom | 24 V / 0,64 A (15,4 W) |
| Spulenwiderstand | 34 Ω kalt |
| Haltemoment | 0,80 N·m motorseitig → ~6,4 N·m am Gelenk (8:1) |
| Öffnen / Einfallen | ~35 ms / ~20 ms |

**Eigene Messungen (24 V):** Anzug ab ~10,5 V / ~266 mA, Abfall bei ~2,5 V / ~60 mA, Dauerbetrieb 24 V heiß ~450 mA. Hysterese ~4:1, 15 min bei 24 V → 65 °C ohne Schwellenverschiebung.

**Topologie (netzlisten-verifiziert):**

```
+48V ── J16.2 ── [Bremsspule 34Ω] ── J16.1 ── TH1(PTC) ──┬── Q2 Drain
                                                          │
        D1 (US2DA, K→+48V, A→Drain) = Freilauf ───────────┤
                                                          ├── R7 150k ── D3 (Sense) ── R8 10k ── GND
                                                     Q2 Source ── GND
```

Gate: D0 → U3 74AHCT1G125 (VCC 5 V, OE̅ auf GND, R3 10 k Eingangs-Pulldown) → R2 22 Ω → Q2, R4 10 k Gate-Pulldown. **Default = FET aus = Bremse eingefallen**, auch beim Booten/Reset.

**Economizer:** Anzug 100 % Duty (48 V) für ~150 ms → Halten bei ~10,4 % Duty (= 5 V Mittelwert, ~130–150 mA, ~0,7 W statt 15 W). Einfallen: PWM = 0.

```
LÖSEN:      PWM 100 %, 100–200 ms  →  PWM ~10,4 % halten  →  danach Motor-Torque
SCHLIESSEN: PWM 0 %  (Anker fällt in ~20 ms)
```

**Sicherheit:** Die Spule hängt an 48 V — ein Not-Aus, der 48 V trennt (Schütz), lässt alle Bremsen zwangsläufig einfallen. Das ist die Hardware-Zwangsabschaltung; ein reiner Software-Not-Aus reicht nicht.

---

## Spannungsdomänen

| Domäne | Quelle | Versorgt |
| --- | --- | --- |
| 48 V | Tether (durchgeschleift) | Motorabzweig, Bremsen-Endstufe |
| 5 V | Tether (durchgeschleift) | TLE4905L, AP2112K-Eingang, U3 (Gate-Treiber) |
| 3,3 V | U1 AP2112K-3.3 (600 mA) | XIAO, SN65HVD230, Pull-up |
| GND | ein gemeinsames Netz | alles |

CAN hat keine eigene GND-Ader (J1/J2 führen nur H/L/Schirm) — die Referenz kommt über die durchgehende Power-GND. **Muss über alle Boards durchgängig sein.**

---

## Layout-Vorgaben (noch offen)

- GND-Pour, Sternpunkt zwischen Power-GND (XT/MR30-Stecker, Q2-Source, 48-V-Bulk) und Signal-GND (MCU, U4, U1, TLE4905L).
- **Bremsen-Schaltknoten-Loop klein** (Q2 / Spule / D1 eng), gepulste 48 V **weg von den CAN-Leitungen**.
- 48-V-Bus auf ~13 A (LRS-600N2-Limit), Motorabzweig auf Einzelmotor-Peak, Bremsenkanal ≥ 2 A.
- Entkopplungs-Cs direkt an den IC-Pins.
- Footprints müssen noch final zugewiesen/geprüft werden (u. a. C11, C12, R6–R11, TH1, J9, J11).
- Das `.kicad_pcb` ist noch nicht mit dem Schaltplan synchronisiert.

---

## Offene Punkte

**Schaltplan**
- [ ] **Zwei leere Labels löschen** bei (83.82, 156.21) und (111.76, 215.9) — sie verschmelzen Hallsensor-Ausgang (`J5.2`) und MOSFET-Gate (`Q2.1`) zu einem Netz. Danach dem Gate-Netz einen Namen geben (`MOSFEST In` hängt aktuell in der Luft).
- [ ] **TVS/Zener (~68 V) vom Schaltknoten nach GND** — ohne sie klingt der Spulenstrom mit τ = L/R ab (verzögertes Einfallen gegenüber den 20 ms), und der Sense-Teiler sieht ungeklemmte Flyback-Spitzen.
- [ ] **Sense-Teiler entschärfen:** 48 V × 10k/160k = 3,0 V, bei 53 V Bus 3,3 V — kein Headroom. R8 auf ~6,8 k und/oder Klemmdiode auf 3V3.
- [ ] R11 (LED-Vorwiderstand) hat noch keinen Wert; Q2 braucht einen echten MPN (V_DS ≥ 100 V, R_DS(on) bei V_GS = 4,5 V).
- [ ] SW2: Symbol `SW_DPST_x2` (4 Pins), nur Unit A benutzt → auf SPST wechseln.
- [ ] Spannungsfestigkeit eintragen: C2/C3 ≥ 63 V, C4 ≥ 100 V. C6 auf 2,2–4,7 µF (DC-Derating am LDO-Ausgang).
- [ ] Sicherung / Inrush-Begrenzung im 48-V-Pfad erwägen (2× 100 µF pro Board × 6 Boards beim Hot-Plug).

**Auslegung**
- [ ] TH1 ist auf 0,64 A (Bremsen-Nennstrom bei 24 V) ausgelegt, der 48-V-Anzugspuls zieht aber ~1,4 A für ~150 ms. Trip-Zeit des PTC bei ~2,2× I_hold gegen die Pulsdauer prüfen — ebenso J16 (JST SH: 1 A/Kontakt).
- [ ] Gate-Treiberleistung: 74AHCT1G125 liefert ±8 mA → ~2 µs Flanken. Entweder PWM auf ~500 Hz–1 kHz (τ der Spule reicht) oder echter Gate-Treiber, sonst spürbare Schaltverluste bei 48 V / 1,4 A.
- [ ] D2 SMAJ5.0A klemmt bei ~9,2 V, Abs-Max VIN des AP2112K ist 6,0 V — schützt gegen Energie, nicht gegen moderate 5-V-Überhöhung.
- [ ] J12/J13 sind beide MR30PW-**M**: für die Daisy Chain Geschlechter prüfen. MR30 = 15 A, bei den geplanten ~13 A am Limit.
- [ ] J6 (UART-Debug) darf **nicht** denselben Footprint wie J4 (Motor, 48 V) bekommen — sonst ist das Motorkabel in den Debug-Port steckbar.
- [ ] Haltemoment unter Payload an J2/J3 verifizieren (~6,4 N·m Bremse vs. 7,5 N·m Motor-Nennmoment), kalt und warm.
- [ ] Spuleninduktivität messen → PWM-Frequenz und Einfallzeit final festlegen.

**Bring-up**
- [ ] Kurzschlusstest 48 V / 5 V / 3,3 V gegen GND, 3,3 V-Rail messen
- [ ] ESP32 flashen, CAN-Loopback, dann `/homing`-Protokoll end-to-end gegen den Pi
- [ ] TLE4905L mit Magnet: sauberer 3,3-V-Pegelwechsel an D1
- [ ] Bremse: Anzug (48 V, 150 ms) → öffnet; Halten bei 10,4 % Duty; Einfallzeit mit/ohne TVS messen
