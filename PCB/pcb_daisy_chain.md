# R0192 „Daisy Chain" Driverlink-PCB

Bus-Verteilplatine, **einmal pro Achse**. Schleift 48 V, 5 V und CAN von Node zu Node durch, zweigt die Motorversorgung ab, wertet den Homing-Hallsensor aus und steuert die Aktuator-Haltebremse. Ersetzt den Arduino-Uno-Prototyp (`microcontroller/r0192_homing.ino`) durch einen **XIAO-ESP32-S3** mit integriertem CAN-Controller (TWAI).

Identische Platine an jeder Achsposition: Bus-Termination per Schalter, positionsabhängige Verbraucher (Bremse) per Bestückungsvariante.

KiCad-Projekt: `PCB/KiCad/DriverDasyChain.*` (4 Lagen: F.Cu / In1 GND / In2 PWR / B.Cu)
Datenblätter: `PCB/R0192.pretty/Datenblätter/` (Bauteile) und `.../Kabel/` (CF77.UL.D, CFBUS.PVC)

---

## Funktionsblöcke

| Block | Umsetzung |
| --- | --- |
| Power-Durchschleife | J12 / J13: GND, +48 V, +5 V. Bulk C2/C3 = 2× 100 µF, C4 100 nF |
| 3,3 V-Erzeugung | U1 AP2112K-3.3 aus 5 V (C8 10 µ / C5 1 µ / C6 1 µ), EN an VIN, D2 SMAJ5.0A am Eingang |
| MCU | U2 XIAO-ESP32-S3, gesockelt (J9/J11), Versorgung über **3V3-Pin**, VBUS unbenutzt |
| CAN-Transceiver | U4 SN65HVD230 (nativ 3,3 V), Rs auf GND (High-Speed), Vref NC, C7 100 nF |
| CAN-Bus | J1/J2 (CANH/CANL/SHLD), Schirm über R6 1 M ‖ C11 100 n, R10 als DNP-Brücke auf GND |
| Terminierung | R5 120 Ω über **beide** Pole von SW1 zuschaltbar (kein Stub im offenen Zustand) — nur an den zwei Busenden einschalten, beide DIP-Positionen ON |
| Motorabzweig | J4 (XT30PW 2+2): CANH, CANL, GND, +48 V |
| Homing-Sensor | J5: 5 V / OUT / GND. R1 4,7 k Pull-up auf **3,3 V**, C10 1 nF, R9 1 k Serienschutz |
| Bremsen-Endstufe | J16 + TH1 + Q2 + D1, Gate über U3 74AHCT1G125 (3,3 V → 5 V) |
| Status-LED | D4 + R11 an D5 |
| UART-Debug | J6: GND / D6 / D7 / +3,3 V |

---

## GPIO-Belegung (XIAO-ESP32-S3)

| Pin | GPIO | Funktion |
| --- | --- | --- |
| D0 | GPIO1 | **Bremse PWM** → U3 → R2 22 Ω → Q2 Gate |
| D1 | GPIO2 | **Hallsensor** (TLE4905L, über R9 1 k) |
| D3 | GPIO4 (ADC1_CH3) | **Bremsen-Schaltknoten-Sense** (Teiler R7 150 k / R8 10 k, C12 100 n) |
| D5 | GPIO6 | Status-LED |
| D6 / D7 | GPIO43 / GPIO44 | UART TX / RX (Debug, J6) |
| D9 / D10 | GPIO8 / GPIO9 | **CAN RX / CAN TX** → U4 Pin 4 (R) / Pin 1 (D) |

> **Firmware:** `ESP32-TWAI-CAN` bzw. `driver/twai.h` statt `autowp/mcp2515`. CAN liegt auf D10/D9 (**nicht** D6/D7 — die sind UART0/Debug). CAN-Protokoll unverändert: achsenspezifische ID, `CMD_ARM` / `RSP_DETECTED` / `RSP_ERROR`.

> **XIAO-Block ist bewusst eine „Insel":** J7/J8 (Board-Signale) sind im Schaltplan nicht mit J9/J11+U2 verdrahtet — dort kommen noch Pins dazu, die mitzählen sollen. Die Zuordnung oben ist positionsbezogen (J7.n ↔ J9.n, J8.n ↔ J11.n).

---

## Steckerkonzept & Kabel

**Prinzip:** polarisierte Lötverbinder überall, wo ein Fehlanschluss Hardware zerstört — Federkraft-/Push-in-Klemmen bei den Signalen. Damit wird **keine Crimpzange** gebraucht, nur eine Aderendhülsenzange.

| Stecker | Typ | Kabel / Ader |
| --- | --- | --- |
| J12 / J13 Power | MR30PW-M, gelötet + polarisiert (20 A / 80 V) | CF77.UL.15.04.D aufgetrennt, 1,5 mm², OD 2,3 mm, **21 A/Ader** (igus) — Bus zieht max. 12,5 A |
| J4 Motor (48 V + CAN) | XT30PW 2+2, gelötet + polarisiert | noch festzulegen |
| J1 / J2 CAN | Federkraftklemme 1×03 | CFBUS.PVC.021 aufgetrennt, 0,5 mm², OD 2,9 mm |
| J5 Hall | Federkraftklemme 1×03 | 0,14 mm², OD 1,1 mm |
| J16 Bremse | Federkraftklemme 1×02 | ≥ 0,14 mm² feindrähtig (0,081 mm² wäre mechanisch zu dünn fürs bewegte Gelenk) |
| J6 UART-Debug | Stiftleiste 1×04 (Dupont) | — |

**Warum Power polarisiert bleibt:** vertauschte 48 V/5 V an J12/J13 zerstören auf *jedem* Board der Kette AP2112K, TLE4905L, U3 und XIAO; 48 V auf CANH an J4 nimmt alle sechs SN65HVD230 mit. MR30/XT30 sind Lötverbinder — dort spart eine Klemme ohnehin keinen Crimpaufwand.

**Bei den Klemmen beachten:** CAN-Paar bis unmittelbar an die Klemme verdrillt lassen (2–3 cm untwistet, max.); Schirm als kurzes Pigtail mit Aderendhülse; Zugentlastung hinter jeder Klemme (Klemmen haben keine); Bestückungsdruck deutlich beschriften.

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

**Soll-Topologie** (im Schaltplan sitzt TH1 noch auf der Low-Side, siehe offene Punkte):

```
+48V ── TH1(PTC) ──┬── J16.2 ── [Bremsspule 34Ω] ── J16.1 ──┬── Q2 Drain
                   │                                        │
                   └────────── D1 (K→TH1-Knoten, A→Drain) ───┤  Freilauf
                                                             ├── R7 150k ── D3 (Sense) ── R8 10k ── GND
                                                        Q2 Source ── GND
```

TH1 gehört auf die **48-V-Seite**: nur dort liegt auch ein Masseschluss einer der beiden Kabeladern hinter dem Schutz. Sitzt er hinter der Spule, wird ein Kurzschluss der 48-V-Ader gegen Gehäuse gar nicht begrenzt.

Gate: D0 → U3 74AHCT1G125 (VCC 5 V, OE̅ auf GND, R3 10 k Eingangs-Pulldown) → R2 22 Ω → Q2, R4 10 k Gate-Pulldown. **Default = FET aus = Bremse eingefallen**, auch beim Booten/Reset.

**Economizer:** Anzug 100 % Duty (48 V) für ~150 ms → Halten bei ~10,4 % Duty (= 5 V Mittelwert, ~147 mA, ~0,7 W statt 15 W). Einfallen: PWM = 0.

```
LÖSEN:      PWM 100 %, 100–200 ms  →  PWM ~10,4 % halten  →  danach Motor-Torque
SCHLIESSEN: PWM 0 %  (Anker fällt in ~20 ms)
```

**TH1 = 0,64 A passt:** Halten (147 mA) = 23 % von I_hold, der 1,41-A-Anzugspuls über 150 ms liegt unter der Trip-Zeit, ein durchlegierter FET (1,41 A dauerhaft) trippt nach Sekunden — lange vor der Isolationsgrenze der Spule (~35 s bis 155 °C). Der PTC diskriminiert also über die Dauer, nicht über den Strom.

**Schaltverluste sind unkritisch:** während der PWM-Phase fließen nur 147 mA (nicht 1,41 A), bei ~0,5 µs Flanken sind das ~35 mW bei 10 kHz. Der 74AHCT1G125 reicht, ein Gate-Treiber ist nicht nötig.

**Sicherheit:** Die Spule hängt an 48 V — ein Not-Aus, der 48 V trennt (Schütz), lässt alle Bremsen zwangsläufig einfallen. Das ist die Hardware-Zwangsabschaltung; ein reiner Software-Not-Aus reicht nicht. Der Sense-Pin ist **Diagnose, keine Schutzfunktion** — er erkennt einen durchlegierten FET, kann ihn aber nicht abschalten.

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

- GND-Pour, Sternpunkt zwischen Power-GND (Motor-/Bus-Stecker, Q2-Source, 48-V-Bulk) und Signal-GND (MCU, U4, U1, TLE4905L).
- **Bremsen-Schaltknoten-Loop klein** (Q2 / Spule / D1 eng), gepulste 48 V **weg von den CAN-Leitungen**.
- 48-V-Bus auf 12,5 A (LRS-600N2-Limit), Motorabzweig auf Einzelmotor-Peak, Bremsenkanal ≥ 2 A.
- Entkopplungs-Cs direkt an den IC-Pins.
- Platinenumriss ist noch nicht gezeichnet; Footprints teils offen (C11, C12, R6–R11, TH1, J9/J11 sind Platzhalter).
- Das `.kicad_pcb` ist noch nicht mit dem Schaltplan synchronisiert.

---

## Offene Punkte

**Schaltplan**
- [ ] **Zwei leere Labels löschen** bei (83.82, 156.21) und (111.76, 215.9) — sie verschmelzen Hallsensor-Ausgang (`J5.2`) und MOSFET-Gate (`Q2.1`) zu einem Netz. Danach dem Gate-Netz einen Namen geben (`MOSFET In` hängt in der Luft).
- [ ] **TH1 auf die 48-V-Seite verlegen** (siehe Soll-Topologie), D1-Kathode auf den Knoten *nach* TH1.
- [ ] **Steckerwechsel eintragen:** J1/J2, J5, J16 auf Federkraftklemmen, J6 auf Stiftleiste.
- [ ] **TVS/Zener (~68 V) vom Schaltknoten nach GND** — nur nötig, wenn die gemessene Spuleninduktivität > ~380 mH liegt (darunter reicht D1 allein für ein Einfallen unter 20 ms). Schützt zusätzlich den Sense-Teiler vor Flyback-Spitzen.
- [ ] **Sense-Teiler entschärfen:** 48 V × 10k/160k = 3,0 V, im gestauchten ADC-Bereich. R8 auf ~6,8 k → 2,08 V, sauber linear.
- [ ] R11 (LED-Vorwiderstand) hat noch keinen Wert; Q2 braucht den MPN im Value-Feld (IRLR3410?), R_DS(on) bei **V_GS = 5 V** prüfen.
- [ ] Spannungsfestigkeit eintragen: C2/C3 ≥ 63 V, C4 ≥ 100 V. C6 auf 2,2–4,7 µF (DC-Derating am LDO-Ausgang).
- [ ] Note für J4 fehlt noch (Leiterquerschnitt Motorabzweig).
- [ ] Sicherung / Inrush-Begrenzung im 48-V-Pfad erwägen (2× 100 µF pro Board × 6 Boards beim Hot-Plug).

**Auslegung**
- [ ] **Wellenimpedanz der CFBUS.PVC.021 prüfen.** Violett + 2×0,5 mm² deutet auf einen Profibus-Typ (150 Ω); CAN erwartet 120 Ω. Falls 150 Ω: **R5 auf 150 Ω** ändern — dann passen Kabel und Abschluss zusammen und der SN65HVD230 sieht 75 Ω statt 60 Ω differentiell.
- [ ] Bei **steckbaren** Klemmen: J1/J2 (CAN) und J5 (Hall) wären beide 3-polig und gegeneinander steckbar → unterschiedliche Raster wählen (z. B. 3,5 mm / 5,0 mm) oder feste Klemmen nehmen.
- [ ] J12/J13 sind beide MR30PW-**M**: Geschlechter für die Daisy Chain prüfen. Der **eine GND-Pin** führt den Rückstrom beider Schienen — unkritisch, solange 5 V nur Logik versorgt.
- [ ] D2 SMAJ5.0A klemmt bei ~9,2 V, Abs-Max VIN des AP2112K ist 6,0 V — schützt gegen Energie, nicht gegen moderate 5-V-Überhöhung.
- [ ] Haltemoment unter Payload an J2/J3 verifizieren (~6,4 N·m Bremse vs. 7,5 N·m Motor-Nennmoment), kalt und warm.
- [ ] Spuleninduktivität messen → entscheidet über die TVS und die PWM-Frequenz.

**Bring-up**
- [ ] Kurzschlusstest 48 V / 5 V / 3,3 V gegen GND, 3,3 V-Rail messen
- [ ] ESP32 flashen, CAN-Loopback, dann `/homing`-Protokoll end-to-end gegen den Pi
- [ ] TLE4905L mit Magnet: sauberer 3,3-V-Pegelwechsel an D1
- [ ] Bremse: Anzug (48 V, 150 ms) → öffnet; Halten bei 10,4 % Duty; Einfallzeit messen
