# R0192 „Daisy Chain" Driverlink-PCB

Bus-Verteilplatine, **einmal pro Achse**. Schleift 48 V, 5 V und CAN von Node zu Node durch, zweigt die Motorversorgung ab, wertet den Homing-Hallsensor aus und steuert die Aktuator-Haltebremse. Ersetzt den Arduino-Uno-Prototyp (`microcontroller/r0192_homing.ino`) durch einen **XIAO-ESP32-S3** mit integriertem CAN-Controller (TWAI).

Identische Platine an jeder Achsposition: Bus-Termination per Schalter, positionsabhängige Verbraucher (Bremse) per Bestückungsvariante.

KiCad-Projekt: `PCB/KiCad/DriverDasyChain.*` (4 Lagen: F.Cu / In1 GND / In2 PWR / B.Cu)
Datenblätter: `PCB/R0192.pretty/Datenblätter/` (Bauteile) und `.../Kabel/` (CF77.UL.D, CFBUS.PVC)

---

## Funktionsblöcke

| Block | Umsetzung |
| --- | --- |
| Power-Durchschleife | J12 / J10 (4-polig): +5 V / GND / GND / +48 V. Bulk C2/C3 = 2× 100 µF, C4 100 nF |
| 3,3 V-Erzeugung | U1 AP2112K-3.3 aus 5 V (C8 10 µ / C5 1 µ / C6 2,2 µ), EN an VIN, D3 SMAJ5.0CA am Eingang (bidirektional) |
| MCU | U2 XIAO-ESP32-S3 (Footprint `R0192:XIAO-ESP32-S3-DIP`, THT-Sockel), Versorgung über **3V3-Pin**, trennbar über SW2 (PCM12SMTR) — beim Flashen über USB öffnen, VBUS = NC |
| CAN-Transceiver | U4 SN65HVD230 (nativ 3,3 V), Rs auf GND (High-Speed), Vref NC, C7 100 nF |
| CAN-Bus | J1/J2 (CANH/CANL/SHLD), Schirm hybrid über R6 1 M ‖ C11 100 n; R10 (0 Ω) = harte Schirmauflage, **nur auf einem Board bestücken** |
| Terminierung | **Split-Termination**: R13 + R12 je 60,4 Ω in Reihe über den Bus, Mittelpunkt über C13 4,7 nF auf GND. Zuschaltbar über SW1 (CTS 219-2LPSTJ, **2-polig**, je ein Pol pro Zweig) — nur an den zwei physischen Busenden einschalten |
| Motorabzweig | J4: CANH, CANL, GND, +48 V (Steckertyp s. u.) |
| Homing-Sensor | J5: 5 V / GND / OUT (Pin 1–3). R1 4,7 k Pull-up auf **3,3 V**, C10 1 nF, R9 6,8 k Serienschutz |
| Bremsen-Endstufe | J3 + F1 (PPTC) + Q2 + D1, Gate über U3 74AHCT1G125 (3,3 V → 5 V) |
| Status-LED | D4 (rot oder grün, 0603) + **R11 470 Ω** an **D2**, active high |
| UART-Debug | J6: GND / D6 / D7 / +3,3 V, TX/RX über R5 / R14 (220 Ω in Serie). Adapter-VCC **nicht** an Pin 4 (= 3,3-V-Schiene) |

---

## GPIO-Belegung (XIAO-ESP32-S3)

| Pin | GPIO | Funktion |
| --- | --- | --- |
| D0 | GPIO1 | **Bremse PWM** → U3 → R2 22 Ω → Q2 Gate |
| D1 | GPIO2 | **Hallsensor** (TLE4905L, über R9 6,8 k) |
| D2 | GPIO3 | **Status-LED** (R11 470 Ω, active high) — dadurch bleiben D4/D5 (SDA/SCL) für I²C frei |
| D3 | GPIO4 (ADC1_CH3) | **Bremsen-Schaltknoten-Sense** (Teiler R7 150 k / R8 6,8 k, C12 100 n) |
| D6 / D7 | GPIO43 / GPIO44 | UART TX / RX (Debug, J6) |
| D9 / D10 | GPIO8 / GPIO9 | **CAN RX / CAN TX** → U4 Pin 4 (R) / Pin 1 (D) |

> **Firmware:** `ESP32-TWAI-CAN` bzw. `driver/twai.h` statt `autowp/mcp2515`. CAN liegt auf D10/D9 (**nicht** D6/D7 — die sind UART0/Debug). CAN-Protokoll unverändert: achsenspezifische ID, `CMD_ARM` / `RSP_DETECTED` / `RSP_ERROR`.

> **Keine Sockel-Symbole mehr:** die früheren Hilfsstecker J7/J8/J9/J11 sind gelöscht; die Signale hängen direkt an U2. Grund: die vier Footprints lagen auf dem Board deckungsgleich auf denselben 14 Pads, und die eine Seite (J8) war gegenüber der physischen Lage zeilenverkehrt — nach der physischen Zuordnung wäre die Bremsen-PWM auf **D6 = UART0 TX** gelandet. Jetzt ist der XIAO-Footprint die einzige Wahrheitsquelle. D4, D5, D8 und VBUS haben No-Connect-Flags.

---

## Steckerkonzept & Kabel

**Prinzip:** Push-in-Klemmen für Power und Bus, JST PH dort, wo die Kabel für Klemmen zu dünn sind. Alles Top-Entry. Drei Bauteilnummern: WAGO 2601-3104 (3×), Phoenix PTSM 0,5/3-2,5-V-THR (2×), JST PH B3B-PH-K (2×), dazu eine Stiftleiste.

| Stecker | Typ | Footprint | Kabel / Ader |
| --- | --- | --- | --- |
| J12 / J10 Power | WAGO **2601-3104**, Push-in, Top-Entry, 17,5 A / AWG 26–14 | `TerminalBlock_WAGO:TerminalBlock_WAGO_2601-3104_1x04_P3.50mm_Vertical` | CF77.UL.15.04.D, alle 4 Adern, 1,5 mm², 21 A/Ader (igus) — Bus zieht max. 12,5 A |
| J4 Motor (CANH/CANL/GND/48 V) | WAGO **2601-3104**, Push-in, Top-Entry, 17,5 A / AWG 26–14 | `TerminalBlock_WAGO:TerminalBlock_WAGO_2601-3104_1x04_P3.50mm_Vertical` | 1,5 mm² Power + 0,5 mm² CAN im selben Block |
| J1 / J2 CAN | Phoenix **PTSM 0,5/3-2,5-V-THR**, Push-in, Top-Entry, 6 A / 160 V, **0,14–0,5 mm² (AWG 26–20)** | `TerminalBlock_Phoenix:TerminalBlock_Phoenix_PTSM-0,5-3-2.5-V-THR_1x03_P2.50mm_Vertical` | CFBUS.PVC.021, 0,5 mm² — **am Maximum der Klemme** |
| J5 Hall | **JST PH B3B-PH-K**, vertikal, Kontakt SPH-002T-P0.5S: 0,05–0,22 mm² (AWG 30–24), OD 0,9–1,5 mm, 2 A | `Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical` | 0,14 mm², OD 1,1 mm — beides mittig im Fenster |
| J3 Bremse | **dieselbe JST PH B3B-PH-K** (nur Pin 1+2 belegt, Pin 3 = Reserve + NC-Flag) | dito | 0,081 mm² (AWG 28), OD 0,8 mm — fest am Motor, nicht änderbar; Leiter im Fenster, OD 0,1 mm darunter |
| J6 UART-Debug | Stiftleiste 1×04 (Dupont) | `Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical` | — |

**Warum J5/J3 kein Klemmentyp sind:** das Bremskabel kommt fest aus dem Motor (Wechsel = Treiber demontieren) und hat 0,081 mm² — 42 % unter dem PTSM-Minimum von 0,14 mm². Der JST-PH-Kontakt deckt 0,05–0,22 mm² ab und passt nativ; nur die Isolations-OD liegt mit 0,8 mm knapp unter den geforderten 0,9 mm. **Abhilfe:** vor dem Krimpen ein kurzes Stück dünnen Schrumpfschlauch über die Isolierung schieben, dann greift der Zugentlastungsflügel. Der stromführende Leiterkrimp ist unkritisch. Zange ist vorhanden; ~24 Krimps über alle sechs Boards.

Zweimal dieselbe 3-polige B3B-PH-K, an J3 bleibt ein Pin frei — nützlich als Reserve, falls eine Bremsader durch Biegewechsel bricht. **Fehlerrichtung ist gutartig:** verliert die Bremsleitung Kontakt, fällt die Bremse ein (stromlos geschlossen).

PTSM nur noch für J1/J2 → 12 Stück für sechs Boards; Kleinmengen über die Phoenix-SAMPLE-Nummer **1701101**, die Suffixe `R44`/`R32` sind Gurtware.

**Flächenbilanz gegenüber dem alten Steckerkonzept** — trotz Klemmen und einem Pol mehr an J12/J10 wird es kleiner:

| | alt | neu | Δ |
| --- | ---: | ---: | ---: |
| J12 / J10 | MR30PW-M 257,5 mm² | WAGO 227 mm² | −31 mm² je Stecker |
| J4 | XT30PW 2+2 250,1 mm² | WAGO 227 mm² | −23 mm² |
| J1 / J2 | Micro-Fit 149,0 mm² | PTSM 54,1 mm² | −95 mm² je Stecker |
| J5 / J3 | JST PH horizontal 76,5 / 59,3 mm² | JST PH **vertikal** 49,0 / 49,0 mm² | −27 / −10 mm² |

Summe über alle sieben Stecker: **1199 → 887 mm²**, also ~310 mm² gespart.

**Alle Klemmen sind Top-Entry** — die Kabel verlassen das Board senkrecht zur Platine. Das bestimmt Einbaulage und Zugentlastungen.

**CAN-Pinreihenfolge der Treiber:** GDS68 und RS05 sind **unterschiedlich** belegt. Ein einmaliger Tausch im Schaltplan hilft daher nicht — die Klemme an J4 wird pro Achse passend verdrahtet. Genau dafür ist der Klemmentyp dort richtig.

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

**Topologie** (netzlisten-verifiziert):

```
+48V ── F1 (PPTC) ─┬── J3.1 ─── [Bremsspule 34Ω] ── J3.2 ───┬── Q2 Drain
                   │                                        │
                   └────────── D1 (K→F1-Knoten, A→Drain) ────┤  Freilauf
                                                             ├── R7 150k ── D3 (Sense) ── R8 6,8k ── GND
                                                        Q2 Source ── GND
```

F1 sitzt bewusst auf der **48-V-Seite**: nur dort liegt auch ein Masseschluss einer der beiden Kabeladern hinter dem Schutz. Hinter der Spule wäre ein Kurzschluss der 48-V-Ader gegen Gehäuse gar nicht begrenzt.

Gate: D0 → U3 74AHCT1G125 (VCC 5 V, OE̅ auf GND, R3 10 k Eingangs-Pulldown) → R2 22 Ω → Q2, R4 10 k Gate-Pulldown. **Default = FET aus = Bremse eingefallen**, auch beim Booten/Reset.

**Economizer:** Anzug 100 % Duty (48 V) für ~150 ms → Halten bei ~10,4 % Duty (= 5 V Mittelwert, ~147 mA, ~0,7 W statt 15 W). Einfallen: PWM = 0.

```
LÖSEN:      PWM 100 %, 100–200 ms  →  PWM ~10,4 % halten  →  danach Motor-Torque
SCHLIESSEN: PWM 0 %  (Anker fällt in ~20 ms)
```

**F1 = Littelfuse 60R050XU** (PPTC, 0,5 A Halte- / 1,0 A Auslösestrom, **60 V**) — Symbol `Device:Polyfuse`, Footprint `R0192:Fuse_Littelfuse_60R050XU` (radial THT, RM 5,1 mm, eigenes STEP-Modell). Die ursprünglich vorgesehene Bourns MF-RHT050-2 ist laut Datenblatt nur für **30 V** spezifiziert und damit an 48 V ungeeignet.

| Betriebsfall | Strom | Verhalten |
| --- | --- | --- |
| Halten | 147 mA = 30 % von I_hold | auch bei 65 °C derated (~0,3 A) noch 2× Marge |
| Anzug | 1,41 A für 150 ms | weit unter der Auslösezeit, geht durch |
| FET durchlegiert | 1,41 A dauerhaft = 2,8× I_hold | trippt im Sekundenbereich — lange vor den ~35 s bis zur Isolationsgrenze der Spule |

Der PTC diskriminiert also über die **Dauer**, nicht über den Strom — Anzugspuls und Fehlerstrom sind gleich groß. **Die 60 V sind die bindende Spec**: gängige SMD-PPTCs sind auf 6–30 V spezifiziert und können an einer 48-V-Schiene beim Auslösen durchzünden. Der PTC-Widerstand (Größenordnung 1 Ω) gegen 34 Ω Spulenwiderstand ist vernachlässigbar.

**Schaltverluste sind unkritisch:** während der PWM-Phase fließen nur 147 mA (nicht 1,41 A), bei ~0,5 µs Flanken sind das ~35 mW bei 10 kHz. Der 74AHCT1G125 reicht, ein Gate-Treiber ist nicht nötig.

**Sicherheit:** Die Spule hängt an 48 V — ein Not-Aus, der 48 V trennt (Schütz), lässt alle Bremsen zwangsläufig einfallen. Das ist die Hardware-Zwangsabschaltung; ein reiner Software-Not-Aus reicht nicht. Der Sense-Pin ist **Diagnose, keine Schutzfunktion** — er erkennt einen durchlegierten FET, kann ihn aber nicht abschalten.

---

## CAN-Bus: Referenz, Schirm, Terminierung

**Das Kabel ist geklärt.** Das igus-Datenblatt (`.../Kabel/IGUS_DE_Datenblatt_chainflex_CFBUS.PVC.pdf`) listet **CFBUS.PVC.020–.022 explizit unter „CAN-Bus/Feldbus"** mit **Wellenwiderstand 120 ± 12 Ω** (≥ 1 MHz) und 41 pF/m. Der 150-Ω-Profibus-Typ ist `.001` (1×2×0,64). Die eingesetzte **CFBUS.PVC.021 (2×0,5)C ist also ein echtes 120-Ω-CAN-Kabel** — die Terminierung bleibt auf 120 Ω gesamt.

**Warum keine eigene GND_CAN-Ader nötig ist.** Die CAN-Klemmen J1/J2 führen nur H/L/Schirm; die Referenz läuft über die Power-GND aus J10/J12, und zum Motortreiber über J4.3. Beides ist bereits vorhanden — der Treiber hat seine CAN-Masse also über dieselbe Klemme wie CANH/CANL. Eine zusätzliche dünne Ader würde daran nichts ändern:

| Pfad zwischen zwei Boards (0,5 m Hop) | Querschnitt | R |
| --- | --- | ---: |
| vorhandene GND über J10/J12 | 2 × 1,5 mm² = 3 mm² | **~3 mΩ** |
| hypothetische GND_CAN-Ader | 0,25 mm² | ~34 mΩ |

Die neue Ader wäre 12× hochohmiger als die schon vorhandene Verbindung und übernähme parallel nur ~8 % des Stroms — das Bezugspotenzial bestimmt der niederohmigste Pfad. Kaskadiert über sechs Achsen (Hop 1 trägt 6 Motoren, Hop 5 einen):

```
ΔU = R_hop · I_Motor · (5+4+3+2+1) = 3 mΩ · 2 A · 15  ≈  90 mV
absurder Worst Case (12,5 A durch alle Hops):          < 200 mV
```

Gegenüber dem Budget des SN65HVD230 — Betrieb −2 … +7 V, **Absolut-Maximum an CANH/CANL nur −4 … +16 V** (Datenblatt, Transientenpuls über 100 Ω bis ±25 V) — sind das ~10 % vom engsten (negativen) Rand. Faktor 10 Marge, kein Grenzfall.

> **Bedingung dafür**: CAN-Kabel und Powerkabel **im selben Bündel / derselben Schleppkette** führen. Dann liegt die GND-Ader physisch neben dem Paar und die Schleifenfläche bleibt klein. Das ist der eigentliche Wirkhebel, nicht der Aderquerschnitt.

> Falls später doch getrennte Wege nötig werden: **CFBUS.PVC.020, (4×0,25)C, ebenfalls 120 Ω**, OD 7,0 mm statt 8,5 mm — dünner in der Kette, und 0,25 mm² sitzt mittig im PTSM-Fenster (0,14–0,5 mm²) statt wie jetzt am Anschlag.

**Schirm — hybride Auflage.** R6 1 M ‖ C11 100 n je Board: DC-getrennt (keine Schleife für die bis zu 12,5 A Motorrückstrom), HF-gebunden. **R10 (0 Ω, harte Auflage) darf nur auf genau einem Board bestückt werden** — am Pi-/Steuerungsende. Sechs harte Auflagen wären genau die Schleife, die das 1 M vermeiden soll. Gehört in den Bestückungsplan.

**Split-Termination** (statt einem 120-Ω-Widerstand):

```
CANH ──[SW2 Pol A]── R13 60,4Ω ──┬── R12 60,4Ω ──[SW2 Pol B]── CANL
                                 │
                                C13 4,7 nF (C0G)
                                 │
                                GND
```

Das gibt die Common-Mode-Dämpfung und -Referenz, die ohne eigene CAN-GND-Ader sonst fehlt. Eckfrequenz `f_c = 1/(2π · (60‖60) · 4,7 nF) ≈ 1,1 MHz` — passend für 1 Mbit/s (TI/Bosch empfehlen 4,7–10 nF).

- **SW1 muss 2-polig sein** (`Switch:SW_DIP_x02`, bestückt: **CTS 219-2LPSTJ**, Footprint `Button_Switch_SMD:SW_DIP_SPSTx02_Slide_6.7x6.64mm_W6.73mm_P2.54mm_LowProfile_JPin`), je ein Pol pro Zweig. Mit einem 1-poligen Schalter bliebe bei offenem Schalter `CANH → 60 Ω → 4,7 nF → GND` hängen (bei 10 MHz ~63 Ω gegen GND, vier nicht terminierte Boards parallel ~16 Ω) — eine einseitige HF-Last auf CANH und damit genau die Unsymmetrie, gegen die die Split-Termination antritt. Gewählt wurde der CTS 219-2LPSTJ (Raster 2,54 mm, 6,7 × 6,6 mm) statt des kleineren Copal CHS-02 (Raster 1,27 mm), weil er ohne Footprint-Wechsel auf das bestehende Layout passt.
- **60,4 Ω, 1 % (E96)** — nicht 60 Ω (kein E-Reihen-Wert) und nicht 5 %: ungleiche Hälften wandeln Common Mode in Differential Mode um.
- **C13 als C0G/NP0**, nicht X7R (DC-Bias-/Temperaturdrift verschiebt die Eckfrequenz).

**Bus-Schutz.** Bei −4 / +16 V Absolut-Maximum an den Busklemmen und 48 V in derselben WAGO-Klemme wie CANH/CANL (J4) gehört an jedes Board ein **CAN-TVS** (NUP2105L / PESD2CANFD o. ä., SOT-23) am J1/J2-Knoten. Galvanische Isolation (ISO1042 + isolierter DC/DC) wäre bei 6 Knoten und ~2–3 m Bus Overkill.

**Stichleitungen** bei 1 Mbit/s kurz halten: Abzweig J4 → Motortreiber möglichst < 30 cm. Die Gesamtlänge ist unkritisch (1 Mbit/s erlaubt 40 m).

---

## Spannungsdomänen

| Domäne | Quelle | Versorgt |
| --- | --- | --- |
| 48 V | Tether (durchgeschleift) | Motorabzweig, Bremsen-Endstufe |
| 5 V | Tether (durchgeschleift) | TLE4905L, AP2112K-Eingang, U3 (Gate-Treiber) |
| 3,3 V | U1 AP2112K-3.3 (600 mA) | XIAO, SN65HVD230, Pull-up |
| GND | ein gemeinsames Netz | alles |

CAN hat keine eigene GND-Ader (J1/J2 führen nur H/L/Schirm) — die Referenz kommt über die durchgehende Power-GND. **Muss über alle Boards durchgängig sein.** Rechnung und Begründung siehe „CAN-Bus: Referenz, Schirm, Terminierung" oben.

**D3 (SMAJ5.0CA, SMA/DO-214AC):** ausgewählt über Sperrspannung und Pulsenergie, **nicht** über den Laststrom — im Normalbetrieb führt sie keinen Strom. Die **CA-Variante ist bidirektional**, damit ist die Einbaurichtung egal (bei der unidirektionalen SMAJ5.0A müsste Pin 1 die Kathode sein, und das Symbol zeigt das nicht an).

Was sie leistet: Transienten und ESD wegstecken, und falls 48 V auf die 5-V-Schiene gelangen, als **Crowbar** in den Kurzschluss gehen und das Netzteil in die Strombegrenzung werfen. Sie klemmt bei ~9,2 V und hält den AP2112K (Abs-Max VIN 6,0 V) bei dauerhafter Überspannung **nicht** am Leben — das ist eine bewusst akzeptierte Grenze.

---

## Stückliste (BOM) — Bestellstand 2026-09-11

Bezeichner laut Schaltplan-BOM, Mengen für **6 Boards** plus Reserve. Bezugsquellen: Mouser-Projekte `R0192` und `R0192_Kleine_Komponenten`, WAGO/Netzteil/Crimpzange bei Reichelt.

### Kondensatoren

| Ref | Wert | Hersteller | Herst.-Nr. | Bestell-Nr. | je Board | bestellt |
| --- | --- | --- | --- | --- | ---: | ---: |
| C1, C7, C9, C11, C12 | 100 nF 50 V X7R 0603 | Yageo | CC0603KPX7R9BB104 | Mouser 603-CC603KPX7R9BB104 | 5 | 35 |
| C4 | 100 nF **100 V** X7R 0805 | Yageo | CC0805KKX7R0BB104 | Mouser 603-CC805KKX7R0BB104 | 1 | 10 |
| C5 | 1 µF 25 V X5R 0603 | Samsung | CL10A105KA8NFNC | Mouser 187-CL10A105KA8NFNC | 1 | 10 |
| C6 | 2,2 µF 16 V X5R 0603 | Samsung | CL10A225KO8NNNC | Mouser 187-CL10A225KO8NNNC | 1 | 10 |
| C8 | 10 µF 25 V X5R 0805 | Samsung | CL21A106KAYNNNG | Mouser 187-CL21A106KAYNNNG | 1 | 10 |
| C10 | 1 nF 50 V **C0G** 0603 | Murata | GRM1885C1H102JA01D | Mouser 81-GRM39C102J50 | 1 | 10 |
| C13 | 4,7 nF 50 V **C0G** 0603 | Murata | GRM1885C1H472JA01D | Mouser 81-GRM1885C1H472JA1D | 1 | 10 |
| C2, C3, C14 | 100 µF **100 V** Elko, Ø10 × 20 mm, RM 5 | Nichicon | UVR2A101MPD1TD | Mouser 647-UVR2A101MPD1TO | 3 | 15 (**für 6 Boards 18 nötig, 3+ nachbestellen**) |

### Widerstände (Yageo RC0603, 1 %, 1/10 W)

| Ref | Wert | Herst.-Nr. | Bestell-Nr. | je Board | bestellt |
| --- | --- | --- | --- | ---: | ---: |
| R1 | 4,7 kΩ | RC0603FR-074K7L | Mouser 603-RC0603FR-074K7L | 1 | 10 |
| R2 | 22 Ω | RC0603FR-1322RL | Mouser 603-RC0603FR-1322RL | 1 | 10 |
| R3, R4 | 10 kΩ | RC0603FR-0710KL | Mouser 603-RC0603FR-0710KL | 2 | 15 |
| R5, R14 | 220 Ω (UART-Serienwiderstände J6) | RC0603FR-07220RL | Mouser 603-RC0603FR-07220RL | 2 | 15 |
| R6 | 1 MΩ | RC0603FR-071ML | Mouser 603-RC0603FR-071ML | 1 | 10 |
| R7 | 150 kΩ | RC0603FR-07150KL | Mouser 603-RC0603FR-07150KL | 1 | 10 |
| R8, R9 | 6,8 kΩ | RC0603FR-076K8L | Mouser 603-RC0603FR-076K8L | 2 | 15 |
| R10 | 0 Ω (5 %), **nur auf einem Board** (Steuerungsende) | RC0603JR-070RL | Mouser 603-RC0603JR-070RL | 0–1 | 10 |
| R11 | 470 Ω | RC0603FR-07470RL | Mouser 603-RC0603FR-07470RL | 1 | 10 |
| R12, R13 | 60,4 Ω | RC0603FR-0760R4L | Mouser 603-RC0603FR-0760R4L | 2 | 15 |

### Halbleiter, Schutz, Schalter

| Ref | Funktion | Hersteller | Herst.-Nr. | Bestell-Nr. | je Board | bestellt |
| --- | --- | --- | --- | --- | ---: | ---: |
| D1 | Freilaufdiode, ultrafast 200 V / 1 A, SMA | Diotec | US1D | Mouser 637-US1D | 1 | 10 |
| D3 | TVS 5 V bidirektional, SMA | Littelfuse | SMAJ5.0CA | Mouser 576-SMAJ5.0CA | 1 | 10 |
| D4 | Status-LED rot, 0603 | Lite-On | LTST-C191KRKT | Mouser 859-LTST-C191KRKT | 1 | 10 |
| Q2 | N-MOSFET 100 V, 0,125 Ω @ V_GS 5 V, TO-252 | Infineon | IRLR3410TRPBF | Mouser 942-IRLR3410TRPBF | 1 | 10 |
| U1 | LDO 3,3 V / 600 mA, SOT-23-5 | Diodes Inc. | AP2112K-3.3TRG1 | Mouser 621-AP2112K-3.3TRG1 | 1 | 10 |
| U2 | XIAO ESP32-S3 (Stiftleisten lose beiliegend) | Seeed Studio | 113991114 | Mouser 713-113991114 | 1 | 6 |
| U3 | Puffer 74AHCT1G125, SOT-25 | Diodes Inc. | 74AHCT1G125QW5-7 | Mouser 621-74AHCT1G125QW5-7 | 1 | 10 |
| U4 | CAN-Transceiver 3,3 V, SOIC-8 | Texas Instruments | SN65HVD230DR | Mouser 595-SN65HVD230DR | 1 | 10 |
| F1 | PPTC **60 V**, 0,5 A hold / 1 A trip, radial RM 5,1 | Littelfuse | 60R050XU | Mouser 576-60R050XU | 1 | 10 |
| SW1 | Split-Termination, DIP 2-polig, Low Profile, J-Bend | CTS | 219-2LPSTJ | Mouser 774-219-2LPSTJ | 1 | 10 |
| SW2 | 3,3-V-Trennung zum XIAO, Schiebeschalter SPDT 300 mA | C&K | PCM12SMTR | Mouser 611-PCM12SMTR | 1 | 7 |
| *neu (z. B. D2)* | CAN-TVS 12 V, 2 Leitungen bidirektional gegen GND, SOT-23 | Nexperia | PESD12VL2BT,215 | Mouser 771-PESD12VL2BT215 | 1 | **noch bestellen** |

### Steckverbinder (Platine)

| Ref | Funktion | Hersteller | Herst.-Nr. | Bestell-Nr. | je Board | bestellt |
| --- | --- | --- | --- | --- | ---: | ---: |
| J1, J2 | CAN-Bus, Push-in 3-polig, RM 2,5 | Phoenix Contact | PTSM 0,5/3-2,5-V THR R44 (1770966) | Mouser 651-1770966 | 2 | 15 |
| J3, J5 | Bremse / Hallsensor, JST PH 3-polig vertikal | JST | B3B-PH-K-S-GW | Mouser 306-B3BPHKSGW | 2 | 15 |
| J4, J10, J12 | Motorabzweig / Power, Hebelklemme 4-polig, RM 3,5 | WAGO | 2601-3104 | Reichelt WAGO 2601-3104 | 3 | 20 |
| J6 | UART-Debug, Stiftleiste 1×4, RM 2,54 | Würth Elektronik | 61300411121 | Mouser 710-61300411121 | 1 | 10 |

### Kabelseite & Werkzeug (nicht auf der Platine)

| Teil | Verwendung | Hersteller | Herst.-Nr. | Bestell-Nr. | bestellt |
| --- | --- | --- | --- | --- | ---: |
| PH-Gehäuse 3-polig | Gegenstecker J3 / J5 | JST | PHR-3 | Mouser 306-PHR-3 | 15 |
| PH-Crimpkontakt AWG 30–24 | J3 (2 Pins) + J5 (3 Pins) | JST | SPH-002T-P0.5S | Mouser 306-SPH-002T-P0.5S | 100 |
| Aderendhülse unisoliert 0,5 mm² × 6 mm | J1 / J2 (CAN + Schirm-Pigtail) | Altech | 2216.0 (H0.50/6) | Mouser 845-2216.0 | 50 |
| Aderendhülse unisoliert 1,5 mm² × 10 mm | J4 / J10 / J12 Power-Adern | Altech | 2222.0 | Mouser 845-2222.0 | 100 |
| Crimpzange Aderendhülsen 0,25–6 mm² | Aderendhülsen | — | CRIMPZANGE AEH | Reichelt | 1 |
| 5-V-Netzteil 90 W / 18 A | 5-V-Schiene (einmal pro Arm) | MEAN WELL | LRS-100-5 | Reichelt MW LRS-100-5 | 1 |

**Bereits vorhanden:** Hallsensoren TLE49x5L und Magnete Ø 4 × 2 mm (Reichelt MAGNET 4.2). **Typ prüfen:** Homing-Ablauf und Firmware setzen einen **TLE4905L** (unipolarer Schalter) voraus; ein **TLE4935L** (bipolarer Latch) gibt mit nur einem Magneten nach dem Überfahren nicht wieder frei.

**Hinweise zur BOM**

- **Schaltplan-Werte nachziehen:** D4 steht noch auf `LED` → `LTST-C191KRKT` (rot).
- **Aderendhülsen:** Die WAGO 2601 ist für 8–9 mm Abisolierlänge ausgelegt; steht die 10-mm-Hülse sichtbar über das Gehäuse, nach dem Crimpen auf ~8 mm kürzen. Die CAN-Adern in **J4 ohne Hülse** klemmen (0,5 × 6 mm ist dort zu kurz; feindrähtig ohne Hülse ist zulässig). Isolierte Hülsen passen nicht: WAGO nimmt isolierte Hülsen nur bis 0,75 mm², PTSM gar nicht.
- **Altech 2222.0:** Mouser beschreibt sie als „H100/10“; nach Altech-Nummernschema (2219/2220 = 1,0 mm², 2221/2222 = 1,5 mm²) ist es die 1,5-mm²-Hülse — beim Auspacken kurz gegenprüfen.
- **Nicht bestellt / nicht nötig:** Buchsenleisten für den XIAO (wird direkt gelötet), Serien-PTC in der 5-V-Einspeisung (noch offener Schaltplan-Punkt, s. u.).

---

## Layout-Vorgaben (noch offen)

- GND-Pour, Sternpunkt zwischen Power-GND (Bus-/Motorklemmen, Q2-Source, 48-V-Bulk) und Signal-GND (MCU, U4, U1, TLE4905L). Beide GND-Pins von J12/J10 breit auf die Massefläche führen.
- **Bremsen-Schaltknoten-Loop klein** (Q2 / Spule / D1 eng), gepulste 48 V **weg von den CAN-Leitungen**.
- **48-V-Bus auf 12,5 A** (LRS-600N2-Limit) — und das schließt Tracks aus. IPC-2221, 1 oz außen, 20 K Anstieg: **~8 mm** für 12,5 A (10 K → ~11 mm). Die Netclass `Power` steht auf 2,0 mm ≈ 4 A, also **3–4× zu schmal**. Deshalb: +48 V und GND als **Pours**, bevorzugt auf F.Cu/B.Cu parallel (je ~4 mm äquivalent). In2.Cu ist im Standard-4-Lagen-Stackup meist **0,5 oz** — dort bräuchte man ~25 mm Breite, also nicht der richtige Ort für den Durchschleifstrom.
- **THT-Pads von J10/J12/J4 auf `solid connection`** statt Thermal Relief — die Wärmefalle schnürt sonst genau den 12,5-A-Pfad ab.
- Lagenwechsel im 48-V-/GND-Pfad vermeiden: eine 0,8/0,4-Via trägt ~1,5–2 A, sonst braucht es ≥ 10 Stück.
- Motorabzweig auf Einzelmotor-Peak, Bremsenkanal ≥ 2 A.
- Entkopplungs-Cs direkt an den IC-Pins. **U1 (SOT-23-5) großzügig Kupfer geben**: ~150 mA × 1,7 V = 0,26 W, ohne Fläche ~50 K Anstieg im Gelenkgehäuse.
- **D3 ans Board-Ende bei J12/J10**, wo die 5 V ankommen — eine TVS wirkt dort, wo sie sitzt, nicht neben dem LDO.
- **48-V-Clearance auf 0,5 mm** anheben (IPC-2221 fordert nur 0,1 mm, aber Staub/Kondensat im Arm). Und die DRC-Constraints stehen auf `min_clearance: 0.0` / `min_track_width: 0.0` — echte Minima eintragen, sonst fängt der DRC nichts ab.
- **Netznamen mit Leerzeichen und Klammern** (`GPIO D3 (XIAO-ESP32-S3)`) vor dem Routen umbenennen (`BRAKE_PWM`, `HALL_IN`, `BRAKE_SENSE`, `BRAKE_SW`, `LED`, `CAN_TX/RX`) — sonst sind Netclass-Patterns und Custom-DRC-Rules unangenehm.
- Platinenumriss: **⌀70 mm Kreis** (Mitte 150/100, r 35), 4× M2 auf r = 26 mm. Auf `Edge.Cuts` liegen noch **acht entartete `point`-Objekte auf (148, 96.5)** — löschen.
- GND-Zone auf In1.Cu ist ein willkürliches Achteck, das nicht zum Kreis passt → neu ziehen. Auf **In2.Cu existiert noch gar keine Zone** (48 V / 5 V / 3,3 V als getrennte Pours).
- Das `.kicad_pcb` ist **nicht mit dem Schaltplan synchronisiert**: F1 fehlt komplett, die TVS heißt dort noch `D2`, und es gibt ein Kollisionsnetz `/GPIO D5#`. Vor dem Routen *Update PCB from Schematic*.

---

## Offene Punkte

**Schaltplan**
- [x] **XIAO-Insel aufgelöst:** J7/J8/J9/J11 gelöscht, Signale direkt an U2. Netzliste geprüft — D0 Bremse, D1 Hall, D2 LED, D3 Sense, D6/D7 UART, D9/D10 CAN, 3V3/GND versorgt, D4/D5/D8/VBUS mit NC-Flag.
- [x] **Zwei leere Labels löschen** bei (83.82, 156.21) — deckungsgleich unter `Brake Gate` — und (111.76, 215.9), auf dem `TLE4905L Out`-Netz. Zwei Labels mit gleichem (leerem) Text verschmelzen Hallsensor-Ausgang und MOSFET-Gate zu **einem** Netz; im letzten PCB-Sync war das `/<NO NET>` mit genau 7 Pads (`J5, R1, C10, R9` + `Q2.G, R2, R4`). Deshalb wirken die neuen Labels nicht. In KiCad: draufklicken → „Clarify Selection" → das Label ohne Namen löschen, oder Selection Filter auf *Labels* und einen Rahmen ziehen. *Geprüft 2026-09-11:* keine leeren Labels mehr, `TLE4905L Out` und `BRAKE GATE` sind getrennte Netze.
- [x] Leeres `text`-Objekt entfernt (auch das zweite bei (359.41, 125.095)).
- [x] **Status-LED-Label korrigieren:** heißt `GPIO D5 (XIAO-ESP32-S3)`, hängt aber an U2.3 = GPIO3 = **D2**. Da D5 ein NC-Flag hat, ist D2 die Absicht → Label auf `GPIO D2` umbenennen (Vorteil: D4/D5 bleiben als SDA/SCL frei). *Erledigt:* Label `GPIO D2`, Netz R11 ↔ U2.3.
- [x] `U1.4 (NC)` hat kein No-Connect-Flag → ERC-Rauschen. *Erledigt:* ERC meldet keine unverbundenen Pins mehr.
- [x] **Split-Termination fertigstellen:** SW2 auf **2-polig** (`Switch:SW_DIP_x02`, `SW_DIP_SPSTx02_Slide_Copal_CHS-02A_W5.08mm_P1.27mm_JPin`), je ein Pol pro Zweig; R12/R13 auf **60,4 Ω 1 %** (E96); C13 als **C0G**. Notiztext im Rahmen („120 Ohm → aktivierbarer Abschlusswiderstand") anpassen. *Erledigt:* SW1 = CTS 219-2LPSTJ (2-polig), R12/R13 = 60,4 Ω 1 %, C13 C0G bestellt. *Rest:* Notiztext im CAN-Rahmen steht noch auf „120 Ohm -> aktivierbarer Abschlusswiderstand“.
- [ ] **CAN-TVS** (NUP2105L / PESD2CANFD, SOT-23) am J1/J2-Knoten ergänzen. *Empfehlung:* **Nexperia PESD12VL2BT,215** (Mouser 771-PESD12VL2BT215), 12 V V_RWM, V_BR 14,2–16,7 V, 19 pF, 200 W, SOT-23. Pin 1 = CANH, Pin 2 = CANL, Pin 3 = GND (jede Leitung gegen GND, **nicht** CANH–CANL). Symbol `Power_Protection:NUP2105L` (gleiche Pinbelegung, Value ändern), Footprint `Package_TO_SOT_SMD:SOT-23`. Die übliche NUP2105L (24 V, V_BR ≥ 26,2 V) klemmt erst oberhalb der −4/+16-V-Grenze des SN65HVD230. Noch nicht im Schaltplan, noch nicht bestellt.
- [ ] **Serienelement vor D3 in die 5-V-Einspeisung** (PPTC 0,5 A oder 1-A-Sicherung). Ohne das kann die SMAJ5.0CA ihre Crowbar-Rolle nicht überleben: bei 48 V auf der 5-V-Schiene müsste sie 12,5 A × 9 V ≈ 112 W verheizen und stirbt — ob kurz oder offen ist Glückssache, und „offen" heißt 48 V auf allem. Realistischer Fehlerfall: die 4-polige WAGO verkehrt herum gesteckt.
- [x] **C2/C3 auf `Device:C_Polarized`** umstellen — Symbol ist ungepolt (`Device:C`), Footprint ist ein Elko (`CP_Radial_D10.0mm_P5.00mm`). Ohne Polaritätsmarkierung im Bestückungsdruck, und bei verpolter 48-V-Einspeisung fliegen beide. Verpolschutz gibt es auf dem Board keinen. *Erledigt.* 3D-Modell: KiCad hat Ø 10 × 20 mm nur mit RM 7,5 — daher Footprint `CP_Radial_D10.0mm_P5.00mm` behalten und im 3D-Modell **Scale Z = 2,0** setzen (das KiCad-Modell ist trotz „height=16mm“ nur ~10 mm hoch → 20 mm, per Render geprüft).
- [ ] **R10 aus dem Value-Feld** (`DNP`) ins DNP-Attribut verschieben, sonst steht ein Widerstand mit Wert „DNP" in der BOM.
- [x] **J6 (UART): 2× 220 Ω in TX/RX** (R5 / R14) — ein versehentlich angesteckter 5-V-USB-Serial-Adapter grillt sonst den ESP32.
- [x] J6-Note „nur Platzhalter -> nicht auflöten“ entfernt — J6 wird bestückt.
- [x] **XIAO-3V3-Backfeed** über **SW2** (PCM12SMTR, 300 mA) entschärft — beim Flashen über USB öffnen. Hintergrund: U1 treibt den 3V3-Pin: beim Flashen über USB kämpfen AP2112K und XIAO-LDO gegeneinander. Entweder „beim Flashen 5 V aus" oder ein 0-Ω-Trennpunkt in der 3V3-Zuleitung.
- [x] Netz `F1.1 / J3.1` (48 V hinter der Sicherung) benennen (`BRAKE_48V`) — im Layout sonst nicht unterscheidbar vom ungeschützten 48 V. *Erledigt:* Label `BRAKE +48V`.
- [x] **PPTC auf die 48-V-Seite verlegt** (jetzt F1), Symbol `Device:Polyfuse`, Footprint `Fuse_Bourns_MF-RHT050` zugewiesen. Value auf Littelfuse **60R050XU** (60 V) umgestellt. Footprint `R0192:Fuse_Littelfuse_60R050XU` zugewiesen (steht auf „Polyfuse“).
- [x] **J12/J10 auf `Conn_01x04` umgestellt**, Belegung +5 V | GND | GND | +48 V verdrahtet.
- [x] **CAN-Pinreihenfolge der Treiber geklärt:** GDS68 und RS05 sind unterschiedlich belegt → kein einmaliger Schaltplan-Tausch möglich, wird pro Achse an der Klemme verdrahtet.
- [ ] **TVS/Zener (~68 V) vom Schaltknoten nach GND** — nur nötig, wenn die gemessene Spuleninduktivität > ~380 mH liegt (darunter reicht D1 allein für ein Einfallen unter 20 ms). Schützt zusätzlich den Sense-Teiler vor Flyback-Spitzen.
- [x] **Sense-Teiler entschärft:** R8 = 6,8 k → 48 V × 6,8/156,8 = 2,08 V, sauber linear. *Nebenbei:* 0 V bei PWM = 0 heißt „FET durchlegiert **oder** Bremsleitung unterbrochen" — beides Fehler, aber nicht unterscheidbar.
- [x] **R11 = 470 Ω** eingetragen (~3 mA an roter/grüner 0603-LED).
- [x] MPNs in die Value-Felder: **D1** US1D, **F1** 60R050XU, **Q2** IRLR3410TRPBF (R_DS(on) = 0,125 Ω bei V_GS = 5 V, geprüft).
- [ ] **D4** Value steht noch auf `LED` → `LTST-C191KRKT` (rot).
- [x] **TVS auf SMAJ5.0CA (D3) gewechselt** — bidirektional, Einbaurichtung damit egal.
- [x] C2/C3 mit Note „mind. 100 V spannungsfest“ versehen.
- [ ] **C4 ≥ 100 V** noch vermerken.
- [x] **C6 auf 2,2 µF** angehoben (DC-Derating am LDO-Ausgang).
- [ ] Note für J4 fehlt noch (Leiterquerschnitt Motorabzweig, 1,5 mm² Power + 0,5 mm² CAN).
- [ ] Sicherung / Inrush-Begrenzung im 48-V-Pfad erwägen (2× 100 µF pro Board × 6 Boards = 1,2 mF beim Hot-Plug).
- [ ] **ERC aufräumen:** 245× `endpoint_off_grid` (Pins/Drahtenden neben dem Raster) und Symbolbibliothek `Seeed_Studio_XIAO_Series` nicht gefunden (verweist auf alten OneDrive-Pfad).
- [ ] **Mis-Plug-Risiko J4 ↔ J10/J12**: dreimal dieselbe WAGO 2601-3104 mit drei Belegungen. Motorkabel in J10 legt CANH auf +5 V. Bestückungsdruck allein reicht bei 6 Boards × 3 Klemmen nicht — unterschiedliche Kabellängen oder anderer Typ für J4.

**Auslegung**
- [x] **Wellenimpedanz der CFBUS.PVC.021 geklärt:** laut igus-Datenblatt **120 ± 12 Ω**, explizit als CAN-Bus-Typ gelistet (Profibus ist `.001` mit 150 Ω). Terminierung bleibt 120 Ω gesamt.
- [x] **Eigene GND_CAN-Ader geprüft — nicht nötig.** Rechnung im Abschnitt „CAN-Bus" oben. Bedingung: CAN- und Powerkabel gemeinsam verlegen.
- [ ] Im Bestückungsplan festhalten: **R10 nur auf einem Board** (Steuerungsende), Split-Termination nur auf den zwei Busenden.
- [ ] Haltemoment unter Payload an **Achse 2/3** verifizieren (~6,4 N·m Bremse vs. 7,5 N·m Motor-Nennmoment), kalt und warm.
- [ ] Spuleninduktivität messen → entscheidet über die TVS und die PWM-Frequenz.

**Bring-up**
- [ ] Kurzschlusstest 48 V / 5 V / 3,3 V gegen GND, 3,3 V-Rail messen
- [ ] ESP32 flashen, CAN-Loopback, dann `/homing`-Protokoll end-to-end gegen den Pi
- [ ] TLE4905L mit Magnet: sauberer 3,3-V-Pegelwechsel an D1
- [ ] Bremse: Anzug (48 V, 150 ms) → öffnet; Halten bei 10,4 % Duty; Einfallzeit messen
- [ ] CAN mit allen 6 Boards bei 1 Mbit/s: Terminierung nur an den Enden, Eye/Fehlerzähler unter laufenden Motoren prüfen (das ist der Test, der die GND-Referenz wirklich beantwortet)
