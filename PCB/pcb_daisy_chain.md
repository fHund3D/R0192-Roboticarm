# R0192 „Daisy Chain" Driverlink-PCB

Bus-Verteilplatine, **einmal pro Achse**. Schleift 48 V, 5 V und CAN von Node zu Node durch, zweigt die Motorversorgung ab, wertet den Homing-Hallsensor aus und steuert die Aktuator-Haltebremse. Ersetzt den Arduino-Uno-Prototyp (`microcontroller/r0192_homing.ino`) durch einen **XIAO-ESP32-S3** mit integriertem CAN-Controller (TWAI).

Identische Platine an jeder Achsposition: Bus-Termination per Schalter, positionsabhängige Verbraucher (Bremse) per Bestückungsvariante.

KiCad-Projekt: `PCB/KiCad/DriverDasyChain.*` (4 Lagen: F.Cu / In1 GND / In2 PWR / B.Cu)
Datenblätter: `PCB/R0192.pretty/Datenblätter/` (Bauteile) und `.../Kabel/` (CF77.UL.D, CFBUS.PVC)

---

## Funktionsblöcke

| Block | Umsetzung |
| --- | --- |
| Power-Durchschleife | J12 / J10 (4-polig): +5 V / GND / GND / +48 V. Bulk C2/C3/C14 = 3× 100 µF / 100 V, C4 100 nF / 100 V |
| 3,3 V-Erzeugung | U1 AP2112K-3.3 aus 5 V (C8 10 µ / C5 1 µ / C6 2,2 µ), EN an VIN, D3 SMAJ5.0CA am Eingang (bidirektional) |
| MCU | U2 XIAO-ESP32-S3 (Footprint `R0192:XIAO-ESP32-S3-DIP`, THT-Sockel), Versorgung über **3V3-Pin**, trennbar über SW2 (PCM12SMTR) — beim Flashen über USB öffnen, VBUS = NC |
| CAN-Transceiver | U4 SN65HVD230 (nativ 3,3 V), Rs auf GND (High-Speed), Vref NC, C7 100 nF |
| CAN-Bus | J1/J2 (CANH/CANL/SHLD), Schirm hybrid über R6 1 M ‖ C11 100 n; R10 (0 Ω) = harte Schirmauflage, **nur auf einem Board bestücken** |
| Terminierung | **Split-Termination**: R13 + R12 je 60,4 Ω in Reihe über den Bus, Mittelpunkt über C13 4,7 nF auf GND. Zuschaltbar über SW1 (CTS 219-2LPSTJ, **2-polig**, je ein Pol pro Zweig) — nur an den zwei physischen Busenden einschalten |
| CAN-TVS | D2 PESD12VL2BT (Symbol `NUP2105L`), CANH und CANL je gegen GND, sitzt direkt vor U4 |
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

**Vertauschen der Klemmen ist gutartig:** An J4, J10 und J12 liegen Pin 3 (GND) und Pin 4 (+48 V) gleich. Motorkabel in J10/J12 → CANH auf +5 V, CANL auf GND. Powerkabel in J4 → +5 V auf CANH, GND auf CANL. Beides legt den Bus lahm, zerstört aber nichts (SN65HVD230 Abs-Max +16 V, PESD12VL2BT klemmt erst ab ~14 V). **Gefährlich ist nur eine umgekehrte Aderreihenfolge** (48 V auf Pin 1): in J10/J12 landen dann 48 V auf der 5-V-Schiene (s. Spannungsdomänen), in J4 48 V auf CANH. Deshalb Aderfarben pro Pin festlegen und jedes Kabel vor dem ersten Einschalten durchmessen.

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
+48V ─┬── F1 (PPTC) ── J3.1 ─── [Bremsspule 34Ω] ── J3.2 ───┬── Q2 Drain
      │                                                     │
      └──────────── D1 (K→+48V vor F1, A→Drain) ────────────┤  Freilauf
                                                            ├── R7 150k ── GPIO D3 (Sense) ── R8 6,8k ── GND
                                                       Q2 Source ── GND
```

**D1-Platzierung (bewusst so gelassen):** Die Kathode hängt am ungesicherten +48 V, nicht zwischen F1 und J3.1. Die Schutzfunktion von F1 hängt davon nicht ab: Liegen dauerhaft 48 V an der Spule (FET durchlegiert), fließt der Strom +48 V → F1 → Spule → Q2 → GND, D1 sperrt dabei, und F1 löst in beiden Varianten gleich aus. Der Unterschied betrifft nur den Freilauf: Hier fließt der Haltestrom auch in der PWM-Aus-Phase durch F1 (dauerhaft ~147 mA statt ~15 mA im Mittel). Mit 30 % von I_hold liegt das weit im grünen Bereich (Tabelle unten), und die ~1 Ω von F1 gegen 34 Ω Spule sind vernachlässigbar.

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

**Warum keine TVS/Zener am Schaltknoten:**

- **Parallel zu D1** (Schaltknoten → GND, Durchbruch über der Busspannung) leitet sie im Normalbetrieb nie: D1 klemmt den Drain bei ~48,7 V (mit Regen Clamp höchstens ~50,7 V). Sie schützt nur Q2, falls D1 fehlt oder offen ist, siehe D5 unten.
- **Anstelle von D1** passt sie zwar auf den SMA-Footprint, aber die Halte-PWM funktioniert dann nicht mehr. In jeder Aus-Phase muss der Spulenstrom über die TVS abgebaut werden, die Spule sieht −20 V (68 V − 48 V) statt −0,7 V. Mittlere Spulenspannung bei 10,4 % Duty: 0,104 × 48 V − 0,896 × 20 V ≈ −13 V. Der Strom bricht zusammen und die Bremse fällt ein. Halten ginge erst ab ~37 % Duty, und dann verheizt die TVS dauerhaft ~6 W (68 V × 147 mA × 63 %). Das überlebt keine SMA-TVS, und der Vorteil des Economizers ist weg.
- Schnelleres Einfallen bräuchte eine Zener in Reihe zu D1 **plus** einen zweiten (High-Side-)Schalter, der den Zener-Pfad nur beim Einfallen freigibt. Das ist ein Thema für V2.

**D5: TVS als Q2-Schutz (umgesetzt 2026-09-13, DNP).** Sie nützt nur, wenn D1 fehlt oder offen ist (z. B. Lötfehler). Ohne Freilaufpfad liefe der Drain beim Abschalten über die 100 V von Q2 hinaus.

| | |
| --- | --- |
| Bauteil | **Littelfuse SMAJ58A** (unidirektional), Mouser **576-SMAJ58A** |
| Daten | V_RWM 58 V, V_BR ≥ 64,4 V, V_C 93,6 V bei 4,3 A, 400 W Pulsleistung, DO-214AC |
| Symbol / Footprint | `Diode:SMAJ58A` / `Diode_SMD:D_SMA` mit 3D-Modell `D_SMA.step`, DNP-Attribut gesetzt. Die Pins heißen im KiCad-Symbol `A1`/`A2`; maßgeblich ist die Pad-Nummer: Pad 1 = Kathode (Band im Bestückungsdruck) |
| Anschluss | Pad 1 (Kathode) an `BRAKE Out`, Pad 2 (Anode) an GND, per Netzliste geprüft |
| Platz | senkrecht zwischen D1 und J3/F1 (x 177,7 / y 91,6). Kathode über eine 1,5-mm-Bahn direkt von D1 (Drain-Knoten), Anode über 1,5 mm auf die GND-Via bei 176,5 / 87,1. F1 sitzt dafür jetzt oberhalb (y ≈ 86) |
| Einlöten | Kathodenband nach unten, Richtung D1/Q2 (Pad 1 bei y 93,6) |

**Warum 58 V:** Die Sperrspannung muss über dem Bus inklusive Regen Clamp (~50 V) liegen, die Klemmspannung unter den 100 V von Q2. Ohne D1 kann die Bremse bei 10,4 % Duty nicht halten (s. oben), der Fehler fällt also sofort auf. Die TVS muss dann vor allem die eine Abschaltung aus der Anzugsphase abfangen. Das ist Notfallschutz, kein Dauerbetrieb. Unidirektional reicht, weil der Drain wegen der Body-Diode von Q2 nie unter −0,7 V fällt.

**Einfallzeit mit D1:** Der Strom klingt mit τ = L/R ab, die Bremse fällt bei ~60 mA. Aus dem Haltezustand (147 mA) dauert das t ≈ τ · ln(147/60) ≈ 0,9 τ. Für < 20 ms darf die Spule also bis ~0,78 H haben (R ≈ 35 Ω). Die alte Grenze „~380 mH" galt für 24-V-Dauerbetrieb mit ~450 mA (ln(450/60) ≈ 2). Direkt aus der Anzugsphase (1,41 A) wären es ≈ 3,2 τ, dort also nur bis ~0,22 H. Der Economizer beschleunigt das Einfallen damit deutlich. Einfallzeit beim Bring-up messen.

**Sicherheit:** Die Spule hängt an 48 V — ein Not-Aus, der 48 V trennt (Schütz), lässt alle Bremsen zwangsläufig einfallen. Das ist die Hardware-Zwangsabschaltung; ein reiner Software-Not-Aus reicht nicht. **Aber verzögert:** Nach dem Trennen halten die Buskondensatoren (6 × ~300 µF ≈ 1,8 mF, dazu ggf. die Ausgangskondensatoren des Netzteils) die 48 V noch eine Weile. Solange der ESP32 an 5 V hängt, läuft die Halte-PWM weiter. Die Bremsen fallen erst, wenn die mittlere Spulenspannung unter ~2,5 V sinkt, bei 10,4 % Duty also unter ~24 V Bus. Bei 0,1–0,2 A Restlast dauert das einige hundert Millisekunden, und die Motoren sind in dieser Zeit schon stromlos. Abhilfe: Die Firmware setzt beim Not-Aus sofort PWM = 0, **oder** das Schütz trennt auch die 5 V — dann schalten R3/R4 den FET sofort ab.

Der Sense-Pin ist **Diagnose, keine Schutzfunktion** — er erkennt einen durchlegierten FET, kann ihn aber nicht abschalten. 0 V bei PWM = 0 heißt „FET durchlegiert **oder** Bremsleitung unterbrochen": Beides ist ein Fehler, lässt sich aber nicht unterscheiden.

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
CANH ──[SW1 Pol A]── R13 60,4Ω ──┬── R12 60,4Ω ──[SW1 Pol B]── CANL
                                 │
                                C13 4,7 nF (C0G)
                                 │
                                GND
```

Das gibt die Common-Mode-Dämpfung und -Referenz, die ohne eigene CAN-GND-Ader sonst fehlt. Eckfrequenz `f_c = 1/(2π · (60‖60) · 4,7 nF) ≈ 1,1 MHz` — passend für 1 Mbit/s (TI/Bosch empfehlen 4,7–10 nF).

- **SW1 muss 2-polig sein** (`Switch:SW_DIP_x02`, bestückt: **CTS 219-2LPSTJ**, Footprint `Button_Switch_SMD:SW_DIP_SPSTx02_Slide_6.7x6.64mm_W6.73mm_P2.54mm_LowProfile_JPin`), je ein Pol pro Zweig. Mit einem 1-poligen Schalter bliebe bei offenem Schalter `CANH → 60 Ω → 4,7 nF → GND` hängen (bei 10 MHz ~63 Ω gegen GND, vier nicht terminierte Boards parallel ~16 Ω) — eine einseitige HF-Last auf CANH und damit genau die Unsymmetrie, gegen die die Split-Termination antritt. Gewählt wurde der CTS 219-2LPSTJ (Raster 2,54 mm, 6,7 × 6,6 mm) statt des kleineren Copal CHS-02 (Raster 1,27 mm), weil er ohne Footprint-Wechsel auf das bestehende Layout passt.
- **60,4 Ω, 1 % (E96)** — nicht 60 Ω (kein E-Reihen-Wert) und nicht 5 %: ungleiche Hälften wandeln Common Mode in Differential Mode um.
- **C13 als C0G/NP0**, nicht X7R (DC-Bias-/Temperaturdrift verschiebt die Eckfrequenz).

**Bus-Schutz.** Bei −4 / +16 V Absolut-Maximum an den Busklemmen und 48 V in derselben WAGO-Klemme wie CANH/CANL (J4) gehört an jedes Board ein **CAN-TVS** (NUP2105L / PESD2CANFD o. ä., SOT-23) am J1/J2-Knoten. *Umgesetzt:* D2 **Nexperia PESD12VL2BT,215**: 12 V V_RWM, V_BR 14,2–16,7 V, 19 pF, 200 W, SOT-23. Pin 1 = CANH, Pin 2 = CANL, Pin 3 = GND (jede Leitung gegen GND, **nicht** CANH–CANL), Symbol `Power_Protection:NUP2105L` mit geändertem Value. Die übliche NUP2105L (24 V, V_BR ≥ 26,2 V) klemmt erst oberhalb der −4/+16-V-Grenze des SN65HVD230. D2 sitzt nicht am J1/J2-Knoten, sondern ~7 mm vor U4 am Ende der Stichleitung (GND-Via direkt daneben). Für den Schutz des Transceivers ist das gleichwertig. Galvanische Isolation (ISO1042 + isolierter DC/DC) wäre bei 6 Knoten und ~2–3 m Bus Overkill.

**Stichleitungen** bei 1 Mbit/s kurz halten: Abzweig J4 → Motortreiber möglichst < 30 cm. Die Gesamtlänge ist unkritisch (1 Mbit/s erlaubt 40 m).

---

## Spannungsdomänen

| Domäne | Quelle | Versorgt |
| --- | --- | --- |
| 48 V | Tether (durchgeschleift) | Motorabzweig, Bremsen-Endstufe |
| 5 V | Tether (durchgeschleift) | TLE4905L, AP2112K-Eingang, U3 (Gate-Treiber) |
| 3,3 V | U1 AP2112K-3.3 (600 mA) | XIAO, SN65HVD230, Pull-up |
| GND | ein gemeinsames Netz | alles |

**Rückspeisung beim Abbremsen:** Das Netzteil kann keinen Strom aufnehmen. Ein zentraler **ODrive Regen Clamp** am Bus begrenzt die Spannung auf ~50 V. Damit bleiben alle Bauteile mit Abstand unter ihren Grenzen: F1 60 V, SMAJ58A (falls bestückt) 58 V Sperrspannung, GDS68 max. 72 V laut Handbuch, Q2 und Elkos 100 V. Die Ausgangsspannung des 48-V-Netzteils muss klar unter der Clamp-Schwelle liegen, sonst arbeitet der Clamp dauerhaft und heizt.

CAN hat keine eigene GND-Ader (J1/J2 führen nur H/L/Schirm) — die Referenz kommt über die durchgehende Power-GND. **Muss über alle Boards durchgängig sein.** Rechnung und Begründung siehe „CAN-Bus: Referenz, Schirm, Terminierung" oben.

**D3 (SMAJ5.0CA, SMA/DO-214AC):** ausgewählt über Sperrspannung und Pulsenergie, **nicht** über den Laststrom — im Normalbetrieb führt sie keinen Strom. Die **CA-Variante ist bidirektional**, damit ist die Einbaurichtung egal (bei der unidirektionalen SMAJ5.0A müsste Pin 1 die Kathode sein, und das Symbol zeigt das nicht an).

Was sie leistet: Transienten und ESD wegstecken, und falls 48 V auf die 5-V-Schiene gelangen, als **Crowbar** in den Kurzschluss gehen und das Netzteil in die Strombegrenzung werfen. Sie klemmt bei ~9,2 V und hält den AP2112K (Abs-Max VIN 6,0 V) bei dauerhafter Überspannung **nicht** am Leben — das ist eine bewusst akzeptierte Grenze.

**Kein Serienelement vor D3 (entschieden 2026-09-13):** Die 5 V laufen als Fläche auf In2 direkt von J10.1 nach J12.1. Ein Serienelement ginge nur mit einem eigenen lokalen 5-V-Netz und Layoutumbau. Stattdessen wird sorgfältig verdrahtet: Aderfarben pro Pin festlegen und jedes Kabel vor dem ersten Einschalten durchmessen (+48 V nur auf Pin 4). Bei umgekehrter Aderreihenfolge liegen 48 V auf der 5-V-Schiene. D3 geht dann als Crowbar in den Kurzschluss und stirbt dabei vermutlich (12,5 A × ~9 V ≈ 112 W). Ob sie kurz oder offen ausfällt, ist Glückssache, und „offen" heißt 48 V auf U1, U3, dem XIAO und dem Hallsensor.

---

## 48-V-Pufferkondensatoren (C2/C3/C14)

Die GDS68 haben kaum eigene Eingangskapazität, und das Board sitzt per M2 direkt am Treiber (kurze J4-Leitung). C2/C3/C14 sind damit praktisch die **Eingangskondensatoren des Treibers**. Ihre Aufgabe ist der Schaltstrom des Wechselrichters (Rippel im PWM-Takt), **nicht** Energie für Beschleunigungen:

```
ΔU = I · t / C
Rippel, ~1 PWM-Periode:   2 A · 40 µs / 300 µF  ≈ 0,3 V     → machbar
Beschleunigung:           5 A · 10 ms / 300 µF  ≈ 167 V     → unmöglich, kommt aus dem Netzteil über die Leitung
```

Mehr µF helfen also kaum. Entscheidend sind **Rippelstrom-Belastbarkeit, Impedanz bei 10–100 kHz und Temperatur** (warmes Gelenkgehäuse).

**Belastung:** GIM6010-8 (48 V): 2,8 A Nennstrom, 17,2 A Blockierstrom. GDS68 laut Handbuch: 6 A Nennstrom, 30 A max. Leitungsstrom. Der Eingangs-Rippelstrom eines 3-Phasen-Wechselrichters liegt grob bei 0,3–0,65 × Phasenstrom, im Nennbetrieb also bei ~1–2 A rms, kurzzeitig deutlich mehr. Pro Elko sind das ~0,3–0,7 A.

| | Nichicon **UVR2A101MPD** (bestellt) | Nichicon **UHE2A820MPD** (Alternative) |
| --- | --- | --- |
| C / U / Maß | 100 µF / 100 V / Ø10 × 20, RM 5 | 82 µF / 100 V / Ø10 × 20, RM 5 → **gleicher Footprint** |
| Serie | Standard („entertainment electronics") | Low Impedance, Long Life |
| Temperatur / Lebensdauer | 85 °C / 2000 h | 105 °C, Long Life |
| tan δ (120 Hz) | 0,08 → ESR ≤ ~1 Ω bei 120 Hz | 0,08 |
| Impedanz 100 kHz | nicht spezifiziert | **0,21 Ω** (20 °C), 0,94 Ω (−10 °C) |
| Rippelstrom | 370 mA (85 °C / 120 Hz), bei ≥ 10 kHz × ~1,5–2 | **518 mA (105 °C / 100 kHz)** |

Quellen: Nichicon-Kataloge UVR und UHE (CAT.8100M).

**Entscheidung (2026-09-13): UVR2A101MPD bleiben.** Die UHE kostet etwa das Doppelte. Für die Belastung reicht die UVR, die UHE wäre nur die robustere Wahl bei dauerhaft hoher Last im warmen Gehäuse.

**Zwei kleinere Elkos statt C14? Lohnt sich nicht.** Innerhalb einer Serie hängt der ESR an der Gesamtkapazität, nicht an der Stückzahl: Bei der UVR ist tan δ für alle 100-V-Werte 0,08, also ist ESR ∝ 1/C. Werte aus dem UVR-Katalog:

| Variante | C gesamt | Rippelstrom gesamt (85 °C / 120 Hz) | Bauhöhe |
| --- | ---: | ---: | ---: |
| 1 × UVR2A101MPD, Ø10 × 20 (jetzt) | 100 µF | 370 mA | 20 mm |
| 2 × UVR2A470MPD, Ø10 × 12,5 | 94 µF | 460 mA | 12,5 mm |
| 2 × UVR2A330MPD, Ø8 × 11,5 | 66 µF | 360 mA | 11,5 mm |

Der ESR bleibt praktisch gleich. Die 47-µF-Variante bringt ~25 % mehr Rippelstrom bei gleichem Durchmesser, die Ø-8-Variante hat weniger Kapazität, also *weniger* Reserve. Einziger echter Vorteil wäre die geringere Bauhöhe. Dafür müsste die dicht belegte Mitte (C14 zwischen C2/C3 und J6) umgeroutet werden. Die wirksamen Hebel liegen woanders:

1. **J4-Adern +48 V/GND kurz und verdrillt** zum Treiber führen: Jeder Zentimeter Schleifenfläche liegt zwischen Elko und Brücke.
2. *Am Treiber:* Hat der GDS68 an den Leistungs-MOSFETs gar keine Keramikkondensatoren, 1–2 × MLCC 2,2–4,7 µF / 100 V (X7R, 1210) direkt an seinen Versorgungspads nachrüsten. Die MHz-Anteile der Schaltflanken kann nur ein Kondensator unmittelbar an der Brücke liefern, nicht das Board über die Leitung.
3. **Keine Polymer- oder Ultra-Low-ESR-Kondensatoren** auf dem Bus: Mit der Leitungsinduktivität zwischen den Boards bilden sie einen kaum gedämpften Schwingkreis, und beim Zuschalten schwingt die Spannung auf bis zu ~2× über. Der ESR der Alu-Elkos dämpft das.
4. 48 V nie unter Spannung stecken, sondern zentral schalten (Schütz/Netzteil).

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
| D2 | CAN-TVS 12 V, 2 Leitungen bidirektional gegen GND, SOT-23 | Nexperia | PESD12VL2BT,215 | Mouser 771-PESD12VL2BT215 | 1 | **noch bestellen** |
| D5 (DNP) | TVS 58 V unidirektional, Q2-Schutz bei fehlendem D1, SMA | Littelfuse | SMAJ58A | Mouser 576-SMAJ58A | 0 (DNP) | **noch bestellen** (Reserve) |

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

- **Aderendhülsen:** Die WAGO 2601 ist für 8–9 mm Abisolierlänge ausgelegt; steht die 10-mm-Hülse sichtbar über das Gehäuse, nach dem Crimpen auf ~8 mm kürzen. Die CAN-Adern in **J4 ohne Hülse** klemmen (0,5 × 6 mm ist dort zu kurz; feindrähtig ohne Hülse ist zulässig). Isolierte Hülsen passen nicht: WAGO nimmt isolierte Hülsen nur bis 0,75 mm², PTSM gar nicht.
- **Altech 2222.0:** Mouser beschreibt sie als „H100/10“; nach Altech-Nummernschema (2219/2220 = 1,0 mm², 2221/2222 = 1,5 mm²) ist es die 1,5-mm²-Hülse — beim Auspacken kurz gegenprüfen.
- **Nicht bestellt / nicht nötig:** Buchsenleisten für den XIAO (wird direkt gelötet), Serien-PTC in der 5-V-Einspeisung (bewusst verworfen, s. Spannungsdomänen).

---

## Layout-Stand (geprüft 2026-09-13)

Geprüft mit `kicad-cli` (ERC; DRC mit Schaltplan-Parität und neu gefüllten Zonen) und einer Widerstandsrechnung der In2-Flächen (Gitter 0,1 mm auf den gefüllten Zonenpolygonen aus pcbnew).

**DRC:** 0 unverbundene Pads, 0 Footprint- und Paritätsfehler. Übrig bleiben 6× Silk-Überlappung C2/C14, 6× Silk am Platinenrand (J10/J12/U2-USB), 1× Texthöhe 0,7 mm („USB-C") und 2× Courtyard H3↔J12 / H4↔J10. Bei den Courtyards Schraubenkopf und Unterlegscheibe gegen das WAGO-Gehäuse mechanisch prüfen.

**Lagen:** F.Cu Signale · In1 GND (eine durchgehende Fläche) · In2 +48 V mit eingeschnittener +5-V-Fläche · B.Cu GND + 3,3-V-Insel + wenige Signale. Im KiCad-Stackup stehen alle Lagen auf 35 µm. **Innenlagen-Kupfer beim Fertiger prüfen**, viele Standard-4-Lagen-Aufbauten haben innen 0,5 oz. Die Rechnung unten deckt beide Fälle ab.

**48 V auf In2 — nachgerechnet:**

| Pfad | R (35 µm) | R (17,5 µm) | Verlust bei 12,5 A (17,5 µm) | engste Stelle (äquiv. Breite) |
| --- | ---: | ---: | ---: | ---: |
| J10.4 → J12.4 | 1,2 mΩ | 2,5 mΩ | ~0,38 W | ~5 mm |
| J10.4 → J4.4 | 1,1 mΩ | 2,2 mΩ | ~0,34 W | ~6 mm |
| J12.4 → J4.4 | 1,3 mΩ | 2,6 mΩ | ~0,40 W | ~5,5 mm |
| +5 V: J10.1 → J12.1 | 3,0 mΩ | 6,1 mΩ | bei ~1 A: ~6 mW | ~2 mm |

Die alte Vorgabe hier („In2 bräuchte ~25 mm Breite") kam aus der IPC-2221-Leiterbahnformel. Die gilt für lange, isolierte Bahnen und ist für eine ganze Lage viel zu konservativ: Die Verluste verteilen sich über ~2500 mm². Die Stromdichte-Spitzen (~2,5 A/mm bei 12,5 A) liegen nur lokal an der unteren Ecke des 5-V-Stegs zwischen J10 und J12 (≈ x 150 / y 82) und an den Pad-Kanten. **48 V ausschließlich auf In2 ist damit in Ordnung, auch bei 0,5 oz.** Optionale Verbesserung: Den 5-V-Steg zwischen J10.1 und J12.1 als ~2-mm-Bahn auf F.Cu oder B.Cu führen. Dann läuft der 48-V-Pfad J10→J12 ohne Umweg.

**Umgesetzt:**

- GND: In1 vollflächig plus B.Cu-Pour. Ein gemeinsamer GND statt Sternpunkt ist bei dieser Größe mit durchgehender Innenlage die bessere Lösung.
- Bremsen-Schaltknoten klein (Q2 / D1 / J3 / F1 eng beieinander). Brems-PWM liegt rechts, CAN links: Das Trennkonzept ist im Layout eingehalten.
- THT-Pads von J10/J12/J4 massiv angebunden (Custom-Rule `Power-Klemmen massiv an Flaechen` in `.kicad_dru`), kein Lagenwechsel im 48-V- und GND-Pfad.
- Bremskanal 1,5 mm auf F.Cu (≥ 2 A).
- D5 (SMAJ58A, DNP) zwischen D1 und J3 ergänzt, F1 dafür nach oben verschoben. `BRAKE +48V` läuft als 1,5-mm-Bahn rechts an D5 vorbei zu J3.1.
- D3 direkt an J12.1, wo die 5 V ankommen.
- 48-V-Clearance 0,5 mm (Netclass `HV_Bus_48V` und Zonen), DRC-Minima eingetragen (Clearance und Track 0,15 mm). Nachgeprüft: Alle 48-V-führenden Netze haben außen ≥ 0,5 mm, auch die Bremsnetze. Die Netzklassen `CAN`/`HV_Brake` haben kein Pattern mehr, das Layout hält die Abstände aber ohnehin ein. Einordnung nach IPC-2221B für 31–50 V: innen (B1) 0,1 mm, außen unbeschichtet (B2) 0,6 mm, außen mit Polymerbeschichtung (B4) 0,13 mm. 0,5 mm mit Lötstopplack ist ok.
- PCB synchron zum Schaltplan (DRC-Parität sauber), In2-Zonen vorhanden, In1-Zone deckt die ganze Platine ab, keine entarteten Punkte mehr auf Edge.Cuts.
- Platinenumriss: Kreis Ø 70 mm (Mitte 150/100) mit Abflachungen → 66,05 × 64 mm, 4× M2.
- C2/C3/C14 als `Device:C_Polarized` (Polaritätsmarkierung im Bestückungsdruck, auf dem Board gibt es keinen Verpolschutz). 3D-Modell: KiCad hat Ø 10 × 20 mm nur mit RM 7,5, daher Footprint `CP_Radial_D10.0mm_P5.00mm` mit **Scale Z = 2,0** (per Render geprüft).

---

## Fertigung

**Gewählt: JLCPCB.** Für ein 4-Lagen-Board unter 100 × 100 mm ist das der günstigste Weg. Bis 150 € Warenwert zieht JLCPCB die Einfuhrumsatzsteuer schon beim Bestellen ein (IOSS), bei DHL fallen also keine Zollgebühren an. Alternativen: **AISLER** (Aachen, Fertigung in Europa, kein Import, dafür teurer) und **PCBWay** (ähnlich wie JLCPCB).

| Option | Wert |
| --- | --- |
| Lagen / Dicke | 4 / 1,6 mm, FR-4 |
| Menge | **10** (das Minimum von 5 reicht nicht für 6 Boards, 10 kosten kaum mehr) |
| Kupfer außen / innen | 1 oz / **0,5 oz** (Standard; real ~15 µm → J10→J12 ≈ 2,9 mΩ, laut Rechnung im Layout-Stand ausreichend) |
| Oberfläche | HASL bleifrei (gut zum Handlöten) oder ENIG |
| Stackup | Standard-Stackup, keine Impedanzkontrolle nötig. Lagenreihenfolge F.Cu / In1 (GND) / In2 (48 V) / B.Cu bestätigen |
| Design-Regeln | Vias 0,6/0,3 mm, Leiterbahn/Abstand ≥ 0,15 mm: im Standardbereich |

Beim Hochladen fragt JLCPCB, wo die Auftragsnummer auf den Bestückungsdruck soll. Wer sie nicht will, wählt die Option zum Positionieren oder Entfernen.

**Export aus KiCad:**

1. Zonen neu füllen (`B`), DRC laufen lassen.
2. *Datei → Fertigungsunterlagen → Gerber:* F.Cu, In1.Cu, In2.Cu, B.Cu, F.Mask, B.Mask, F.Silkscreen, B.Silkscreen, Edge.Cuts. Danach *Bohrdateien erzeugen* (Excellon, mm).
3. Alles in ein ZIP packen, hochladen und im Gerber-Viewer des Fertigers alle vier Kupferlagen, die Bohrungen und den Umriss prüfen.

---

## Offene Punkte

Stand 2026-09-13: ERC 0 Fehler / 4 Warnungen, DRC 0 offene Pads (nur Silk-/Courtyard-Meldungen). Notiztexte, Values (D4), Notes (C2/C3/C4/C14/J4) und die DNP-Attribute von R10 und D5 sind aktuell. Erledigte Punkte sind gelöscht, ihre Begründungen stehen in den Abschnitten oben.

**Schaltplan / Layout**

- [ ] **ERC-Warnung Bibliothekspfad:** `sym-lib-table` zeigt für `Seeed_Studio_XIAO_Series` noch auf den alten OneDrive-Pfad. Harmlos, weil das Symbol im Schaltplan eingebettet ist. Sauber wäre, die `.kicad_sym` ins Repo zu legen (neben `R0192.pretty`) und per `${KIPRJMOD}` einzubinden.
- [ ] **3 ERC-Warnungen `endpoint_off_grid` an D5:** Pin 2 und die GND-Leitung (x 55,2–62,9 mm / y 147,3 mm) liegen neben dem 1,27-mm-Raster. D5 und das GND-Symbol aufs Raster schieben. Dabei im Brake-Notiztext `D5 -> TVS, Q2-Schutz falls D1 fehlt (DNP)` ergänzen.

**Auslegung / Entscheidungen**

- [ ] **Not-Aus-Verzögerung der Bremsen:** Nach dem Trennen der 48 V halten die Buskondensatoren die Bremsen noch einige hundert Millisekunden offen (s. „Sicherheit" im Bremsen-Abschnitt). Entscheiden: Firmware setzt beim Not-Aus sofort PWM = 0, und/oder das Schütz trennt auch die 5 V.
- [ ] **48 V nie unter Spannung stecken** (~1,8 mF Buskapazität, Überschwingen durch Leitungsinduktivität). Zentral am Netzteil bzw. über das Schütz schalten; eine Inrush-Begrenzung gehört, falls überhaupt, dorthin und nicht auf jedes Board.
- [ ] Haltemoment unter Payload an **Achse 2/3** verifizieren (~6,4 N·m Bremse vs. 7,5 N·m Motor-Nennmoment), kalt und warm.
- [ ] Im Bestückungsplan festhalten: **R10 bleibt DNP** (bei Bedarf von Hand am Steuerungsende nachlöten, nie auf mehreren Boards), Split-Termination nur an den zwei Busenden.

**Bestellung**

- [ ] Mouser: D2 PESD12VL2BT,215 (771-PESD12VL2BT215), 6 Stück + Reserve, und D5 SMAJ58A (576-SMAJ58A), einige als Reserve.
- [ ] 3+ UVR2A101MPD nachbestellen (für 6 Boards sind 18 nötig, 15 bestellt).
- [ ] PCB bei JLCPCB bestellen (s. „Fertigung"): Zonen neu füllen, DRC, Gerber und Bohrdaten exportieren, im Gerber-Viewer des Fertigers prüfen.

**Bring-up**

- [ ] **Vor dem ersten Einschalten jedes Kabel durchmessen:** An J10/J12 darf +48 V nur auf Pin 4 liegen. Es gibt kein Serienelement vor D3 (s. Spannungsdomänen).
- [ ] Kurzschlusstest 48 V / 5 V / 3,3 V gegen GND, 3,3-V-Schiene messen
- [ ] ESP32 flashen, CAN-Loopback, dann `/homing`-Protokoll end-to-end gegen den Pi
- [ ] TLE4905L mit Magnet: sauberer 3,3-V-Pegelwechsel an D1
- [ ] Bremse: Anzug (48 V, 150 ms) → öffnet; Halten bei 10,4 % Duty; **Einfallzeit aus dem Haltezustand messen** (Ziel < 20 ms, entspricht L ≤ ~0,78 H, s. „Einfallzeit mit D1")
- [ ] CAN mit allen 6 Boards bei 1 Mbit/s: Terminierung nur an den Enden, Eye/Fehlerzähler unter laufenden Motoren prüfen (das ist der Test, der die GND-Referenz wirklich beantwortet)
