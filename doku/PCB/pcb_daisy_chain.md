# R0192 "Daisy Chain" Driverlink-PCB — Konzept & Pre-Fab-Doku

## Kontext & Ziel

Das **Daisy-Chain-Board** ist die Bus-Verteilplatine, die pro Roboterachse einmal verbaut wird. Sie schleift die zentralen Versorgungen vom Rack-Tether (48 V, 5 V) sowie den CAN-Bus von Node zu Node durch und zweigt an jeder Achse die Motorversorgung ab. Zusätzlich wertet sie lokal den Homing-Hall-Sensor (TLE4905L) aus, **steuert die Aktuator-Haltebremse** (Achsen 1–3) und bietet Erweiterungsanschlüsse für spätere Sensorik/LEDs.

Damit ist das Board die Hardware-Entsprechung zum Software-Homing-Node (`microcontroller/r0192_homing.ino`): Der bisherige Arduino-Uno-Prototyp mit separatem MCP2515 wird durch den **XIAO-ESP32-S3** mit integriertem CAN-Controller (TWAI) ersetzt.

**Designprinzip:** Identische Platine an jeder Achsposition einsetzbar — alle positionsabhängigen Eigenschaften (v. a. Bus-Termination) sind per Jumper konfigurierbar. Positionsabhängige *Verbraucher* (Bremse) werden per Bestückungsvariante (DNP) unterschieden.

---

## Funktionsblöcke

| Block | Funktion |
| --- | --- |
| **48 V-Bus-Durchschleife** | Eingang → Ausgang (Daisy Chain) + Abzweig zum Motor + Speisung der Bremsen-Endstufe |
| **5 V-Bus-Durchschleife** | Eingang → Ausgang (Daisy Chain), versorgt Logik (optional 5-V-DC-Halten der Bremse) |
| **CAN-Bus-Durchschleife** | CAN_H / CAN_L von Node zu Node, Termination jumperbar |
| **3,3 V-Erzeugung** | AP2112K-3.3 LDO aus 5 V, versorgt XIAO (3V3-Pin), Transceiver & Logik |
| **MCU** | XIAO-ESP32-S3 (gesockelt, tauschbar), CAN via TWAI, versorgt über 3V3-Pin |
| **CAN-Transceiver** | SN65HVD230, nativer 3,3-V-CAN-Transceiver, Rs auf GND (High-Speed) |
| **Homing-Sensor** | TLE4905L Hall (Open-Collector), Pegelwandlung via Pull-up |
| **Bremsen-Endstufe** (Verbraucher, nur Achsen 1–3) | Low-Side-N-MOSFET direkt an **48 V**, PWM-Economizer (Anzug-Puls + sparsames Halten), Freilauf + TVS-Klemmung, per ESP32-PWM-GPIO |
| **Lüfter-Provision** (optional/DNP, alle Boards) | 2-Pin-Header + Low-Side-MOSFET + Freilaufdiode, per ESP32-GPIO (an/aus/PWM); **Versorgungsschiene offen** seit Wegfall des 24-V-Bucks (s. u.) |
| **Erweiterung** | Freie GPIOs (D0/D1/D3) + 3,3 V + 5 V auf Stiftleiste |

---

## Schlüssel-Designentscheidungen

### MCP2515 eliminiert
Der ESP32-S3 hat einen integrierten CAN-Controller (TWAI). Dadurch entfallen MCP2515, Quarz, 2× 22 pF und der SPI-Pull-up gegenüber dem Arduino-Prototyp. Der SN65HVD230 bleibt als physischer Transceiver zwingend erforderlich. Die TX/RX-Pins des ESP32 (D6_TX / D7_RX) werden direkt mit dem SN65HVD230 (D / R) verbunden.

> **Firmware-Konsequenz:** Umstieg von `autowp/mcp2515` auf `ESP32-TWAI-CAN` bzw. natives `driver/twai.h`. Das CAN-Protokoll (achsenspezifische CAN-ID, `CMD_ARM` / `RSP_DETECTED` / `RSP_ERROR`) bleibt unverändert. TX/RX-Pin-Zuordnung muss zwischen PCB und Firmware konsistent dokumentiert sein.

### Echter 3,3-V-Transceiver: SN65HVD230 statt TJA1051
Der **SN65HVD230** ist ein **nativer 3,3-V-CAN-Transceiver** (V_CC = 3,3 V, Logikpegel referenzieren direkt 3,3 V, spezifiziert bis 1 Mbit/s). Damit liegt der **gesamte** Transceiver auf 3,3 V — passend zum XIAO-ESP32-S3, der mit 3,3-V-Ausgangssignalen arbeitet. Kein 5-V-Pegel, keine Pegelwandlung nötig.

> **Korrektur gegenüber früherem Stand:** Geplant war ein **TJA1051T** „3,3-V-seitig versorgt". Der klassische TJA1051(T) ist aber ein **5-V-Transceiver** (V_CC 4,5–5,5 V) und referenziert seine Logikpegel an V_CC — bei 3,3 V V_CC erzeugt er keine spec-konformen CAN-Pegel, und die Plain-Variante hat keinen separaten Logik-Pin. Korrekt wären entweder TJA1051T**/3** (V_CC = 5 V, VIO = 3,3 V) oder eben der **SN65HVD230** (komplett 3,3 V). Gewählt: **SN65HVD230** — single-rail 3,3 V, keine 5-V-Referenz am Transceiver.

**Pin-Belegung SN65HVD230:**

- **D** (Pin 1, TXD) ← ESP32 D6_TX, **R** (Pin 4, RXD) → ESP32 D7_RX
- **V_CC** (Pin 3) = 3,3 V, **GND** (Pin 2)
- **Rs** (Pin 8) **auf GND** → High-Speed-Mode (nötig für 1 Mbit/s; nicht floaten lassen)
- **Vref** (Pin 5, V_CC/2-Ausgang) **unbenutzt → NC** (NC-Flag im Schaltplan)
- **CANH** (Pin 7) / **CANL** (Pin 6) → Bus

Die 3,3 V werden lokal per AP2112K-3.3 aus der durchgeschleiften 5 V-Schiene erzeugt; es ist **kein** 48 V→3,3 V-Wandler nötig, da 5 V ohnehin über den Tether mitgeführt wird (Quelle: MeanWell LRS-50).

### TLE4905L — Open-Collector, Versorgung 5 V, Pull-up auf 3,3 V
Der TLE4905L benötigt mindestens 3,8 V Versorgung und wird daher aus 5 V betrieben. Sein Open-Collector-Ausgang wird über einen Pull-up (R1, 10 kΩ) auf **3,3 V** gezogen — der High-Pegel wird vom Pull-up bestimmt, nicht von der Sensorversorgung. Das ergibt einen für den ESP32-GPIO sicheren 3,3 V-Pegel und vermeidet, den GPIO mit 5 V zu beschädigen. Entkopplung: 100 nF (C1) direkt am Sensor.

> Pull-up bleibt bei 10 kΩ, da die Sensorleitung kurz ist (≤ 10 cm). Bei längeren Leitungen wäre 4,7 kΩ störungsärmer.

### Bus-Termination jumperbar
CAN-Termination (120 Ω) darf **nur an den beiden physischen Busenden** sitzen, nicht auf jedem Node (Impedanzproblem — Befund aus früherem Review). Realisierung: 120 Ω fest bestückt, über Jumper (JP1) parallel zu CAN_H/CAN_L zuschaltbar. So ist dieselbe Platine an jeder Position einsetzbar; Jumper nur an den Endknoten gesteckt.

### ESP32 gesockelt
Der XIAO-ESP32-S3 wird über Buchsenleisten (J12–J15) aufgesteckt, nicht verlötet → einfacher Austausch bei Defekt. Footprints müssen exakt zum XIAO-Rastermaß passen.

### XIAO-Versorgung über den 3V3-Pin (single-rail 3,3 V)
Der XIAO wird **direkt über den 3V3-Pin (Pin 13) aus der AP2112K-3,3-V-Schiene** versorgt; der **5V/VBUS-Pin (Pin 14) bleibt unverbunden** (NC-Flag). Der 3V3-Pin des XIAO ist bidirektional und von Seeed als geregelter 3,3-V-**Eingang** freigegeben — der Onboard-LDO wird damit umgangen. So läuft das ganze Board single-rail auf 3,3 V (Logik) + 5 V (TLE4905L, AP2112K-Eingang, optional Bremsen-Halten).

> **Caveats:**
> - **USB-Flashen:** Liegt beim Flashen 5 V auf VBUS (USB-C), speist der Onboard-LDO parallel zur AP2112K auf den 3V3-Knoten. Meist tolerierbar; sauberer ist, die Board-3,3 V beim Flashen nicht gleichzeitig anzulegen oder den XIAO zum Flashen abzustecken.
> - **AP2112K-Budget (600 mA):** Für den reinen CAN-Homing-Node (kein WLAN, ~40–80 mA + ~15 mA Transceiver) reichlich. WLAN-Sendespitzen (~350–500 mA) sind nicht eingeplant — Erweiterungssensorik daher stromsparend halten.

### Bremsen-Ansteuerung auf dem Board (PWM-Economizer aus 48 V, per-Achse softwaregesteuert)

> **Korrektur gegenüber früherem Stand (zwei Iterationen):**
> 1. Ursprünglich war angenommen, die Failsafe-Bremsen würden vom Treiber (GDS68/RS05) versorgt/angesteuert. Das ist **falsch**: Die SteadyWin-Bremsen wurden mit nach außen geführten, unangeschlossenen Kabeln geliefert. Der GDS68-„brake interface" (Abschnitt 2.4.7) ist **nicht per CAN/Software** steuerbar (hängt nur am Treiber-Power-Zustand) und spannungs-/stromseitig undokumentiert. Die Bremse wird daher **auf dem Board vom ESP32** gesteuert.
> 2. Zwischenzeitlich war ein **48 V→24 V-Buck** vorgesehen, um die 24-V-Nennbremse zu versorgen. Auch das **entfällt**: Messungen zeigen, dass die Bremse mit einem **PWM-Economizer direkt aus 48 V** (Anzug-Puls) und einem sparsamen 5-V-Mittelwert-Halten betrieben werden kann — **kein 24-V-Buck, kein Spannungsteiler** nötig.

**Bremse — Kenndaten (SteadyWin STW-S035):** Identifiziert über Spulendurchmesser 48 mm; **dieselbe Bremse in GIM6010 und GIM8108** verbaut. Aufbau: **stromlos-geschlossen** (fail-safe, federbetätigt), Lösen durch Bestromen der Spule.

| Parameter | Wert (Datenblatt) |
| --- | --- |
| Nennspannung | 24 V DC |
| Nennstrom | 0,64 A |
| Nennleistung | 15,36 W |
| Spulenwiderstand | 34,03 Ω (kalt) |
| Haltemoment (motorseitig) | 0,80 N·m |
| Haltemoment am Gelenk (× 8:1) | ~6,4 N·m |
| Isolierklasse | F (155 °C) |
| Öffnungszeit (Anker zieht an) | ~35 ms |
| Einfallzeit (Anker fällt ab) | ~20 ms |

> ⚠️ Das Haltemoment am Gelenk (~6,4 N·m) liegt **unter** dem Motor-Nennmoment (7,5 N·m). An Schulter/Ellbogen (J2/J3) muss das statische Halten unter voller Gravitationslast + 1 kg Payload — kalt **und** warm — noch verifiziert werden.

**Eigene Messwerte (eine Bremse, bei verifizierten 24 V):**

| Übergang | Schwelle | Strom |
| --- | --- | --- |
| **Anzug / Öffnen** (aus geschlossen) | ~10,5 V | ~266 mA |
| **Abfall / Schließen** (aus offen) | ~2,5 V | ~60 mA |
| Sicher offen (Referenzpunkt) | 3,5 V | ~78 mA |
| Dauerbetrieb 24 V (heiß) | 24 V | ~450 mA |

- **Hysterese ~4:1** — Anzug braucht ~266 mA, Halten trägt bis herunter auf ~60 mA. Das ist die physikalische Grundlage des Economizers.
- **Abfall ist strombasiert (~60 mA)**, nicht spannungsbasiert; Temperatur verschiebt nur die zugehörige Spannung leicht.
- **Warmtest:** 15 min bei 24 V → extern ~65 °C. Schwellen praktisch unverändert (Anzug weiter ~10,5 V) → Economizer empirisch validiert.

**Ansteuerkonzept (Economizer, zweiphasig):**

| Phase | Vorgabe | Strom (ca.) | Leistung |
| --- | --- | --- | --- |
| **Anzug** | 48 V für ~100–200 ms (PWM ~100 %) | ~1,4 A (kalt) | kurzzeitig, unkritisch |
| **Halten** | 5 V Mittelwert (PWM ~10,4 % = 5 V / 48 V) | ~130–150 mA | ~0,7 W |

- **Kein 24 V nötig, kein Spannungsteiler.** Die Anzugsschwelle liegt bei 10,5 V — direkt aus 48 V anziehen zieht härter und schneller an. Ein 48-V-Puls über 100–200 ms ist thermisch (wenige Joule) und isolationsseitig (Klasse F) unkritisch.
- **5 V ist exakt die validierte Haltespannung** (~2× Marge über der 60-mA-Abfallschwelle, auch heiß). Konstantspannung reicht, **keine Stromregelung** nötig.
- Ersparnis: **~0,7 W statt ~15 W** pro Bremse.

**Schaltungstopologie (Primärvariante: ein Rail, 48 V, Low-Side-FET, PWM):**

```
   48V ──┬──────────────┐
         │            [Bremsspule ~34Ω]
        (D_fw)            │
    Freilauf ◄────────────┤  ← Schaltknoten (+ TVS-Klemmung ~60–75 V für schnelles Einfallen)
         │                │
   48V ──┘            [ N-MOSFET ] ← Gate über R_g vom MCU-PWM
                          │         + R_pd (Gate→GND, ~10k)
   GND ───────────────────┘
```

- **Anzug:** PWM ~100 % Duty für ~150 ms.
- **Halten:** Duty auf ~**10,4 %** (= 5 V / 48 V Mittelwert), einige kHz PWM-Frequenz — hoch genug für glatten Strom, über der Hörschwelle.
- **Einfallen (Bremse zu):** PWM komplett aus.

**Bauteil-Startwerte (pro Bremse):**

| Bauteil | Anforderung / Startwert |
| --- | --- |
| N-MOSFET (Low-Side) | Logic-Level (voll durch bei V_GS = 3,3 V), V_DS ≥ 60 V, I_D ≥ 3 A mit Reserve. Falls 3,3 V-Ansteuerung grenzwertig: kleiner Gate-Treiber oder Level-Shift auf 5 V. |
| Freilaufdiode D_fw | Schottky/Ultrafast, V_R ≥ 60 V (Bus + Ringing), I ≥ 2 A, nahe an der Spule |
| Gate-Widerstand R_g | ~22–100 Ω (Flanken zähmen, EMV) |
| Gate-Pulldown R_pd | ~10 kΩ Gate→GND — **fail-safe:** FET AUS (Bremse zu), wenn der MCU-GPIO floatet / bootet / stromlos ist |
| TVS/Zener-Klemmung | ~60–75 V über dem Schaltknoten für **garantiert schnelles Einfallen** (s. Sicherheit); **nicht** dieselbe Diode wie die PWM-Rezirkulation |
| Snubber (optional) | RC über den FET, falls Ringing am Schaltknoten stört |

**Alternative: DC-Halten aus der 5-V-Schiene.** Da 5 V die Haltespannung ist, kann das Halten auch als reiner DC-Anschluss an die 5-V-Schiene über einen zweiten Low-Side-Pfad erfolgen (kein Ripple, ruhiger, weniger EMV in der langen Haltephase). Nur der 48-V-Anzug wird gepulst. Kostet einen zweiten Schalter + Entkopplung (Break-before-make oder Steuerdioden, damit 48 V nie auf die 5-V-Schiene durchschlägt). Für den ersten Wurf ist Ein-Rail-PWM einfacher.

**Failsafe-Eigenschaft:** Default = MOSFET aus = Bremse eingefallen (stromlos = zu). Gate-Pulldown erzwingt das beim Booten/Reset. Der Homing-Node auf dem Board bekommt damit Zusatzaufgaben: Bremse (und optional Lüfter) dieser Achse. Die Protokoll-Erweiterung Pi↔Node für „Bremse lüften/einfallen" ist Folgearbeit.

### Sicherheit (Bremsen-Endstufe, kritisch)
1. **Zwangsabschaltung über Hardware.** Das Lösen darf **nicht** allein an einem MCU-GPIO hängen — ein Firmware-Hänger würde die Bremse offen lassen. Brake-Enable in die bestehende **zweikanalige Sicherheitskette / Not-Aus** legen, sodass Stromausfall oder Not-Aus die Endstufe **unabhängig von der Software** totlegt → Federn schließen.
2. **Schnelles Einfallen.** Eine reine Freilaufdiode lässt den Strom mit τ = L/R (einige ms) abklingen → verzögertes Einfallen (grenzwertig gegenüber den 20 ms mechanischer Einfallzeit). Für **garantiert** schnelles Einfallen beim Abschalten eine **Zener-/TVS-Klemmung** (~60–75 V) — nicht dieselbe Diode wie die PWM-Rezirkulation. Cleanste Lösung: **Zwei-Schalter-Endstufe** (High- + Low-Side), PWM-Rezirkulation über die Bodydiode, beim Shutdown beide öffnen → Strom in die TVS.
3. **Gate-Pulldown** für definierten AUS-Zustand beim Booten (s. o.).

### Lüfter-Provision (optional, DNP, auf jedem Board)
Reserve für einen kleinen Achslüfter, falls passive Kühlung nicht reicht (v. a. bei nicht-Alu-Gehäuseteilen): **2-Pin-Lüfter-Header + Low-Side-MOSFET (ESP32-GPIO, an/aus oder PWM) + Freilaufdiode**. Standardmäßig **nicht bestückt** — passive-first bleibt Default. Footprints trotzdem im ersten Layout mitziehen, damit später kein Redesign nötig ist.

> **Offen seit Wegfall des 24-V-Bucks:** Der Lüfter hing zuvor am board-universellen 24-V-Buck. Der ist mit dem Economizer entfallen. Ein bestückter Lüfter braucht daher eine eigene Versorgung — Optionen: 48-V-PWM (nur mit 48-V-fähigem Lüfter, selten), 5-V-Schiene (kleiner 5-V-Lüfter, LRS-50-Budget beachten) oder ein kleiner lokaler Buck nur bei Bestückung. Entscheidung offen; da DNP nicht blockierend fürs erste Layout.

---

## Spannungsdomänen

| Domäne | Quelle | Versorgt | Stecker |
| --- | --- | --- | --- |
| **48 V** | Tether-Bus (durchgeschleift) | Motorabzweig, **Bremsen-Endstufe (PWM)** | XT60PW (Bus), XT30PB(2+2)-M (Motor) |
| **5 V** | Tether-Bus (durchgeschleift) | TLE4905L, AP2112K-Eingang (**nicht** XIAO), optional Bremsen-DC-Halten | XT30PW |
| **3,3 V** | AP2112K-3.3 (aus 5 V) | XIAO (3V3-Pin), SN65HVD230, Pull-up, Erweiterung | — (on-board) |
| **GND** | gemeinsam (ein Netz) | alle | über alle Power-Stecker |

> **Wegfall der 24-V-Domäne:** Der zuvor geplante 48 V→24 V-Buck ist mit dem PWM-Economizer entfallen — die Bremse ist jetzt ein **48-V-Verbraucher** (gepulst) mit 5-V-Mittelwert im Halten. Das Board läuft damit auf drei Schienen (48 V / 5 V / 3,3 V). Die Bremsspule liegt zwischen 48 V und dem Schaltknoten des Low-Side-FET; im Halten fließt nur ~130–150 mA (PWM-Mittelwert).

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
| Bremse (nur J1–J3) | JST/Schraubklemme (2-polig) | Bremsspule; führt Schaltknoten (48-V-Puls) + GND |
| Erweiterung | Stiftleiste (J10, 6-polig) | D0/D1/D3 + 3,3 V + 5 V + GND |

> **CAN-GND:** Die XH-2A-CAN-Stecker führen nur CAN_H/CAN_L. Eine gemeinsame GND-Referenz zwischen allen Nodes ist für CAN zwingend — sie wird über die durchgehende GND der Power-Schienen (48 V/5 V) sichergestellt. Verifizieren, dass GND wirklich durchgängig über alle Boards verbunden ist.

---

## Masseführung (GND-Konzept)

**Ein einziges GND-Netz** (`GND`) für das gesamte Board — eine gemeinsame Referenz ist Pflicht (sonst kein CAN). Entscheidend ist die *Layout-Führung*, nicht eine Netztrennung:

- Elektrisch ein Netz, layout-seitig gedankliche Trennung in **Power-GND** (48 V-Rückpfad: XT60-Bus, XT30-Motor, **Bremsen-FET-Source**, 48 V-Kondensatorbank) und **Signal-GND** (ESP32, SN65HVD230, TLE4905L, AP2112K + deren Cs).
- Beide treffen sich an **einem Sternpunkt**, idealerweise am 5 V/GND-Eingang bzw. am 48 V-Bulk-Elko.
- Durchgehende **GND-Massefläche** (Pour), bei 2-Layer typischerweise Bottom.
- Motor-/Bus-GND mit breitem Kupfer **direkt** zwischen XT-Steckern und Kondensatorbank — nicht quer durch die Logik-Zone.
- **Bremsen-Schaltknoten-Loop klein halten** (FET, Spule, Freilaufdiode, TVS eng beieinander) und den gepulsten 48-V-Schaltknoten **weg von den CAN-Leitungen** führen — sonst koppelt das PWM-Schalten in den Bus.
- Logik räumlich von den 48 V-Leistungsbahnen entfernt platzieren, sodass kein Motor-/Bremsstrom geometrisch durch die Logik-Masse fließt.

---

## Stützkondensatoren & Schutz

| Bauteil | Wert | Position | Zweck |
| --- | --- | --- | --- |
| C2, C3 | 100 µF / **≥ 63 V** | 48 V-Bus | Bulk-Puffer, Regen-Spikes, Bremsen-Anzug-Peak |
| C4 | 100 nF | 48 V-Bus | HF-Entkopplung |
| C8 | 10 µF | AP2112K VIN (5 V) | Bulk gegen Lasteinbrüche/Spikes |
| C5 | 1 µF | AP2112K VIN | HF-Stabilität LDO-Eingang |
| C6 | 1 µF | AP2112K VOUT | LDO-Stabilität Ausgang |
| C7 | 100 nF | SN65HVD230 VCC (3,3 V) | IC-Entkopplung |
| C1 | 100 nF | TLE4905L VCC | Sensor-Entkopplung |

> **AP2112K-Hinweis:** Absolute-Max VIN = 6,0 V. Bei 5 V-Nominal ist das knapp; C8 (10 µF) dämpft Überschwinger. Optional als zusätzlicher Schutz eine TVS-Diode (z. B. SMAJ5.0A) zwischen 5 V und GND am Eingang.
>
> **Bremsen-Anzug-Peak:** Der ~1,4-A-Anzugsstrom (pro Bremse, ~150 ms) wird aus dem 48-V-Bulk (C2/C3) und dem Netzteil gedeckt. Bei gleichzeitigem Anziehen mehrerer Bremsen ggf. zeitlich staffeln (s. Strom-Dimensionierung).

---

## Strom-Dimensionierung (Kupfer)

- **48 V-Bus-Durchschleife (XT60):** auf das Netzteil-Limit auslegen — LRS-600N2-48 liefert max. ~12,5 A. Bus-Bahnen auf ~13 A dimensionieren.
- **48 V-Motorabzweig (XT30):** auf den **Spitzenstrom eines einzelnen Motors** (GIM8108) auslegen, nicht auf die Summe.
- **Theoretische Summe** aller Motor-Spitzenströme = 83,6 A — wird vom Netzteil ohnehin nie geliefert und tritt real nicht gleichzeitig auf; für die Bus-Dimensionierung **nicht** maßgeblich.
- **5 V-Schiene:** LRS-50 liefert max. ~10 A bei 5 V → Obergrenze. XT30PW gewählt, da JST-XH stromseitig zu knapp.
- **Bremsen-Endstufe (48 V, nur Achsen 1–3):** Anzugsstrom pro Bremse ≈ 48 V / 34 Ω ≈ **1,4 A** (kalt), ~150 ms. Bei gleichzeitigem Anziehen aller drei (Enable/Homing) ~4,2 A Transient auf 48 V — für die LRS-600N2-48 unkritisch; bei Bedarf zeitlich staffeln. **Haltestrom gesamt (3 Bremsen) ~0,45 A** (PWM-Mittelwert), vernachlässigbar. FET/Diode/Bahnen auf ≥ 3 A pro Kanal mit Reserve auslegen.

---

## Status & ERC

ERC: **0 Errors / 0 Warnings** (Stand letzter Review — vor Integration der Bremsen-Endstufe; nach dem Bremsen-Redesign erneut prüfen).

Behobene ERC-Punkte:
- PWR_FLAG auf +48 V, +5 V (Eingänge) und +3,3 V (AP2112K-Ausgang) gesetzt.
- NC-Flags auf ungenutzte ESP32-Sockel-Pins **inkl. VBUS/5V-Pin (Pin 14)** des XIAO.
- NC-Flag auf SN65HVD230 Vref (Pin 5).
- AP2112K VIN + EN sauber an +5 V verbunden.

Manuell verifiziert (ERC-blind):
- TX/RX-Zuordnung ESP32 (D6_TX/D7_RX) ↔ SN65HVD230 (D/R) korrekt.
- SN65HVD230 Rs (Pin 8) auf GND (High-Speed-Mode für 1 Mbit/s).
- XIAO über 3V3-Pin (Pin 13) versorgt, VBUS (Pin 14) unverbunden.

---

## Pre-Fab-Checkliste

### Schaltplan
- [x] MCP2515 entfernt, ESP32-TWAI genutzt
- [x] CAN-Transceiver SN65HVD230 (nativ 3,3 V) statt TJA1051; Rs (Pin 8) auf GND, Vref (Pin 5) NC
- [x] TX/RX korrekt zugeordnet (ESP32 D6_TX/D7_RX ↔ SN65HVD230 D/R)
- [x] XIAO über 3V3-Pin (Pin 13) versorgt, VBUS (Pin 14) unverbunden/NC
- [x] TLE4905L: Vcc 5 V, Pull-up (10 k) auf 3,3 V, 100 nF Entkopplung
- [x] Termination jumperbar (JP1, 120 Ω)
- [x] AP2112K mit C5/C6/C8, VIN+EN an 5 V
- [x] Decoupling: C7 (SN65HVD230), C1 (Sensor), 48 V-Bank (C2/C3/C4)
- [x] PWR_FLAGs gesetzt, ERC sauber (vor Bremsen-Redesign)
- [ ] **24 V-Buck entfernt** (durch PWM-Economizer ersetzt) — sicherstellen, dass keine 24-V-Netze/-Bauteile mehr im Schaltplan hängen
- [ ] C2/C3 (100 µF) ≥ 63 V (besser 100 V) und C4 (100 nF) ≥ 63 V Spannungsfestigkeit am 48-V-Bus prüfen
- [ ] Optional: TVS (SMAJ5.0A) am 5 V-Eingang
- [ ] Bremsen-Endstufe (nur Achsen 1–3): Low-Side-Logic-Level-N-MOSFET (V_DS ≥ 60 V, I_D ≥ 3 A) an 48 V, Freilaufdiode (V_R ≥ 60 V) über die Spule, **TVS-/Zener-Klemmung (~60–75 V)** für schnelles Einfallen, Gate-R_g (~22–100 Ω), Gate-Pulldown (~10 k, Default = Bremse eingefallen), Bremsstecker; **ESP32-PWM-fähigen GPIO** zuweisen
- [ ] Bremsen-Sicherheit: Brake-Enable in die zweikanalige Not-Aus-Kette legen (Hardware-Zwangsabschaltung unabhängig vom MCU-GPIO)
- [ ] Entscheidung **Ein-Rail-PWM vs. 5 V-DC-Halten** final treffen (bei 5-V-Halten zweiter Schalter + Entkopplung/Break-before-make)
- [ ] Lüfter-Provision (optional/DNP): 2-Pin-Header + Low-Side-MOSFET + Freilaufdiode, Gate-Pulldown; **Versorgungsschiene entscheiden** (48 V-PWM / 5 V / lokaler Buck) — s. offene Punkte
- [ ] Bei SteadyWin die Bremsen-Nennwerte final bestätigen (STW-S035, 24 V, Haltemoment vs. Schwerkraftlast je Achse)

### Layout (nächster Schritt)
- [ ] Footprints allen Bauteilen zugewiesen
- [ ] XT60PW / XT30PW / XT30PB(2+2)-M / XH-2A Footprints beschafft/importiert (teils nicht in Standard-Lib)
- [ ] ESP32-Sockel-Footprint (J12–J15) mechanisch gegen XIAO geprüft
- [ ] GND-Pour mit Sternpunkt-Führung (Power-GND vs. Signal-GND)
- [ ] 48 V-Bus-Kupfer auf ~13 A, Motorabzweig auf Einzelmotor-Peak, Bremsen-Kanal ≥ 3 A
- [ ] **Bremsen-Schaltknoten-Loop klein** (FET/Spule/D_fw/TVS eng), 48-V-Schaltknoten **weg von CAN-Leitungen**
- [ ] Logik räumlich von 48 V-Leistungsbahnen getrennt platziert
- [ ] Entkopplung-Cs nah an den jeweiligen IC-Pins platziert
- [ ] DRC sauber

### Bring-up (nach Fertigung)
- [ ] Sichtprüfung Lötung, Kurzschlusstest 48 V/5 V/3,3 V gegen GND
- [ ] 3,3 V-Rail messen (AP2112K-Ausgang)
- [ ] ESP32 ohne Bus flashen, CAN-Loopback-Test
- [ ] CAN-Kommunikation mit Pi (`/homing`-Protokoll) end-to-end
- [ ] TLE4905L mit Magnet: sauberer 3,3 V-Pegelwechsel am GPIO
- [ ] **Bremse:** Anzug-Puls (48 V, ~150 ms) → Öffnen bestätigen; Halten bei ~10,4 % Duty (5 V avg) verifizieren; Einfallzeit mit/ohne TVS-Klemmung messen; Not-Aus zwingt PWM = 0 **und** trennt hardwareseitig

---

## Firmware-Sequenz (Bremse, pro Node)

```
BREMSE LÖSEN:
  PWM = 100%  (48V)      → 100–200 ms warten (Öffnen ~35 ms + Reserve)
  PWM = ~10,4% (5V avg)  → Halten
  danach: Motor darf Drehmoment aufbauen

BREMSE SCHLIESSEN / NOT-AUS:
  PWM = 0%               → Anker fällt in ~20 ms
  (Motor-Torque vorher/gleichzeitig weg)
```

- Nach dem Lösen mindestens ~35 ms warten, bevor der Motor Moment aufbaut.
- Not-Aus zwingt PWM = 0 **und** trennt hardwareseitig (Sicherheit Punkt 1).

---

## Offene Punkte / spätere Optionen
- **Haltemoment-Test** unter Payload in ungünstigster Pose an J2/J3 (0,80 N·m Bremse, ~6,4 N·m am Gelenk vs. 7,5 N·m Motor-Nennmoment). Kalt **und** warm.
- **Spuleninduktivität L** messen → PWM-Frequenz und τ (Einfallzeit) final festlegen.
- Schwellen **pro Bremse** prüfen (können leicht abweichen); Auslegung auf die schwächste Bremse.
- **3,3 V-Gate-Ansteuerung:** FET-Auswahl gegen tatsächliche R_DS(on) bei V_GS = 3,3 V prüfen, ggf. Gate-Treiber/Level-Shift auf 5 V.
- Entscheidung **Ein-Rail-PWM vs. 5 V-DC-Halten** final treffen.
- **Lüfter-Versorgung** nach Wegfall des 24-V-Bucks festlegen (48 V-PWM / 5 V / lokaler Buck), falls DNP-Provision je bestückt wird.
- TVS am 5 V-Eingang final entscheiden.
- XT30PB-Signalpins: Verwendung definieren oder einfachen XT30 nutzen.
- Sicherung/PTC im 48 V-Pfad pro Board erwägen (Steckergeometrie ersetzt keine Überstromsicherung); auch im Bremsen-Zweig erwägen.
- Erweiterungsstecker-Last bei AP2112K-Budget (600 mA) berücksichtigen, falls dort Sensorik versorgt wird.
