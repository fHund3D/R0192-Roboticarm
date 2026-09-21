# R0192 Not-Aus: Hardware-Abschaltung der 48-V-Schiene

**Status: Entwurf 2026-09-21 (Steuerspannung auf ein eigenes 24-V-Netzteil geändert) — nicht gebaut, nicht bestellt.** Auslegung und Teileauswahl stehen, die Entscheidungen unter „Offene Punkte" fehlen noch.

Der bestehende Software-Not-Aus (`/e_stop` → `/robot_estop` → GDS68 `Estop()` / RS05 `Motor_Stop_Running`) kappt das Drehmoment auf **Treiber-Ebene**. Das ist eine Software-Kette über CAN — sie setzt voraus, dass Pi, CAN-Bus und Treiber-Firmware leben. Dieses Dokument beschreibt die **zweite, davon unabhängige Ebene**: ein Schütz, das die 48 V physisch trennt.

---

## Randbedingungen

| | |
| --- | --- |
| Zu schaltender Kreis | 48 V DC, **12,5 A Dauer**, ~25 A für 5 s (200-%-Peak des LRS-600N2) |
| Verfügbare Hilfsspannungen | Am Arm **nur 5 V** (LRS-50) und **48 V** — es gibt kein 24-V-Netz im Arm, die Haltebremsen werden PWM-gechoppt direkt aus 48 V betrieben (s. [PCB-Doku](../PCB/pcb_daisy_chain.md), „Bremsen-Endstufe"). **Für die Not-Aus-Kette steht ein separates 24-V-Netzteil zur Verfügung** (vorhanden, bisher ungenutzt). |
| Bus-Kapazität hinter dem Schütz | 6 × ~300 µF ≈ **1,8 mF** (C2/C3/C14 je Board) plus Treiber-Eingangskapazität |
| Rückspeisung | ODrive Regen Clamp am Bus, klemmt auf ~50 V |
| Not-Aus-Taster | **vorhanden** (Pilz, rastend, Öffnerkontakt) |
| Einbauort | Steuerungs- & Versorgungsschrank (Item-Profil-Gehäuse), zwischen Netzteil und Han-E-24-Umbilical |

---

## Einbaureihenfolge

```
230 V AC ─[Hauptschalter/FI]─┬─► MeanWell LRS-600N2 (48 V)
                             │        │
                             │      [F2: 30 A KFZ-MIDI oder 20 A träge]
                             │        │
                             │      ┌─┴─ R_pre 47 Ω ─ K3 ─┐   Vorladung
                             │      │                     │
                             │      └═╪═ K1 (DC-Schütz) ══┘
                             │        │
                             │   ┌────┴──────────────┬─────────────────────┐
                             │   │                   │                     │
                             │  ODrive Regen Clamp  Bleeder 10 k/2 W   Han-E 24 ─► Daisy-Chain-Boards
                             │  + 2 Ω / 50 W                                        ─► GDS68 / RS05
                             │
                             ├─► 24-V-Netzteil ──────────────────────────► Not-Aus-Kette (K1/K2/K3-Spulen)
                             │                                              galvanisch getrennt vom 48-V-Bus
                             │
                             └─► MeanWell LRS-50 (5 V) ──────────────────► Pi, CAN, Daisy-Chain-Logik
                                                                            NICHT geschaltet
```

**Drei Festlegungen, die dabei wichtig sind:**

1. **Die Regen Clamp hängt auf der Lastseite von K1.** Wird 48 V getrennt während der Arm fährt, speisen die Motoren über die Body-Dioden der Treiber-Brücken zurück. Läge die Clamp auf der Netzteilseite, hätte diese Energie keinen Abfluss mehr und die Busspannung liefe über die 72 V des GDS68 bzw. die 100 V von Q2/Elkos hinaus. Auf der Lastseite arbeitet sie unverändert weiter.
2. **Die 5 V werden nicht geschaltet.** Sonst rebooten Pi, CAN-Transceiver und alle sechs Homing-Nodes bei jedem Not-Aus, und die Diagnose ist genau dann weg, wenn man sie braucht. Die Motortreiber hängen ohnehin an 48 V.
3. **Die Steuerspannung wird vor dem Schütz abgegriffen**, sonst könnte sich K1 nie selbst einschalten.

**Warum nicht auf der 230-V-Seite trennen?** Billiger (AC-Schütze löschen im Nulldurchgang), aber das LRS-600 hat eine Hold-up-Zeit im zweistelligen Millisekundenbereich und danach entladen sich noch 1,8 mF Buskapazität. Dazu kommt ein Kaltstart des Netzteils bei jedem Not-Aus. Für einen Roboterarm ist DC-seitig trennen richtig.

---

## Steuerspannung: eigenes 24-V-Netzteil

**Entscheidung 2026-09-21: eigenes 24-V-Netzteil**, nicht aus dem 48-V-Bus abgeleitet.

| Variante | Bewertung |
| --- | --- |
| **5 V** (LRS-50) | In der 25-A-DC-Klasse gibt es praktisch keine 5-V-Spulen. Die Panasonic HE-V fängt bei 6 V an, und die Anzugsspannung liegt typisch bei 75 % der Nennspannung (≈ 4,5 V) — mit der Spulenerwärmung steigt sie weiter. Für ein Sicherheitselement zu knapp. |
| **aus 48 V abgeleitet** (z. B. Recom R-78HB12-0.5, 17–72 V → 12 V) | Funktioniert und spart ein Netzteil — hat aber eine unangenehme Rückkopplung: Bei einem Kurzschluss auf der Lastseite geht das LRS-600 in die Konstantstrombegrenzung, die Busspannung fällt unter den Eingangsbereich des Moduls, K1 fällt ab, der Kurzschluss ist weg, das Netzteil erholt sich, K1 zieht wieder an … Ein Schütz, das gegen einen Kurzschluss klappert, verschweißt. Die Selbsthaltung über K2 fängt das nach dem ersten Durchgang ab, aber die Abhängigkeit bleibt. |
| **eigenes 24-V-Netzteil** ✔ | Galvanisch unabhängig vom 48-V-Bus, damit existiert die Rückkopplung oben gar nicht. 24-V-Spulen sind der Industriestandard → größte Auswahl bei Schütz und Hilfsrelais. Bei ~80 mA Spulenstrom ist der Leitungsabfall zum Not-Aus-Taster irrelevant. |

**Strombedarf:** K1-Spule ~80 mA, K2 ~40 mA, K3 ~40 mA, Optokoppler ~10 mA → **unter 200 mA**. Jedes Kleinnetzteil genügt.

**Bedingung: gleicher Netzschalter und dieselbe Sicherung wie der Rest des Schranks.** Sonst kann die Steuerspannung stehen, während der Schrank scheinbar aus ist — genau die Überraschung, die man beim Arbeiten an der Verdrahtung nicht will.

Fällt das 24-V-Netzteil aus, fällt K1 ab: der Ausfall führt in den sicheren Zustand, und der Pi sieht es über den Statuseingang.

---

## Schaltung

```
+24 V ─[F3 0,5 A]─┬─ Not-Aus (Öffner 1) ─┬─[Reset-Taster NO]─┬─ K2-Spule ─ GND
                  │                      └─[K2 Kontakt 1]────┘   Selbsthaltung
                  │
                  ├─ K2 Kontakt 2 ─ Q1 (Pi GPIO) ─ K1-Spule ─ GND        48-V-Schütz
                  ├─ K2 Kontakt 3 ─ Q3 (Pi GPIO) ─ K3-Spule ─ GND        Vorladerelais
                  └─ K2 Kontakt 4 ─ Optokoppler ─► Pi GPIO (Eingang)     Status
```

Die 24-V-Masse der Steuerkette und die 48-V-Masse an **einem** Punkt im Schrank verbinden (Sternpunkt), damit Pi-Statuseingang und MOSFET-Source ein gemeinsames Bezugspotenzial haben. Der Optokoppler hält die 24 V trotzdem vom Pi fern.

Über K1- und K3-Spule je eine Freilaufdiode **in Reihe mit einer Z-Diode** (oder eine TVS, z. B. P6KE33A — V_RWM 28,2 V, sicher über den 24 V). Eine nackte 1N4007 verlangsamt den Spulenabfall spürbar, und beim Schütz heißt langsamer Abfall längerer Lichtbogen.

Q1/Q3: logic-level MOSFET (AO3400A o. ä.) mit **10 k Gate-Pulldown**. Die Pi-GPIOs sind beim Boot hochohmig — mit dem Pulldown ist der sichere Zustand „aus" der Default, auch während des Bootens und nach einem Absturz.

### Verhalten

| Ereignis | Wirkung |
| --- | --- |
| Not-Aus gedrückt | K2 fällt → K1 fällt → 48 V weg. Rein mechanisch, die Software kann das nicht überstimmen. |
| Not-Aus entriegelt | **Bleibt aus.** Die Selbsthaltung um den Reset-Taster ist unterbrochen, K2 zieht erst nach dem Drücken von Reset wieder an. Wiederanlaufschutz in Hardware. |
| Pi setzt GPIO low (`/e_stop`) | Q1 sperrt → K1 fällt. K2 bleibt angezogen, d. h. der Pi kann über `/robot_reset` wieder einschalten, ohne dass jemand zum Schrank läuft. |
| Pi bootet / stürzt ab | GPIO hochohmig → Pulldown → K1 offen. |
| 48 V oder Netzteil weg | Steuerspannung weg → K1 offen. |

Der Preis für den Komfort in Zeile 4: nach einem **Software**-Not-Aus kann die Software auch wieder einschalten. Der Hardware-Taster bleibt davon unberührt — der braucht immer den physischen Reset.

> **Minimalvariante ohne K2:** Die HEV2aN hat zwei Kontakte. Man kann Kontakt B als Selbsthaltung benutzen und K2 komplett sparen. Dann bricht aber auch jeder Software-Not-Aus die Selbsthaltung, und man muss nach *jedem* `/e_stop` physisch am Schrank quittieren. Für den Dauerbetrieb sauberer, für die Entwicklung lästig.

---

## Schütz K1

**Panasonic HEV2aN-P-DC24V** (HE-V-Serie, „High Capacity DC Cutoff Relays"), Mouser.

| | |
| --- | --- |
| Kontakte | 2 Form A, AgNi |
| Strom | 25 A |
| Schaltvermögen | 20 A @ 400 V DC (ein Kontakt), 25 A @ 600 V DC (beide in Serie), max. 1000 V DC |
| Spule | 6 / 9 / 12 / 15 / 24 V DC |

Bei 48 V ist das Teil massiv unterfordert — die Lichtbogenenergie skaliert mit U·I, und 400 V DC sind Faktor 8 über dem Arbeitspunkt. Genau dieser Abstand ist die Reserve, die man bei einem Sicherheitselement will.

**Beide Kontakte parallel** schalten: die Serienschaltung bringt nur mehr Spannungsfestigkeit, die hier niemand braucht, parallel gibt es thermische Reserve für den 25-A-Peak. (Beim Öffnen trägt der zuerst öffnende Kontakt den Lichtbogen allein — bei 48 V unkritisch.)

**Die typische Falle beim Aussuchen:** Distributor-Filter mischen Spulen- und Kontaktspannung. Ein TE T9AS1D12-48 ist eine **48-V-Spule** mit 30 A Kontakt — aber nur **30 V DC** Schaltvermögen. Die meisten „30 A"-Leistungsrelais sind 30 A / 250 V **AC** und 30 A / 30 V **DC**.

Alternative mit mehr Reserve, falls der Strom später steigt: Gigavac **GX11** / **GX14** (50–150 A, 12–800 V DC, Spule auch 24 V) — liegen aber bei ~100 €+.

---

## Vorladung (K3 + R_pre)

Hinter dem Schütz liegen ~1,8 mF, davor die geladenen Ausgangskondensatoren des LRS-600. Schließt K1 direkt, fließt ein Ausgleichsstrom, der nur durch den Leitungswiderstand begrenzt ist — drei-stellige Ampere für einige Mikrosekunden. Das verschweißt auf Dauer die Kontakte.

**Ablauf, vom Pi gesteuert** (K1 wird ohnehin nur vom Pi eingeschaltet, der Taster schaltet nur ab):

```
1. Q3 an   → K3 zieht an, Bus lädt über R_pre 47 Ω     τ = 47 Ω · 1,8 mF ≈ 85 ms
2. 500 ms warten                                        (≈ 6 τ, Bus auf > 99 %)
3. Q1 an   → K1 zieht an (beide Seiten gleiches Potenzial, kein Inrush)
4. Q3 aus  → K3 fällt ab (trennt spannungslos, K1 ist parallel)
```

R_pre: **47 Ω / 10 W Drahtwiderstand**. Spitzenstrom 48 V / 47 Ω ≈ 1 A, Energie pro Ladevorgang ½·C·U² ≈ 2,1 J — für einen 10-W-Typ ein Klacks. K3 kann ein beliebiges kleines 24-V-Relais sein: es schließt in 1 A und öffnet spannungsfrei.

> **Alternative ohne K3/Q3:** R_pre fest parallel zu den K1-Kontakten. Dann liegen beide Seiten immer auf gleichem Potenzial und der Inrush entfällt ohne jede Sequenz. Preis: nach dem Not-Aus rieseln dauerhaft ~22 mA in den Bus. Das reicht für kein Drehmoment und auch nicht für die Halte-PWM der Bremsen (147 mA pro Achse), aber „Bus ist garantiert spannungsfrei" gilt dann nicht mehr — beim Arbeiten am Arm muss man den Hauptschalter benutzen.

**Bleeder** 10 k / 2 W auf der Lastseite: entlädt die 1,8 mF nach dem Trennen in ~1 min auf ungefährliche Werte, Verlustleistung 0,23 W. Gespeicherte Energie ist mit ~2 J ohnehin harmlos, aber definierte Verhältnisse sind angenehmer.

---

## Bremsen beim Not-Aus

Die Haltebremsen sind stromlos geschlossen und hängen an 48 V — das Trennen lässt sie also **zwangsläufig** einfallen. Aber verzögert: solange die Busspannung aus den 1,8 mF noch steht und der ESP32 an 5 V weiterläuft, läuft auch die Halte-PWM weiter. Die Bremse fällt erst unter ~2,5 V mittlerer Spulenspannung, bei 10,4 % Duty also erst unter ~24 V Bus (Details in der [PCB-Doku](../PCB/pcb_daisy_chain.md), „Bremsen-Endstufe → Sicherheit").

**Vorschlag: Bus-Unterspannung auf dem vorhandenen Sense-Pin erkennen.** Der Bremsen-Sense (GPIO D3, Teiler R7 150 k / R8 6,8 k am Schaltknoten) misst in der PWM-**Aus**-Phase die Busspannung, weil der Knoten dann über D1 bzw. über die Spule am Bus hängt. Teilerverhältnis 6,8/156,8 = 0,0434 → 48 V ergeben 2,08 V am ADC, gut im Bereich.

```
Firmware: in der PWM-Aus-Phase samplen.
          U_bus < 40 V für > 2 ms  →  PWM = 0
```

Die Bremse fällt dann in ihren ~20 ms statt nach einigen hundert Millisekunden, **ohne zusätzliche Hardware** und ohne dass der ESP32 vom Pi informiert werden muss. Das ist der bessere der beiden Wege, die die PCB-Doku als offenen Punkt führt (die Alternative „Schütz trennt auch die 5 V" kostet CAN und Diagnose).

Zu verifizieren beim Bring-up: dass der Knoten in der Aus-Phase tatsächlich sauber am Bus liegt und der ADC-Wert nicht von der Freilaufphase verfälscht wird.

---

## ROS-Anbindung

Die Zustandsmaschine (`robot_state_manager`) bleibt unverändert, sie bekommt nur zwei zusätzliche Aktionen und einen Eingang:

| ROS-Schnittstelle | bisher | neu zusätzlich |
| --- | --- | --- |
| `/e_stop` | Treiber-Torque-Cut über `/robot_estop` | **danach** Q1 aus → K1 fällt |
| `/robot_reset` | `/robot_clear_faults` | Vorladesequenz, dann Q1 an |
| `/robot_state` | 5 Zustände | Not-Aus-Eingang (Optokoppler) erzwingt `DISABLED`, solange K2 abgefallen ist |

**Reihenfolge bei `/e_stop` ist wichtig:** erst der Treiber-Stop über CAN, dann ~50 ms später das Schütz. Dann sind die Endstufen bereits aus, bevor die Busspannung wegbricht, und die Regen Clamp muss weniger abfangen. Der Hardware-Taster geht natürlich direkt auf die Spule, ohne jede Verzögerung.

Drei GPIOs: 2 Ausgänge (Q1, Q3), 1 Eingang (Not-Aus-Status über Optokoppler, damit die 12 V nicht an den Pi gehen).

---

## Stückliste

| Pos | Teil | Bezug | ca. |
| --- | --- | --- | ---: |
| K1 | Panasonic **HEV2aN-P-DC24V** | Mouser | ~30 € |
| — | 24-V-Netzteil für die Steuerkette (≥ 200 mA) | **vorhanden** | — |
| K2 | Finder **55.34.9.024.0040** (4 Wechsler, 24 V DC) + Fassung 94.74 | Reichelt | ~12 € |
| K3 | kleines 24-V-Relais, 1 Schließer | Reichelt | ~4 € |
| R_pre | 47 Ω / 10 W Draht | Reichelt | ~2 € |
| Bleeder | 10 k / 2 W | Reichelt | ~1 € |
| Q1, Q3 | AO3400A + 10 k Pulldown | Mouser | — |
| — | P6KE33A (über K1/K3-Spule), 1N4007 | Mouser | — |
| F2 | KFZ-MIDI-Halter + 30 A | Reichelt | ~6 € |
| F3 | Feinsicherung 0,5 A träge + Halter | Reichelt | ~2 € |
| — | Reset-Taster (Schließer) | Reichelt | ~5 € |
| — | Optokoppler PC817 + 2,2 k Vorwiderstand | Mouser | — |

Not-Aus-Taster ist vorhanden. **Prüfen:** ob er einen zweiten Öffner für die Statusrückmeldung hat — sonst kommt der Status wie oben über K2 Kontakt 4.

---

## Grenzen

- **Kategorie 0 einkanalig.** Keine Zweikanaligkeit, keine Kontaktrückführung, kein Verschweiß-Nachweis. Wer das braucht, ersetzt K2 durch ein Sicherheitsrelais (Pilz PNOZ X2.8P, Phoenix PSR, Siemens 3SK1, ~80–130 €), das beide Öffner überwacht und die Rückmeldekontakte von K1 auswertet. Die übrige Schaltung bleibt gleich.
- **Laufendes Homing wird nicht abgebrochen.** Der `HomingController` fährt Achse 1 an `motors_enabled_` vorbei. Das Drehmoment ist nach dem Schütz zwar weg, aber der Abbruch-Hook in der Software fehlt weiterhin (s. `CLAUDE.md`, „v1-Grenze").
- **Kein PE-/Isolationskonzept** in diesem Dokument. 48 V SELV, aber der 230-V-Teil des Schranks ist ein eigenes Thema.

## Offene Punkte

- [ ] K2-Variante (Software darf wieder einschalten) vs. Minimalvariante (Selbsthaltung über K1-Kontakt B, immer physischer Reset) entscheiden
- [ ] Vorladung: Pi-sequenziert mit K3, oder R_pre fest parallel mit dem Rest-Rieselstrom
- [ ] Bus-Sense-Trip in der ESP32-Firmware umsetzen und beim Bring-up gegen die reale Einfallzeit messen
- [ ] Kontakte des vorhandenen Not-Aus-Tasters prüfen (Anzahl Öffner, DC-Angaben)
- [ ] Spulenleistung der HEV2aN aus dem Datenblatt gegen die Leistung des vorhandenen 24-V-Netzteils gegenrechnen (erwartet ~80 mA, Gesamtkette < 200 mA)
- [ ] 24-V-Netzteil auf denselben Netzschalter / dieselbe Sicherung legen wie das 48-V- und 5-V-Netzteil
- [ ] Platz und Montage im Steuerungsschrank (Hutschiene?) festlegen
