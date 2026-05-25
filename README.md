# Roboticarm-R0192

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Der **R0192** ist ein hochpräziser 6-Achsen-Roboterarm, der auf einem hybriden System aus SteadyWin- und RobStride-Aktuatoren basiert. Dieses Repository enthält das vollständige ROS 2 Jazzy-Ökosystem für die CAN-Bus-Kommunikation, die Hardware-Abstraktion, die Bahnplanung via MoveIt 2 und das Bedienpanel via Foxglove Studio.

Die gesamte Steuerungslogik (MoveIt 2, ros2_control, CAN-Treiber) läuft auf einem **Raspberry Pi 5**. Als Bedienpanel dient **Foxglove Studio** auf einem externen Gerät (MacBook) — verbunden über Netzwerk via `foxglove_bridge`. Achsen 1 und 4 sind vollständig in Betrieb; die restlichen Achsen werden nach und nach nachgerüstet.

---

## 🚀 Projekt-Roadmap

### Software (ROS 2 & OS)
- [x] OS-Setup: Ubuntu 24.04 & ROS 2 Jazzy Installation
- [x] Workspace & Paketstruktur initialisiert
- [x] Treiber-Integration: SteadyWin GDS68 (Achse 1–3)
- [x] Treiber-Integration: RobStride RS05 (Achse 4–6)
- [x] CAN-Kommunikation & Hardware Interface (ros2_control, 100 Hz, Closed-Loop)
- [x] URDF/XACRO Modellierung inkl. 3D-Visualisierung & Kollisionsgeometrien
- [x] MoveIt 2 Integration — Bahnplanung & Trajektorienausführung auf Achsen 1 + 4 verifiziert
- [x] Encoder-Homing beim Start (Hardware Interface setzt internen Nullpunkt)
- [ ] **Aktuell:** Foxglove Tech-Pendant einrichten
  - `foxglove_bridge` in Bringup integrieren (RPi → MacBook via WebSocket)
  - Foxglove Studio: 3D-Ansicht, Buttons für E-Stop / Motor-Enable / Homing
  - ROS-Services für Bedienfunktionen implementieren
- [ ] Globaler Software-Notaus (E-Stop) — über Foxglove-Button auslösbar
- [ ] Homing-Sequenz mit TLE4905L Hall-Sensoren als Absolutreferenz
- [ ] Echtzeit-Optimierung (RT-Kernel & CPU-Shielding auf RPi 5)

#### Projektstruktur

```
roboticarm_r0192_ws/
├── src/
│   ├── r0192_bringup/         # Bringup Sequenz (real + simuliert)
│   ├── r0192_canbus/          # CAN-Bus Treiber: GDS68, RS05, CanCommunication
│   ├── r0192_controller/      # JTC-Konfiguration, manueller Slider-Node
│   ├── r0192_description/     # URDF/XACRO, STL-Meshes, RViz-Konfiguration
│   ├── r0192_hardware/        # ros2_control Hardware Interface (SystemInterface)
│   ├── r0192_moveit/          # MoveIt 2: SRDF, Kinematik, Planung, Launch
│   └── r0192_remote/          # (Geplant) ROS-Services für Foxglove-Bedienpanel
├── doku/                      # Datenblätter & Treiber-Dokumentation
├── build/                     # Build-Artefakte (ignoriert)
├── install/                   # Install-Artefakte (ignoriert)
└── log/                       # Logs (ignoriert)
```

#### System-Architektur

```
Foxglove Studio (MacBook)
  │  WebSocket ws://<rpi-ip>:8765  [bidirektional]
  │  Visualisierung + Bedienelemente (E-Stop, Enable, Homing)
foxglove_bridge (RPi)
  │
MoveIt 2 ──► ros2_control (100 Hz, Closed-Loop)
                 │
         R0192SystemHardware
                 │
    ┌────────────┴────────────┐
GDS68Driver              RS05Driver
(Achsen 1–3)             (Achsen 4–6)
    └────────────┬────────────┘
             SocketCAN (can0, 1 Mbit/s)
             MKS CANable Pro → Motoren
```

### Hardware & Mechanik
- [x] Kinematische Auslegung & Motorauswahl
- [x] Grobes mechanisches Design (CAD)
- [x] Beschaffung Primär-Aktuatoren (GIM6010 & RSO5)
- [ ] Detail-Konstruktion & Optimierung der Steifigkeit
- [ ] Full-Scale 3D-Druck Prototyping (PLA/PETG)
- [ ] **Geplant:** Substitution kritischer Strukturbauteile durch CNC-Aluminium

### Elektronik (PCB Design)
- [x] Schaltungskonzept & Leistungsplanung
- [x] Design Daisy-Chain PCB (PCB 2) - *Draft*
- [ ] Design Breakout-Board (PCB 1) - *Main Power Distribution*
- [ ] PCB-Fertigung (PCBWay) & Bestückung
- [ ] Systemintegration & EMV-Tests

---

## 🛠 Technische Spezifikationen

### Achskonfiguration

> **Aktuell verfügbare Hardware:** Achse 1 (GIM6010-8) und Achse 4 (RS05). Die restlichen Motoren werden zu einem späteren Zeitpunkt nachgekauft.

| Achse | Motor | Treiber | Besonderheit | Lagerung | Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Axis 1** | [GIM6010-8](https://steadywin.cn/en/pd.jsp?id=116&fromColId=0#_pp=0_757_3) | GDS68 | High Torque Base | [Kreuzlager RU66](https://de.aliexpress.com/item/1005008094900625.html) | ✅ Vorhanden |
| **Axis 2** | [GIM8108-8](https://steadywin.cn/en/pd.jsp?id=133&fromColId=0#_pp=0_757_3) | GDS68 | Hauptausleger | [Kreuzlager RU42](https://de.aliexpress.com/item/1005008094900625.html) | 🔜 Geplant |
| **Axis 3** | [GIM6010-8](https://steadywin.cn/en/pd.jsp?id=116&fromColId=0#_pp=0_757_3) | GDS68 | Ellenbogen | - | 🔜 Geplant |
| **Axis 4** | [RS05](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature/RS05) | Intern | RobStride Dynamics | - | ✅ Vorhanden |
| **Axis 5** | [RS05](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature/RS05) | Intern | 1:2 Übersetzung (Torque) | [Kreuzlager RU28](https://de.aliexpress.com/item/1005008094900625.html) | 🔜 Geplant |
| **Axis 6** | [RS05](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature/RS05) | Intern | Endeffektor Rotation | - | 🔜 Geplant |



### Elektronik & Steuerung
* **Recheneinheit:** Raspberry Pi 5 (8GB RAM)
* **CAN-Interface:** MKS CANable Pro (Isoliert), SocketCAN (`can0`)
* **CAN-Bitrate:** 1 Mbit/s (info: 500 kbit/s GDS68 Werkseinstellung)
* **Energieversorgung:**
    * Bus-Spannung: 48V (MeanWell LRS-600N2)
    * Logik-Spannung: 5V (MeanWell LRS-50)
* **Sensorik:** Hall-Effekt-Sensoren (TLE4905L) für Nullpunktkalibrierung
* **Schnittstellen:** XT60PW (Power-Bus), XT30PW (Motor-Abgriff), XH-2A (CAN-Bus)

> **Hinweis zu URDF-Werten:** Alle Gelenkgrenzen, Trägheitsmomente und Schwerpunkte in der URDF sind derzeit Platzhalter und werden nach Abschluss des CAD-Designs mit realen Werten befüllt.

---

## Beitragen

1. Fork das Repository
2. Erstelle einen Feature-Branch (`git checkout -b feature/AmazingFeature`)
3. Commit deine Änderungen (`git commit -m 'Add some AmazingFeature'`)
4. Push zum Branch (`git push origin feature/AmazingFeature`)
5. Öffne einen Pull Request

## Lizenz

Dieses Projekt ist unter der MIT License lizenziert - siehe die [LICENSE](LICENSE) Datei für Details.

## Kontakt

Bei Fragen oder Problemen: Erstelle ein Issue im GitHub Repository.
