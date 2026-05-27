# Roboticarm-R0192

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Der **R0192** ist ein hochpräziser 6-Achsen-Roboterarm, der auf einem hybriden System aus SteadyWin- und RobStride-Aktuatoren basiert. Dieses Repository enthält das vollständige ROS 2 Jazzy-Ökosystem für die CAN-Bus-Kommunikation, die Hardware-Abstraktion, die Bahnplanung via MoveIt 2 sowie ein Arduino-basiertes Hall-Sensor-Homing.

Die gesamte Steuerungslogik (MoveIt 2, ros2_control, CAN-Treiber) läuft auf einem **Raspberry Pi 5**. Als Bedienpanel ist ein eigenes **Web-Interface** (`r0192_remote`) geplant; Foxglove Studio und RViz dienen als Debug-Werkzeuge. Achsen 1 und 4 sind vollständig in Betrieb; die restlichen Achsen werden nach und nach nachgerüstet.

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
- [x] `foxglove_bridge` in Bringup integriert (Debug-Tool, Port 8765, `use_foxglove:=true`)
- [x] **Homing-Sequenz** mit TLE4905L Hall-Sensoren — Arduino-Firmware (Achse 1) + ROS-Service `/homing` in `r0192_hardware` eingebettet; zweiseitiger Bisektionsalgorithmus implementiert
- [ ] **Aktuell: Hardware-Test Homing** — Arduino + Magnet an Achse 1 anschließen und end-to-end validieren
- [ ] **Aktuell: Web-Interface** (`r0192_remote`) — eigenes Bedienpanel für E-Stop, Motor-Enable, Homing, Arm-Steuerung
- [ ] Globaler Software-Notaus (E-Stop)
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
│   └── r0192_remote/          # (Geplant) Web-Interface als Operator-Pendant
├── doku/                      # Datenblätter & Treiber-Dokumentation
├── build/                     # Build-Artefakte (ignoriert)
├── install/                   # Install-Artefakte (ignoriert)
└── log/                       # Logs (ignoriert)
```

#### System-Architektur

```
Web-Interface (Browser/MacBook)          Debug-Tools (optional)
  │  HTTP / WebSocket                    RViz  ◄──  ROS-Topics
  │  E-Stop, Enable, Homing, Steuerung   Foxglove ◄─ foxglove_bridge (:8765)
r0192_remote (RPi, geplant)
  │
MoveIt 2 ──► ros2_control (100 Hz, Closed-Loop)
                 │
         R0192SystemHardware
          ├─ /homing Service (eingebettet)
                 │
    ┌────────────┴────────────┐
GDS68Driver              RS05Driver
(Achsen 1–3)             (Achsen 4–6)
    └────────────┬────────────┘
             SocketCAN (can0, 1 Mbit/s)
             MKS CANable Pro → Motoren
                 │
    Arduino Uno R3 (je 1 pro Achse)
    TLE4905L Hall-Sensor + MCP2515
    Homing-Node (CAN 0x100 ↔ 0x000)
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
* **Homing-Sensorik:** TLE4905L Hall-Effekt-Sensoren (je 1 Arduino Uno R3 + MCP2515 CAN-Transceiver pro Achse)
* **Schnittstellen:** XT60PW (Power-Bus), XT30PW (Motor-Abgriff), XH-2A (CAN-Bus)

> **Hinweis zu URDF-Werten:** Alle Gelenkgrenzen, Trägheitsmomente und Schwerpunkte in der URDF sind derzeit Platzhalter und werden nach Abschluss des CAD-Designs mit realen Werten befüllt.

---

---

## Quickstart (RPi)

```bash
# 1. CAN-Interface hochfahren (einmalig pro Boot)
sudo ip link set can0 up type can bitrate 1000000

# 2. Build (nach Code-Änderungen)
cd ~/roboticarm_r0192_ws
colcon build --symlink-install
source install/setup.bash

# 3. Roboterarm starten (ros2_control + MoveIt + RViz)
ros2 launch r0192_bringup real_robot.launch.py

# 4. Optional: Foxglove-Bridge für Remote-Debug mitstarten
ros2 launch r0192_bringup real_robot.launch.py use_rviz:=false use_foxglove:=true

# 5. Homing auslösen (zu beliebigem Zeitpunkt nach Start)
ros2 service call /homing std_srvs/srv/Trigger {}

# Homing-Parameter zur Laufzeit anpassen (optional)
ros2 param set /r0192_homing homing_vel 0.1
ros2 param set /r0192_homing zero_offset 0.05
```

> **Homing-Ablauf (Achse 1):** Pi sendet CAN `0x100` → Arduino scharf gestellt → Achse sweept langsam in +/– Richtung → TLE4905L erkennt Magnet → Arduino antwortet mit CAN `0x000/0xFF` → Pi stoppt, merkt Positionen P1/P2 → fährt zur Mitte → setzt Encoder auf 0.

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
