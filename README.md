# Roboticarm-R0192

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Der **R0192** ist ein hochpräziser 6-Achsen-Roboterarm, der auf einem hybriden System aus SteadyWin- und RobStride-Aktuatoren basiert. Dieses Repository enthält das vollständige ROS 2 Jazzy-Ökosystem für die Simulation (MoveIt 2), die CAN-Bus-Kommunikation und die Hardware-Abstraktion.

---

## 🚀 Projekt-Roadmap

### Software (ROS 2 & OS)
- [x] OS-Setup: Ubuntu 24.04 & ROS 2 Jazzy Installation
- [x] Workspace & Paketstruktur initialisiert
- [x] Treiber-Integration: SteadyWin GDS68 (Achse 1-3)
- [x] Basis-Node für CAN-Kommunikation & Einzelmotortest
- [x] URDF/XACRO Modellierung inkl. 3D-Visualisierung
- [x] Kollisionsgeometrien für MoveIt 2 integriert
- [x] RVIZ & MoveIt 2 Integration
- [ ] **Aktuell:** Integration der RobStride RSO5 Treiber (Achse 4-6)
- [ ] Closed-Loop Verbindung zwischen RVIZ-Planung und Hardware
- [ ] Implementierung der Homing-Sequenz (TLE4935L Hall-Sensoren)
- [ ] Globaler Software-Notaus & Fehlerbehandlung
- [ ] Echtzeit-Optimierung (RT-Kernel & CPU-Shielding)

#### Projektstruktur

```
roboticarm_r0192_ws/
├── src/
│   ├── r0192_bringup/         # Bringup Sequenz für den Roboterarm
│   ├── r0192_canbus/          # CAN-Bus Steuerung und Treiber der Motoren
│   ├── r0192_controller/      # Controller für den Roboterarm
│   ├── r0192_description/     # Robot-Beschreibung
│   ├── r0192_moveit/          # MoveIt für ROS
│   ├── r0192_msgs/            # Nachrichten/Services
│   ├── r0192_remote/          # Remote-Steuerung
│   └── r0192_utils/           # Utilities
├── doku/                      # Zusätzliche Dokumente für Treiber usw.
├── build/                     # Build-Artefakte (ignoriert)
├── install/                   # Install-Artefakte (ignoriert)
└── log/                       # Logs (ignoriert)
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
| Achse | Motor | Treiber | Besonderheit | Lagerung |
| :--- | :--- | :--- | :--- | :--- |
| **Axis 1** | [GIM6010-8](https://steadywin.cn/en/pd.jsp?id=116&fromColId=0#_pp=0_757_3) | GDS68 | High Torque Base | [Kreuzlager RU66](https://de.aliexpress.com/item/1005008094900625.html) |
| **Axis 2** | [GIM8108-8](https://steadywin.cn/en/pd.jsp?id=133&fromColId=0#_pp=0_757_3) | GDS68 | Hauptausleger | [Kreuzlager RU42](https://de.aliexpress.com/item/1005008094900625.html) |
| **Axis 3** | [GIM6010-8](https://steadywin.cn/en/pd.jsp?id=116&fromColId=0#_pp=0_757_3) | GDS68 | Ellenbogen | - |
| **Axis 4** | [RSO5](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature/RS05) | Intern | RobStride Dynamics | - |
| **Axis 5** | [RSO5](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature/RS05) | Intern | 1:2 Übersetzung (Torque) | [Kreuzlager RU28](https://de.aliexpress.com/item/1005008094900625.html) |
| **Axis 6** | [RSO5](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature/RS05) | Intern | Endeffektor Rotation | - |



### Elektronik & Steuerung
* **Recheneinheit:** Raspberry Pi 5 (8GB RAM)
* **CAN-Interface:** MKS CANable Pro (Isoliert)
* **Energieversorgung:** * Bus-Spannung: 48V (MeanWell LRS-600N2)
    * Logik-Spannung: 5V (MeanWell LRS-50)
* **Sensorik:** Hall-Effekt-Sensoren (TLE4935L) für Nullpunktkalibrierung
* **Schnittstellen:** XT60PW (Power-Bus), XT30PW (Motor-Abgriff), XH-2A (CAN-Bus)

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
