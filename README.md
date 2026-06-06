# Roboticarm-R0192

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Der **R0192** ist ein hochpräziser 6-Achsen-Roboterarm, der auf einem hybriden System aus SteadyWin- und RobStride-Aktuatoren basiert. Dieses Repository enthält das vollständige ROS 2 Jazzy-Ökosystem für die CAN-Bus-Kommunikation, die Hardware-Abstraktion, die Bahnplanung via MoveIt 2 sowie ein Arduino-basiertes Hall-Sensor-Homing.

Die gesamte Steuerungslogik (MoveIt 2, ros2_control, CAN-Treiber) läuft auf einem **Raspberry Pi 5**. Eine zentrale **Betriebszustands-Maschine** (`robot_state_manager`) koordiniert Motor-Enable, Jogging, MoveIt-Ausführung, Homing und Notaus als sich gegenseitig ausschließende Zustände. Bedient wird der Arm aktuell über ein **RViz-Jog-Panel** (Teach-Pendant); als Produktiv-Bedienpanel ist ein eigenes **Web-Interface** (`r0192_remote`) geplant. Foxglove Studio und RViz dienen als Debug-Werkzeuge. Achsen 1 und 4 sind vollständig in Betrieb; die restlichen Achsen werden nach und nach nachgerüstet.

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
- [x] **Homing-Sequenz** mit TLE4905L Hall-Sensoren — Arduino-Firmware (Achse 1) + ROS-Service `/homing` in `r0192_hardware` eingebettet; zweiseitiger Kantenanlauf (Kante A → Überfahren → Kante B → Mitte) implementiert
- [x] **RViz Jog-Panel / Teach-Pendant** (`r0192_rviz_plugins`) über MoveIt Servo: 3 Modi (Joints / Cartesian / Tool), Speed-Slider, Press-and-Hold
- [x] **Zentrale Zustandsmaschine** (`robot_state_manager` + `r0192_interfaces`): 5 exklusive Zustände DISABLED / HOLD / JOG / MOVEIT / HOMING via `/set_robot_state` + `/robot_state`
- [x] **Notaus** (`/e_stop`) — erzwingt DISABLED aus jedem Zustand, latchender Treiber-Torque-Cut (GDS68 `Estop()`, RS05 stop) + Treiber-Reset (`/robot_reset` → `Clear_Errors`) zur Wiederinbetriebnahme
- [ ] **Aktuell: Hardware-Test Homing** — Arduino + Magnet an Achse 1 anschließen und end-to-end validieren
- [ ] **Aktuell: Web-Interface** (`r0192_remote`) — eigenes Bedienpanel auf Basis von `/set_robot_state` + `/robot_state`
- [ ] Echter Hardware-Notaus (laufenden Homing-Sweep abbrechen, ggf. Power-Cut/Schütz statt nur Treiber-Torque-Aus)
- [ ] Echtzeit-Optimierung (RT-Kernel & CPU-Shielding auf RPi 5)

#### Projektstruktur

```
roboticarm_r0192_ws/
├── src/
│   ├── r0192_bringup/         # Bringup Sequenz (real + simuliert)
│   ├── r0192_canbus/          # CAN-Bus Treiber: GDS68, RS05, CanCommunication
│   ├── r0192_controller/      # JTC-Konfiguration, manueller Slider-Node
│   ├── r0192_description/     # URDF/XACRO, STL-Meshes, RViz-Konfiguration
│   ├── r0192_interfaces/      # Custom msgs/srvs: RobotState, SetRobotState
│   ├── r0192_hardware/        # ros2_control HW-Interface + robot_state_manager
│   ├── r0192_moveit/          # MoveIt 2: SRDF, Kinematik, Planung, Servo, Launch
│   ├── r0192_rviz_plugins/    # RViz Jog-Panel / Teach-Pendant (Operator-Panel)
│   └── r0192_remote/          # (Geplant) Web-Interface als Operator-Pendant
├── doku/                      # Datenblätter & Treiber-Dokumentation
├── build/                     # Build-Artefakte (ignoriert)
├── install/                   # Install-Artefakte (ignoriert)
└── log/                       # Logs (ignoriert)
```

#### System-Architektur

```
RViz Jog-Panel  /  Web-Interface (geplant)        Debug-Tools (optional)
  │  /set_robot_state, /e_stop                     RViz  ◄──  ROS-Topics
  │  /robot_state (latched)                        Foxglove ◄─ foxglove_bridge (:8765)
  ▼
robot_state_manager  (zentrale Zustandsmaschine)
  │  koordiniert: /robot_enable, switch_controller, pause_servo, /homing
  ├──────────────► MoveIt 2 + Servo
  │                     │
  ▼                     ▼
ros2_control (100 Hz, Closed-Loop)  ──►  arm_controller / gripper_controller
                 │
         R0192SystemHardware
          ├─ /robot_enable Service (Motor-Torque an/aus)
          ├─ /homing Service (eingebettet, HomingController)
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
    Homing-Node (CAN-ID 0x100, Codes in Data[0]: CMD_ARM/DETECTED/ERROR)
```

#### Betriebszustände (Robot State Manager)

Der `robot_state_manager` ist die *Single Source of Truth* für den Betriebszustand. Vorher war der Zustand implizit über drei verstreute Flags verteilt (`motors_enabled_`, Homing-aktiv, Servo-Pause) — jetzt 5 exklusive Zustände mit erzwungenen Übergängen:

| Zustand | Motor-Torque | `arm_controller` | Servo | Bedeutung |
| :--- | :--- | :--- | :--- | :--- |
| `DISABLED` | aus | inaktiv | pausiert | Idle, drehmomentfrei (Startzustand) |
| `HOLD` | an | inaktiv | pausiert | Servos an, Hardware hält letzte Soll-Pos |
| `JOG` | an | aktiv | aktiv | Teach-Pendant über MoveIt Servo |
| `MOVEIT` | an | aktiv | pausiert | move_group darf Trajektorien ausführen |
| `HOMING` | an | (Homing managed) | pausiert | Homing-Sequenz läuft |

Übergänge: `DISABLED ⇄ HOLD`, und aus `HOLD` heraus `→ JOG / MOVEIT / HOMING` (jeweils zurück nach `HOLD`). `/e_stop` erzwingt `DISABLED` aus **jedem** Zustand und kappt das Drehmoment **auf Treiber-Ebene** (GDS68 `Estop()` 0x002 latchend, RS05 stop). Die Treiber sind danach bewusst in einem Fehlerzustand „gefangen" — `/robot_reset` (→ Hardware `Clear_Errors` / RS05-Fault-Clear) löst das, erst dann ist `DISABLED→HOLD` wieder erlaubt. Clients (RViz-Panel, künftiges Web-Interface) steuern ausschließlich über `/set_robot_state` und spiegeln das latched `/robot_state` — nie direkt `/robot_enable` / `/homing` / `pause_servo`.

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

# 5. Bedienung über die Zustandsmaschine (oder das RViz Jog-Panel)
#    Motoren scharf schalten (DISABLED -> HOLD):
ros2 service call /set_robot_state r0192_interfaces/srv/SetRobotState "{requested_state: 1}"
#    MoveIt-Ausführung freigeben (HOLD -> MOVEIT):
ros2 service call /set_robot_state r0192_interfaces/srv/SetRobotState "{requested_state: 3}"
#    Homing starten (HOLD -> HOMING, kehrt automatisch nach HOLD zurück):
ros2 service call /set_robot_state r0192_interfaces/srv/SetRobotState "{requested_state: 4}"
#    Notaus (erzwingt DISABLED aus jedem Zustand, latchender Treiber-Stop):
ros2 service call /e_stop std_srvs/srv/Trigger {}
#    Reset nach Notaus (löst die Treiber-Fehler, nötig vor erneutem Enable):
ros2 service call /robot_reset std_srvs/srv/Trigger {}

# Aktuellen Zustand beobachten (latched)
ros2 topic echo /robot_state

# Homing-Parameter zur Laufzeit anpassen (optional)
ros2 param set /r0192_homing homing_vel 0.1
ros2 param set /r0192_homing zero_offset 0.05
```

> **Hinweis:** `/homing` und `/robot_enable` existieren weiterhin als Low-Level-Services, sollten aber **nicht direkt** aufgerufen werden — das umgeht die zentrale Zustands-Erzwingung. Immer über `/set_robot_state` gehen.

> **Homing-Ablauf (Achse 1):** Pi sendet CAN `0x100` (`CMD_ARM`) → Arduino scharf gestellt → Achse sweept langsam zum Magneten → TLE4905L erkennt Kante A → Arduino antwortet `0x100`/`Data[0]=0xFF` (`RSP_DETECTED`) → Pi überfährt ~25°, fährt von der Gegenseite zurück bis Kante B → Mitte = (P1+P2)/2 → Software-Home-Offset im GDS68-Treiber gesetzt (nicht `Set_Linear_Count`, da kontinuierlicher Mehrumdrehungs-Encoder).

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
