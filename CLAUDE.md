# R0192 Robotic Arm – ROS 2 Jazzy Workspace

## Claude Maintenance Instructions

This file is loaded automatically at the start of every Claude Code conversation.
**Keep it up to date**: after implementing features, resolving known issues, or changing hardware/config, update the relevant sections (e.g. move a task from "Open" to done, fix a "Known Issue" entry, adjust parameter tables).

---

## Project Overview

The **R0192** is a custom 6-DOF robotic arm with a parallel gripper, built for ROS 2 Jazzy on Ubuntu 24.04. The compute unit is a **Raspberry Pi 5 (8 GB RAM)**. All motor communication runs over **SocketCAN** (`can0`) via a **MKS CANable Pro** (isolated USB-CAN adapter). The arm uses a hybrid actuator system:

- **Axes 1–3**: SteadyWin GIM motors with **GDS68** driver (standard 11-bit CAN frames)
- **Axes 4–6**: RobStride **RS05** motors with internal driver (extended 29-bit CAN frames)
- **Gripper**: prismatic joint pair (joint_7 + mirrored joint_8)

GitHub: https://github.com/fHund3D/R0192-Roboticarm

---

## Documentation (`/doku`)

Driver and hardware datasheets are stored locally in `/doku`:

| File | Content |
|------|---------|
| `doku/Actuatoren/SteadyWin Actuator/GDS68 Treiber.pdf` | GDS68 driver protocol, CAN commands, register map |
| `doku/Actuatoren/RobStride Dynamics/RS05User Manual.pdf` | RS05 motor user manual, CAN protocol, configuration |
| `doku/Actuatoren/SteadyWin Actuator/GIM6010-8.png` | GIM6010-8 motor datasheet image |
| `doku/Actuatoren/SteadyWin Actuator/GIM8108-8.png` | GIM8108-8 motor datasheet image |
| `doku/Netzteil Meanwell/LRS-600N2-spec.pdf` | Main 48 V PSU datasheet |
| `doku/Netzteil Meanwell/HRP-600N3-spec.pdf` | Alternative PSU datasheet |

Always consult these before implementing or modifying driver communication logic.

---

## Hardware Configuration

| Axis  | Motor      | Driver | CAN ID | Torque Range    | Status         | Remarks                |
|-------|------------|--------|--------|-----------------|----------------|------------------------|
| 1     | GIM6010-8  | GDS68  | 0x01   | ±11 Nm          | **Available**  | Base rotation, RU66 cross-roller |
| 2     | GIM8108-8  | GDS68  | 0x02   | ±22 Nm          | Not yet bought | Shoulder, RU42 cross-roller |
| 3     | GIM6010-8  | GDS68  | 0x03   | ±11 Nm          | Not yet bought | Elbow                  |
| 4     | RS05       | RS05   | 0x04   | ±5.5 Nm         | **Available**  | RobStride              |
| 5     | RS05       | RS05   | 0x05   | ±5.5 Nm (1:2)   | Not yet bought | RU28 cross-roller      |
| 6     | RS05       | RS05   | 0x06   | ±5.5 Nm         | Not yet bought | End-effector rotation  |

**Only axes 1 and 4 have physical motors available.** Axes 2, 3, 5, 6, and the gripper use passthrough feedback (command = state) so the full software stack runs without hardware.

**Power supply**: 48 V bus (MeanWell LRS-600N2), 5 V logic (MeanWell LRS-50)  
**Connectors**: XT60PW power bus, XT30PW motor tap, XH-2A CAN bus  
**Homing**: TLE4905L Hall-effect sensors for zero-point calibration (one Arduino Uno R3 per axis via MCP2515 CAN transceiver)  

---

## Package Structure

```
src/
├── r0192_bringup/       # Central launch entry points (real + simulated)
├── r0192_canbus/        # CAN drivers: CanCommunication, GDS68Driver, RS05Driver
├── r0192_controller/    # JointTrajectoryController config + manual slider node
├── r0192_description/   # URDF/xacro, STL meshes, Gazebo/Rviz configs
├── r0192_hardware/      # ros2_control hardware interface (SystemInterface plugin)
├── r0192_moveit/        # MoveIt 2 config: SRDF, kinematics, joint limits, launch
└── r0192_remote/        # (Planned) Web-based remote control interface
```

---

## Architecture Overview

```
Web-Interface (MacBook/Browser) ──── HTTP/WS ────► r0192_remote (RPi)
  [Operator-Pendant:                                      │
   E-Stop, Enable, Homing, Arm-Steuerung]                 │
                                                          ▼
                                         MoveIt 2  ──►  ros2_control (100 Hz)
                                                            │
                                                  ┌─────────┴──────────┐
                                                  │  arm_controller    │  gripper_controller
                                                  │ (JointTrajectory)  │  (JointTrajectory)
                                                  └─────────┬──────────┘
                                                            │
                                                  R0192SystemHardware
                                                     read() / write()
                                                            │
                                                   ┌────────┴────────┐
                                              GDS68Driver       RS05Driver
                                              (axes 1–3)        (axes 4–6)
                                                   └────────┬────────┘
                                                        SocketCAN (can0 @ 1 Mbit/s)
                                                            │
                                             ┌──────────────┴──────────────┐
                                        Arduino (Achse 1)  …  Arduino (Achse 6)
                                         TLE4905L + MCP2515              (je 1 pro Achse)
                                         Homing-Sensor-Node

Debug (lokal):
  RViz ◄──── ROS-Topics     Foxglove Studio ◄── foxglove_bridge (Port 8765)
  (MoveIt-Planungspanel)     (Topic-Inspektion, Waveforms)
```

**Feedback strategy per joint:**
- `joint_1` (GDS68, axis 1): real CAN feedback from motor. Mode set to Position Control (3) + Passthrough (1) via `Set_Controller_Mode(3,1)` on activate; commands sent via `MIT_Control`.
- `joint_4` (RS05, axis 4): real CAN feedback from motor. Commands sent via `MIT_Control`.
- `joint_2/3/5/6/7`: **passthrough** — last commanded value is reported as state (zero error, no motor needed)
- `joint_8`: derived from `joint_7` as mimic (multiplier −1)

**Axis presence detection:** At `on_configure`, each physical axis (1 and 4) is probed over CAN with a 200 ms timeout:
- GDS68 (axis 1): sends `Get_Encoder_Estimates()` (CMD 0x009), waits for any standard frame from `node_id`
- RS05 (axis 4): sends `Get_Device_ID()` (comm_type 0x00), waits for any extended frame with `sender_id == node_id` (bits 8–15)

**RS05 extended-frame caveat**: `Get_Device_ID()` uses comm_type `0x00`, making the CAN ID equal to just `node_id` (e.g. `0x04`). Since `0x04 < 0x7FF`, `sendFrame()` would normally send it as an 11-bit standard frame, which the RS05 ignores. `Get_Device_ID()` therefore passes `force_extended=true` to `sendFrame()`. The same issue applies to any RS05 command with comm_type `0x00`; all other comm_types produce IDs > `0x7FF` and are unaffected.

Result stored in `axis1_present_` / `axis4_present_`. Axes that don't respond are silently treated as virtual (passthrough). Initialization, CAN sends, and stop commands are gated on these flags.

CAN bitrate: **500 kbit/s** (GDS68 factory default). Will be raised to 1 Mbit/s later.

---

## CAN Protocol Details

### GDS68 (Axes 1–3) — Standard 11-bit frames
- **CAN-ID encoding**: `(node_id << 5) | cmd_id`
- **MIT Control** (`cmd=0x008`): position (±12.5 rad, 16-bit), speed (±65 rad/s, 12-bit), KP (0–500), KD (0–5), torque (±50 Nm, 12-bit)
- Feedback frames: Heartbeat (0x001), MIT FB (0x008), Encoder Estimates (0x009), Torques (0x01C), Powers (0x01D)
- State protected by `data_mutex_`

### RS05 (Axes 4–6) — Extended 29-bit frames
- **Arbitration ID encoding**: `(comm_type << 24) | (torque_int << 8) | motor_id`
- **MIT Control** (`type=0x01`): position (±4π rad), speed (±50 rad/s), KP (0–500), KD (0–5), torque (±5.5 Nm) — all 16-bit
- Feedback type `0x02`: position, velocity, torque, temperature, mode, fault
- State protected by `data_mutex_`

---

## ros2_control Interface

Defined in [r0192_description/urdf/r0192_ros2_control.xacro](src/r0192_description/urdf/r0192_ros2_control.xacro).  
Hardware plugin: `r0192_hardware/R0192SystemHardware` (registered via `r0192_hardware_plugin.xml`).

Each joint exposes:
- **Command interfaces**: `position`, `velocity`, `effort`, `kp`, `kd`
- **State interfaces**: `position`, `velocity`, `effort`

The background thread `canRxThread()` receives CAN frames asynchronously without blocking the 100 Hz read/write cycle. CAN init failure (e.g. `can0` not up) causes `on_configure` to log a warning and continue in full virtual mode (no error return).

Per-axis presence flags (`axis1_present_`, `axis4_present_`) are set during `on_configure` via `probePresent(200)` — see "Axis presence detection" in Architecture Overview. All motor operations (enable, write, stop) are gated on these flags; unplugged axes automatically fall back to passthrough.

---

## Joint Limits (URDF)

| Joint   | Type      | Axis | Range                | Effort  | Velocity  |
|---------|-----------|------|----------------------|---------|-----------|
| joint_1 | revolute  | Z    | [−π, +π]             | 30 N·m  | 10 rad/s  |
| joint_2 | revolute  | Y    | [−π/2, +π/2]         | 30 N·m  | 10 rad/s  |
| joint_3 | revolute  | Y    | [−π/4, +5π/4]        | 30 N·m  | 10 rad/s  |
| joint_4 | revolute  | X    | [−π, +π]             | 30 N·m  | 10 rad/s  |
| joint_5 | revolute  | Y    | [−π, +π]             | 30 N·m  | 10 rad/s  |
| joint_6 | revolute  | X    | [−π, +π]             | 30 N·m  | 10 rad/s  |
| joint_7 | prismatic | Y    | [0, 0.025 m]         | 30 N    | 10 m/s    |
| joint_8 | prismatic | Y    | mimic joint_7 (×−1)  | —       | —         |

**All values above are placeholders.** Real joint ranges, effort limits, velocity limits, inertias, and center-of-mass positions will be replaced with CAD-derived values later.

---

## Build & Run

### Build
```bash
cd ~/roboticarm_r0192_ws
colcon build --symlink-install
source install/setup.bash
```

### CAN interface setup (run once per boot, before launch)
```bash
sudo ip link set can0 up type can bitrate 1000000
```

### Launch (real robot — startet alles)
```bash
ros2 launch r0192_bringup real_robot.launch.py
```
Startreihenfolge: robot_state_publisher → controller_manager → controller-spawner (joint_state_broadcaster, arm_controller, gripper_controller) → MoveIt + RViz (nach 3 s Verzögerung).

```bash
# Debug-Modus: MoveIt + RViz (Standard, für Planung lokal)
ros2 launch r0192_bringup real_robot.launch.py use_rviz:=true

# Monitoring-Modus: Foxglove Bridge statt RViz (kein RViz-Overhead)
ros2 launch r0192_bringup real_robot.launch.py use_rviz:=false use_foxglove:=true

# Beides gleichzeitig (z. B. Debuggen + Foxglove monitoring)
ros2 launch r0192_bringup real_robot.launch.py use_rviz:=true use_foxglove:=true
```

### Launch (simulation with Gazebo)
```bash
ros2 launch r0192_bringup simulated_robot.launch.py
```

### Display only (RViz, kein Hardware-Interface)
```bash
ros2 launch r0192_description display.launch.py
```

### MoveIt only (ohne Controller Manager)
```bash
ros2 launch r0192_moveit moveit.launch.py is_sim:=false
```

---

## MoveIt 2 Configuration

- **IK solver**: KDL (`kinematics.yaml`) — TRAC-IK als robustere Alternative wenn nötig
- **Planning groups**: `r0192_arm` (joints 1–6), `gripper` (joint 7)
- **Home position**: all joints at 0
- **Planners**: default RRTConnect via OMPL; Pilz für Cartesian paths
- **Controllers**: `arm_controller` und `gripper_controller` als FollowJointTrajectory-Actions
- **Velocity/acceleration scaling**: 0.1 (konservativ — erst erhöhen wenn URDF-Limits real sind)
- **xacro mappings** in `moveit.launch.py`: `is_sim=false, is_ignition=false` (konsistent mit controller)

---

## Operator Interface

### Geplantes Web-Interface (r0192_remote — Produktivbetrieb)

Das zukünftige **Bedienpanel** ist ein eigenes Web-Interface / Programm, das auf dem MacBook (oder beliebigem Browser) läuft und den Roboterarm vollständig steuert. Es kommuniziert direkt mit dem RPi (ROS 2 Topics/Services über HTTP/WebSocket). Das Paket `r0192_remote` ist dafür vorgesehen.

Geplante Bedienfunktionen und ihr ROS-Interface:

| Funktion | ROS-Interface | Typ |
|----------|--------------|-----|
| Notaus | `/e_stop` | `std_msgs/Bool` (Topic, latched) |
| Motoren aktivieren | `/robot_enable` | `std_srvs/SetBool` (Service) |
| Homing auslösen | `/homing` | `std_srvs/Trigger` (Service) |
| Arm-Status | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| Trajektorie visualisieren | `/display_planned_path` | `moveit_msgs/DisplayTrajectory` |

### Foxglove Studio (Debug-Tool)

Foxglove dient ausschließlich als **Debug-Werkzeug** (Topic-Inspektion, Waveform-Ansicht, TF-Visualisierung) — **kein Produktivbetrieb**. Die `foxglove_bridge` läuft optional via Launch-Argument.

```bash
# Debug mit Foxglove (zusätzlich zu oder statt RViz)
ros2 launch r0192_bringup real_robot.launch.py use_foxglove:=true

# Nur RViz (Standard-Debug)
ros2 launch r0192_bringup real_robot.launch.py use_rviz:=true
```

Foxglove Bridge Setup (RPi):
```bash
sudo apt install ros-jazzy-foxglove-bridge
```
Node in Launch: `package='foxglove_bridge'`, `executable='foxglove_bridge'`, `port: 8765`  
Verbindung in Foxglove Studio: **Foxglove WebSocket** → `ws://<rpi-ip>:8765`

**Hinweis**: Die native `foxglove_bridge` ist wesentlich performanter als `rosbridge_suite`. Neuere Bridge-Versionen verwenden das `foxglove.sdk.v1` Protokoll — Foxglove Studio aktuell halten.

### Relevante Debug-Topics

| Topic | Inhalt |
|-------|--------|
| `/joint_states` | Aktuelle Gelenkpositionen |
| `/tf`, `/tf_static` | Koordinatensystem-Baum |
| `/robot_description` | URDF (für 3D-Render) |
| `/display_planned_path` | Geplante MoveIt-Trajektorie |
| `/diagnostics` | System-Diagnose |

---

## Homing-Architektur (Arduino + TLE4905L)

### Konzept

Jede Achse hat einen **eigenen Arduino Uno R3** als dedizierter Homing-Node. Der Arduino verbindet sich über einen **MCP2515 SPI-CAN-Transceiver** mit dem CAN-Bus (1 Mbit/s, 8 MHz Quarz am MCP2515). Ein **TLE4905L** Hall-Effekt-Sensor detektiert einen an der rotierenden Achse befestigten Magneten.

Aktueller Stand: **Achse 1 vollständig implementiert** — Arduino-Firmware (`microcontroller/r0192_homing.ino`) und ROS-seitiger Service (in `r0192_hardware`). Achsen 2–6 folgen mit identischer Firmware (nur CAN-IDs anpassen).

### CAN-Protokoll (Pi ↔ Arduino)

| Richtung | CAN-ID | DLC | Data | Bedeutung |
|----------|--------|-----|------|-----------|
| Pi → Arduino | `0x100` | 1 | axis_id | Homing-Befehl: Sensor scharf stellen |
| Arduino → Pi | `0x000` | 1 | `0xFF` | Magnet erkannt: Achse stoppen |

Arduino geht nach Senden der Bestätigung in **Standby** zurück (wartet auf nächsten `0x100`).

### Homing-Ablauf (zweiseitiger Bisektionsalgorithmus)

1. **Pass 1 (vorwärts)**: Pi aktiviert Homing-Modus (`0x100` → Arduino), sendet dann kontinuierlich Bewegungsbefehl in eine Richtung. Arduino überwacht TLE4905L (LOW = Magnet). Bei Detektion: Arduino sendet `0x00 / 0xFF` → Pi stoppt Achse. Position P1 merken.
2. **Pass 2 (rückwärts)**: Gleicher Ablauf in Gegenrichtung. Pi schickt erneut `0x100`, bewegt Achse zurück bis Magnet wieder detektiert. Position P2 merken.
3. **Mitte berechnen**: Magnet-Mittelpunkt = `(P1 + P2) / 2` → Achse auf Mittelpunkt fahren.
4. **Zero setzen**: Achse auf gewünschte Nullposition fahren (ggf. mit konfiguriertem Offset zum Magnetmittelpunkt) und Encoder-Nullpunkt im Hardware Interface setzen.

### ROS-seitiger Homing-Service (in `r0192_hardware`)

Der `/homing`-Service ist **direkt im Hardware-Interface-Plugin** eingebettet, nicht als separater Node. Beim `on_activate()` wird automatisch ein Sub-Node `r0192_homing` mit dem Service erstellt.

- Service: `/homing` (`std_srvs/Trigger`) — verfügbar solange das Hardware-Interface aktiviert ist
- Während Homing: `homing_active_`-Flag sperrt `write()` für Achse 1 → kein Konflikt mit arm_controller
- Arduino-ACK wird vom bestehenden `canRxThread()` erkannt und per `arduino_ack_`-Atomic weitergegeben
- Nach Homing: `hw_positions_[joint_1]` und `hw_cmd_positions_[joint_1]` werden auf 0 synchronisiert

Parameter zur Laufzeit änderbar via `ros2 param set /r0192_homing <name> <value>`:

| Parameter | Default | Bedeutung |
|-----------|---------|-----------|
| `homing_vel` | 0.15 | Suchgeschwindigkeit (rad/s) |
| `homing_kd` | 2.0 | Velocity-Gain in MIT_Control (KD, KP=0) |
| `hold_kp` | 30.0 | Positions-Gain nach Kantenerkennung |
| `hold_kd` | 1.0 | Dämpfungs-Gain nach Kantenerkennung |
| `zero_offset` | 0.0 | Offset vom Magnetmittelpunkt zur Nullposition (rad) |
| `homing_timeout` | 60.0 | Max. Sekunden pro Sweep-Richtung |

**Bekannte Einschränkung**: Nach Abschluss des Homings kann der arm_controller versuchen, die Achse auf eine vorherige Zielposition zurückzufahren (interne Trajektorie ist nicht synchronisiert). Für jetzt akzeptiert — später durch Controller-Deaktivierung während Homing beheben.

### Arduino-Firmware Details

- Bibliothek: `mcp2515` (autowp)
- CAN-Bitrate: `CAN_1000KBPS`, MCP_8MHZ
- Hall-Sensor-Pin: Digital 3 (INPUT_PULLUP), LOW = Magnet erkannt
- CS-Pin MCP2515: Digital 10
- Debug-Modus über `DEBUG_MODE`-Flag in der Firmware deaktivierbar
- Für Achsen 2–6: `MSG_SLAVE_ID` (jetzt `0x100`) auf achsenspezifische ID anpassen

---

## Key Open Tasks

**Aktueller Teststand (2026-05): Achsen 1 + 4 vollständig getestet — CAN-Feedback, Positions-Tracking und MoveIt-Planung funktionieren auf beiden Achsen.**

- [x] Motor-Test: Achse 1 (GDS68) über MoveIt — MIT_Control, CAN-Feedback, Positions-Tracking OK
- [x] Motor-Test: Achse 4 (RS05) über MoveIt — MIT_Control, CAN-Feedback, Positions-Tracking OK
- [x] Encoder-Homing (on_activate) validiert: Hardware Interface setzt beim Start den internen Nullpunkt
- [x] Achsen-Erkennung per CAN-Probe: `probePresent(200ms)` in `on_configure`, automatisches Passthrough für nicht angeschlossene Achsen
- [x] `foxglove_bridge` in `real_robot.launch.py` integriert (nur als Debug-Tool, `use_foxglove` Launch-Arg, Port 8765)

**Next: Homing-System (Arduino-basiert)**
- [x] Arduino-Firmware für Achse 1 implementiert (`microcontroller/r0192_homing.ino`)
- [x] ROS 2 Homing-Service `/homing` implementiert (eingebettet in `r0192_hardware`, `std_srvs/Trigger`)
- [x] Zweiseitiger Bisektions-Algorithmus implementiert (P1 + P2 → Mittelpunkt → Zero setzen)
- [x] `write()` wird während Homing gesperrt (`homing_active_` Flag), kein CAN-Konflikt mit ros2_control
- [ ] Hardware-Test: Arduino-Homing auf Achse 1 end-to-end validieren (benötigt Arduino + Magnet an Achse)
- [ ] Arduino-Firmware auf Achsen 2–6 erweitern (achsenspezifische `MSG_SLAVE_ID`)

**Next: Web-Interface (r0192_remote)**
- [ ] r0192_remote Paket aufbauen: Web-basiertes Bedienpanel als Operator-Interface
- [ ] ROS-Services für E-Stop, Motor-Enable, Homing implementieren (Backend für Web-Interface)
- [ ] Notaus-Implementierung (global software E-Stop)

**Wenn weitere Motoren gekauft sind (Achsen 2, 3, 5, 6, Greifer):**
- [ ] Passthrough durch echte Treiber-Instanzen ersetzen (GDS68 für 2+3, RS05 für 5+6)
- [ ] Greifer-Controller mit physischer Hardware verbinden
- [ ] URDF-Platzhalter durch CAD-Werte ersetzen (Limits, Trägheit, CoM)

**Später / System-Ebene:**
- [ ] RT-Kernel / CPU-Shielding auf RPi 5 evaluieren
- [x] CAN-Bitrate auf 1 Mbit/s erhöhen (beide Treiber konfigurieren)
- [ ] `on_init` Deprecation-Warnung: Migration auf `HardwareComponentInterfaceParams` API

---

## Known Issues & Gotchas

- **joint_7 URDF-Limit falsch**: `lower="0.15" upper="0.0"` — lower > upper ist ungültig. MoveIt kann Greifer-Planung ablehnen. Korrigieren auf `lower="0.0" upper="0.025"` (oder echte CAD-Werte).
- **MIT Control KP/KD**: Werden jetzt aus `<param name="kp">` / `<param name="kd">` in `r0192_ros2_control.xacro` geladen (joint_1: KP=50/KD=1, joint_4: KP=30/KD=0.5). Werte dort anpassen — kein Rebuild nötig (symlink-install), nur neu starten.
- **GDS68 Feedback-Routing in canRxThread**: `(std_id >> 5) == 0x01` matched nur Achse 1. Wenn Achsen 2 und 3 später hinzukommen, Routing erweitern.
- **Deprecation-Warnung**: `on_init(const HardwareInfo&)` ist in Jazzy deprecated. Funktioniert noch, Migration auf `HardwareComponentInterfaceParams` ist ausstehend.
- **Simulation Launch**: `simulated_robot.launch.py` referenziert `r0192_remote` (noch nicht vorhanden) — beim Simulationsstart auskommentiert lassen.
- **KDL IK**: Kann an Singularitäten scheitern. Alternativ: TRAC-IK oder pick_ik.
- **libserial**: Taucht in alten Paketabhängigkeiten auf, wird nicht genutzt. Kann entfernt werden.
- **JTC open_loop_control**: In `r0192_controllers.yaml` muss `open_loop_control: false` bleiben, da die interne Euler-Integration des Controllers bei den schnellen Pilz-PTP-Splines stark abweicht und zu `PATH_TOLERANCE_VIOLATED` führt.
- **Segfault bei Shutdown**: `move_group` und `rviz2` produzieren gelegentlich einen Segfault am Ende (Teardown). Dies ist ein bekannter Upstream-Bug im Lifecycle-Management von ROS 2 und hat keine Auswirkungen auf den Betrieb.
