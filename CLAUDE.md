
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
├── r0192_bringup/          # Central launch entry points (real + simulated)
├── r0192_canbus/           # CAN drivers: CanCommunication, GDS68Driver, RS05Driver
├── r0192_controller/       # JointTrajectoryController config + manual slider node
├── r0192_description/      # URDF/xacro, STL meshes, Gazebo/Rviz configs
├── r0192_interfaces/       # Custom msgs/srvs/actions: RobotState, SetRobotState, ExecuteProgram
├── r0192_hardware/         # ros2_control hardware interface + robot_state_manager node
├── r0192_moveit/           # MoveIt 2 config: SRDF, kinematics, joint limits, launch
├── r0192_program_executor/ # Program backend: YAML loader + /execute_program action server
├── r0192_rviz_plugins/     # RViz operator panel (Homing- + Motoren-EIN/AUS-Buttons)
└── r0192_remote/           # (Planned) Web-based remote control interface

programs/                   # Robot programs (YAML): points.yaml + program_*.yaml
doku/schemas/               # JSON Schemas for programs/points (VS Code + docs source of truth)
.vscode/                    # Committed engineering config: YAML schema mapping, snippets
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
- `joint_1` (GDS68, axis 1): real CAN feedback from motor. Mode set to Position Control (3) + Passthrough (1) via `Set_Controller_Mode(3,1)` on activate; commands sent via `MIT_Control`. **Position/velocity feedback comes exclusively from the periodic Encoder_Estimates frame (0x009 @ `encoder_rate_ms`, default 10 ms = 100 Hz)** — so feedback streams continuously even when the motor is idle/disabled (no MIT command needed). The MIT response (0x008) is no longer used for pos/vel (only best-effort torque). **Enable `encoder_rate_ms = 10` in the SteadyWin tool and `Save_Configuration` (0x01F)**, otherwise no 0x009 frames arrive and joint_1 reports nothing.
- `joint_4` (RS05, axis 4): real CAN feedback from motor. Commands sent via `MIT_Control`. **While enabled**, active reporting (Type 24, `0x18`) is turned on over CAN via `Actively_Reports_Frame(1.0f)` (in `on_activate()` and re-sent on every `/robot_enable`-enable) → the RS05 streams its Type-2 feedback frame @ 10 ms (100 Hz). No tool required (set purely over CAN); not persisted to flash. The Type-2 frame is parsed identically whether it arrives as a MIT response or an active report (same `case 0x02`), so there is no GDS68-style scale-mixing issue. **While disabled**, the RS05 has **no ODrive-style idle**: `Motor_Stop_Running` puts it in Reset mode where it streams nothing (motor + active reporting both off — "off is off"). So `read()` falls back to **polling `mechPos` (`0x7019`) + `mechVel` (`0x701B`) via Type-17 single-parameter reads** (read-only, does NOT energize the motor) at ~20 Hz, so joint_4 still tracks when back-driven by hand — symmetric to joint_1. **Hardware-to-verify**: (a) Type-17 index endianness (little-endian assumed; if no reply, flip to big-endian), (b) `mechPos`/`mechVel` scale matches the decoded Type-2 feedback (incl. mechanical-zero offset).
- `joint_2/3/5/6/7`: **passthrough** — last commanded value is reported as state (zero error, no motor needed)
- `joint_8`: derived from `joint_7` as mimic (multiplier −1)

**Axis presence detection:** At `on_configure`, each physical axis (1 and 4) is probed over CAN with a 200 ms timeout:
- GDS68 (axis 1): sends `Get_Encoder_Estimates()` (CMD 0x009), waits for any standard frame from `node_id`
- RS05 (axis 4): sends `Get_Device_ID()` (comm_type 0x00), waits for any extended frame with `sender_id == node_id` (bits 8–15)

**RS05 extended-frame caveat**: `Get_Device_ID()` uses comm_type `0x00`, making the CAN ID equal to just `node_id` (e.g. `0x04`). Since `0x04 < 0x7FF`, `sendFrame()` would normally send it as an 11-bit standard frame, which the RS05 ignores. `Get_Device_ID()` therefore passes `force_extended=true` to `sendFrame()`. The same issue applies to any RS05 command with comm_type `0x00`; all other comm_types produce IDs > `0x7FF` and are unaffected.

Result stored in `axis1_present_` / `axis4_present_`. Axes that don't respond are silently treated as virtual (passthrough). Initialization, CAN sends, and stop commands are gated on these flags.

CAN bitrate: **1 Mbit/s** (bus + both drivers configured; GDS68 factory default was 500 kbit/s). Set per boot via `ip link set can0 up type can bitrate 1000000`; the Arduino homing nodes run `CAN_1000KBPS` to match.

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

### Start-Sicherheits-Check: Encoder innerhalb der Achsenlimits

In `on_activate()` werden die Motoren zuerst aktiviert und Encoder-Feedback angefordert; **bevor** die Achsen genullt werden, prüft `encodersWithinLimits()`, ob jede **physisch vorhandene** Achse laut **RAW-Encoder** (vor jeglichem Nullen, `home_offset_`/Mechanical-Zero noch nicht gesetzt) innerhalb ihrer URDF-Positionslimits steht. Die Limits kommen aus `info_.limits` (von ros2_control aus den `<limit lower/upper>`-Tags in [r0192.urdf.xacro](src/r0192_description/urdf/r0192.urdf.xacro) geparst, keyed nach Joint-Name). Virtuelle/passthrough-Achsen haben keinen Encoder und werden übersprungen (starten ohnehin bei 0).

Liegt eine Achse außerhalb `[min_position, max_position]`, loggt der Check einen `RCLCPP_ERROR` mit Joint-Name, Ist-Position und Limits, stoppt Motoren + RX-Thread (`stopMotorsAndRx()`) und gibt `CallbackReturn::ERROR` zurück → die Komponente erreicht **nicht** den Active-State, der Arm startet nicht. `stopMotorsAndRx()` wird von `on_deactivate()` und diesem Fehlerpfad gemeinsam genutzt.

**Hinweis (GDS68 Mehrumdrehungs-Position)**: joint_1 liefert eine kontinuierliche Position (Magnet bei `≈ −1.87 ± k·2π`). Steht die Achse in einer höheren Umdrehung (z. B. +4.43 rad), schlägt der Check gegen das ±π-Limit zu Recht an — die Achse ist physisch außerhalb des erlaubten Single-Turn-Bereichs.

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
| joint_7 | prismatic | Y    | [0, 0.0025 m]        | 30 N    | 10 m/s    |
| joint_8 | prismatic | Y    | mimic joint_7 (×−1)  | —       | —         |

**All values above are placeholders.** Real joint ranges, effort limits, velocity limits, inertias, and center-of-mass positions will be replaced with CAD-derived values later.

**Geometrie ist 1:1 (real skaliert)**: Die URDF-Geometrie wurde von der ~10× zu großen Platzhalter-Skala auf reale Größe gebracht — Mesh-`scale` über die Property `mesh_scale="0.001 0.001 0.001"` (STL in mm → m), alle Gelenkursprünge in echten Metern (joint_1 @ 0.1751 m usw.), Greifer-Hub `[0, 0.0025 m]`. Dadurch zeigen RViz/Jog-Panel reale mm und die MoveIt-Servo-Singularitätsschwellen passen mit den Defaults. **Gelenk-Winkel-Limits, Effort, Trägheit, CoM bleiben Platzhalter.**

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

## Betriebszustands-Maschine (Robot State Manager)

Der **`robot_state_manager`**-Node (Executable in `r0192_hardware`, [robot_state_manager.cpp](src/r0192_hardware/src/robot_state_manager.cpp)) ist die **zentrale, autoritative Zustandsmaschine** des Arms (Single Source of Truth). Vorher war der Betriebszustand implizit über drei verstreute Flags verteilt (`motors_enabled_` im Hardware-Interface, `homing_->isActive()`, Servo-Pause im JogPanel) — ohne erzwungene gegenseitige Ausschließung. Der Manager bündelt das in **5 sich gegenseitig ausschließende Zustände** und koordiniert die bestehenden Services.

| State | `/robot_enable` | `arm_controller` | Servo (`pause_servo`) | Bedeutung |
|-------|-----------------|------------------|-----------------------|-----------|
| `DISABLED` (0) | aus | inaktiv | pausiert | Idle, drehmomentfrei (Startzustand) |
| `HOLD` (1) | an | **inaktiv** | pausiert | Servos an, Hardware hält letzte Soll-Pos |
| `JOG` (2) | an | aktiv | **aktiv** | Teach-Pendant über MoveIt Servo |
| `MOVEIT` (3) | an | aktiv | pausiert | move_group darf Trajektorien ausführen |
| `HOMING` (4) | an | (Homing managed) | pausiert | Homing-Sequenz läuft |

**Harte Gatterung über den `arm_controller`**: In `HOLD` ist der `arm_controller` **deaktiviert**. Die Hardware hält die Position trotzdem, weil `write()` bei `motors_enabled_` jeden Zyklus `MIT_Control(hw_cmd_positions_, …)` mit dem letzten Sollwert sendet — unabhängig von aktiven Controllern. Erst das Aktivieren des `arm_controller` (`MOVEIT`/`JOG`) öffnet den Pfad: in `HOLD` ist der FollowJointTrajectory-Action-Server unten, MoveIt kann **nichts** bewegen. Beim Reaktivieren liest der JTC den aktuellen Zustand und hält dort (kein Snap). Gleiches Muster wie `HomingController::setArmControllerActive()`.

**Erlaubte Übergänge** (alles andere wird mit klarer Meldung abgelehnt):
```
DISABLED ⇄ HOLD            (Servos an/aus)
HOLD → JOG    → HOLD
HOLD → MOVEIT → HOLD
HOLD → HOMING → HOLD       (HOMING→HOLD automatisch nach Service-Abschluss)
```
`JOG`/`MOVEIT`/`HOMING` sind **nur aus `HOLD`** erreichbar (Servos müssen an sein). Während `HOMING` (oder einem laufenden Übergang) werden neue Anfragen mit „busy" abgelehnt. `requested_state == current_state` ist ein No-op-Erfolg.

**Aktionen pro Übergang** (Manager ruft die bestehenden Services): `/robot_enable` (SetBool), `/controller_manager/switch_controller` (arm_controller aktiv/inaktiv), `/servo_node/pause_servo` (SetBool), `/homing` (Trigger, **asynchron** — die Service-Antwort kehrt sofort mit `HOMING` zurück, ein Worker-Thread setzt nach Abschluss `HOLD`). Der `HomingController` reaktiviert am Ende den `arm_controller`; da `HOLD` ihn inaktiv erwartet, deaktiviert der Manager ihn danach erneut.

**Startup-Reconcile**: Der Controller-Spawner startet `arm_controller` aktiv, die Hardware aber drehmomentfrei. Der Manager initialisiert auf `DISABLED` und deaktiviert `arm_controller` einmalig beim Start (tolerant, falls der Controller-Manager noch nicht hochgefahren ist).

**ROS-Schnittstelle** (Custom-Interfaces im Paket [r0192_interfaces](src/r0192_interfaces)):
- `/robot_state` — `r0192_interfaces/RobotState` (latched/`transient_local`): `uint8 state` (Konstanten `DISABLED`/`HOLD`/`JOG`/`MOVEIT`/`HOMING`) + `string status`. Publiziert bei jedem Wechsel.
- `/set_robot_state` — `r0192_interfaces/SetRobotState`: `uint8 requested_state` → `bool success, string message, uint8 current_state`.
- `/e_stop` — `std_srvs/Trigger`: **Notaus**, erzwingt `DISABLED` aus **jedem** Zustand (anders als der normale `DISABLED←HOLD`-Übergang). Immer verfügbar.
- `/robot_reset` — `std_srvs/Trigger`: **Reset nach Notaus** — löst die gelatchten Treiber-Fehler (siehe unten). Nötig, bevor `DISABLED→HOLD` wieder funktioniert.

**Notaus (`/e_stop`) + Treiber-Reset (`/robot_reset`)**: Der Notaus schneidet das Drehmoment **auf Treiber-Ebene** über den neuen Hardware-Service `/robot_estop` (nicht nur per `write()`-Gate): GDS68 `Estop()` (CMD 0x002) latcht die Achse in einen ESTOP-Fehlerzustand, RS05 `Motor_Stop_Running()`. Danach pausiert der Manager Servo, deaktiviert den `arm_controller` und setzt `state = DISABLED`. Das Drehmoment-Kappen läuft in `handleEStop()` **vor** dem Mutex (Sicherheit zuerst). Läuft gerade ein Homing (`busy_`), wird das Homing per `estop_`-Flag angewiesen, in `DISABLED` statt `HOLD` zu landen; das Drehmoment ist bereits weg.

**Treiber sind nach dem Notaus bewusst „gefangen"** (latched fault) — ein reines Re-Enable würde von den Treibern ignoriert. Recovery erfordert einen expliziten `/robot_reset`: der Manager ruft den Hardware-Service `/robot_clear_faults` (GDS68 `Clear_Errors()` 0x018 + zurück in Idle; RS05 `Motor_Stop_Running(clear_faults=true)`), löst das Latch und bleibt in `DISABLED`. Erst danach lässt der Manager (und auch das Hardware-Interface selbst, das `estop_latched_` führt) `DISABLED→HOLD` wieder zu. `DISABLED→HOLD` bei aktivem Latch wird mit „E-STOP latched — call /robot_reset" abgelehnt.

**v1-Grenze**: Ein laufender Homing-Sweep wird **nicht** physisch unterbrochen — der `HomingController` fährt Achse 1 direkt (an `motors_enabled_` vorbei). Das Drehmoment ist zwar gekappt, aber ein echter Hardware-Notaus (Homing-Abbruch-Hook + ggf. Schütz/Power-Cut) ist Folgearbeit.

**Nebenläufigkeit**: Die Executor-Thread-Callbacks (`handleSetState`/`handleEStop`/`handleReset`) sind durch den Single-Threaded-Spin serialisiert und teilen sich `client_node_`. Der **Homing-Worker läuft in einem eigenen Thread** und benutzt daher eine **separate `homing_node_`** mit eigenen Clients — sonst würden bei einem `/e_stop` während des Homings zwei Executoren dieselbe Node gleichzeitig spinnen, was rclcpp verbietet (war ein latenter Crash-Bug). Nicht-Homing-Übergänge sind kurz, daher ist die Serialisierung des Notaus dahinter unkritisch.

Der Node läuft in `real_robot.launch.py` mit, ist im virtuellen Modus voll testbar (fehlende Services wie `/homing` ohne Achse 1 → Übergang wird sauber abgelehnt; `/e_stop`/`/robot_reset` setzen dann nur das Torque-Gate) und ist für das geplante `r0192_remote`-Web-Interface wiederverwendbar.

---

## Programm-System (r0192_program_executor)

Industrielle Trennung **Engineering ↔ Operations** (Plan: [doku/program_ide_plan.md](doku/program_ide_plan.md); Phasen 0–2 fertig, Phase 3 RViz-Run-Panel als Nächstes):

- **Engineering (VS Code)**: Programme/Punkte als YAML in `programs/` (git-versioniert). Live-Validierung über JSON Schemas in `doku/schemas/` (`.vscode/settings.json`-Mapping, Extension `redhat.vscode-yaml`, Snippets `r0192-program`/`move_j`/`move_l`/`wait`/`point-joint`/`point-pose`). `.gitignore` whitelisted `.vscode/{settings,extensions,r0192.code-snippets}`.
- **Backend**: Node `r0192_program_executor` (in `real_robot.launch.py`), Action **`/execute_program`** (`r0192_interfaces/action/ExecuteProgram`, Goal = Dateipfad, relativ zu Parameter `programs_dir`, Default `~/roboticarm_r0192_ws/programs`). Loader ([program_loader.cpp](src/r0192_program_executor/src/program_loader.cpp)) ist die **einzige** Parse-/Validierstelle (strikte Meldungen, unbekannte Keys abgelehnt).
- **Datenmodell**: Punkte (`points.yaml`, Map by name, `type: joint` mit 6 rad-Werten joint_1..6 oder `type: pose` in `base_link`) ↔ Programme (`program_*.yaml`, Steps `move_j`/`move_l`/`wait`, Referenz **nur per Punktname**, `velocity`/`acceleration` = MoveIt-Skalierungsfaktoren (0, 1], optionale `defaults`, Fallback 0.1).
- **State-Integration**: Executor ist reiner `/set_robot_state`-Client — `HOLD → MOVEIT` vor dem ersten Step, `MOVEIT → HOLD` am Ende/Cancel/Fehler, aber **nur wenn der Zustand noch MOVEIT ist** (nach `/e_stop` → DISABLED fasst er nichts an; ein HOLD-Request hieße Motoren einschalten). Goal aus falschem Zustand → ABORTED mit Manager-Meldung.
- **Ausführung v1**: sequenziell via `MoveGroupInterface` (lazy erstellt — move_group startet 3 s nach dem Executor). `move_j` akzeptiert joint+pose-Punkte, `move_l` ist schema-gültig, wird aber bis Phase 6 beim Goal abgelehnt. Cancel → `MoveGroupInterface::stop()` bricht blockierendes plan/execute, Ergebnis CANCELED. Feedback: Step-Index/Label/Total + Status (LOADING/PLANNING/MOVING/WAITING).
- **Getestet (virtuell, 2026-06-11)**: 4-Step-Demo SUCCEEDED + zurück in HOLD; Cancel mid-program → CANCELED + HOLD; Goal aus DISABLED → ABORTED; ungültiges YAML → ABORTED mit präziser Meldung. **Hardware-Test steht aus.**

---

## Operator Interface

### Geplantes Web-Interface (r0192_remote — Produktivbetrieb)

Das zukünftige **Bedienpanel** ist ein eigenes Web-Interface / Programm, das auf dem MacBook (oder beliebigem Browser) läuft und den Roboterarm vollständig steuert. Es kommuniziert direkt mit dem RPi (ROS 2 Topics/Services über HTTP/WebSocket). Das Paket `r0192_remote` ist dafür vorgesehen.

Geplante Bedienfunktionen und ihr ROS-Interface:

| Funktion | ROS-Interface | Typ | Status |
|----------|--------------|-----|--------|
| Notaus | `/e_stop` | `std_srvs/Trigger` (Service) | **implementiert** (Robot State Manager → Hardware `/robot_estop`; latchender Treiber-Torque-Cut aus jedem Zustand). Echter HW-Power-Cut + Homing-Abbruch folgt |
| Reset nach Notaus | `/robot_reset` | `std_srvs/Trigger` (Service) | **implementiert** (Robot State Manager → Hardware `/robot_clear_faults`; löst Treiber-Latch) |
| Zustand setzen (DISABLED/HOLD/JOG/MOVEIT/HOMING) | `/set_robot_state` | `r0192_interfaces/SetRobotState` (Service) | **implementiert** (Robot State Manager) |
| Aktueller Zustand | `/robot_state` | `r0192_interfaces/RobotState` (Topic, latched) | **implementiert** (Robot State Manager) |
| Motoren aktivieren | `/robot_enable` | `std_srvs/SetBool` (Service) | **implementiert** (in `r0192_hardware`; vom State Manager genutzt) |
| Notaus (Treiber-Ebene) | `/robot_estop` | `std_srvs/Trigger` (Service) | **implementiert** (in `r0192_hardware`; latchender Treiber-Stop, vom State Manager genutzt) |
| Treiber-Fault-Reset | `/robot_clear_faults` | `std_srvs/Trigger` (Service) | **implementiert** (in `r0192_hardware`; Clear_Errors / RS05-Fault-Clear, vom State Manager genutzt) |
| Homing auslösen | `/homing` | `std_srvs/Trigger` (Service) | **implementiert** (in `r0192_hardware`; vom State Manager genutzt) |
| Arm-Status | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | geplant |
| Trajektorie visualisieren | `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | vorhanden (MoveIt) |

> **Empfehlung:** Neue Clients (RViz-Panel, Web-Interface) steuern Betriebszustände **ausschließlich** über `/set_robot_state` und spiegeln `/robot_state`, nicht direkt über `/robot_enable` / `/homing` / `pause_servo` — sonst wird die zentrale Zustands-Erzwingung umgangen.

### RViz Operator-Panel (`r0192_rviz_plugins`)

> **Hinweis (2026-06):** Die Bedienelemente des `OperatorPanel` (Homing, Servo-Enable) sind in das kombinierte **`JogPanel`** gewandert (siehe unten) und in [moveit.rviz](src/r0192_moveit/config/moveit.rviz) zeigt nur noch das JogPanel („R0192 Control"). Die `OperatorPanel`-Klasse bleibt registriert (manuell via **Panels → Add New Panel** ladbar), wird aber nicht mehr im Default-Layout angezeigt. Der folgende Abschnitt beschreibt weiterhin die identische Service-Semantik.

Debug-/Bequemlichkeits-Panel direkt in RViz (kein Ersatz für das geplante `r0192_remote`-Web-Interface). Das Plugin `r0192_rviz_plugins/OperatorPanel` ([operator_panel.cpp](src/r0192_rviz_plugins/src/operator_panel.cpp)) bietet einen Button, einen Toggle + Status-Label:

- **Homing** → ruft `/homing` (`std_srvs/Trigger`) asynchron auf. **Nach Erfolg** löst das Panel automatisch einen RViz-Display-Reset aus (siehe unten), damit MoveIts Planungs-Startzustand auf die frisch gehomte Pose nachzieht.
- **Enable-Toggle** (checkable Button) → ruft `/robot_enable` (`std_srvs/SetBool`): aktiviert = Motoren an (`data=true`, grün), deaktiviert = Motoren aus (`data=false`, rot). Ist der Service nicht erreichbar, springt der Toggle in den vorherigen Zustand zurück.

**Auto-Reset nach Homing**: Nach dem Re-Zeroing zeigt MoveIts MotionPlanning-Display sonst noch den alten (vor dem Homing gesetzten) Query-/Startzustand, was zu ungewollten Planungszuständen führt. Manuell behebt das der **Reset**-Button in RViz. Das Panel macht das jetzt automatisch: ~500 ms nach erfolgreichem `/homing` (Zeit, damit die genullten `/joint_states` MoveIts Planning-Scene-Monitor erreichen) ruft `resetDisplays()` `getDisplayContext()->getRootDisplayGroup()->reset()` auf — derselbe rekursive Display-Reset, den `VisualizationManager::resetTime()` (der Reset-Button) auslöst. Läuft auf dem RViz-GUI-Thread (Service-Callback), daher Qt/Ogre-sicher.

Das Panel ist in [moveit.rviz](src/r0192_moveit/config/moveit.rviz) registriert und erscheint daher beim `real_robot.launch.py`-Start automatisch (sonst manuell via **Panels → Add New Panel → r0192_rviz_plugins/OperatorPanel**). Die Service-Clients hängen am RViz-Node; Aufrufe sind asynchron (GUI blockiert nie). Sind die Services nicht da (Hardware nicht aktiv / Achse 1 fehlt für `/homing`), zeigt das Status-Label das rot an.

**`/robot_enable`-Semantik** (siehe `setMotorsEnabled()` in [r0192_hardware_interface.cpp](src/r0192_hardware/src/r0192_hardware_interface.cpp)): `false` setzt `motors_enabled_` und schaltet die vorhandenen Achsen in Idle/Stop → `write()` sendet kein MIT_Control mehr (Drehmoment weg). `true` schaltet wieder Closed-Loop und setzt das Soll auf die aktuelle Ist-Position. **Caveat**: Bei aktivem `arm_controller` überschreibt der JTC das Soll im nächsten Zyklus; driftet eine Achse im deaktivierten Zustand (Schwerkraft), kann es beim Re-Enable einen Ruck geben — Achse 1 (vertikale Drehachse) ist praktisch nicht betroffen.

**Start drehmomentfrei**: Beim Hochfahren bestromt `on_activate()` die physischen Achsen nur kurz für den Safety-Check (`encodersWithinLimits()`) und das Zeroing; **danach** werden die Achsen wieder in Idle/Stop geschaltet und `motors_enabled_ = false` gesetzt. Der Arm startet also **drehmomentfrei** — der Operator schaltet die Motoren bewusst über den Enable-Toggle im RViz-Operator-Panel (`/robot_enable` mit `data=true`) scharf. Bis dahin sendet `write()` kein MIT_Control. (Virtueller Modus ohne CAN bleibt unberührt: dort ist `motors_enabled_` egal, da nichts gesendet wird.) Der Enable-Toggle im Panel startet daher auf „aus" (rot).

### RViz Jog-Panel / Teach-Pendant (`r0192_rviz_plugins/JogPanel`)

Teach-Pendant-artiges Verfahr-Panel ([jog_panel.cpp](src/r0192_rviz_plugins/src/jog_panel.cpp)), das den Arm in **drei Modi** über **MoveIt Servo** verfährt. Die UI ist **englisch** (einheitlich mit RViz). Dieselben **6× (− / +)-Tasten** werden je Modus umbeschriftet:

| Modus (UI-Label) | Tasten-Bedeutung | Servo-Schnittstelle |
|------------------|------------------|---------------------|
| **Joints** | Joint 1 … 6 direkt | `control_msgs/JointJog` auf `/servo_node/delta_joint_cmds` |
| **Cartesian (Base)** | X, Y, Z, RX, RY, RZ im `base_link`-Frame | `geometry_msgs/TwistStamped` (`frame_id=base_link`) auf `/servo_node/delta_twist_cmds` |
| **Tool** | X, Y, Z, RX, RY, RZ im TCP-Frame | `TwistStamped` (`frame_id=tcp`) auf demselben Topic |

Layout pro Zeile: **[−][+]** links (beide Tasten auf einer Seite), Name mittig, **Live-Wert** rechts. Die Live-Spalte (10-Hz-Timer `onValueTick()`) zeigt im Joints-Modus die aktuellen Gelenkwinkel in **°** (3 Nachkommastellen, aus `/joint_states`), im Cartesian/Tool-Modus die aktuelle **TCP-Pose im Basis-Frame** (X/Y/Z in **mm** mit 1 Nachkommastelle, RPY in °, via TF `base_link`→`tcp`). Das Panel hält dafür ein eigenes `/joint_states`-Abo und einen `tf2_ros::TransformListener`.

**Positions-Anzeige-Skalierung** (`kModelToRealScale = 1.0` in [jog_panel.cpp](src/r0192_rviz_plugins/src/jog_panel.cpp)): Die URDF ist jetzt **1:1** (Mesh-`scale="0.001"`, Gelenkursprünge in echten Metern), daher zeigt die mm-Anzeige direkt reale Werte. Der Faktor bleibt als benannte Konstante für den Fall einer künftigen Skalen-Diskrepanz erhalten.

**Zustands-Maschinen-Client**: Das JogPanel ist seit der Einführung des **Robot State Manager** (siehe „Betriebszustands-Maschine" oben) ein **reiner Client** der zentralen Zustandsmaschine. Es ruft Betriebszustände **nicht mehr direkt** (`/robot_enable`, `/homing`, `pause_servo`) auf, sondern fordert Übergänge über `/set_robot_state` an und spiegelt den autoritativen `/robot_state` (latched). Die UI ist **state-getrieben** (`updateUiForState()`): nur Buttons gültiger Übergänge sind freigegeben, und die Button-Zustände folgen dem **bestätigten** `/robot_state` (kein optimistisches Umschalten). Das Servo-Command-Streaming selbst (Command-Type + Press-and-Hold-Publishing) bleibt im Panel und läuft **nur im `JOG`-Zustand**.

Bedienelemente:
- **Modus-Radios** (Joints / Cartesian / Tool) → setzen den Servo-Command-Type via `/servo_node/switch_command_type` (`moveit_msgs/ServoCommandType`: `JOINT_JOG`/`TWIST`).
- **Geschwindigkeits-Slider** (1–100 %) → skaliert die Befehlsmagnitude. Servo läuft mit `command_in_type: "unitless"`, d. h. das Panel sendet Werte in [−1, 1]; die Max-Geschwindigkeit setzen `scale.linear/rotational/joint` in [servo.yaml](src/r0192_moveit/config/servo.yaml). Effektive Geschwindigkeit = Slider-% × scale. (Dies ist der „Schritt"-/Speed-Regler.)
- **Master-Toggle „Enable Jog"**: fordert `JOG` (an) bzw. `HOLD` (aus) über `/set_robot_state` an (der Manager macht intern `pause_servo` + `arm_controller`). Nur aus `HOLD`/`JOG` klickbar; die ±-Tasten sind nur im `JOG`-Zustand freigegeben.

**Notaus-Button** (`EMERGENCY STOP`, rot, ganz oben im Panel, **immer** klickbar): ruft `/e_stop` (`std_srvs/Trigger`) — die **einzige** Ausnahme von der „alles über `/set_robot_state`"-Regel, da der Notaus aus **jedem** Zustand sofort nach `DISABLED` zwingen muss. Schneidet das Drehmoment auf Treiber-Ebene (latchend). Die UI zieht danach über `/robot_state` auf `DISABLED` nach.

**Reset-Button** (`Reset (clear faults)`, direkt unter dem Notaus, **nur in `DISABLED` freigegeben**): ruft `/robot_reset` (`std_srvs/Trigger`) — löst die nach einem Notaus gelatchten Treiber-Fehler, sodass `DISABLED→HOLD` (Servos-Enable) wieder funktioniert. Ohne Reset bleiben die Treiber „gefangen" und der Enable-Toggle hat keine Wirkung.

**Operator-Bedienelemente** (unter „Enable Jog", alle über `/set_robot_state` geroutet):
- **Servos-Enable-Toggle** → `HOLD` (an, grün) / `DISABLED` (aus, rot); nur aus `DISABLED`/`HOLD` klickbar; startet auf „aus" (Arm startet drehmomentfrei).
- **MoveIt-Toggle** → `MOVEIT` (an) / `HOLD` (aus); nur aus `HOLD`/`MOVEIT` klickbar. Aktiviert den `arm_controller`, sodass move_group Trajektorien ausführen darf — in `HOLD` ist Ausführung **gesperrt**.
- **Homing** → fordert `HOMING` an (nur aus `HOLD`); Auto-Display-Reset (`resetDisplays()`) ~500 ms nach dem automatischen `HOMING→HOLD`-Übergang.
- **„Goal Marker"-Toggle** → blendet MoveIts Query-Goal-State aus/ein, indem das Panel die Property `Planning Request → Query Goal State` des MotionPlanning-Displays setzt (`findMotionPlanningDisplay()` → `subProp(...)->setValue(...)`). Praktisch beim Jogging: der orange Ziel-Marker liegt sonst über dem realen Arm-Modell und stört die Sicht auf die Ist-Pose. Default „shown" (wie in moveit.rviz).

**Singularität in Cartesian/Tool**: Servo bremst/stoppt bei echten Singularitäten mit `Very close to a singularity, emergency stop`. Schwellen in [servo.yaml](src/r0192_moveit/config/servo.yaml) stehen auf Default (`lower_singularity_threshold: 17`, `hard_stop_singularity_threshold: 30`) — passend, seit das Modell **real-skaliert** ist (~0,18 m Glieder; vorher waren sie wegen der 10× zu großen Platzhalter-Geometrie temporär auf 200/400 angehoben). **Joints-Modus ist nicht betroffen** (invertiert die Jacobi-Matrix nicht). Eine echte Handgelenk-Singularität bleibt bei joint_5 = 0 (joint_4 und joint_6 dann achsparallel um X) — dort vor Cartesian/Tool-Jogging joint_5 ein paar Grad wegfahren. Schwellen live tunebar: `ros2 param set /servo_node hard_stop_singularity_threshold <x>`.

**TCP-/Tool-Frame**: Der Tool-Modus verfährt im Frame `tcp` (in [r0192.urdf.xacro](src/r0192_description/urdf/r0192.urdf.xacro) als fester Kind-Frame von `gripper_base`, rpy `0 π/2 0`). Damit folgt er der ROS-Industrial-`tool0`-Konvention: **+Z = Anflug-/Stoßrichtung**, +Y = Greifer-Öffnungsachse. `gripper_base` selbst hat die Anflugrichtung auf **X** (deshalb der +90°-Dreh um Y, der Z auf X abbildet). `tcp` ist ein reiner TF-Frame (keine Visual/Collision) und nicht Teil der IK-Kette (Gruppe `r0192_arm` endet an `gripper_base`). Position sitzt vorerst auf `gripper_base` — mit echten CAD-Werten an den realen Fingerspitzen-TCP verschieben.

**Press-and-Hold**: Solange eine ±-Taste gedrückt ist, streamt ein QTimer (50 Hz) den Befehl; beim Loslassen wird ein Null-Befehl gesendet (sofortiger Stopp). 

**Koexistenz mit MoveIt**: Servo startet mit Command-Type `INVALID` und greift erst nach `switch_command_type` in `arm_controller` ein. Das Master-Toggle übernimmt beim Aktivieren die Kontrolle über den Controller; beim Deaktivieren (`pause_servo(true)`) gibt es ihn an `move_group` zurück. **Im Jog-Modus also nicht gleichzeitig MoveIt-Trajektorien ausführen.** Jogging erfordert eingeschaltete Motoren (Enable-Toggle), da Servo über den JTC → `write()` → `MIT_Control` läuft.

MoveIt Servo wird vom `servo_node` in [moveit.launch.py](src/r0192_moveit/launch/moveit.launch.py) gestartet (Launch-Arg `use_servo:=true`, Default an). Konfig in [servo.yaml](src/r0192_moveit/config/servo.yaml): Gruppe `r0192_arm`, Ausgabe als `trajectory_msgs/JointTrajectory` auf `/arm_controller/joint_trajectory`, Butterworth-Smoothing, Kollisions- + Singularitätsbremsung aktiv, `is_primary_planning_scene_monitor: false` (move_group besitzt die Primär-Scene). Das Panel ist in [moveit.rviz](src/r0192_moveit/config/moveit.rviz) registriert und erscheint beim Start automatisch.

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

Jede Achse hat einen **eigenen Arduino Uno R3 (später den Seeed Studio XIAO ESP32-S3)** als dedizierter Homing-Node. Der Arduino verbindet sich über einen **MCP2515 SPI-CAN-Transceiver** mit dem CAN-Bus (1 Mbit/s, 8 MHz Quarz am MCP2515). Ein **TLE4905L** Hall-Effekt-Sensor detektiert einen an der rotierenden Achse befestigten Magneten.

Aktueller Stand: **Achse 1 vollständig implementiert** — Arduino-Firmware (`microcontroller/r0192_homing.ino`) und ROS-seitiger Service (in `r0192_hardware`). Achsen 2–6 folgen mit identischer Firmware (nur CAN-IDs anpassen).

### CAN-Protokoll (Pi ↔ Arduino)

Beide Richtungen nutzen **dieselbe achsenspezifische CAN-ID** (Achse 1 = `0x100`, Achse 2 = `0x101`, … Achse 6 = `0x105`). Die Bedeutung steckt im **Daten-Byte `Data[0]`** (Message-Code), nicht in der ID. So spart man eine zweite ID pro Achse und der Bus bleibt übersichtlich. Da der Bus pro Achse nur zwei Teilnehmer hat (Pi + ein Arduino) und der Arduino seine eigenen TX-Frames nicht zurückliest, gibt es keine Verwechslung.

| Richtung | CAN-ID | DLC | `Data[0]` | Code | Bedeutung |
|----------|--------|-----|-----------|------|-----------|
| Pi → Arduino   | `0x100` | 1 | `0x01` | `CMD_ARM`     | Sensor scharf stellen (Überwachung starten) |
| Arduino → Pi   | `0x100` | 1 | `0xFF` | `RSP_DETECTED`| Magnet erkannt → Pi stoppt Achse |
| Arduino → Pi   | `0x100` | 1 | `0xEE` | `RSP_ERROR`   | Fehler/Timeout: kein Magnet innerhalb der Zeit gefunden |

Der Arduino reagiert **nur** auf Frames mit korrekter ID **und** `Data[0] == CMD_ARM`; alle anderen Frames (inkl. eventueller Antwort-Frames) werden ignoriert. Nach Senden von `RSP_DETECTED` oder `RSP_ERROR` geht der Arduino in **Standby** zurück (wartet auf nächstes `CMD_ARM`).

### Homing-Ablauf (zweiseitiger Kantenanlauf)

Der Magnet hat eine endliche Breite — der TLE4905L löst je nach Anfahrtrichtung an einer anderen Kante aus. Der wahre Mittelpunkt liegt zwischen beiden Schaltkanten. Statt nach der ersten Detektion in die Gegenrichtung zu sweepen, bis der Sensor _irgendwo_ wieder auslöst, wird gezielt nur ein kleines Stück über die erste Kante hinausgefahren und dann von der anderen Seite zurück angefahren. Da die ungefähre Lage nach Pass 1 bekannt ist, ist das deutlich schneller und genauer.

- **Vorbedingungen**: Zu Beginn wird `home_offset_` auf 0 zurückgesetzt (Sequenz läuft komplett in Roh-Koordinaten — sonst korrumpiert ein zweiter Homing-Lauf den Offset). Danach **Range-Check**: liegt die rohe Encoder-Position außerhalb `±max_start_angle` (Default ±π ≈ ±180°), bricht der Service mit Fehler ab. ±180° ist wichtig, weil der GDS68 eine **kontinuierliche Mehrumdrehungs-Position** liefert (siehe Gotchas) — der Magnet erscheint sonst alle 2π erneut (mehrere Lösungen). Pass 1 muss daher **in Richtung des Magneten** sweepen (`search_dir`), damit er innerhalb ±180° gefunden wird.
0. **Vorab-Check (schon auf Magnet?)**: Pi schickt `CMD_ARM` und hält die Achse ~500 ms still. Meldet der Arduino sofort `RSP_DETECTED`, sitzt die Achse bereits auf dem Magneten — dann wird **entgegen der Pass-1-Richtung** (`−search_dir`) um `overshoot_angle` zurückgefahren, um den Magneten zu verlassen. Sonst wäre die erste Kantenerkennung mitten im Magneten und damit undefiniert.
1. **Pass 1 (Kante A)**: Pi schickt `CMD_ARM` an den Arduino, fährt die Achse dann **langsam** (`homing_vel`, klein für hohe Positionsgenauigkeit) in `search_dir`-Richtung. Arduino überwacht TLE4905L (LOW = Magnet). Bei Detektion sendet er `RSP_DETECTED` → Pi stoppt sofort. Position **P1** merken.
2. **Überfahren**: Pi fährt in **gleicher Richtung** noch ~25° (`overshoot_angle`) weiter, bis der Magnet sicher verlassen ist (Sensor wieder HIGH). Damit ist der Anlauf für Pass 2 von der Gegenseite frei.
3. **Pass 2 (Kante B)**: Pi schickt erneut `CMD_ARM`, fährt die Achse jetzt **in Gegenrichtung** langsam zurück, bis der Arduino den Magneten erneut detektiert (`RSP_DETECTED`). Position **P2** merken — dies ist die gegenüberliegende Schaltkante.
4. **Mitte berechnen**: Magnet-Mittelpunkt = `(P1 + P2) / 2` → Achse auf Mittelpunkt fahren.
5. **Zero setzen**: Achse auf gewünschte Nullposition fahren (`zero_target = Mitte + zero_offset`) und **Software-Home-Offset** im GDS68-Treiber setzen (`set_home_offset(zero_target)`). **Nicht** `Set_Linear_Count(0)`: der GDS68 nutzt einen ODrive-artigen **Absolut-Encoder**, dessen Position jeden Zyklus aus dem Absolutwert neu abgeleitet wird — `Set_Linear_Count` hält also nicht. Der Offset ist encoder-unabhängig (siehe CAN-Protokoll-Abschnitt).

Antwortet der Arduino in Pass 1 oder Pass 2 mit `RSP_ERROR` (oder kommt innerhalb `homing_timeout` gar keine Antwort), bricht der Service mit `success=false` ab und die Achse bleibt unkalibriert.

### ROS-seitiger Homing-Service (`HomingController` in `r0192_hardware`)

Die Homing-Logik ist in eine **eigene Klasse `HomingController`** ausgelagert ([homing_controller.hpp](src/r0192_hardware/include/r0192_hardware/homing_controller.hpp) / [homing_controller.cpp](src/r0192_hardware/src/homing_controller.cpp)), damit das Hardware-Interface auf die Echtzeit-`read()`/`write()`-Schleife fokussiert bleibt. Das Hardware-Interface erstellt im `on_activate()` (nur wenn Achse 1 vorhanden) eine Instanz und ruft `start()`; im `on_deactivate()` `stop()`.

- Der `HomingController` besitzt einen **eigenen Node `r0192_homing`** + `/homing`-Service (`std_srvs/Trigger`) + Executor-Thread → blockierende Sequenz stört die 100-Hz-Schleife nicht.
- Während Homing: `homing_->isActive()` sperrt `write()` für Achse 1 → kein Konflikt mit arm_controller.
- Arduino-Antworten werden vom bestehenden `canRxThread()` erkannt und per `homing_->notifyArduinoFrame(data[0])` weitergegeben (filtert auf `RSP_DETECTED`/`RSP_ERROR`).
- Nach dem Re-Zeroing ruft der Controller den `on_zeroed`-Callback des Hardware-Interfaces → **alle Achsen** werden auf die Home-Pose (0) gesetzt (State + Command), damit RViz/MoveIt nach dem Homing einen sauberen Nullzustand ohne Phantom-Offsets zeigen. Ausnahme: eine *echte*, nicht gehomte Achse (nur joint_4 möglich) wird übersprungen — ihr State kommt aus dem CAN-Feedback, ein erzwungenes 0-Kommando würde sie real fahren. Aktuell ist außer Achse 1 nichts eingesteckt, d. h. alle übrigen Achsen sind virtuell und werden sauber auf 0 gesetzt.
- Protokoll-Konstanten (`AXIS_CAN_ID`, `CMD_ARM`, `RSP_DETECTED`, `RSP_ERROR`) sind im `HomingController` definiert und mit der Arduino-Firmware konsistent zu halten.

Parameter zur Laufzeit änderbar via `ros2 param set /r0192_homing <name> <value>`:

| Parameter | Default | Bedeutung |
|-----------|---------|-----------|
| `homing_vel` | 0.3 | Suchgeschwindigkeit (rad/s) — Rampenrate des Positions-Sollwerts beim Sweep |
| `search_kp` | 20.0 | Positions-Gain (KP) während des Sweeps — gibt dem Motor genug Moment, um dem rampenden Sollwert zu folgen |
| `homing_kd` | 1.0 | Velocity-Gain (KD) während des Sweeps |
| `move_vel` | 0.5 | Geschwindigkeit (rad/s) für Überfahrt (25°) und Anfahrt der Mitte — Rampenrate, damit nicht zu schnell angefahren wird |
| `hold_kp` | 50.0 | Positions-Gain (KP) für Moves / Halten nach Kantenerkennung |
| `hold_kd` | 1.0 | Dämpfungs-Gain (KD) für Moves / Halten nach Kantenerkennung |
| `overshoot_angle` | 0.436 | Weiterfahrt in gleicher Richtung nach Kante A, bevor Pass 2 startet (rad, ≈ 25°) |
| `zero_offset` | 0.0 | Offset vom Magnetmittelpunkt zur Nullposition (rad) |
| `search_dir` | −1.0 | Sweep-Richtung von Pass 1 (+1/−1) — **zum Magneten hin** wählen, damit er innerhalb ±180° gefunden wird (Achse 1: Magnet bei ≈ −1.87 rad → −1.0) |
| `max_start_angle` | 3.1416 | Range-Check beim Start: Abbruch wenn \|rohe Position\| > diesem Wert (rad, ≈ ±180°). ±180° macht den Magneten eindeutig (sonst mehrere Lösungen pro Umdrehung) |
| `homing_timeout` | 60.0 | Max. Sekunden pro Anlauf-Richtung (Pi-seitig); zusätzlich eigener Timeout im Arduino |
| `managed_controller` | `arm_controller` | Controller, der während des Homings deaktiviert und danach reaktiviert wird |

**Controller-Handling während Homing**: Zu Beginn der Sequenz wird `managed_controller` (Default `arm_controller`) über `/controller_manager/switch_controller` **deaktiviert** und am Ende wieder **aktiviert**. Sonst würde der JointTrajectoryController nach dem Re-Zeroing die Achse sofort auf seinen alten (vor dem Homing gesetzten) Sollwert zurückfahren — bei Reaktivierung liest der JTC stattdessen den aktuellen Zustand (joint_1 = 0) ein und hält dort. Der Service-Call läuft auf einem separaten Client-Node (`r0192_homing_client`), damit er aus dem blockierenden `/homing`-Callback heraus gespint werden kann. Ist der Controller-Manager nicht erreichbar, wird nur gewarnt und das Homing läuft trotzdem durch.

### Arduino-Firmware Details

- Bibliothek: `mcp2515` (autowp)
- CAN-Bitrate: `CAN_1000KBPS`, MCP_8MHZ
- Hall-Sensor-Pin: Digital 3 (INPUT_PULLUP), LOW = Magnet erkannt
- CS-Pin MCP2515: Digital 10
- **CAN-ID**: eine einzige achsenspezifische ID (`AXIS_CAN_ID`, Achse 1 = `0x100`) für RX **und** TX; Unterscheidung über `Data[0]`-Code (`CMD_ARM` / `RSP_DETECTED` / `RSP_ERROR`). Für Achsen 2–6 nur `AXIS_CAN_ID` anpassen (`0x101`…`0x105`).
- **Arming-Bedingung**: Arduino wird nur scharf, wenn `can_id == AXIS_CAN_ID` **und** `Data[0] == CMD_ARM` (`0x01`). Andere Frames werden ignoriert.
- **Timeout im Arduino**: Nach Empfang von `CMD_ARM` läuft ein Timeout (`HOMING_TIMEOUT_MS`, z. B. 30000 ms). Wird der Magnet bis dahin nicht erkannt, sendet der Arduino `RSP_ERROR` (`0xEE`) und geht in Standby — er hängt **nicht** mehr in der Schleife fest. (Vorher: kein Timeout → Endlosschleife bei fehlendem Hall-Signal.)
- **Debug-Modus**: `DEBUG_MODE`-Flag schaltet serielle Ausgaben global ab. Im aktivierten Zustand wird **nur** geloggt, was die eigene `AXIS_CAN_ID` betrifft (gefiltert in `printCanFrame` / Empfangslogik) — Frames anderer Achsen werden nicht ausgegeben, damit der Monitor bei mehreren Arduinos am selben Bus lesbar bleibt.

---

## Key Open Tasks

**Aktueller Teststand (2026-05): Achsen 1 + 4 vollständig getestet — CAN-Feedback, Positions-Tracking und MoveIt-Planung funktionieren auf beiden Achsen.**

- [x] Motor-Test: Achse 1 (GDS68) über MoveIt — MIT_Control, CAN-Feedback, Positions-Tracking OK
- [x] Motor-Test: Achse 4 (RS05) über MoveIt — MIT_Control, CAN-Feedback, Positions-Tracking OK
- [x] GDS68-Feedback auf periodisches 0x009 (Encoder_Estimates) umgestellt — Pos/Vel kommen kontinuierlich (auch bei deaktivierten Motoren), getestet OK
- [x] RS05-Feedback über aktives Melden (Type 24 @ 10 ms, per CAN aktiviert) — streamt im **aktivierten** Zustand; bestätigt, dass der RS05 im Reset-Modus (deaktiviert) **nicht** meldet (kein ODrive-Idle)
- [ ] Hardware-Test: RS05 Feedback-im-deaktivierten-Zustand via Type-17-Polling (`mechPos`/`mechVel`) — prüfen, dass joint_4 beim Handverdrehen trackt; **Index-Endianness** (LE angenommen) und **Skala** (`mechPos` vs. Type-2-Position inkl. Mechanical-Zero) verifizieren
- [x] Encoder-Homing (on_activate) validiert: Hardware Interface setzt beim Start den internen Nullpunkt
- [x] Achsen-Erkennung per CAN-Probe: `probePresent(200ms)` in `on_configure`, automatisches Passthrough für nicht angeschlossene Achsen
- [x] `foxglove_bridge` in `real_robot.launch.py` integriert (nur als Debug-Tool, `use_foxglove` Launch-Arg, Port 8765)
- [x] Start-Sicherheits-Check `encodersWithinLimits()` in `on_activate()`: physische Achsen müssen laut RAW-Encoder innerhalb der URDF-Limits liegen, sonst `CallbackReturn::ERROR` (Arm startet nicht)
- [ ] Hardware-Test: Achse außerhalb der Limits parken → `on_activate` bricht mit Fehler ab; innerhalb der Limits → normaler Start

**Next: Homing-System (Arduino-basiert)**
- [x] Arduino-Firmware für Achse 1 implementiert (`microcontroller/r0192_homing.ino`)
- [x] ROS 2 Homing-Service `/homing` implementiert (eingebettet in `r0192_hardware`, `std_srvs/Trigger`)
- [x] `write()` wird während Homing gesperrt (`homing_->isActive()`), kein CAN-Konflikt mit ros2_control
- [x] Homing-Logik in eigene Klasse `HomingController` ausgelagert (`homing_controller.hpp`/`.cpp`)
- [x] Protokoll auf einheitliche ID + Daten-Codes umgestellt (eine `AXIS_CAN_ID` für RX/TX, `CMD_ARM`/`RSP_DETECTED`/`RSP_ERROR` in `Data[0]`) — Firmware **und** `r0192_hardware`
- [x] Arduino-Timeout ergänzt (`HOMING_TIMEOUT_MS` → `RSP_ERROR`), kein Hängenbleiben bei fehlendem Hall-Signal
- [x] Debug-Filter in Firmware: nur Frames der eigenen `AXIS_CAN_ID` werden geloggt
- [x] Algorithmus auf Kantenanlauf umgestellt (Kante A → `overshoot_angle` überfahren → Kante B → Mitte)
- [x] Sweep + Moves auf gerampten Positions-Sollwert umgestellt (`search_kp`/`move_vel`) — Motor fährt selbst, keine Snaps mehr
- [x] `arm_controller` wird während Homing via `switch_controller` deaktiviert/reaktiviert (kein Zurückfahren auf alten Sollwert nach dem Homing)
- [x] Nullsetzen über Software-Home-Offset im GDS68-Treiber statt `Set_Linear_Count(0)` (Absolut-Encoder → 0x019 hält nicht)
- [x] Nach Homing wird die komplette Home-Pose (alle Achsen 0) gesetzt — Achse 1 + virtuelle Achsen, echte Nicht-Home-Achse (joint_4) ausgenommen
- [x] Doppel-Homing-Bug behoben: `home_offset_` wird zu Beginn jeder Sequenz auf 0 zurückgesetzt (Sequenz arbeitet in Roh-Koordinaten)
- [x] Vorab-Check „schon auf Magnet?" + Zurückfahren entgegen Pass-1-Richtung
- [x] Range-Check beim Start (`max_start_angle`, Default ±π = ±180°) — eindeutiger Magnet trotz kontinuierlicher Mehrumdrehungs-Position
- [x] Sweep-Richtung konfigurierbar (`search_dir`, Default −1.0 für Achse 1) — Magnet wird innerhalb ±180° gefunden
- [x] joint_1 Drehrichtung korrigiert (URDF-Achse `0 0 -1`) — Modell dreht wie echter Motor
- [ ] Hardware-Test: Nach Homing meldet joint_1 ≈ 0 (≈ −1.87 roh) und hält die Magnet-Mitte; Re-Homing klappt; Richtung in RViz korrekt
- [ ] Optional: Start-Zeroing in `on_activate()` ebenfalls auf `set_home_here()` umstellen (0x008/0x009-Skala beachten)
- [ ] Arduino-Firmware auf Achsen 2–6 erweitern (achsenspezifische `MSG_SLAVE_ID`)

**Operator-Bedienung**
- [x] RViz Operator-Panel (`r0192_rviz_plugins`): Buttons für Homing + Motoren EIN/AUS, in `moveit.rviz` registriert
- [x] `/robot_enable` (`std_srvs/SetBool`) im Hardware-Interface implementiert (Motor-Drehmoment an/aus, `write()`-Gate)
- [x] Arm startet drehmomentfrei (`on_activate` setzt `motors_enabled_=false` nach Safety-Check/Zeroing) — Operator schaltet bewusst via Enable-Toggle
- [x] RViz Jog-Panel (`JogPanel`): Teach-Pendant mit 3 Modi (Achsen / Kartesisch-Basis / Werkzeug-Tool), 6×(−/+)-Tasten, Speed-Slider, über MoveIt Servo
- [x] MoveIt Servo integriert (`servo_node` in `moveit.launch.py`, `use_servo` Launch-Arg, `servo.yaml`) — Ausgabe an `arm_controller`
- [x] Zentrale Zustandsmaschine (`robot_state_manager` + `r0192_interfaces`): 5 exklusive Zustände DISABLED/HOLD/JOG/MOVEIT/HOMING, `/set_robot_state` + `/robot_state`, in `real_robot.launch.py`; virtuell getestet (Übergänge, Rejections, Homing-Auto-Return)
- [x] JogPanel auf State-Maschinen-Client umgebaut (state-getriebene UI, neuer MoveIt-Toggle, kein Direktzugriff mehr auf `/robot_enable`/`/homing`/`pause_servo`)
- [ ] Hardware-Test Zustandsmaschine: DISABLED→HOLD schaltet Drehmoment scharf; in HOLD ignoriert move_group (Action-Server inaktiv); MOVEIT führt aus; JOG joggt; HOMING kehrt automatisch nach HOLD zurück
- [ ] Hardware-Test: Panel-Buttons gegen echte Hardware (Homing-Trigger, Motoren EIN/AUS, Drift-/Ruck-Verhalten beim Re-Enable prüfen)
- [ ] Hardware-Test Jog-Panel: alle 3 Modi gegen Achse 1/4 verfahren; Twist-Frame-/Rotationsverhalten (`apply_twist_commands_about_ee_frame`) auf Hardware tunen; Speed-Slider-Bereich kalibrieren
- [ ] Optional Jog-Panel: echter Inkrement-/Schritt-Modus (fixer Weg pro Klick) zusätzlich zum Press-and-Hold

**Programm-System (Plan: doku/program_ide_plan.md)**
- [x] Phase 0: Schemas/Interfaces final abgestimmt (Datenmodell im Plan-Dokument)
- [x] Phase 1 Backend MVP: `r0192_program_executor` + `/execute_program`-Action, YAML-Loader, `move_j`/`wait`, State-Übergänge über `/set_robot_state`, Cancel, Feedback — virtuell getestet (Acceptance erfüllt)
- [x] Phase 2 VS-Code-Integration: JSON Schemas (`doku/schemas/`), `.vscode/` (Schema-Mapping, Snippets, Extension-Empfehlung), README-Abschnitt
- [ ] Phase 2 Acceptance manuell in VS Code verifizieren (Snippet → Live-Validierung → Autocomplete/Hover)
- [ ] Hardware-Test Phase 1: Demo-Programm gegen Achse 1/4 fahren
- [ ] Phase 3: RViz Run-Panel (`ProgramPanel` in `r0192_rviz_plugins`, runtime-only, KEIN Editor)
- [ ] Phase 4: Punkt-Services (`TeachPoint`/`ListPoints`/`DeletePoint`) + Teach-UI
- [ ] Phase 5+: Pause/Override, `move_l`, Pilz (siehe Plan)

**Next: Web-Interface (r0192_remote)**
- [ ] r0192_remote Paket aufbauen: Web-basiertes Bedienpanel als Operator-Interface
- [x] ROS-Services für Motor-Enable (`/robot_enable`) und Homing (`/homing`) implementiert (Backend, auch vom RViz-Panel genutzt)
- [x] E-Stop `/e_stop` (`std_srvs/Trigger`) im State Manager: erzwingt `DISABLED` aus jedem Zustand; latchender **Treiber-Torque-Cut** über Hardware-`/robot_estop` (GDS68 `Estop()` 0x002, RS05 stop); Torque-Kappen vor dem Mutex; Notaus-Button im JogPanel
- [x] Treiber-Reset nach Notaus: `/robot_reset` (State Manager) → Hardware-`/robot_clear_faults` (GDS68 `Clear_Errors()`, RS05 Fault-Clear-Stop); löst das `estop_latched_`-Gate; Reset-Button im JogPanel (nur in DISABLED)
- [x] Nebenläufigkeits-Fix: Homing-Worker des State Managers nutzt eigene `homing_node_` (kein paralleles Spinnen derselben Node bei `/e_stop` während Homing)
- [ ] Hardware-Test Notaus: `/e_stop` kappt Drehmoment auf Treiber-Ebene; Re-Enable ohne Reset wird abgelehnt; nach `/robot_reset` läuft `DISABLED→HOLD` wieder
- [ ] Echter Hardware-Notaus: laufenden Homing-Sweep abbrechen (HomingController-Abort-Hook), ggf. Power-Cut/Schütz statt nur Treiber-Torque-Aus
- [ ] Optional: `/e_stop` zusätzlich als latched Topic spiegeln, damit andere Nodes (Web-Interface) auf den Notaus-Zustand reagieren können

**Wenn weitere Motoren gekauft sind (Achsen 2, 3, 5, 6, Greifer):**
- [ ] Passthrough durch echte Treiber-Instanzen ersetzen (GDS68 für 2+3, RS05 für 5+6)
- [ ] Greifer-Controller mit physischer Hardware verbinden
- [x] URDF-Geometrie auf 1:1 real skaliert (Mesh `mesh_scale=0.001`, Gelenkursprünge in echten Metern, Greifer-Hub 2,5 mm)
- [ ] URDF-Platzhalter durch CAD-Werte ersetzen (Winkel-Limits, Trägheit, CoM; reale Maße verifizieren)

**Später / System-Ebene:**
- [ ] RT-Kernel / CPU-Shielding auf RPi 5 evaluieren
- [x] CAN-Bitrate auf 1 Mbit/s erhöhen (beide Treiber konfigurieren)
- [ ] `on_init` Deprecation-Warnung: Migration auf `HardwareComponentInterfaceParams` API

---

## Known Issues & Gotchas

- **joint_7 Greifer-Limit**: aktuell `lower="0.0" upper="0.0025"` (gültig, real-skaliert). Realer Hub mit CAD-Werten verifizieren — 2,5 mm ist nur der skalierte Platzhalter.
- **MIT Control KP/KD**: Werden jetzt aus `<param name="kp">` / `<param name="kd">` in `r0192_ros2_control.xacro` geladen (joint_1: KP=50/KD=1, joint_4: KP=30/KD=0.5). Werte dort anpassen — kein Rebuild nötig (symlink-install), nur neu starten.
- **GDS68 Feedback-Routing in canRxThread**: `(std_id >> 5) == 0x01` matched nur Achse 1. Wenn Achsen 2 und 3 später hinzukommen, Routing erweitern.
- **GDS68 kontinuierliche Mehrumdrehungs-Position / `Set_Linear_Count` hält nicht**: Der GDS68 (ODrive-Protokoll) liefert eine **kontinuierliche Position über mehrere Umdrehungen** (kein Single-Turn-Absolutwert). Folgen: (1) Der Magnet erscheint je nach Umdrehungszahl bei `magnet ± k·2π` (im Test bei −1.87 / +4.43 / +10.71 rad). Deshalb Range-Check `±max_start_angle` (±180°) beim Homing-Start → eindeutige Lösung. (2) `Set_Linear_Count(0)` (CMD 0x019) wird vom kontinuierlichen Positionswert sofort überschrieben. Nullsetzen erfolgt daher über einen **Software-Offset** im Treiber: `set_home_offset(raw)` / `set_home_here()`; `get_current_position()` liefert `raw − offset`, `MIT_Control()` addiert den Offset wieder auf. Das Homing nutzt das (setzt Offset zu Beginn auf 0, rechnet in Roh-Koordinaten, am Ende `set_home_offset(zero_target)`). **Achtung**: `on_activate()` ruft beim Start noch `Set_Linear_Count(0)` auf (Altlast, wirkungslos) und setzt `hw_cmd` auf die rohe Startposition — bis ein Homing gelaufen ist, meldet joint_1 den Rohwert (ggf. außerhalb der URDF-Limits). Später ggf. Start-Zeroing auf `set_home_here()` umstellen.
- **joint_1 Drehrichtung / URDF-Achse**: Der reale Motor dreht entgegen der Modell-Default-Richtung. Korrigiert über `<axis xyz="0 0 -1"/>` für joint_1 in `r0192.urdf.xacro` (statt `0 0 1`) — sonst dreht das RViz-Modell spiegelverkehrt zum echten Motor. Der Regelkreis selbst ist korrekt (Encoder-/Command-Vorzeichen konsistent, Ziele werden erreicht); es war rein eine Modell-Konvention.
- **GDS68 Positions-Skala 0x008 vs 0x009**: MIT-Feedback (0x008) liefert Position im ±12.5-rad-Feld, Encoder-Estimates (0x009) in Umdrehungen mit Getriebefaktor (`revToRad`, 8:1). **Seit der Umstellung füllt nur noch 0x009 `current_pos_`/`current_vel_`** (0x008 liefert nur best-effort Drehmoment) — d. h. die Laufzeit-Feedback-Skala ist jetzt die 0x009-`revToRad`-Skala. **Offen / Hardware-zu-verifizieren**: Das Homing setzt den Home-Offset weiterhin aus den während des MIT-Sweeps gelesenen Positionen — die kommen jetzt ebenfalls aus 0x009, also ist die Homing→Laufzeit-Feedback-Kette in sich konsistent. Die verbleibende Annahme ist, dass `revToRad(0x009)` **dieselbe Skala** ist wie das MIT-Befehlsfeld (`MIT_Control`-Position): Nur dann liest die Rückmeldung den kommandierten Winkel korrekt zurück. Muss einmal an Achse 1 geprüft werden — falls ein konstanter Faktor (z. B. 8er-Getriebe) auftaucht, müssen `revToRad`-Konvention und `MIT_Control`-Positionskodierung aneinander angeglichen werden.
- **Deprecation-Warnung**: `on_init(const HardwareInfo&)` ist in Jazzy deprecated. Funktioniert noch, Migration auf `HardwareComponentInterfaceParams` ist ausstehend.
- **Simulation Launch**: `simulated_robot.launch.py` referenziert `r0192_remote` (noch nicht vorhanden) — beim Simulationsstart auskommentiert lassen.
- **KDL IK**: Kann an Singularitäten scheitern. Alternativ: TRAC-IK oder pick_ik.
- **libserial**: Taucht in alten Paketabhängigkeiten auf, wird nicht genutzt. Kann entfernt werden.
- **JTC open_loop_control**: In `r0192_controllers.yaml` muss `open_loop_control: false` bleiben, da die interne Euler-Integration des Controllers bei den schnellen Pilz-PTP-Splines stark abweicht und zu `PATH_TOLERANCE_VIOLATED` führt.
- **Segfault bei Shutdown**: `move_group` und `rviz2` produzieren gelegentlich einen Segfault am Ende (Teardown). Dies ist ein bekannter Upstream-Bug im Lifecycle-Management von ROS 2 und hat keine Auswirkungen auf den Betrieb.
