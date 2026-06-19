# Roboticarm-R0192

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Der **R0192** ist ein 6-Achsen-Roboterarm, der auf einem hybriden System aus SteadyWin- und RobStride-Aktuatoren basiert. Dieses Repository enthält das vollständige ROS 2 Jazzy-Ökosystem für die CAN-Bus-Kommunikation, die Hardware-Abstraktion, die Bahnplanung via MoveIt 2 sowie ein Arduino-basiertes Hall-Sensor-Homing.

Die gesamte Steuerungslogik (MoveIt 2, ros2_control, CAN-Treiber) läuft auf einem **Raspberry Pi 5**. Eine zentrale **Betriebszustands-Maschine** (`robot_state_manager`) koordiniert Motor-Enable, Jogging, MoveIt-Ausführung, Homing und Notaus als sich gegenseitig ausschließende Zustände. Bedient wird der Arm über ein **RViz-Jog-Panel**, das gezielt zu einem Teach-Pendant-Ersatz ausgebaut wird (später ggf. durch ein echtes Hardware-Teach-Pendant ersetzbar); ein **Web-Interface** (`r0192_remote`) bleibt eine optionale Zukunftsoption. Foxglove Studio spielt nur eine kleine Rolle zum Debuggen. **Alle sechs Aktuatoren sind inzwischen beschafft und montagebereit**; softwareseitig sind die GDS68- und RS05-Antriebsstränge an je einer Achse verifiziert, die Inbetriebnahme der übrigen Achsen läuft.

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
- [x] **Programm-System** (`r0192_program_executor`) — industrielle Trennung Engineering (YAML in VS Code, JSON-Schema-Validierung) ↔ Operations (RViz-Run-Panel „R0192 Program"). Action `/execute_program`, KRL/Pilz-Steps `ptp`/`lin`/`circ` (+ Legacy `move_j`/`move_l`/`wait`), Stop-at-each-point **und** Pilz-Blend-Modus mit `c_dis`, Pause/Resume/Speed-Override, Punkt-Teach. Ersetzt für die Programmierung das ursprünglich geplante Web-Interface
- [ ] **Aktuell: Hardware-Test Homing** — Arduino + Magnet an Achse 1 anschließen und end-to-end validieren
- [ ] **Aktuell: RViz-Jog-Panel zum Teach-Pendant ausbauen** — bedienerfreundlich möglichst nah an ein echtes Teach-Pendant (später ggf. durch ein Hardware-Pendant ersetzbar); ein Web-Interface (`r0192_remote`) bleibt optionale Zukunftsoption
- [ ] **Hardware-Notaus über Schütz/Relay** — schaltet die 48-V-Versorgung hart ab (zusätzlich zum latchenden Treiber-Torque-Cut); laufenden Homing-Sweep abbrechen
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
- [x] Beschaffung **aller 6 Aktuatoren** abgeschlossen (4× SteadyWin GIM @ GDS68, 2× RobStride RS05)
- [ ] Detail-Konstruktion & Optimierung der Steifigkeit
- [ ] Full-Scale 3D-Druck Prototyping (PLA/PETG)
- [ ] **Geplant:** Substitution kritischer Strukturbauteile durch CNC-Aluminium
- [ ] **Steuerungs- & Versorgungsschrank** (Item-/Profil-5-Alu, 20×20) — separates Gehäuse für beide Netzteile, ODrive Regen Clamp & Raspberry Pi als externe Steuerung/Stromversorgung des Arms

### Elektronik (PCB Design)
- [x] Schaltungskonzept & Leistungsplanung
- [x] Design Daisy-Chain PCB (PCB 2) - *Draft*
- [ ] Design Breakout-Board (PCB 1) - *Main Power Distribution*
- [ ] PCB-Fertigung (PCBWay) & Bestückung
- [ ] Systemintegration & EMV-Tests

---

## 🛠 Technische Spezifikationen

### Achskonfiguration

> **Alle sechs Aktuatoren sind beschafft.** Achsen 1–4 nutzen SteadyWin-GIM-Motoren am **GDS68**-Treiber (11-bit-Standard-CAN), Achsen 5–6 RobStride **RS05** (29-bit-Extended-CAN). Achse 4 wurde von RS05 auf einen **GIM6010-8** umgestellt (24 V, ohne Bremse — der zuerst beschaffte Motor); Achsen 1–3 sind die 48-V-Varianten **mit Bremse** — sie legen den größten Weg zurück und tragen die größte Last; die Bremse hält sie stromlos in Position und verhindert ein Zusammenfallen des Arms ohne Strom. Achsen 5–6 sind RS05 @ 48 V.

| Achse | Motor | Treiber | Spannung / Bremse | Besonderheit | Lagerung | Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Axis 1** | [GIM6010-8](https://openelab.de/products/steadywin-gim6010-8-planetengetriebemotor?variant=54009308873041) | GDS68 | 48 V · mit Bremse | High Torque Base | [Kreuzlager RU66](https://de.aliexpress.com/item/1005008094900625.html) | ✅ Vorhanden |
| **Axis 2** | [GIM8108-8](https://openelab.de/products/steadywin-gim8108-8?variant=54041139609937) | GDS68 | 48 V · mit Bremse | Hauptausleger | [Kreuzlager RU42](https://de.aliexpress.com/item/1005008094900625.html) | ✅ Vorhanden |
| **Axis 3** | [GIM6010-8](https://openelab.de/products/steadywin-gim6010-8-planetengetriebemotor?variant=54009308873041) | GDS68 | 48 V · mit Bremse | Ellenbogen | - | ✅ Vorhanden |
| **Axis 4** | [GIM6010-8](https://openelab.de/products/steadywin-gim6010-8-planetengetriebemotor?variant=54009308873041) | GDS68 | 24 V · ohne Bremse | Zuerst beschaffter Motor (ersetzt RS05) | - | ✅ Vorhanden |
| **Axis 5** | [RS05](https://openelab.de/products/robstride05-qdd-55nm-joint-motor-robotik?variant=52331724931409) | Intern | 48 V | 1:2 Übersetzung (Torque) | [Kreuzlager RU28](https://de.aliexpress.com/item/1005008094900625.html) | ✅ Vorhanden |
| **Axis 6** | [RS05](https://openelab.de/products/robstride05-qdd-55nm-joint-motor-robotik?variant=52331724931409) | Intern | 48 V | Endeffektor Rotation | - | ✅ Vorhanden |

> **Code-Hinweis:** Achse 4 ist jetzt ein GIM6010-8 am **GDS68**-Treiber (11-bit-Standard-CAN) statt bisher RS05 (29-bit-Extended-CAN). Im Hardware-Interface/Treiber-Mapping ist Achse 4 noch der RS05-Zuordnung zugewiesen — die Code-Migration steht aus.



### Elektronik & Steuerung
* **Recheneinheit:** Raspberry Pi 5 (8GB RAM)
* **CAN-Interface:** MKS CANable Pro (Isoliert), SocketCAN (`can0`)
* **CAN-Bitrate:** 1 Mbit/s (info: 500 kbit/s GDS68 Werkseinstellung)
* **Energieversorgung:**
    * Bus-Spannung: 48 V — MeanWell LRS-600-48
    * Logik-Spannung: 5 V — MeanWell LRS-50
    * Regen-/Bremsenergie: [ODrive Regen Clamp](https://eu.odriverobotics.com/shop/odrive-regen-clamp) am 48-V-Bus — verhindert Überspannung beim Abbremsen/Rückspeisen der Motoren
* **Steuerungs- & Versorgungsschrank:** separates Gehäuse aus Item-/Profil-5-Aluprofilen (20×20) — nimmt beide Netzteile, die Regen Clamp und den Raspberry Pi auf und bildet die externe Steuerung & Stromversorgung des Arms
* **Umbilical-Steckverbindung (Schrank ↔ Arm):** Heavy-Duty-Rechteckstecker im **Han-E-Format, 24-polig** (Typ HDC-HE-024, Harting-Han-24E-kompatibel) — 16 A/Kontakt, 500 V, 0,5–6 mm² Schraubanschluss, Metallgehäuse (Schirm-/PE-Anbindung), IP65. Führt 48 V, 5 V, Bremse (Achsen 1–3) und CAN in einem Stecker; CAN-Trio in einer Ecke mit GND-Guard, Kabelschirm aufs Metallgehäuse
* **Homing-Sensorik:** TLE4905L Hall-Effekt-Sensoren (je 1 Arduino Uno R3 + MCP2515 CAN-Transceiver pro Achse)
* **Schnittstellen:** XT60PW (Power-Bus), XT30PW (Motor-Abgriff), XH-2A (CAN-Bus)

#### Verkabelung

Auslegung mit dem **200%-Kurzzeit-Peak** der Netzteile (48 V: 12,5 A Dauer → ~25 A Peak). Cu hat thermische Trägheit — der Peak ist eher für die **Schutzorgane** (träge Sicherung) relevant als für den Querschnitt.

| Kreis | Querschnitt | Kabeltyp |
| :--- | :--- | :--- |
| **48 V Trunk** (Box → Arm-Basis) | 2,5 mm² (≈14 AWG) | Silikon-Litze, feindrähtig (Klasse 5/6) |
| 48 V → Motor (Abzweig) | 1,0–1,5 mm² | Silikon-Litze (Achse 2 / GIM8108 → 1,5 mm²) |
| **5 V Logik** | 1,0 mm² (≈18 AWG) | Silikon-Litze (Querschnitt wegen Spannungsabfall, nicht Strom) |
| Bremse (Achsen 1–3, falls separat) | 0,5–0,75 mm² | Silikon-Litze (wenig Strom, kurzer Inrush) |
| **CAN** | 2×2×0,34 mm² | geschirmte, verdrillte CAN-Busleitung, **120 Ω** |

**Bewegung an den Gelenken** ist entscheidend für die Lebensdauer:

- **Drehachsen 1, 4, 6** (tordieren) → **torsionsfähige Roboterleitung** (z. B. Lapp ÖLFLEX ROBOT, igus chainflex), keine reine Biege-Schleppkettenleitung.
- **Biegeachsen 2, 3, 5** → Schleppketten-/Dauerbiege-Leitung genügt.
- Innerhalb eines Glieds (unbewegt) → einfache Silikon-Litze.

**CAN-Hinweise:** dediziertes 120-Ω-Buskabel (nicht zwei Adern aus dem Powerkabel), `CAN_GND` als Referenz mitführen, Schirm **steuerboxseitig** auf PE/Metallgehäuse (Han-E-Stecker); 48 V und CAN nicht ungeschirmt im selben Mantel bündeln.

**Materialkosten (grob, DE-Retail):** Silikon-Litze für Power/5V/Bremse ist günstig (~30–50 €), CAN-Leitung ~15–25 €. Kostentreiber ist die Torsions-/Roboterleitung an den Drehachsen. Gesamt: **~60–100 €** maker-pragmatisch (Silikon-Litze überall), **~120–220 €** mit Marken-Roboterleitung an den Gelenken; dazu Kleinkram (Aderendhülsen, Schrumpfschlauch, Gewebeschlauch) ~25–50 €.

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

## Roboterprogramme (Engineering in VS Code, Ausführung über ROS 2)

Das Projekt folgt der industriellen Trennung **Engineering ↔ Operations** (siehe [doku/program_ide_plan.md](doku/program_ide_plan.md)): Programme werden in **VS Code** geschrieben (YAML mit Schema-Validierung), ausgeführt werden sie vom Backend **`r0192_program_executor`** über die Action **`/execute_program`** — per CLI oder über das **RViz-Run-Panel „R0192 Program"** (`r0192_rviz_plugins/ProgramPanel`, erscheint automatisch beim `real_robot.launch.py`-Start): Programm wählen, Run/Stop, Live-Markierung des laufenden Steps, Status „Step X / Y". Das Panel ist bewusst runtime-only — editiert wird ausschließlich in VS Code.

**Dateien** (im Repo, git-versioniert):

```
programs/
├── points.yaml          # Punktdatenbank: benannte Ziele (joint / pose)
└── program_*.yaml       # Programme: Steps ptp / lin / circ (KRL/Pilz) + wait
doku/schemas/            # JSON Schemas = Source of Truth (VS Code + Loader)
```

**Schreiben in VS Code:** Die empfohlene Extension `redhat.vscode-yaml` (siehe `.vscode/extensions.json`) validiert beide Dateitypen live gegen die Schemas in `doku/schemas/` (Mapping in `.vscode/settings.json`): ungültige Step-Typen/Felder werden markiert, gültige Felder autocompleted, Hover zeigt Doku. Snippets: `r0192-program` (neues Programm), `ptp`, `lin`, `circ`, `wait`, `point-joint`, `point-pose` (plus `move_j`/`move_l` als deprecated Aliase).

**Step-Vokabular:** Zwei koexistierende Sätze, beide gültig:

| Step | Bewegung | Parameter |
|------|----------|-----------|
| `ptp` | Punkt-zu-Punkt (Pilz PTP), joint- oder pose-Ziel | `vel`, `acc`, `c_dis` |
| `lin` | Kartesische Gerade (Pilz LIN), nur pose-Ziel | `vel`, `acc`, `c_dis` |
| `circ` | Kreisbogen (Pilz CIRC) über `via`-Punkt zum Ziel, beide pose | `via`, `vel`, `acc`, `c_dis` |
| `wait` | Pause (Sekunden) | `duration` |
| `move_j` / `move_l` | **deprecated** — Legacy OMPL / KDL, weiterhin lauffähig | `velocity`, `acceleration` |

Wichtige Regeln: Programme referenzieren Punkte **nur per Name** aus `points.yaml` (keine Inline-Posen); `vel`/`acc` (bzw. `velocity`/`acceleration`) sind **MoveIt-Skalierungsfaktoren (0, 1]**, keine physikalischen Geschwindigkeiten; `c_dis` ist der Blend-Radius in Metern (0 = Halt am Punkt), nur im Blend-Modus wirksam.

**Zwei Run-Modi** (Goal-Feld `blend`, im Run-Panel der Toggle „Blend through (Pilz)"):
- `blend: false` (Default) — *Stop at each point*: jeder Move einzeln geplant/ausgeführt, `c_dis` ignoriert, `circ` abgelehnt.
- `blend: true` — *Blend through*: zusammenhängende Move-Steps gehen als **eine** Pilz-`MotionSequenceRequest` an `/sequence_move_group` (`blend_radius = c_dis`, letztes Segment hält). Pflicht für `circ`. Für `circ` müssen Start, `via` und Ziel **nicht-kollinear** sein (sonst „Plane for motion is not properly defined"). `wait`-Steps trennen eine Sequenz.

**Simulationsmodus** (Goal-Feld `dry_run`, im Run-Panel der Toggle „Simulate (dry run)"): `dry_run: true` plant das ganze Programm und animiert es **nur als RViz-Geist** (`/display_planned_path`) — der echte Arm bewegt sich **nicht**, der Roboterzustand bleibt unangetastet (kein MOVEIT, Motoren bleiben aus/an wie sie sind), `wait`-Steps werden übersprungen. Läuft aus **jedem** Zustand und ist die saubere Vorschau vor dem echten Lauf. Häkchen raus → echter Lauf auf dem Arm.

**Ausführen** (Arm muss in `HOLD` sein; der Executor schaltet selbst `HOLD → MOVEIT → HOLD`):

```bash
# Stop at each point (Default)
ros2 action send_goal /execute_program \
  r0192_interfaces/action/ExecuteProgram "{program_path: program_demo.yaml}" -f

# Blend through (Pilz-Sequenz mit c_dis-Blending; Pflicht für circ)
ros2 action send_goal /execute_program \
  r0192_interfaces/action/ExecuteProgram "{program_path: program_blend_demo.yaml, blend: true}" -f
```

Relative Pfade werden gegen `programs/` aufgelöst (Parameter `programs_dir`/`points_file` des Executor-Nodes). Action-Cancel stoppt die laufende Bewegung (`MoveGroupInterface::stop()`) und kehrt sauber nach `HOLD` zurück. Nach einem Notaus (`/e_stop`) fasst der Executor den Zustand nicht an — die Wiederinbetriebnahme läuft wie immer über `/robot_reset` + `/set_robot_state`.

**Pause, Resume & Speed-Override** (auch als Button/Slider im Run-Panel):

```bash
# Pause nach dem aktuellen Step (Zustand bleibt MOVEIT, Arm hält Position)
ros2 service call /pause_program std_srvs/srv/Trigger {}
ros2 service call /resume_program std_srvs/srv/Trigger {}

# Globaler Speed-Override (klemmt auf [0.1, 1.0], wirkt ab dem NÄCHSTEN Step)
ros2 service call /set_program_override r0192_interfaces/srv/SetProgramOverride "{override: 0.3}"
ros2 topic echo /program_override --once     # autoritativer Ist-Wert (latched)
```

**Punkte teachen** (Phase 4, auch über die „Points"-Gruppe im Run-Panel): `/teach_point` speichert die aktuelle Position (joint oder pose, nur in `HOLD`/`JOG`), `/list_points` listet, `/delete_point` löscht. Explizite Werte editiert man direkt in `points.yaml` (VS Code, schema-validiert) — Achtung: ein Teach/Delete-Rewrite erhält Hand-Kommentare nicht.

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
