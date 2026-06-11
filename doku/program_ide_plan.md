# R0192 Program Executor & RViz IDE — Implementierungsplan

## Kontext & Ziel

Erweiterung des bestehenden R0192-ROS2-Projekts um eine industriell-typische Trennung zwischen **Engineering** (Programmieren auf dem PC) und **Operations** (Bedienen am Roboter):

- **VS Code als Engineering-Tool**: Programme werden dort geschrieben/editiert — YAML mit JSON-Schema-Validierung, Autocomplete, Git-Integration.
- **RViz-Panel als Operator-/Runtime-Interface**: Programm laden, starten, pausieren, stoppen, Override, aktuelle Programmzeile anzeigen, Status-Monitoring.

Dieses Modell entspricht der etablierten Aufteilung in der industriellen Robotik (KUKA WorkVisual + SmartPAD, ABB RobotStudio + FlexPendant, Fanuc Roboguide + iPendant) und ist deutlich pragmatischer, als einen vollständigen Programmeditor in RViz nachzubauen.

## Architektur-Prinzipien (nicht verhandelbar)

1. **Trennung Backend / UI / Editor**: Backend (Executor) ist die einzige Stelle mit Geschäftslogik. RViz-Panel = Runtime-Client. VS Code = Editor. Kommunikation nur über ROS 2 Actions/Services und Dateien (YAML).
2. **State-Management nur über `/set_robot_state`**: Programmausführung läuft im `MOVEIT`-State, am Ende immer zurück nach `HOLD`. Niemals direkt `/robot_enable`, `/homing`, `pause_servo` oder Controller-Switch aufrufen.
3. **MoveIt sequenziell für v1**: `MoveGroupInterface` step-by-step. Pilz Industrial Motion Planner kommt erst in einer späteren Phase.
4. **Punkte vs. Programme getrennt**: Programme referenzieren Punkte per Name aus einer separaten Punktdatei — keine inline-Posen in Schritten.
5. **Programme sind YAML, kein Custom-DSL**: Files sind menschen- und maschinenlesbar, git-diff-bar und ohne Spezialtools editierbar.

## Rollenverteilung der Tools

| Tool                | Rolle               | Aufgaben                                                              |
| ------------------- | ------------------- | --------------------------------------------------------------------- |
| **VS Code**         | Engineering         | Programme schreiben/editieren, Punktnamen referenzieren, Git, Review  |
| **RViz Run-Panel**  | Operator / Runtime  | Laden, Run/Pause/Stop, aktuelle Zeile, Override, Status, Notaus       |
| **RViz Jog-Panel**  | Operator / Teach    | Manuell fahren, Punkt teachen (existiert bereits)                     |
| **Backend (ROS 2)** | Logik               | YAML lesen, MoveIt aufrufen, State-Übergänge, Cancel, Punkt-Services  |

## Pakete

- **NEU**: `r0192_program_executor` — Backend (Action Server, YAML-I/O, Punktverwaltung)
- **Erweitern**: `r0192_rviz_plugins` — zweites Panel "Program Panel" (Run-Panel, KEIN Editor)
- **Erweitern**: `r0192_interfaces` — neue msgs/srvs/actions
- **NEU** (Config-only): `.vscode/`-Verzeichnis im Repo-Root + JSON Schemas in `doku/schemas/`

## Datenmodell (FINAL — in Phase 0 abgestimmt, 2026-06-11)

### Punktdatei `points.yaml`

```yaml
version: 1
points:
  pick_home:
    type: joint
    joints: [0.0, -0.5, 1.2, 0.0, 1.0, 0.0]   # rad, Reihenfolge joint_1..joint_6
  drop_position:
    type: pose
    frame: base_link                            # v1: nur base_link erlaubt
    position: {x: 0.3, y: 0.2, z: 0.4}          # m
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

Festgelegt:
- **Map keyed by name** (kein Array) — Namens-Eindeutigkeit gratis, direkte Referenz, JSON-Schema-Validierung via `patternProperties`. Namensregel `^[A-Za-z_][A-Za-z0-9_]*$`.
- **`joints:`** statt generischem `values:` (expliziter, review-freundlicher). Genau **6 Werte** (Gruppe `r0192_arm`); Greifer ist kein Punkt-Bestandteil.
- `frame`: String mit Default `base_link`; Executor lehnt in v1 andere Frames ab.
- `version: 1` für spätere Migrationen.

### Programmdatei `program_*.yaml`

```yaml
version: 1
name: "Pick and Drop Demo"
description: "Holt Objekt von A und legt es bei B ab"   # optional
defaults:                # optional, gilt für Steps ohne eigene Angabe
  velocity: 0.2
  acceleration: 0.2
steps:
  - type: move_j
    target: pick_home
    velocity: 0.3        # Skalierungsfaktor (0, 1], NICHT rad/s
    acceleration: 0.3
  - type: wait
    duration: 2.0        # Sekunden
  - type: move_l
    target: drop_position
    velocity: 0.1
```

Festgelegt:
- **Step-Typen v1: `move_j`, `move_l`, `wait`** — keine Loops/Conditions.
- **`velocity`/`acceleration` sind einheitenlose Skalierungsfaktoren (0, 1]** (MoveIt `velocity_scaling_factor`); physikalische Einheiten erst mit Pilz (Phase 7). Override-Slider (Phase 5) multipliziert sich drauf.
- **`move_j` akzeptiert joint- und pose-Punkte** (pose → `setPoseTarget`); **`move_l` nur pose-Punkte** (v1).
- Optionales `name:` pro Step für die Anzeige im Run-Panel.

### Storage-Layout (final)

```
<workspace>/programs/          # im Repo (git-diff-bar), Default des Executors
├── points.yaml
└── program_*.yaml
doku/schemas/                  # JSON Schemas (Phase 2) = Source of Truth
```

Im Workspace statt `~/.r0192/`: Programme sind git-versioniert (Prinzip 5), das `.vscode/`-Schema-Mapping greift im geöffneten Workspace, Engineering-Workflow läuft dort. Pfade als ROS-Parameter (`programs_dir`, `points_file`) überschreibbar.

### ROS-Interfaces (final, in `r0192_interfaces`)

- **`action/ExecuteProgram.action`** — Goal: `string program_path` (nur Pfad, kein Inhalt — Datei bleibt Source of Truth, Validierung nur im Backend-Loader). Result: `success`, `message`, `steps_completed`. Feedback: `current_step`, `total_steps`, `step_type`, `step_label`, `status` (`STATUS_LOADING/PLANNING/MOVING/WAITING`).
- **`srv/TeachPoint.srv`** (Phase 4) — `name`, `type` (`TYPE_JOINT`/`TYPE_POSE`), `overwrite` → `success`, `message`.
- **`srv/ListPoints.srv`** (Phase 4) — → `names[]`, `types[]`.
- **`srv/DeletePoint.srv`** (Phase 4) — `name` → `success`, `message`.
- **Kein `SavePoint` in v1** — explizite Werte schreibt man in VS Code direkt in `points.yaml` (Schema-validiert); ein Service wäre ein redundanter zweiter Schreibpfad. Bei Bedarf später additiv.

Die JSON Schemas in `doku/schemas/` (Phase 2) sind die Source of Truth für VS-Code-Autocomplete und Validierung.

---

## ToDo

### Phase 0 — Design & Schema festklopfen (kein Code) ✅ (2026-06-11)

- [x] YAML-Schema für Punkte final festlegen (joint vs. pose, Frame-Handling) — siehe „Datenmodell (FINAL)"
- [x] YAML-Schema für Programme final festlegen (Step-Typen, Parameter pro Typ)
- [x] Step-Typen für v1 entschieden: `move_j`, `move_l`, `wait` — keine Loops/Conditions
- [x] `ExecuteProgram.action` festgelegt (Goal: nur Programm-Pfad; Feedback: Step-Index/-Typ/-Label + Status; Result: success/message/steps_completed)
- [x] Service-Interfaces für Punkte festgelegt: `TeachPoint`, `ListPoints`, `DeletePoint` — **kein `SavePoint` in v1** (VS Code editiert `points.yaml` direkt)
- [x] Dateipfade & Storage-Layout festgelegt: `<workspace>/programs/` + `programs/points.yaml` (im Repo, per ROS-Parameter überschreibbar)

### Phase 1 — Backend MVP (`r0192_program_executor`) ✅ (2026-06-11)

- [x] Neues Paket angelegt, in Workspace + Bringup registriert (`real_robot.launch.py`, Node `r0192_program_executor`)
- [x] `ExecuteProgram.action` in `r0192_interfaces` ergänzt (Punkt-Services folgen in Phase 4)
- [x] YAML-Loader mit Schema-Validierung (`program_loader.cpp`: Punkte + Programme, strikte Fehlermeldungen `<file>: <step/point>: <problem>`, unbekannte Keys abgelehnt)
- [x] Action Server `ExecuteProgram` lauffähig (`wait`-Step mit Cancel-responsiven 50-ms-Ticks)
- [x] `move_j` implementiert: Punkt aus DB → `MoveGroupInterface` (lazy erstellt, da move_group später startet) → planen → ausführen; joint- und pose-Ziele
- [x] State-Übergang im Executor: vor Start `HOLD → MOVEIT`, am Ende `MOVEIT → HOLD` — nur wenn der Zustand noch `MOVEIT` ist (nach `/e_stop` wird nichts angefasst)
- [x] Cancel-Verhalten: Action-Cancel ruft `MoveGroupInterface::stop()`, Worker beendet als CANCELED, sauber nach `HOLD` (getestet)
- [x] Feedback publiziert: Step-Index, Label, Gesamt-Steps, Status (LOADING/PLANNING/MOVING/WAITING)
- [x] **Acceptance erfüllt** (virtueller Modus, 2026-06-11): 4-Schritt-Demo (`programs/program_demo.yaml`) läuft via `ros2 action send_goal` durch → SUCCEEDED, Zustand zurück in `HOLD`. Zusätzlich getestet: Cancel mid-program → CANCELED + `HOLD`; Goal aus `DISABLED` → ABORTED mit Manager-Meldung; ungültiger Step-Typ → ABORTED mit präziser Loader-Meldung. **Hardware-Test an Achse 1/4 steht aus.**

### Phase 2 — VS-Code-Integration (Engineering) ✅ implementiert (2026-06-11, Acceptance manuell offen)

> Klein, aber sehr früh wertvoll: macht das Schreiben/Bearbeiten von Programm- und Punktdateien angenehm und sicher. Damit kannst du im weiteren Verlauf Programme zum Testen schnell und ohne Tippfehler schreiben.

- [x] JSON Schema für `program.yaml` geschrieben (`doku/schemas/program.schema.json`)
- [x] JSON Schema für `points.yaml` geschrieben (`doku/schemas/points.schema.json`)
- [x] `.vscode/extensions.json` mit Empfehlung für `redhat.vscode-yaml` angelegt
- [x] `.vscode/settings.json` mit Schema-Mapping ergänzt (bestehende Settings beibehalten); `.gitignore` von `\.vscode/` auf selektives Whitelisting umgestellt (settings/extensions/snippets committet, Rest ignoriert)
- [x] `.vscode/r0192.code-snippets`: `r0192-program`, `move_j`, `move_l`, `wait`, `point-joint`, `point-pose`
- [x] Im Haupt-README dokumentiert (Abschnitt „Roboterprogramme")
- [ ] **Acceptance** (manuell in VS Code zu verifizieren): Neue `program_*.yaml` aus Snippet erzeugen, ungültiger Step-Typ wird live markiert, Autocomplete + Hover funktionieren. (Schemas wurden bereits maschinell gegen die Beispieldateien validiert, inkl. Negativtest.)

### Phase 3 — RViz Run-Panel MVP (`r0192_rviz_plugins`)

> Bewusst minimaler Scope: nur Runtime, kein Editor. Editiert wird in VS Code.

- [ ] Panel-Klasse `ProgramPanel` (Subklasse von `rviz_common::Panel`) anlegen
- [ ] In `plugins_description.xml` registrieren, CMakeLists/Build erweitern
- [ ] UI-Layout: Datei-Picker, Programm-Anzeige (read-only mit Syntax-Highlighting via `QSyntaxHighlighter`), Buttons Run/Stop
- [ ] Action-Client für `ExecuteProgram` einbauen
- [ ] Live-Highlighting der aktuell ausgeführten Zeile/des Steps aus Action-Feedback
- [ ] Stop-Button → Action-Cancel → Executor stoppt sauber nach `HOLD`
- [ ] Status-Display: aktueller Step, "Step X von Y", aktueller Robot-State
- [ ] **Acceptance**: YAML-Programm laden, anzeigen, ausführen, stoppen — komplett über UI. Aktuelle Programmzeile ist während der Ausführung sichtbar markiert.

### Phase 4 — Punktverwaltung & Teach

- [ ] Services `SavePoint`, `DeletePoint`, `ListPoints` im Backend implementieren
- [ ] `TeachPoint`: holt aktuelle Joint-States bzw. TCP-Pose via TF und speichert unter Namen
- [ ] Punktdatei live nachladen (File-Watcher oder Service-getriggert)
- [ ] UI-Erweiterung (im Run-Panel oder als separates Teach-Panel): Punktliste-View, "Teach"-Button (nur aktiv im `JOG`-State), Rename, Delete
- [ ] **Acceptance**: Mit Jog-Panel hinfahren → "Teach as P1" → P1 in Punktliste sichtbar → in Programm (VS Code) per Name referenzierbar mit Autocomplete.

### Phase 5 — Pause & Override

> Hier zwei Features mit ehrlichen technischen Caveats — bitte im Backend sauber implementieren, nicht "irgendwie reinhacken".

- [ ] **Pause-Implementierung**: Da MoveIt-Trajektorien nicht echt mid-execution pausierbar sind, wird "Pause" als *Stop nach aktuellem Step* implementiert. Resume = Weitermachen ab Step N+1. Im UI klar so kommunizieren ("Pause after current step").
- [ ] **Speed-Override-Slider** im UI (0.1 – 1.0)
- [ ] Override wirkt auf den **nächsten** Step (Skalierung von `velocity`/`acceleration` beim Planen). Live-Override auf laufender Trajektorie ist NICHT Teil von v1 — kommt mit Pilz (Phase 7).
- [ ] Im Backend: Override-Wert als Service oder Topic, Executor liest ihn vor jedem Plan-Aufruf
- [ ] **Acceptance**: Override auf 0.3 setzen → nächster `move_j` ist sichtbar langsamer. Pause während eines Programms → Roboter hält nach aktuellem Step, Resume setzt den Ablauf fort.

### Phase 6 — MoveL (Kartesisch) & Visualisierung

- [ ] `move_l` im Executor: `computeCartesianPath()` für gerade Linien
- [ ] Punkt-Visualisierung im RViz 3D-View (interaktive Marker für alle Punkte des aktuellen Programms, mit Namen-Labels)
- [ ] Optional: Trajektorien-Preview vor Ausführung (planen, anzeigen, dann erst ausführen)
- [ ] **Acceptance**: Gemischtes `move_j`/`move_l`-Programm läuft sauber, Punkte sind im 3D-View sichtbar und benannt.

### Phase 7 — Pilz Industrial Motion Planner (optional, mittelfristig)

- [ ] Pilz im MoveIt-Setup als Planning Pipeline aktivieren
- [ ] Step-Typen erweitern: `move_ptp`, `move_lin`, `move_circ` mit `blend_radius`
- [ ] JSON Schema entsprechend erweitern (VS Code lernt die neuen Typen automatisch mit)
- [ ] Executor-Modus "Pilz Sequence": ganzes Programm als `MoveGroupSequence`-Request statt sequenziell
- [ ] UI-Toggle "Stop at each point" vs. "Blend through"
- [ ] **Echtes Live-Override** auf laufender Trajektorie wird hier endlich möglich

### Phase 8 — Robustheit & Polish

- [ ] Fehlerbehandlung im Executor (Planung schlägt fehl, Trajektorie schlägt fehl, Notaus während Ausführung)
- [ ] Unit-Tests für YAML-Parser, Integrationstest für Executor (mit Sim oder ros2_control Mock)
- [ ] README-Eintrag im Hauptrepo: kurze Bedienanleitung Run-Panel + Engineering-Workflow in VS Code
- [ ] Roadmap im Haupt-README anpassen (geplantes Web-Interface durch "RViz-Run-Panel + VS-Code-Engineering" ersetzen)

### Phase 9 (optional) — Eigene VS-Code-Extension

> Nur bei echtem Bedarf. Bis dahin reicht JSON Schema + `.vscode/`-Config.

- [ ] Validierung von Punktnamen gegen Live-Punktdatenbank (per ROS-Bridge oder File-Read)
- [ ] "Send to robot"-Button direkt aus VS Code
- [ ] "Go to point definition" (Cmd-Click auf Punktnamen springt zu `points.yaml`)
- [ ] Inline-Diagnostics für nicht-existierende Punkte

---

## Kickoff-Prompt für AI Coding Agent

> Diesen Prompt kannst du Claude Code oder einem anderen Coding-Assistenten geben, um eine Phase zu starten. Setze `[PHASE]` und die Tasks/Acceptance der jeweiligen Phase aus dem ToDo oben ein.

```text
Du arbeitest am ROS 2 Jazzy Projekt R0192-Roboticarm
(Repo: fHund3D/R0192-Roboticarm). Lies zuerst CLAUDE.md und README.md
im Repo-Root für den Projektkontext und die bestehende Paketstruktur.

# Ziel
Wir erweitern das Projekt um eine industriell-typische Trennung
zwischen Engineering und Operations:
- VS Code als Engineering-Tool (Programme schreiben, YAML + JSON Schema)
- RViz-Panel als Operator-/Runtime-Interface (Laden, Run, Pause, Stop,
  Override, aktuelle Programmzeile)
- Backend in ROS 2 (Action Server, YAML-Loader, Punktverwaltung)

# Architektur-Prinzipien (NICHT VERHANDELBAR)
1. Strikte Trennung Backend/UI/Editor: Backend ist die einzige Stelle
   mit Geschäftslogik. RViz-Panel = Runtime-Client. VS Code = Editor.
   Kommunikation nur über ROS 2 Actions/Services und Dateien (YAML).
2. State-Management ausschließlich über /set_robot_state (siehe
   robot_state_manager). Niemals direkt /robot_enable, /homing,
   pause_servo oder Controller-Switch aufrufen. Programmausführung
   läuft im MOVEIT-State, am Ende immer zurück nach HOLD.
3. MoveIt sequenziell für v1: MoveGroupInterface step-by-step.
   Pilz Industrial Motion Planner ist eine spätere Phase, jetzt nicht.
4. Punkte vs. Programme getrennt: Programme referenzieren Punkte per
   Name. Keine inline-Posen in Programmschritten.
5. Programme sind YAML, kein Custom-DSL. Die Schemas in doku/schemas/
   sind die Source of Truth.

# Neue/erweiterte Pakete
- NEU: r0192_program_executor (Backend)
- Erweitern: r0192_rviz_plugins (Run-Panel, KEIN Editor)
- Erweitern: r0192_interfaces (neue msgs/srvs/actions)
- NEU (Config-only): .vscode/ + doku/schemas/

# Aktuelle Phase
[PHASE-NAME, z.B. "Phase 1 — Backend MVP"]

# Tasks dieser Phase
[Hier die Checkbox-Tasks der Phase aus dem ToDo einfügen]

# Acceptance Criteria
[Hier den Acceptance-Punkt der Phase einfügen]

# Vorgehen
1. Erst die Schema-/Interface-Entscheidungen der Phase mit mir abstimmen,
   BEVOR du Code schreibst. Zeige mir geplante msg/srv/action-Definitionen
   und YAML-/JSON-Schemas, warte auf mein OK.
2. Danach inkrementell implementieren: kleine, testbare Commits.
3. Nach jedem Sub-Task: kurze Zusammenfassung der Änderung + wie ich es
   testen kann (Befehle, erwartetes Verhalten).
4. Bei Unsicherheit über Repo-Konventionen: im bestehenden Code nachsehen
   (z.B. wie r0192_rviz_plugins/JogPanel aufgebaut ist) und konsistent bleiben.

# Was du NICHT tun sollst
- KEINEN Programm-Editor ins RViz-Panel einbauen. Editieren ist VS-Code-Sache.
  RViz-Panel ist Runtime-only (Laden, Run/Pause/Stop, Override, Status).
- Keine UI-Logik ins Backend, keine Backend-Logik ins UI.
- Keine direkten Aufrufe von Low-Level-Services unter Umgehung des
  robot_state_manager.
- Keine Web-/HTTP-/REST-Schnittstellen. Nur native ROS 2 Pub/Sub/Service/Action.
- Keine echte VS-Code-Extension in dieser Phase (außer du arbeitest explizit
  an Phase 9). Bis dahin reicht JSON Schema + .vscode/-Config.
- Keine Annahmen über bestehende APIs ohne sie im Repo verifiziert zu haben.
- Keine eigenständigen Refactorings am bestehenden Code ohne Rücksprache.
```

---

## Nutzung dieses Dokuments

Dieses File z.B. als `doku/PROGRAM_IDE_PLAN.md` ins Repo committen. Phasen sequenziell abarbeiten — jeweils Tasks und Acceptance der Phase in den Kickoff-Prompt einsetzen, dem Coding-Agent geben, nach Abschluss Checkboxen abhaken und Acceptance manuell verifizieren. README-Roadmap im Hauptrepo erst am Ende der relevanten Phasen anpassen.
