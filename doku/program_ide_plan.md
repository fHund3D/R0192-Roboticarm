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

## Datenmodell (Vorschlag, finalisiert in Phase 0)

### Punktdatei `points.yaml`

```yaml
points:
  pick_home:
    type: joint
    values: [0.0, -0.5, 1.2, 0.0, 1.0, 0.0]
  drop_position:
    type: pose
    frame: base_link
    position: {x: 0.3, y: 0.2, z: 0.4}
    orientation: {x: 0, y: 0, z: 0, w: 1}
```

### Programmdatei `program_xyz.yaml`

```yaml
name: "Pick and Drop Demo"
description: "Holt Objekt von A und legt es bei B ab"
steps:
  - type: move_j
    target: pick_home
    velocity: 0.3
    acceleration: 0.3
  - type: wait
    duration: 2.0
  - type: move_l
    target: drop_position
    velocity: 0.1
```

Diese Schemas werden in Phase 2 in offizielle JSON Schemas überführt, die VS Code für Autocomplete und Validierung nutzt.

---

## ToDo

### Phase 0 — Design & Schema festklopfen (kein Code)

- [ ] YAML-Schema für Punkte final festlegen (joint vs. pose, Frame-Handling)
- [ ] YAML-Schema für Programme final festlegen (Step-Typen, Parameter pro Typ)
- [ ] Step-Typen für v1 entscheiden (Empfehlung: `move_j`, `move_l`, `wait` — keine Loops/Conditions)
- [ ] `ExecuteProgram.action` skizzieren (Goal: Programm-Pfad oder -Inhalt; Feedback: aktueller Step + Status; Result: Erfolg/Fehler)
- [ ] Service-Interfaces für Punkte skizzieren (`SavePoint`, `DeletePoint`, `ListPoints`, `TeachPoint`)
- [ ] Dateipfade & Storage-Layout festlegen (z.B. `~/.r0192/programs/`, `~/.r0192/points.yaml`)

### Phase 1 — Backend MVP (`r0192_program_executor`)

- [ ] Neues Paket anlegen, in Workspace + Bringup registrieren
- [ ] Custom Messages/Action in `r0192_interfaces` ergänzen
- [ ] YAML-Loader mit Schema-Validierung (Punkte + Programme)
- [ ] Action Server `ExecuteProgram` als Skelett (nur `wait`-Step) lauffähig bekommen
- [ ] `move_j` implementieren: Punkt aus DB → `MoveGroupInterface` → planen → ausführen
- [ ] State-Übergang im Executor: vor Start `HOLD → MOVEIT`, am Ende `MOVEIT → HOLD`
- [ ] Cancel-Verhalten: bei Action-Cancel laufenden Move abbrechen, sauber nach `HOLD`
- [ ] Feedback publizieren: aktueller Step-Index, Step-Name, Gesamt-Steps, Status
- [ ] **Acceptance**: `ros2 action send_goal /execute_program …` mit 3-Schritt-Programm (move_j, wait, move_j) läuft durch und kehrt sauber nach `HOLD` zurück.

### Phase 2 — VS-Code-Integration (Engineering)

> Klein, aber sehr früh wertvoll: macht das Schreiben/Bearbeiten von Programm- und Punktdateien angenehm und sicher. Damit kannst du im weiteren Verlauf Programme zum Testen schnell und ohne Tippfehler schreiben.

- [ ] JSON Schema für `program.yaml` schreiben (`doku/schemas/program.schema.json`)
- [ ] JSON Schema für `points.yaml` schreiben (`doku/schemas/points.schema.json`)
- [ ] `.vscode/extensions.json` mit Empfehlung für `redhat.vscode-yaml` anlegen
- [ ] `.vscode/settings.json` mit Schema-Mapping (Filepattern → Schema)
- [ ] `.vscode/r0192.code-snippets` mit Snippets für `move_j`, `move_l`, `wait`
- [ ] Im Haupt-README kurz dokumentieren, wie Programme in VS Code geschrieben werden
- [ ] **Acceptance**: Neue `program_*.yaml` aus einem Snippet erzeugen, ungültiger Step-Typ wird live als Fehler markiert, gültige Felder werden autocompleted, Hover zeigt Doku.

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
