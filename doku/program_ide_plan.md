# R0192 Program Executor & RViz IDE — Implementierungsplan

## Kontext & Ziel

Erweiterung des bestehenden R0192-ROS2-Projekts um eine industriell-typische Trennung zwischen **Engineering** (Programmieren auf dem PC) und **Operations** (Bedienen am Roboter):

- **VS Code als Engineering-Tool**: Programme werden dort geschrieben/editiert — YAML mit JSON-Schema-Validierung, Autocomplete, Git-Integration.
- **RViz-Panel als Operator-/Runtime-Interface**: Programm laden, starten, pausieren, stoppen, Override, aktuelle Programmzeile anzeigen, Status-Monitoring.

Dieses Modell entspricht der etablierten Aufteilung in der industriellen Robotik (KUKA WorkVisual + SmartPAD, ABB RobotStudio + FlexPendant, Fanuc Roboguide + iPendant) und ist deutlich pragmatischer, als einen vollständigen Programmeditor in RViz nachzubauen.

## Architektur-Prinzipien (nicht verhandelbar)

1. **Trennung Backend / UI / Editor**: Backend (Executor) ist die einzige Stelle mit Geschäftslogik. RViz-Panel = Runtime-Client. VS Code = Editor. Kommunikation nur über ROS 2 Actions/Services und Dateien (YAML).
2. **State-Management nur über `/set_robot_state`**: Programmausführung läuft im `MOVEIT`-State, am Ende immer zurück nach `HOLD`. Niemals direkt `/robot_enable`, `/homing`, `pause_servo` oder Controller-Switch aufrufen.
3. **MoveIt sequenziell für v1**: `MoveGroupInterface` step-by-step. Pilz Industrial Motion Planner kommt erst in einer späteren Phase. *(Update: in Phase 7 umgesetzt — `ptp`/`lin`/`circ` über Pilz, plus Blend-Modus via `/sequence_move_group`. Der sequenzielle `MoveGroupInterface`-Pfad bleibt für „stop at each point" erhalten.)*
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

> **Update 2026-06-13**: Dieser Abschnitt beschreibt das in Phase 0 abgestimmte **v1-Modell**. Es wurde in den Phasen 5/7/8 **additiv erweitert** (das v1-Modell bleibt unverändert gültig). Die Erweiterungen — KRL-Vokabular `ptp`/`lin`/`circ`, die Goal-Felder `blend`/`dry_run`, der `STATUS_PAUSED`-Feedback-Status und die Pause-/Override-Interfaces — sind unten im Block **„Erweiterungen seit v1"** zusammengefasst. **Source of Truth** für das Endmodell sind die JSON Schemas in `doku/schemas/` und die `.action`/`.srv`-Dateien in `r0192_interfaces`.

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

### Erweiterungen seit v1 (Phasen 5/7/8 — additiv, v1 bleibt gültig)

**KRL/Pilz-Step-Vokabular (Phase 7)** — koexistiert mit den Legacy-Steps; `move_j`/`move_l` sind als *deprecated, prefer ptp/lin* markiert:

```yaml
steps:
  - type: ptp                # Pilz PTP (Punkt-zu-Punkt), joint- oder pose-Ziel
    target: pick_home
    vel: 0.3                  # Skalierungsfaktor (0,1]  — KRL-Steps nutzen vel/acc (NICHT velocity/acceleration)
    acc: 0.3
  - type: lin                # Pilz LIN (kartesische Linie), nur pose-Ziel
    target: drop_position
    vel: 0.1
    c_dis: 0.02              # optionaler Blend-Radius in m (nur im blend-Modus wirksam)
  - type: circ               # Pilz CIRC (Kreisbogen), nur pose-Ziel
    target: arc_end
    via: arc_mid             # Hilfspunkt (Pose), zwingend; nicht kollinear zu Start/Ziel
    vel: 0.1
```

- **Zwei Run-Modi (Goal-Feld `blend`)**: `false` = „stop at each point" (je Step ein Plan via `MoveGroupInterface`, `c_dis` ignoriert, `circ` abgelehnt); `true` = „blend through" (zusammenhängende Move-Steps als ein `MotionSequenceRequest` an `/sequence_move_group`, `blend_radius=c_dis`, `circ`-`via` als „interim"-Constraint). `wait`-Steps trennen eine Sequenz.
- **Simulationsmodus (Goal-Feld `dry_run`)**: plant das ganze Programm als Pilz-Sequenz mit `plan_only`, animiert es auf `/display_planned_path` (RViz-Geist), bewegt den Arm **nicht** und ändert den Zustand **nicht** (kein MOVEIT); läuft aus jedem Zustand. *(Erfüllt zugleich die in Phase 6 offen gelassene „Trajektorien-Preview".)*
- **Pose-Referenzlink für `lin`/`circ`/`move_l`**: kartesische Steps brauchen ein abgewinkeltes Handgelenk — KDL/Pilz können keine Linie durch die joint_5=0-Singularität.

**ExecuteProgram.action — additive Felder** (Endstand): Goal zusätzlich `bool blend`, `bool dry_run`. Feedback zusätzlich `STATUS_PAUSED=4` (zwischen Steps pausiert; `current_step` = nächster Step). `step_type` deckt nun `move_j`/`move_l`/`wait`/`ptp`/`lin`/`circ` ab.

**Pause & Override (Phase 5) — zusätzliche Interfaces**:
- `srv/SetProgramOverride.srv` → Service `/set_program_override` (klemmt [0.1, 1.0]) + latched Topic `/program_override` (`std_msgs/Float32`, Ist-Wert).
- `/pause_program` + `/resume_program` (`std_srvs/Trigger`) — „Pause after current step", Zustand bleibt `MOVEIT`.

**Punkt-Visualisierung (Phase 6)**: latched `MarkerArray` auf `/program_points_markers` (Kugel + Namens-Label je Punkt).

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

### Phase 3 — RViz Run-Panel MVP (`r0192_rviz_plugins`) ✅ (2026-06-12, Acceptance erfüllt)

> Bewusst minimaler Scope: nur Runtime, kein Editor. Editiert wird in VS Code.

- [x] Panel-Klasse `ProgramPanel` (Subklasse von `rviz_common::Panel`) angelegt
- [x] In `plugin_description.xml` registriert, CMakeLists/package.xml erweitert (`rclcpp_action`); zusätzlich im Default-Layout `moveit.rviz` als „R0192 Program" — erscheint beim Start automatisch
- [x] UI-Layout: Datei-Combo über `programs/` (+ Refresh + Browse, Verzeichnis wird in der RViz-Config persistiert), Programm-Anzeige read-only mit YAML-`QSyntaxHighlighter`, Buttons Run/Stop
- [x] Action-Client für `ExecuteProgram`; Goal mit absolutem Pfad, Datei wird vor Run frisch von Platte gelesen (View = was der Executor lädt)
- [x] Live-Highlighting des aktuell ausgeführten Steps (Step-Index → `- `-Items nach `steps:` im Dokument, Amber-Block + Auto-Scroll)
- [x] Stop-Button → Action-Cancel → Executor stoppt Bewegung (`stop()`) und kehrt sauber nach `HOLD` zurück
- [x] Status-Display: „Step X / Y — label [status]", Robot-State-Label, Statuszeile; state-getriebene UI (Run nur in `HOLD`/`MOVEIT`, Dateiauswahl während Run gesperrt)
- [x] **Acceptance erfüllt** (manuell in RViz, 2026-06-12): Programm über UI geladen, ausgeführt und gestoppt; Step-Highlight sichtbar; State-Übergänge MOVEIT→HOLD im Log bestätigt.

### Phase 4 — Punktverwaltung & Teach ✅ implementiert (2026-06-12, Acceptance manuell offen)

- [x] Services `/teach_point`, `/list_points`, `/delete_point` im Backend implementiert (kein `SavePoint` — Phase-0-Entscheidung: explizite Werte schreibt VS Code direkt). YAML-Writer schreibt atomar (tmp+rename), schema-valide; **Achtung: Hand-Kommentare in `points.yaml` überleben einen Teach/Delete-Rewrite nicht** (Header-Kommentar in der Datei weist darauf hin)
- [x] `TeachPoint`: Joint-Typ aus `/joint_states` (joint_1..6), Pose-Typ via TF `base_link` → `pose_reference_link` (Param, Default `gripper_base` = EE-Link der Planning Group, damit die Pose beim Abspielen exakt reproduziert wird). Nur in `HOLD`/`JOG` erlaubt
- [x] Punktdatei live nachladen: service-getriggert — jede List/Teach/Delete-Anfrage und jedes Programm-Goal liest die Datei frisch (kein File-Watcher nötig)
- [x] UI-Erweiterung im Run-Panel (`ProgramPanel`, Gruppe „Points"): Punktliste (Name + Typ), Refresh, Teach (Name + joint/pose, nur in `HOLD`/**`JOG`** aktiv, Overwrite-Rückfrage), Delete mit Bestätigungsdialog. **Rename bewusst weggelassen** — Umbenennen ist eine Engineering-Aktion (VS Code, mit Suchen/Ersetzen der Referenzen)
- [x] Services per CLI getestet (2026-06-12, virtuell): List/Teach joint/Teach pose/Overwrite-Ablehnung/Delete/Unknown-Delete/State-Gating alle OK; regenerierte `points.yaml` schema-valide; Demo-Programm läuft aus der regenerierten Datei
- [x] **Acceptance erfüllt** (real, 2026-06-12): Teach joint+pose über das Panel bestätigt (inkl. Pose-Teach auf Achse 4); TF-Listener-Bug gefixt (eigener Spin-Thread). *Hinweis: Namens-Autocomplete über Dateigrenzen kann ein statisches JSON Schema nicht leisten — das kommt erst mit der VS-Code-Extension (Phase 9); das Schema validiert das Namensmuster.*

### Phase 5 — Pause & Override ✅ (2026-06-12, Acceptance erfüllt)

> Hier zwei Features mit ehrlichen technischen Caveats — bitte im Backend sauber implementieren, nicht "irgendwie reinhacken".

- [x] **Pause-Implementierung**: „Pause after current step" — `/pause_program` + `/resume_program` (`std_srvs/Trigger`, Executor). Laufender Step wird beendet, dann hält der Worker vor dem nächsten Step; **Zustand bleibt `MOVEIT`** (Goal besitzt die Zustandsmaschine weiter — kein HOLD-Ping-Pong, kein JOG-Konfliktfenster), JTC hält die Position. Feedback-Status `STATUS_PAUSED=4` (additiv in `ExecuteProgram.action`); Cancel aus der Pause heraus funktioniert. UI-Button „Pause (after step)"/„Resume", paused-Flag folgt dem Action-Feedback
- [x] **Speed-Override-Slider** im UI (10–100 %), folgt dem autoritativen latched Topic statt optimistisch zu schalten
- [x] Override wirkt auf den **nächsten** Step (Multiplikation der `velocity`/`acceleration`-Skalierung beim Planen; (0,1]×[0.1,1] bleibt in (0,1]). Live-Override auf laufender Trajektorie kommt mit Pilz (Phase 7)
- [x] Backend: **Service `/set_program_override`** (`SetProgramOverride.srv`, klemmt auf [0.1, 1.0]) + **latched Topic `/program_override`** (`std_msgs/Float32`, Ist-Wert) — Haus-Muster wie `/set_robot_state` + `/robot_state`; Executor liest den Wert vor jedem Plan-Aufruf; Reset auf 1.0 bei Neustart
- [x] Headless getestet (2026-06-12, virtuell): Override 0.3 angewendet / 0.05→0.1 geklemmt / latched Topic korrekt; Pause ohne Programm abgelehnt; Pause mid-run → hält nach aktuellem Step (PAUSED vor Step 2), bleibt stabil, Resume → SUCCEEDED (4 Steps); Cancel während Pause → CANCELED
- [x] **Acceptance erfüllt** (manuell, 2026-06-12): Override 0.21 → nächster `move_j` ~4× langsamer (Log); Pause vor Step 3 → Resume setzt fort, Programm SUCCEEDED.

### Phase 6 — MoveL (Kartesisch) & Visualisierung ✅ implementiert (2026-06-12, Acceptance manuell offen)

- [x] `move_l` im Executor: `computeCartesianPath()` (eef_step via Param `cartesian_eef_step`, Default 5 mm). **Retiming serverseitig**: die Skalierungsfaktoren (`velocity`/`acceleration` × Override) gehen im Service-Request mit; move_group wendet TOTG mit den `joint_limits.yaml`-Limits an (das Client-Robotermodell hat keine Beschleunigungslimits — clientseitiges TOTG schlug deshalb fehl). `fraction < 99.9 %` → sauberer Abbruch mit Prozentangabe; Null-Distanz (< 2 Trajektorienpunkte) = No-op-Erfolg
- [x] **Wichtige Erkenntnis (getestet)**: KDL kann **keine Linie durch die Handgelenk-Singularität joint_5 = 0** verfolgen (alle bisherigen Demo-Punkte lagen dort → 0 % feasible). `move_l`-Punkte brauchen ein abgewinkeltes Handgelenk; Fehlermeldung weist darauf hin. Langfristige Alternative: TRAC-IK (siehe Known Issues)
- [x] Punkt-Visualisierung im RViz 3D-View: `MarkerArray` auf **`/program_points_markers`** (latched) — Kugel + Namens-Label je Punkt aus `points.yaml` (pose = orange, joint = blau via FK, sobald das Robotermodell verfügbar ist); Republish bei Teach/Delete/List; Display „ProgramPoints" in `moveit.rviz`. *Bewusst keine interaktiven Marker* — ohne Edit-Funktion (Editieren = VS Code) wäre Interaktivität nur Schein
- [x] Optional: Trajektorien-Preview vor Ausführung — **nachträglich durch den Dry-Run-Modus (Phase 8) erfüllt** (`dry_run`-Goal plant + animiert das Programm als RViz-Geist, ohne den Arm zu bewegen)
- [x] Headless getestet (2026-06-12, virtuell): gemischtes `move_j`/`move_l`-Programm (`programs/program_linear_demo.yaml`, Punkte `bent_a`/`bent_b`/`lin_bent`) SUCCEEDED; Wiederholung mit Null-Distanz-`move_l` SUCCEEDED; Marker-Topic liefert Kugel+Label für alle Punkte; Singularitäts-Fall bricht sauber mit klarer Meldung ab
- [ ] **Acceptance** (manuell in RViz): `program_linear_demo.yaml` läuft sichtbar (PTP, PTP, Linearbewegung), Punkte im 3D-View sichtbar und benannt.

### Phase 7 — Pilz Industrial Motion Planner + KRL-Vokabular ✅ funktional abgeschlossen (2026-06-13)

> **Status**: Vokabular, sequenzielles Pilz-Routing und der Blend-/Sequence-Modus (inkl. `circ`) sind implementiert und real bestätigt. **Einzige bewusste Limitation**: echtes Live-Override auf einer laufenden Trajektorie (braucht Stop+Replan) — als Folgearbeit eingeordnet, siehe `[~]`-Punkt unten.

> Ab hier macht es Sinn, sich beim Befehlsvokabular an etablierten Industrie-Sprachen zu orientieren statt es selbst zu erfinden. **KUKA KRL** (Robot Language der klassischen KR-Industrieserie) ist die natürliche Wahl, weil ihre Kernbefehle (`PTP`, `LIN`, `CIRC`) **exakt das sind, was Pilz erwartet** — die Step-Typen mappen sich quasi von selbst. KRL ist in Schulungsmaterial und Lehrbüchern frei dokumentiert, IP-rechtlich unproblematisch. (Hinweis: KUKA Sunrise / iiwa Java-API NICHT als Referenz nehmen — proprietär, nur mit Lizenz zugänglich.)

- [x] Pilz im MoveIt-Setup als Planning Pipeline aktiviert (`moveit.launch.py`: `planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])`, `pilz_cartesian_limits.yaml` vorhanden)
- [x] Neue Step-Typen mit KRL-Vokabular eingeführt: `ptp`, `lin`, `circ` mit `vel`, `acc`, `c_dis` (Blend-Radius in m); `circ` zusätzlich mit `via` (Hilfspunkt). Loader (`program_loader.cpp`) + `crossValidate` erweitert (lin/circ brauchen Pose-Ziele, circ-`via` muss Pose sein)
- [x] **Backward Compatibility** — bewusst **konservativer** als der ursprüngliche Plan: statt `move_j`/`move_l` intern auf `ptp`/`lin` umzubenennen, **koexistieren** beide Vokabulare. `move_j`/`move_l`/`wait` behalten ihre **validierten** Phase-1/6-Pfade (OMPL bzw. KDL `computeCartesianPath`) **unverändert** (Null-Regressionsrisiko); `ptp`/`lin`/`circ` sind die neuen Pilz-Pfade. Alte Programme laufen ohne Migration weiter; Schema + Snippets markieren `move_j`/`move_l` als *deprecated, prefer ptp/lin*
- [x] JSON Schema erweitert (`program.schema.json`): `ptp`/`lin`/`circ` + `blendRadius`-Definition parallel zu `move_j`/`move_l`; headless gegen alle Beispielprogramme + Negativtests validiert (jsonschema)
- [x] Snippets aktualisiert (`r0192.code-snippets`): `ptp`/`lin`/`circ` als Default-Snippets, Programm-Skelett nutzt `ptp`; `move_j`/`move_l` als „(deprecated)" behalten
- [x] Executor-Routing (sequenziell): `ptp` → Pilz `PTP`, `lin` → Pilz `LIN` (`setPlanningPipelineId("pilz_industrial_motion_planner")` + `setPlannerId`, danach Reset auf `ompl`); `c_dis > 0` wird geloggt + ignoriert (Blending fehlt noch); `circ` zur Laufzeit sauber abgelehnt (Datenmodell steht, kommt mit dem Sequence-Modus — gleiches Muster wie `move_l` vor Phase 6)
- [x] Executor-Modus „Pilz Sequence" (`runBlendedRun`): bei `blend == true` (neues additives Goal-Feld in `ExecuteProgram.action`) werden zusammenhängende Move-Steps als **ein** `MotionSequenceRequest` an die **`/sequence_move_group`**-Action geschickt (Planner je Step, `blend_radius = c_dis`, letztes Item zwingend 0; `circ`-`via` als „interim"-Path-Constraint; Start-State leer → Pilz verkettet). Wait-Steps trennen eine Sequenz. Cancel bricht das Sequence-Goal ab. Capability `pilz_industrial_motion_planner/MoveGroupSequenceAction` in `moveit.launch.py` ergänzt. Damit ist auch die **`circ`-Laufzeit** frei (im Stop-at-each-point-Modus weiterhin sauber abgelehnt: „circ requires blend mode")
- [x] UI-Toggle „Blend through (Pilz)" im ProgramPanel (`QCheckBox`, setzt `goal.blend`; während Run gesperrt)
- [x] Polish (2026-06-13): erstes Sequence-Item bekommt den Live-Robot-State als Start-State → beseitigt das kosmetische `Found empty JointState message`-Log-Rauschen (move_group fährt ohnehin vom Ist-Zustand); CIRC-Fehlermeldung um den Kollinear-/Ebenen-Hinweis erweitert
- [~] **Echtes Live-Override** auf laufender Trajektorie: **als Limitation eingeordnet, nicht umgesetzt** — MoveIt/Pilz kann eine bereits gesendete Trajektorie nicht ohne Stop+Replan umskalieren. Override wirkt daher (wie dokumentiert) ab dem nächsten Step/der nächsten Sequenz. Echtes Mid-Trajektorien-Override bräuchte einen Stop-und-Replan-Mechanismus (eigener Folge-Task, kein reiner Phase-7-Scope)
- [x] Demos: `programs/program_blend_demo.yaml` (ptp+ptp mit `c_dis`+lin), `programs/program_circ_demo.yaml` (CIRC-Template); beide schema-valide
- [x] **Acceptance Blend bestätigt** (manuell gegen move_group, 2026-06-13): `program_blend_demo.yaml` mit `blend: true` → „MoveGroupSequenceAction" + „trajectory_blender_transition_window" + Intersection-Points → **eine** geblendete Trajektorie, SUCCEEDED. circ-Pfad bestätigt erreichbar (Pilz „Generating CIRC trajectory"); das Template scheitert erwartungsgemäß an kollinearen Platzhalter-Punkten → braucht reichbare, nicht-kollineare Bogen-Punkte
- [x] Headless getestet (2026-06-13, virtuell): Schema-Validierung (alle Programme + 3 Negativtests OK); C++-Loader akzeptiert `program_krl_demo.yaml` (`ptp`/`lin`), lehnt `circ`-ohne-`via`, joint-`via`, joint-Ziel bei `lin` und Fremd-Key `velocity` bei `ptp` mit präzisen Meldungen ab; Build sauber. Demo `programs/program_krl_demo.yaml` (`ptp`+`lin`)
- [x] **Pilz-Laufzeit bestätigt** (manuell gegen laufende move_group, 2026-06-13): `program_krl_demo.yaml` lief sichtbar über den Pilz-Pfad durch — „Generating PTP trajectory" (ptp) + „Generating LIN trajectory" (lin), beide Ausführungen SUCCEEDED, Programm beendet → State HOLD. (Segfault beim Ctrl-C-Teardown = bekannter Upstream-Bug, harmlos.) **Offen: circ/Blending** (Sequence-Modus)

### Phase 8 — Robustheit & Polish 🚧 in Arbeit (2026-06-13)

- [x] **Unit-Tests für den YAML-Parser** (`test/test_program_loader.cpp`, `ament_add_gtest`): 25 Tests, alle grün — Punkte (joint/pose, Quaternion-Norm, Frame, Keys, Namen, Version), beide Step-Vokabulare (`ptp`/`lin`/`circ`/`move_j`/`move_l`/`wait`, vel/acc/c_dis/via, Defaults), Reject-Pfade (circ-ohne-via, negatives c_dis, Skalierung außer Bereich, Fremd-Keys, leere Steps, …), `crossValidate`, `savePoints`-Round-Trip, `isValidPointName`, `Step::label`. Loader-Quelle wird in den Test einkompiliert (kein Link gegen den Node)
- [x] README-Eintrag im Hauptrepo aktualisiert: Step-Vokabular-Tabelle (KRL + Legacy), beide Run-Modi (`blend`), `circ`-Nicht-Kollinearität, Blend-CLI-Beispiel
- [x] Roadmap im Haupt-README angepasst: Programm-System als erledigt eingetragen (ersetzt für die Programmierung das geplante Web-Interface)
- [x] Fehlerbehandlung Executor — Notaus während Ausführung: bereits robust (der Worker spiegelt `/robot_state`; nach `/e_stop` → DISABLED gibt der Sequence-/Move-Action-Result `!=SUCCEEDED` zurück, der Executor bricht sauber ab und fasst den Zustand **nicht** an). Planungs-/Ausführungsfehler liefern präzise Meldungen (inkl. CIRC-Kollinear-Hinweis)
- [x] **Dry-Run / Simulationsmodus** (2026-06-13): additives Goal-Feld `dry_run`. `true` = ganzes Programm wird als Pilz-Sequenz **nur geplant** (`planning_options.plan_only`), die geplanten Trajektorien werden auf `/display_planned_path` publiziert und als RViz-Geist animiert (Dauer-getaktet, cancel-responsiv) — **der echte Arm bewegt sich nicht**, **kein** State-Wechsel (kein MOVEIT, Motoren bleiben wie sie sind), Waits werden übersprungen, läuft aus **jedem** Zustand. `circ`/Blending werden mitsimuliert (Sequence-Pfad). UI: Toggle „Simulate (dry run)" im ProgramPanel; Run ist im Dry-Run aus jedem Zustand freigegeben. Löst zugleich die „Vorschau zu schnell/überlappt"-Frage: Simulation = Geist allein, echter Lauf = man schaut auf den echten Arm
- [ ] Integrationstest für Executor (mit Sim oder ros2_control Mock) — offen (braucht laufende move_group/Controller; v1 über die manuellen Hardware-Läufe abgedeckt)

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

---

## Anhang: Vokabular-Referenzen für spätere Erweiterungen

Wenn ab Phase 7 das Step-Vokabular über die Move-Befehle hinaus erweitert wird (z.B. IO-Operationen, Wartebedingungen, Greifer-Befehle, Frame-/Tool-Konzepte), sind diese öffentlich dokumentierten Industrie-Sprachen als Inspiration sinnvoll:

- **KUKA KRL** — Move-Befehle (`PTP`, `LIN`, `CIRC`), `WAIT FOR`, `OUT`, Frame-Konzepte (`$BASE`, `$TOOL`). Sehr gut dokumentiert in Schulungsmaterial und Lehrbüchern. **Direkter Nutzen für Pilz**, daher Hauptreferenz ab Phase 7.
- **URScript** (Universal Robots) — Manual frei online verfügbar, breites Befehlsspektrum (Force-Modi, IO, Conditional Waits), pythonische Lesbarkeit. Gute Referenz wenn das Vokabular über reine Bewegungen hinaus wachsen soll.
- **ABB RAPID** — Sehr ausgereift, breit dokumentiert; gute Referenz für fortgeschrittene Konzepte wie Speed-Data, Zone-Data, Tool-Data, falls das System mal ein "professionelles" Datentyp-Modell bekommen soll.

**Nicht als Referenz nehmen**: KUKA Sunrise / LBR iiwa Java-API (RoboticsAPI). Proprietär, nur mit Sunrise.Workbench-Lizenz zugänglich. Der Java-Stack würde außerdem nicht in eine ROS-2-Architektur passen.

**Rechtliche Grenze**: Befehlsnamen, Parameter-Konventionen und Konzepte zu übernehmen ist üblich und legitim — APIs sind in dieser Form nicht durch Copyright geschützt. Direkte Implementierungen, Originaldokumentation oder Code-Snippets aus proprietären Quellen NICHT 1:1 übernehmen.
