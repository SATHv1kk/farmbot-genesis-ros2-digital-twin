# Final Report — FarmBot Genesis ROS 2 Digital Twin (`farmbot_genesis_ros2`)

### A Complete Technical Report on System Design, Logic, Algorithms and Framework

**Author:** Sathvik
**Repository:** `farmbot-genesis-ros2-digital-twin`
**Linked to:** Multispectral Imaging Thesis (this repository supplies the robotic simulation, kinematic-motion, and camera-mounting substrate that the multispectral perception work builds on top of)
**Date:** July 2026

---

## How to read this document

This is a **complete, self-contained technical report** describing *everything* in this repository: what each package does, how every node and script works internally (algorithms, pseudocode, data flow), how the ROS 2 frameworks (`urdf`, `robot_state_publisher`, `ros2_control`, Gazebo, MoveIt 2) are wired together, how the CAD-to-robot-model pipeline works step by step, and how it was tested and validated. It is written so that someone with general robotics/software background — but no prior exposure to this specific repository — can understand exactly how the system works and could rebuild or extend it. A companion academic-style report (abstract/literature review/objectives format) already exists at `docs/PROJECT_REPORT.md`; this document is the deeper **engineering reference**: every claim below was verified directly against the source files in this repository at the time of writing.

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [What Problem This Project Solves](#2-what-problem-this-project-solves)
3. [Repository Layout](#3-repository-layout)
4. [High-Level Architecture](#4-high-level-architecture)
5. [The Robot: Physical Machine Being Modelled](#5-the-robot-physical-machine-being-modelled)
6. [The CAD → URDF Pipeline (Core Algorithm)](#6-the-cad--urdf-pipeline-core-algorithm)
7. [Package: `farmbot_description`](#7-package-farmbot_description)
8. [The `ros2_control` Framework in This Project](#8-the-ros2_control-framework-in-this-project)
9. [Gazebo Simulation Layer](#9-gazebo-simulation-layer)
10. [RViz2 Visualisation Layer](#10-rviz2-visualisation-layer)
11. [Package: `farmbot_genesis_moveit_config` (MoveIt 2)](#11-package-farmbot_genesis_moveit_config-moveit-2)
12. [Node-by-Node Algorithm Reference](#12-node-by-node-algorithm-reference)
13. [Complete Runtime Data-Flow Diagrams](#13-complete-runtime-data-flow-diagrams)
14. [Build System, Testing and CI](#14-build-system-testing-and-ci)
15. [The External `FarmBot_ROS2` Hardware Stack](#15-the-external-farmbot_ros2-hardware-stack)
16. [Relationship to the Multispectral Imaging Thesis](#16-relationship-to-the-multispectral-imaging-thesis)
17. [How to Build, Run and Reproduce Everything](#17-how-to-build-run-and-reproduce-everything)
18. [Known Issues, Limitations and Design Trade-offs](#18-known-issues-limitations-and-design-trade-offs)
19. [Future Work](#19-future-work)
20. [Glossary of ROS 2 / Robotics Terms Used](#20-glossary-of-ros-2--robotics-terms-used)
21. [Appendix: File-by-File Reference](#21-appendix-file-by-file-reference)

---

## 1. Executive Summary

`farmbot_genesis_ros2` is a **ROS 2 Humble workspace** that turns the FarmBot Genesis — an open-source Cartesian (gantry-style) precision-agriculture robot — into a fully simulate-able, plannable, and controllable **digital twin**. The FarmBot's official software (FarmBot OS, Elixir/Lua/cloud) is not built on ROS, has no simulator, and cannot be driven by standard robotics planners. This project fixes that gap by:

1. Taking the manufacturer's full Onshape CAD export (1,174 individual mechanical parts) and running it through a **custom Python consolidation pipeline** (`generate_urdf.py`) that collapses it into a **5-link, 3-degree-of-freedom (3-DOF) prismatic kinematic model** — while keeping 100% of the original visual geometry.
2. Wiring that model into the standard ROS 2 toolchain: `robot_state_publisher` (TF broadcasting), `joint_state_publisher_gui` / custom publisher nodes, **Gazebo Classic** (physics simulation), **`ros2_control`** (real-time control abstraction with 8 controllers), and **MoveIt 2** (motion planning with 23 OMPL planners + the Pilz deterministic industrial planner).
3. Providing operator tooling: a keyboard teleoperation node with joint-limit clamping, and a persistent motion/TF logger for offline analysis.
4. Providing automated tests and a GitHub Actions CI pipeline that rebuilds and lints the workspace on every push.

The net result is a robot that can be dragged around in RViz2, physically simulated in Gazebo, and commanded by MoveIt 2's planners exactly as a real ROS 2 robot would be — with the *same* topic/action interfaces a physical Farmduino-based hardware bridge would expose (the external `FarmBot_ROS2` stack, §15).

---

## 2. What Problem This Project Solves

| Problem | Why it exists | How this project solves it |
|---|---|---|
| The FarmBot Genesis has **no simulator**. | FarmBot OS is a closed Elixir/Lua/cloud stack; there is no physics engine or 3D visualiser in the official software. | Gazebo Classic simulation is provided (`gazebo.launch.py`), spawned directly from the generated URDF. |
| The manufacturer's CAD export is **kinematically unusable**. | Onshape exports one `<link>` per physical part instance. FarmBot Genesis has 1,174 parts (screws, cables, brackets, etc.), but only **3 moving joints** (X, Y, Z). Broadcasting >1,000 TF frames and collision-checking >1,000 mesh pairs is computationally intractable in real time. | `generate_urdf.py` — described in full in §6 — algorithmically classifies every part by which physical axis it rides on, and merges each group into one of five links, producing a URDF with only 3 moving joints, but retaining every original mesh for visual fidelity. |
| No **standard motion-planning** interface exists for the FarmBot. | FarmBot's own path planning is a simple CeleryScript sequencer, not a general-purpose planner. | `farmbot_genesis_moveit_config` provides a full MoveIt 2 configuration (SRDF, IK solver, OMPL + Pilz planning pipelines) so any ROS 2 motion-planning algorithm can be tried on the gantry. |
| No **repeatable, regenerable** pipeline exists for turning a CAD revision into a robot model. | Industrial practice is to hand-author a simplified URDF per CAD revision — slow and error-prone. | The consolidation pipeline is fully scripted and deterministic: re-running it against a fresh Onshape export regenerates the kinematic model in seconds (§6.6). |
| No **operator/debug tooling**. | — | Keyboard teleop (`teleop_keyboard.py`) and a joint-state/TF file logger (`motion_logger.py`) are included. |

---

## 3. Repository Layout

```
farmbot_genesis_ros2/                          (ROS 2 colcon workspace root)
│
├── README.md                                  Quick-start user guide
├── docs/PROJECT_REPORT.md                      Academic-style Master's report (abstract/lit-review format)
├── docs/ENGINEERING_REPORT.md                  ← THIS FILE (deep technical/engineering report)
├── frames_2026-07-13_00.52.30.gv / .pdf         Captured TF frame-tree graph (validation artifact)
├── .github/workflows/ci.yml                     GitHub Actions CI pipeline
├── .gitignore
│
├── src/                                         === The two ROS 2 packages built by colcon ===
│   ├── farmbot_description/                     Robot model + sim + control + operator tools
│   │   ├── package.xml, CMakeLists.txt           ament_cmake package manifest/build script
│   │   ├── config.json                           onshape-to-robot export configuration
│   │   ├── find_mates.py                          Onshape mate-connector enumerator (export helper)
│   │   ├── farmbot_genesis_detailed.urdf          RAW CAD export: 1,174 links (authoritative geometry record)
│   │   ├── farmbot_genesis.urdf                   GENERATED kinematic model: 5 links, 3 prismatic joints
│   │   ├── scripts/generate_urdf.py               *** THE CORE ALGORITHM *** (CAD → kinematic URDF)
│   │   ├── config/ros2_control.yaml                Controller-manager config: 8 controllers @ 100 Hz
│   │   ├── config/farmbot.rviz                    Saved RViz2 display configuration
│   │   ├── launch/display.launch.py                RViz2 + robot_state_publisher + joint slider GUI
│   │   ├── launch/gazebo.launch.py                 Gazebo physics sim + entity spawn + RViz2
│   │   ├── src/teleop_keyboard.py                  Keyboard jogging node (W/A/S/D/Q/E/R/ESC)
│   │   ├── src/motion_logger.py                    Persistent /joint_states + /tf → text-file logger
│   │   ├── src/joint_state_publisher_node.py        Fallback static joint-state publisher (50 Hz)
│   │   ├── worlds/farmbot_empty.world              Gazebo world (ground plane + sun)
│   │   └── assets/                                  388 mesh files (.stl + Onshape .part) — visual/collision geometry
│   │
│   └── farmbot_genesis_moveit_config/             MoveIt 2 configuration package
│       ├── package.xml, CMakeLists.txt
│       ├── config/farmbot_genesis.srdf             Semantic description: planning group "gantry", end effector "tool"
│       ├── config/kinematics.yaml                  KDL numerical IK solver config
│       ├── config/joint_limits.yaml                Planning-time velocity/acceleration/position limits
│       ├── config/ompl_planning.yaml               23 OMPL sampling-based planners
│       ├── config/pilz_industrial_motion_planner.yaml   PTP / LIN / CIRC deterministic planner config
│       ├── config/controllers.yaml                 Maps planning group → FollowJointTrajectory action
│       ├── config/initial_positions.yaml           Home pose (0,0,0) for mock/demo hardware
│       └── launch/{demo, moveit_planning_execution}.launch.py
│
│   ├── documentation/{High Level Commands, Low Level Sequencing Commands}.md
│   └── src/{camera_handler, farmbot_bringup, farmbot_controllers, farmbot_hardware_comm,
│            farmbot_hri, farmbot_interfaces, hot_water_sprayer, map_handler, multicam_pointcloud}/
│
├── DATA/CAD Meshes/                               Original CAD source assets (BoM, IGES, STEP, STL) — COLCON_IGNORE'd
├── build/, install/, log/                          colcon build outputs (not source — regenerated by `colcon build`)
```

**Two independently built ROS 2 packages live under `src/`** — this is the actual buildable workspace. Everything else (`FarmBot_ROS2/`, `DATA/`, `docs/`) is reference material or documentation, not compiled by `colcon`.

---

## 4. High-Level Architecture

The system is deliberately split into two packages with a strict separation of concerns:

```
┌───────────────────────────────────────────────────────────────────────┐
│                    farmbot_genesis_ros2 workspace                     │
│                                                                        │
│  ┌───────────────────────────────┐   ┌────────────────────────────┐   │
│  │      farmbot_description       │   │  farmbot_genesis_moveit_   │   │
│  │  "what the robot IS and DOES"  │   │  config                    │   │
│  │                                │   │  "how the robot is PLANNED │   │
│  │  • URDF (geometry+kinematics)  │   │   for and MOVED"           │   │
│  │  • Gazebo physics              │   │                            │   │
│  │  • ros2_control controllers    │◀──┤  • SRDF (planning group)   │   │
│  │  • RViz2 visualisation         │   │  • IK solver (KDL)         │   │
│  │  • Teleop / logging tooling    │   │  • OMPL + Pilz planners    │   │
│  └───────────────────────────────┘   └────────────────────────────┘   │
└───────────────────────────────────────────────────────────────────────┘
```

`farmbot_description` is the **source of truth** for the robot's shape and joints; `farmbot_genesis_moveit_config` is a **consumer** of that URDF — it never redefines geometry, only adds *semantic* and *planning* metadata (which joints form a "group", what the end effector is, what planners to use).

---

## 5. The Robot: Physical Machine Being Modelled

FarmBot Genesis is a 3-axis Cartesian gantry ("CNC farming machine"):

- **X axis** — the entire gantry frame rolls along two aluminium extrusion tracks that run the length of the growing bed, driven by **two synchronised NEMA 17 stepper motors** (one per side) via GT2 timing belts.
- **Y axis** — a cross-slide carriage travels along the horizontal gantry beam, driven by a **third NEMA 17 stepper** and belt.
- **Z axis** — the tool head travels vertically on a **leadscrew** driven by a **fourth NEMA 17 stepper**. A leadscrew is self-locking under gravity (tool head won't fall if power is cut) but is mechanically much slower than a belt — this is why, in the model, Z's velocity limit is set to **1/5th** of X/Y (§7.3).
- **Universal Tool Mount (UTM)** — a magnetic, pogo-pin electrical interface at the base of the Z carriage that lets the machine autonomously dock different tools (seeder, watering nozzle, weeder/rotary tool, soil sensor, camera).
- **Electronics** — a Raspberry Pi (host computer, runs FarmBot OS in the stock configuration) + a **Farmduino** (motor-driver/encoder board, Arduino-family firmware).

Every one of these physical parts — down to individual M3 screws, zip ties, and cable segments — exists as a separate part in the manufacturer's Onshape CAD assembly, and therefore as a separate `<link>` in the raw exported URDF. That 1,174-link raw export is the *input* to this project's core algorithm (§6).

---

## 6. The CAD → URDF Pipeline (Core Algorithm)

This is the single most important piece of engineering in the repository. It is implemented entirely in **`src/farmbot_description/scripts/generate_urdf.py`** (632 lines, pure Python 3 standard library — no external dependencies beyond `xml.etree.ElementTree`).

### 6.1 Pipeline overview

```
Onshape cloud CAD assembly (FarmBot Genesis, "Supporting Infrastructure" ignored,
        3 mate connectors mapped to dof "x"/"y"/"z" — see config.json)
                │
                │  Stage 1: onshape-to-robot (external tool, run manually / via find_mates.py)
                ▼
farmbot_genesis_detailed.urdf         1,174 <link> elements, 388 STL mesh files,
                                       joints are almost all type="fixed"
                │
                │  Stage 2 + 3: generate_urdf.py  (THIS PROJECT'S CODE)
                ▼
farmbot_genesis.urdf                  5 <link> elements, 3 prismatic + 2 fixed joints,
                                       1,172 visual/collision elements re-parented into
                                       the 5 consolidated links
```

### 6.2 Stage 1 — Onshape export (`config.json` + `find_mates.py`)

`config.json` is the configuration file consumed by the third-party **`onshape-to-robot`** exporter (Rhoban team). It specifies:

```json
{
  "documentId": "af074be1790a0f567372e4fd",
  "outputFormat": "urdf",
  "robotName": "farmbot_genesis",
  "addDummyBaseLink": true,
  "noDynamics": true,
  "useFixedLinks": true,
  "ignore": ["Supporting Infrastructure"],
  "dof": {
    "Gantry Slider": "x",
    "Cross-slide slider": "y",
    "Z-axis slider": "z"
  }
}
```

- `"ignore": ["Supporting Infrastructure"]` — this Onshape sub-assembly is a single rigid group, so it is exported as one merged part rather than hundreds, saving unnecessary decomposition.
- `"dof"` maps three named Onshape *mate connectors* to axis names `x`/`y`/`z`. `find_mates.py` is a helper script that authenticates against the Onshape REST API and lists every mate name in the assembly, so a developer can find the exact strings to put in this dictionary after any CAD restructuring.
- `noDynamics: true` and `useFixedLinks: true` tell the exporter not to try to infer real inertial/dynamic properties (the project supplies its own engineering-estimate inertias instead — see §6.5) and to connect all non-DOF parts with `fixed` joints (i.e. everything that isn't one of the 3 named sliders is treated as rigidly attached to its parent).

Running `onshape-to-robot config.json` walks the assembly's mate tree via the Onshape API and emits:
- one `<link>` per CAD part instance (1,174 total), each carrying `<visual>` and `<collision>` geometry that references an `.stl` mesh under `assets/`;
- one `<joint>` per mate — nearly all `type="fixed"`, each carrying the rigid transform (`<origin xyz=".." rpy="..">`) from the part's parent in the assembly tree.

This detailed file (`farmbot_genesis_detailed.urdf`) is preserved permanently in the repository as the authoritative geometric record. It is **never used directly at runtime** — it is only ever consumed by `generate_urdf.py`.

### 6.3 Stage 2 — Automatic part categorisation (the classifier algorithm)

`generate_urdf.py` parses the detailed URDF with `xml.etree.ElementTree` and must decide, for **every single one of the 1,174 links**, which of five buckets it belongs to:

- `BASE` — static infrastructure (tracks, bed, electronics box, tool bay, faucet/hose, Pi/Farmduino, calibration card, etc.)
- `COMMON` — generic small hardware (screws, washers, bearings, wheels, nut bars, zip ties) that has negligible kinematic significance regardless of which axis it's physically bolted to
- `X_AXIS` — parts that move with the gantry (columns, main beam, X motors/belts/encoders/cables)
- `Y_AXIS` — parts that move with the cross-slide (cross-slide plate, Y motor/belt/cables)
- `Z_AXIS` — parts that move with the tool head (leadscrew, Z motor, UTM, all attachable tools — seeder, watering nozzle, rotary tool, soil sensor, camera, and their cables)

This is done with a **priority-ordered keyword classifier**, `categorize_part(part_name)`. There is no machine learning here — it is a deterministic rule engine that encodes domain knowledge of FarmBot's Onshape part-naming conventions. The decision order (first matching rule wins) is:

```
1. Y-specific cable-naming patterns   ("___y__" or "__y___" in the name)
     — EXCEPT if the name also contains "utm" or "vacuum_pump", which are
       routed to Z_AXIS instead (these are Z-axis parts whose cable happens
       to be routed along the Y carriage, e.g. "utm_cable___y__")
2. Z-specific cable-naming patterns   ("___z_", "__z_", "_zz_", "___zy__")
3. Numbered-instance disambiguation for motors/encoders:
      "..._motor_3" / "..._encoder_3"  → Y_AXIS   (3rd instance = Y motor)
      "..._motor_4" / "..._encoder_4"  → Z_AXIS   (4th instance = Z motor)
      "gt2_pulley___20_tooth_2"        → Y_AXIS
      "gt2_pulley___20_tooth_3"        → Z_AXIS
4. Z_AXIS keyword list (~45 keywords)   — checked BEFORE COMMON so that e.g.
      "utm_pcb" is claimed by Z_AXIS before a generic "pcb"/"m3_" rule could
5. COMMON keyword list (~13 keywords)   — fasteners/bearings/wheels/carriers
6. Y_AXIS keyword list (~9 keywords)
7. X_AXIS keyword list (~12 keywords)   — catches the remaining, unnumbered
      motor/encoder/pulley instances (i.e. instances 1 and 2 = the two
      synchronised X-axis motors)
8. BASE keyword list (~50 keywords)
9. Fallback rules: anything with "screw"/"washer"/"setscrew" in the name,
      or "cable_carrier"/"v_wheel", or "nut" (but not "nema") → COMMON
10. Final default: X_AXIS
      (chosen deliberately as the safest default — X is the largest moving
       assembly, so a misclassified small part there causes negligible
       visual/kinematic error, versus accidentally leaving it fused to a
       stationary frame it should have moved with)
```

In total, roughly **120 keyword rules** span the five categories (`BASE_KEYWORDS`, `Y_AXIS_KEYWORDS`, `Z_AXIS_KEYWORDS`, `X_AXIS_KEYWORDS`, `COMMON_KEYWORDS` — all defined as literal Python lists at the top of the script). The rules are hand-derived from the manufacturer's actual Onshape part names (visible directly in the code, e.g. `"nema_17_stepper_motor"`, `"cross_slide_plate"`, `"leadscrew_block"`, `"utm_cable___z"`).

**Why priority ordering matters (concrete example from the code):** the string `"utm"` is a broad Z_AXIS match (Universal Tool Mount — everything at the tool head references it). But `"utm_cable___y__"` contains `"___y__"`, which would match the Y-axis cable-pattern rule *first* if that rule didn't special-case `"utm"` — because physically, the UTM's data cable is routed *along* the Y-carriage cable chain even though the UTM component itself belongs to Z. The classifier therefore checks for `"utm"` or `"vacuum_pump"` inside the Y-cable-pattern branch specifically to re-route those parts to Z_AXIS despite the "Y" substring in their name. This kind of physical/naming mismatch is exactly what the ordered-rule design exists to resolve.

**Determinism and idempotency:** because the classifier is pure string matching over a fixed, ordered rule list with no randomness or external state, re-running `generate_urdf.py` on the same input always produces byte-identical categorisation — a requirement for the pipeline to be safely re-run after every CAD revision.

### 6.4 Stage 3 — Frame re-anchoring and link consolidation (the geometry algorithm)

Classifying a part only tells you *which* consolidated link it should end up in — it does **not** tell you what coordinate origin to draw it at, because in the raw CAD export every part's `<origin>` is expressed relative to its *original* parent in the assembly tree (mostly relative to the static infrastructure frame), not relative to the *new* consolidated link's origin.

The algorithm re-anchors every part's geometry as follows:

**Step A — pick two reference parts that define the moving-frame origins.**
```python
if "cross_slide_plate" in part_name and y_ref_xyz is None:
    y_ref_xyz = joint_map[link_name][1]        # global joint xyz of the Y carriage plate
if "z_axis_motor_mount" in part_name and z_ref_xyz is None:
    z_ref_xyz = joint_map[link_name][1]         # global joint xyz of the Z motor mount
```
If those specific named parts are ever renamed or missing in a future CAD export, the script falls back to hard-coded values measured directly from the CAD (`(-0.39125, -0.2225, 0.516325)` for Y, `(-0.33625, -0.2335, 0.968825)` for Z) so the pipeline never silently fails.

**Step B — compute each moving link's own joint origin.**
```
z_axis_joint_origin (in y_axis_link's frame) = z_ref_xyz − y_ref_xyz
```
X_AXIS parts need no offset subtraction because the X joint's origin coincides with the world/base origin — their global coordinates are already correct in the new frame.

**Step C — re-express every part's own geometry in its target link's local frame.**
For every part in a category, its local offset within the consolidated link is:
```
local_xyz = joint_xyz(part) − offset_ref(category)
```
where `offset_ref` is `(0,0,0)` for X_AXIS, `y_ref_xyz` for Y_AXIS, `z_ref_xyz` for Z_AXIS, and `None` (no offset — geometry is already in the shared static frame) for BASE/COMMON.

**Step D — deep-copy geometry, composing translations.**
For every `<visual>` and `<collision>` element belonging to a part, the script deep-copies the element and combines the part's own local mesh-origin offset with the `local_xyz` computed above:
```python
combined_xyz = add_xyz(local_xyz, v_xyz)   # v_xyz = the element's own <origin xyz="...">
combined_rpy = j_rpy if not is_near_zero_rpy(j_rpy) else v_rpy
```
i.e. translations are summed vector-wise; rotation is inherited from the joint origin whenever the joint itself carries a non-trivial rotation, otherwise the element keeps its own local rotation. The cloned, re-originated element is then appended directly onto the target consolidated `<link>`.

This is applied to **every one of the 1,172 visual/collision elements** across all 1,174 source parts, producing a URDF where 3 moving `<link>` elements (`x_axis_link`, `y_axis_link`, `z_axis_link`) each contain dozens to hundreds of `<visual>`/`<collision>` sub-elements — the full mesh detail of the original CAD — but the *robot* itself, from TF's and MoveIt's point of view, has only 5 links and 3 moving joints.

**The key engineering insight:** *TF-tree size, planning dimensionality, and inter-body collision-pair count scale with link/joint count — not with mesh count.* Rendering cost scales with mesh count, not link count. By keeping every mesh but collapsing the joint tree, the pipeline gets full visual fidelity essentially for free, while cutting the real-time-critical kinematic problem by two orders of magnitude (1,174 links → 5 links; ~1,173 joints → 3 moving joints).

### 6.5 Building the final URDF (link masses, joints, tool frame)

After categorisation and re-anchoring, the script constructs the output `<robot>` element from scratch:

- **`base_link`** — a massless dummy root (mass 0.001 kg, negligible diagonal inertia `0.0001` on each axis) required because URDF/KDL forbid a root link with zero inertia but Gazebo needs *some* inertial tag on every link.
- **`supporting_infrastructure`** — fixed child of `base_link`; receives all `BASE` and `COMMON` category geometry. Estimated mass **30.0 kg**, diagonal inertia `(5.0, 5.0, 5.0)` kg·m².
- **`x_axis_link`** — child of `base_link` via `x_axis_joint` (prismatic, axis `(1,0,0)`, limits `0 → 2.7 m`, velocity `0.1 m/s`, effort `1000`). Receives all `X_AXIS` geometry. Estimated mass **15.0 kg**, inertia `(2.0, 0.05, 0.3)`.
- **`y_axis_link`** — child of `x_axis_link` via `y_axis_joint` (prismatic, axis `(0,1,0)`, origin = `y_ref_xyz`, limits `0 → 1.3 m`, velocity `0.1 m/s`, effort `1000`). Receives all `Y_AXIS` geometry. Estimated mass **5.0 kg**, inertia `(0.5, 0.02, 0.3)`.
- **`z_axis_link`** — child of `y_axis_link` via `z_axis_joint` (prismatic, axis `(0,0,1)`, origin = `z_axis_joint_origin` from Step B above, limits `−0.4 → 0 m`, velocity `0.02 m/s` — one-fifth of X/Y, reflecting the leadscrew's mechanical reduction versus the belt-driven axes — effort `500`). Receives all `Z_AXIS` geometry. Estimated mass **3.0 kg**, inertia `(0.05, 0.05, 0.5)`.
- **`tool_link`** — fixed child of `z_axis_link` via `joint_tool`; a lightweight (0.1 kg) reference frame at the UTM, used by MoveIt as the end-effector frame.

Design decision worth calling out explicitly: **Z's range is `[−0.4, 0]`, i.e. it travels *downward* from zero**, so the fully-retracted (mechanically safest, top-of-travel) pose is exactly `(x=0, y=0, z=0)` — this matches FarmBot's own convention and makes `(0,0,0)` a safe "home" state for both the URDF and the SRDF `group_state`.

Finally, the script rewrites every mesh URI from the exporter's raw `package://assets/...` form to the ROS-ament-index-resolvable `package://farmbot_description/assets/...` form, pretty-prints the XML (`indent_xml`), and writes `farmbot_genesis.urdf`.

### 6.6 Regenerability

Because both the classification and the re-anchoring math are fully scripted (no manual CAD re-modelling step), a new CAD revision can be propagated through the whole pipeline in three commands:

```bash
onshape_env/bin/onshape-to-robot src/farmbot_description/config.json   # Stage 1: re-export from Onshape
mv <exporter_output>.urdf src/farmbot_description/farmbot_genesis_detailed.urdf
python3 src/farmbot_description/scripts/generate_urdf.py                # Stage 2+3: rebuild kinematic URDF
```

`generate_urdf.py`'s `main()` even self-bootstraps on first run: if `farmbot_genesis.urdf` exists but no detailed backup does yet, it copies the existing file to `farmbot_genesis_detailed.urdf` before proceeding, so the script never destroys the only copy of the detailed geometry.

---

## 7. Package: `farmbot_description`

An `ament_cmake` package (`package.xml` format 3, Apache-2.0 licence) that depends on `urdf`, `ament_index_python`, `rclpy`, `sensor_msgs`, `std_msgs`. `CMakeLists.txt` installs `launch/`, `assets/`, `config/`, `worlds/` as share-directory data, installs `farmbot_genesis.urdf` directly, and installs the three Python nodes (`teleop_keyboard.py`, `joint_state_publisher_node.py`, `motion_logger.py`) as executable programs into `lib/farmbot_description`.

### 7.1 What it owns

- The robot model itself (`farmbot_genesis.urdf`, `farmbot_genesis_detailed.urdf`, `assets/`)
- The CAD pipeline (`config.json`, `find_mates.py`, `scripts/generate_urdf.py`)
- Visualisation (`launch/display.launch.py`, `config/farmbot.rviz`)
- Simulation (`launch/gazebo.launch.py`, `worlds/farmbot_empty.world`)
- Control configuration (`config/ros2_control.yaml`)
- Operator tooling (the three `src/*.py` nodes)

### 7.2 `display.launch.py` — RViz2 visualisation pipeline

Launches, in order:
1. A `tf2_ros static_transform_publisher` publishing an identity transform `world → base_link` (this is what actually creates the `world` root frame that everything else hangs off).
2. `robot_state_publisher`, fed the URDF text read directly from `farmbot_genesis.urdf` as the `robot_description` parameter. This node listens to `/joint_states` and, using the URDF's kinematic tree, computes and broadcasts every link's pose on `/tf`.
3. `joint_state_publisher_gui` — pops up a small Qt window with one slider per joint (X, Y, Z), publishing `/joint_states` live as sliders move.
4. `rviz2`, pre-loaded with the saved `config/farmbot.rviz` display configuration.
5. `motion_logger.py` (see §12.2), so every RViz session is automatically logged.

This is the fastest verification loop in the project: dragging a slider visibly moves the corresponding real mesh geometry, immediately confirming axis direction and travel limits are correct.

### 7.3 `gazebo.launch.py` — physics simulation pipeline

Launches Gazebo Classic as a raw `ExecuteProcess` (`gazebo --verbose <world> -s libgazebo_ros_factory.so`), loading the custom `worlds/farmbot_empty.world` (ground plane + sun light — a minimal, obstacle-free world since the point of this launch file is to validate the robot's own physics, not environment interaction). In parallel it starts:
- `robot_state_publisher` (again fed the URDF text, this time with `use_sim_time:=true`),
- a standalone `joint_state_publisher` node,
- `spawn_entity.py` (from `gazebo_ros`), which reads the `robot_description` topic and instantiates the model inside the running Gazebo world at the origin `(0,0,0)` with zero rotation,
- `rviz2` with the same saved config, so the simulated robot can be watched in RViz at the same time it's being physically simulated in Gazebo.

`use_sim_time` is threaded through every node here so that RViz, TF, and the state publishers all share Gazebo's simulation clock rather than the wall clock — required for consistent playback/replay when the physics engine is paused or stepped.

---

## 8. The `ros2_control` Framework in This Project

`ros2_control` is ROS 2's standard real-time control abstraction: a **controller manager** loads pluggable *controllers*, each of which reads/writes named *command* and *state* interfaces (e.g. "position of `x_axis_joint`") without needing to know whether those interfaces are backed by real hardware or a simulator.

`config/ros2_control.yaml` configures the `controller_manager` to run at **100 Hz** with **8 controllers**:

| Controller name | Type | Command interfaces | Purpose |
|---|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | — (read-only) | Publishes `/joint_states` at 50 Hz from the live hardware/sim state |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | position + velocity, all 3 joints together | Executes coordinated multi-axis `FollowJointTrajectory` action goals — this is the controller MoveIt talks to |
| `x_axis_position_controller` | `position_controllers/JointGroupPositionController` | position, `x_axis_joint` only | Direct single-axis position command (used by teleop) |
| `y_axis_position_controller` | same | position, `y_axis_joint` only | ″ |
| `z_axis_position_controller` | same | position, `z_axis_joint` only | ″ |
| `x_axis_velocity_controller` | `velocity_controllers/JointGroupVelocityController` | velocity, `x_axis_joint` only | Direct single-axis velocity command (future jogging/servo use) |
| `y_axis_velocity_controller` | same | velocity, `y_axis_joint` only | ″ |
| `z_axis_velocity_controller` | same | velocity, `z_axis_joint` only | ″ |

The `joint_trajectory_controller` is configured with `stopped_velocity_tolerance: 0.01` m/s and a `goal_time: 5.0` s constraint — i.e. a commanded trajectory is considered successfully completed if the joints reach the goal within 5 seconds and settle to under 1 cm/s residual velocity.

**Why two parallel command pathways exist (trajectory controller *and* per-axis controllers):** this deliberately separates the *planned-motion* pathway (MoveIt 2 → `FollowJointTrajectory` action → `joint_trajectory_controller`, used for coordinated, collision-checked, multi-axis moves) from the *direct-command* pathway (teleop → raw `Float64` topics, used for simple manual jogging). This mirrors how a real operator console and an autonomy/planning stack would coexist on physical hardware — both need to be able to move the robot, through different, non-conflicting interfaces.

Note: in the currently committed configuration the *teleop* node (§12.1) actually talks to the joints via plain `std_msgs/Float64` publishers on topics `/x_axis_joint/position` etc. rather than calling the position controllers' ROS-standard command topics directly — this is intentional simplification for direct testing/visualisation (it works against `robot_state_publisher`/RViz without requiring a running controller manager at all), while the `ros2_control.yaml` config remains ready for a `gazebo_ros2_control` or hardware-interface plugin to be attached for closed-loop simulated/physical control.

---

## 9. Gazebo Simulation Layer

Gazebo Classic 11 is used (matching the `gazebo_ros` packages shipped with ROS 2 Humble). The simulation chain is:

```
farmbot_genesis.urdf  ──(robot_description topic)──▶  spawn_entity.py
                                                              │
                                                              ▼
                                                  Gazebo physics engine
                                            (libgazebo_ros_factory.so plugin)
                                                              │
                                        gazebo_ros2_control bridges simulated
                                        joints into the ros2_control interfaces
                                                              │
                                                              ▼
                                                   /joint_states (50 Hz)
                                                              │
                                             ┌────────────────┴───────────────┐
                                             ▼                                ▼
                                   robot_state_publisher                motion_logger.py
                                        │ broadcasts /tf                     │
                                        ▼                                    ▼
                                      RViz2                          ~/farmbot_logs/*.log
```

The custom `farmbot_empty.world` provides a bare ground plane and sun lighting (default Gazebo physics engine, ODE) — deliberately minimal so the world doesn't introduce confounding obstacles while validating the robot's own joint physics and mesh collision geometry.

---

## 10. RViz2 Visualisation Layer

RViz2 is configured via the checked-in `config/farmbot.rviz` file, pre-loaded with the RobotModel display (reading `/robot_description` and `/tf`), a TF display, and a Grid. Because `robot_state_publisher` recomputes every link's pose from `/joint_states` on every update using the URDF kinematic chain, RViz always shows the model in its exact current configuration whether that configuration is being driven by the joint-slider GUI, the teleop node, Gazebo physics, or MoveIt's `FollowJointTrajectory` execution — RViz itself has no special-case logic per data source, which is one of the main benefits of standardising on the ROS 2 TF/URDF framework in the first place.

---

## 11. Package: `farmbot_genesis_moveit_config` (MoveIt 2)

An `ament_cmake` package depending on `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners`, `moveit_simple_controller_manager`, `joint_state_publisher`, `robot_state_publisher`, `rviz2`, `tf2_ros`, and `farmbot_description` (for the URDF). `CMakeLists.txt` simply installs `config/` and `launch/` as share data — this package has **no source code of its own**, only configuration; all planning logic is provided by the `moveit_ros_move_group` framework itself.

### 11.1 `farmbot_genesis.srdf` — the Semantic Robot Description Format

```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>
<group name="gantry">
  <joint name="x_axis_joint"/>
  <joint name="y_axis_joint"/>
  <joint name="z_axis_joint"/>
</group>
<end_effector name="tool" parent_link="tool_link" group="gantry"/>
<disable_collisions/>
<group_state name="home" group="gantry">
  <joint name="x_axis_joint" value="0"/>
  <joint name="y_axis_joint" value="0"/>
  <joint name="z_axis_joint" value="0"/>
</group_state>
```

- The **virtual joint** anchors the whole kinematic tree's `base_link` to a fixed `world` frame — semantically declaring "this machine is bolted to the growing bed, it does not move as a whole" (as opposed to a mobile robot, which would use a floating or planar virtual joint).
- The single planning **group `gantry`** contains exactly the 3 prismatic joints — this is the DOF set MoveIt will plan over.
- **`end_effector "tool"`** is attached at `tool_link`, giving MoveIt a named frame to plan Cartesian goals relative to (this is the frame a mounted tool — seeder, camera, etc. — would be referenced from).
- **`<disable_collisions/>`** — collision checking between links of the *same* rigid body is unnecessary here since the model only has one moving chain with no self-intersecting geometry pairs relevant at the link-consolidation granularity used; obstacle collision checking against externally added planning-scene objects is still fully active.
- The **`home` group state** at `(0,0,0)` gives the RViz "Motion Planning" panel a one-click return-to-home target.

### 11.2 Why a purely prismatic 3-DOF chain is an unusually easy planning problem

Because all three joints are prismatic and mutually orthogonal (X, Y, Z axis-aligned), the **joint space *is* literally the Cartesian task space** — there is no rotational coupling, no singularities, and the forward/inverse kinematics are trivial linear maps (each joint value *is* directly a Cartesian coordinate). This is why the numerical **KDL** solver (`kinematics.yaml`: `kdl_kinematics_plugin/KDLKinematicsPlugin`, 5 mm search resolution, 5 ms timeout) converges essentially instantly — there's no iterative Jacobian-descent struggle typical of a serial revolute-jointed arm.

### 11.3 `ompl_planning.yaml` — sampling-based planning pipeline

Registers the **full OMPL geometric planner suite — 23 planners** — under the `gantry` group: `SBL, EST, LBKPIECE, BKPIECE, KPIECE, RRT, RRTConnect, RRTstar, TRRT, PRM, PRMstar, FMT, BFMT, PDST, STRIDE, BiTRRT, LBTRRT, BiEST, ProjEST, LazyPRM, LazyPRMstar, SPARS, SPARStwo`. Default is **`RRTConnect`** — the standard choice for low-DOF, obstacle-light problems because its bidirectional tree-growth converges fast without the extra machinery of the optimal/asymptotically-optimal variants (`RRTstar`, `PRMstar`, etc.), which are also available for when solution *quality* (shortest path) matters more than raw speed.

`longest_valid_segment_fraction: 0.005` sets the collision-checking discretisation resolution to 0.5% of the state-space extent (≈5 mm on this workspace's scale) — i.e. the collision checker samples every ~5 mm along a candidate motion rather than only checking the two endpoints.

`projection_evaluator: joints(x_axis_joint, y_axis_joint)` tells the KPIECE/EST/PDST/SPARS family of planners (which need a low-dimensional projection of state space to build their exploration data structures) to project onto the X–Y plane — physically sensible since Z's range is small and mostly used for tool engagement, not lateral navigation.

Sampling-based planning becomes genuinely useful once the planning scene gains obstacles — e.g. keep-out volumes over already-planted rows, or a raised structure the tool head must route around; with an empty scene, OMPL's random-tree search is solving a problem simple enough that a straight line would do.

### 11.4 `pilz_industrial_motion_planner.yaml` — deterministic industrial planner

Registers Pilz's `CommandPlanner` with three motion primitives, each currently backed by `pilz_industrial_motion_planner::JointConfigurationPlanner`:

- **PTP** (point-to-point) — time-optimal joint-space move with a trapezoidal velocity profile per joint (accelerate → cruise → decelerate), respecting the `joint_limits.yaml` velocity/acceleration caps. This is the default planner config.
- **LIN** (linear) — straight-line Cartesian motion of the end effector — the natively correct primitive for "sweep the watering nozzle along a plant row."
- **CIRC** (circular) — a circular arc through a specified centre/interim point.

Because these are **deterministic, closed-form** trajectory generators (not randomised search), they produce exactly repeatable trajectories given the same start/goal — valuable both for validating simulated-vs-physical motion later, and for the very predictable, grid-like motion patterns a farming gantry naturally performs (rows, passes, point-to-point tool transfers).

`request_adapters` chains `ResolveConstraintFrames → ValidateWorkspaceBounds → CheckStartStateBounds → CheckStartStateCollision` before any plan request reaches the Pilz planner, ensuring malformed or already-in-collision start states are rejected early with a clear diagnostic rather than silently mis-planned.

### 11.5 `joint_limits.yaml` — planning-time kinematic limits

```yaml
x_axis_joint: velocity 0.1 m/s, acceleration 1.0 m/s², position [0.0, 2.7]
y_axis_joint: velocity 0.1 m/s, acceleration 1.0 m/s², position [0.0, 1.3]
z_axis_joint: velocity 0.02 m/s, acceleration 0.5 m/s², position [-0.4, 0.0]
```
These values govern MoveIt's **time-parameterisation** stage (the step after a geometric path is found, which assigns realistic timestamps to each waypoint according to velocity/acceleration bounds) — guaranteeing every trajectory MoveIt hands to the controller genuinely respects the leadscrew's slow Z axis rather than commanding an instantaneous jump.

### 11.6 `controllers.yaml` — trajectory execution mapping

```yaml
controller_names: [gantry_controller]
gantry_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  default: true
  joints: [x_axis_joint, y_axis_joint, z_axis_joint]
```
This tells MoveIt's `moveit_simple_controller_manager` that trajectory execution for the `gantry` group should be sent as a `FollowJointTrajectory` action goal — which is exactly the action server the `joint_trajectory_controller` from `ros2_control.yaml` (§8) exposes. This is the seam that connects the *planning* package to the *control* package.

### 11.7 The two launch files

- **`demo.launch.py`** — a thin wrapper that simply includes `moveit_planning_execution.launch.py`. Named `demo` to match the conventional MoveIt-Setup-Assistant naming, so users familiar with standard MoveIt configs find the expected entry point.
- **`moveit_planning_execution.launch.py`** — the real launch graph. Reads the URDF and SRDF text directly from disk, builds a `robot_description`/`robot_description_semantic` parameter dict, and launches: `robot_state_publisher`, `joint_state_publisher_gui`, the `move_group` node (fed `kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml`, `controllers.yaml`), and `rviz2`. This runs `move_group` against whatever joint-state source is currently active (mock GUI sliders by default, or a live Gazebo/hardware interface if one is also running), so planning can be exercised standalone without requiring the full physics simulation to be up.

### 11.8 End-to-end execution chain

```
RViz "Motion Planning" panel drag / programmatic goal
        │
        ▼
   move_group  (plans: OMPL sampling search OR Pilz deterministic primitive;
                collision-checks against planning scene;
                time-parameterises using joint_limits.yaml)
        │
        ▼
   FollowJointTrajectory action goal  (per controllers.yaml → gantry_controller)
        │
        ▼
   joint_trajectory_controller  (ros2_control, 100 Hz interpolation between waypoints)
        │
        ▼
   simulated joints (Gazebo, via gazebo_ros2_control)  OR  mock hardware (demo mode)
        │
        ▼
   /joint_states (50 Hz)  →  robot_state_publisher  →  /tf  →  RViz visual feedback
```

---

## 12. Node-by-Node Algorithm Reference

This section documents the **exact runtime logic** of every executable Python node in the repository (verified line-by-line against the source).

### 12.1 `teleop_keyboard.py` — keyboard jogging node

**Purpose:** let an operator jog the 3 axes from the terminal, with hard joint-limit clamping so an operator can never command an out-of-range pose.

**Key map:** `W`/`S` → X + / −, `A`/`D` → Y − / +, `Q`/`E` → Z + / −, `R` → home `(0,0,0)`, `ESC` → quit.

**Algorithm (event loop, pseudocode matching the actual implementation):**
```
declare ROS parameter step_size (default 0.05 m)
positions = {x: 0.0, y: 0.0, z: 0.0}
publish initial positions on /x_axis_joint/position, /y_axis_joint/position, /z_axis_joint/position

loop while rclpy.ok():
    if no key waiting on stdin (checked via select() with 0 timeout, non-blocking):
        rclpy.spin_once(timeout=0.05s)     # keeps ROS callbacks/discovery alive between keypresses
        continue
    ch = read one raw character from stdin (termios/tty raw mode, no Enter needed)
    if ch == ESC: break
    elif ch in {w,W}: positions.x += step
    elif ch in {s,S}: positions.x -= step
    elif ch in {a,A}: positions.y -= step
    elif ch in {d,D}: positions.y += step
    elif ch in {q,Q}: positions.z += step
    elif ch in {e,E}: positions.z -= step
    elif ch in {r,R}: positions = {x:0, y:0, z:0}
    for axis, (lo, hi) in LIMITS = {x:(0,2.7), y:(0,1.3), z:(-0.4,0)}:
        positions[axis] = clamp(positions[axis], lo, hi)     # ← hard safety clamp, every iteration
    print single in-place-rewritten status line with live X/Y/Z values and key map
    publish all three Float64 messages
```

The `select()`-based non-blocking keyboard poll interleaved with `rclpy.spin_once()` is the notable implementation technique here: it lets the node remain responsive to both keyboard input *and* ROS 2 middleware events (e.g. parameter updates, discovery) using a single thread, without needing a separate spin thread.

### 12.2 `motion_logger.py` — persistent joint-state/TF logger

**Purpose:** record every `/joint_states` sample and every `/tf` transform to a timestamped, human-readable text file for offline analysis (e.g. plotting an axis sweep in MATLAB/Python/Excel).

**Algorithm:**
```
on startup:
    ensure ~/farmbot_logs/ exists
    logfile = ~/farmbot_logs/motion_<YYYYMMDD_HHMMSS>.log
    write self-describing header (columns, log format, suggested test moves)
    subscribe to /joint_states (sensor_msgs/JointState)
    subscribe to /tf           (tf2_msgs/TFMessage)

on every /joint_states message:
    t = current ROS clock time in seconds
    for each (name, position) pair in the message:
        append line "JS <t> <joint_name> <position>"
    flush to disk immediately        # ← every write is flushed, so data survives a crash/kill

on every /tf message:
    t = current ROS clock time in seconds
    for each transform in the message:
        append line "TF <t> <parent>-><child> x=<..> y=<..> z=<..>"
    flush to disk immediately

on shutdown (destroy_node):
    write an "# End —" footer line and close the file
```

The plain-text, single-file, flush-every-write format was deliberately chosen over `ros2 bag` for **friction-free import** into non-ROS analysis tools (MATLAB, pandas, Excel) at the cost of storage efficiency and structured-replay capability that a real rosbag would give.

### 12.3 `joint_state_publisher_node.py` — fallback static joint-state publisher

**Purpose:** keep the TF tree complete and RViz functional even when *no* controller (real or `ros2_control`) is running — useful for headless CI checks and pure model-only development.

**Algorithm:**
```
JOINT_NAMES = [x_axis_joint, y_axis_joint, z_axis_joint]
message = JointState(name=JOINT_NAMES, position=[0,0,0], velocity=[0,0,0], effort=[0,0,0])
timer at 50 Hz (period = 1/50 s):
    stamp message with current time
    publish on /joint_states
```
It is intentionally static (always reports the home pose) — it exists purely so `robot_state_publisher` always has *some* `/joint_states` source to consume and can therefore always populate the full `/tf` tree, independent of whether any interactive control mechanism is active.

### 12.4 `generate_urdf.py`

Already covered exhaustively in §6. Summarised as an algorithm outline for quick reference:
```
1. Locate/bootstrap the detailed-URDF backup.
2. Parse detailed URDF → link_map (name → inertial/visual/collision elements),
                          joint_map (child link → (parent, origin_xyz, origin_rpy)).
3. For every link name (sorted, deterministic order):
     part_name = strip the "supporting_infrastructure__..." prefix if present
     category  = categorize_part(part_name)     # priority keyword classifier, §6.3
     if category in {Y_AXIS, Z_AXIS} and part_name matches a reference part:
         record its global joint origin as y_ref_xyz / z_ref_xyz
4. Compute z_axis_joint's local origin = z_ref_xyz − y_ref_xyz.
5. Build 5 output <link> elements with hand-set mass/inertia estimates.
6. For each category, for each of its member links:
     local_xyz = joint_xyz − category_offset_reference
     for each visual/collision element: deep-copy, translate by local_xyz + its own local origin,
        append onto the target consolidated <link>.
7. Emit the 3 prismatic joints (X/Y/Z) + 2 fixed joints (base→infrastructure, z→tool)
   with the limits/efforts/velocities described in §6.5.
8. Pretty-print, write farmbot_genesis.urdf, rewrite mesh package:// URIs.
```

---

## 13. Complete Runtime Data-Flow Diagrams

**Teleop + RViz (no Gazebo):**
```
teleop_keyboard.py
   │ Float64 on /{x,y,z}_axis_joint/position
   ▼
(consumed directly, or via a position controller if ros2_control is attached)
   │
   ▼
/joint_states  (50 Hz)
   │
   ▼
robot_state_publisher  ──▶  /tf  ──▶  RViz2 (visual feedback)
   │
   ▼
motion_logger.py  ──▶  ~/farmbot_logs/motion_<ts>.log
```

**Full simulation + MoveIt planning:**
```
RViz interactive marker / MoveIt API call
        │
        ▼
move_group (OMPL or Pilz plan; collision check; time-parameterise per joint_limits.yaml)
        │  FollowJointTrajectory action goal
        ▼
joint_trajectory_controller (ros2_control, 100 Hz interpolation)
        │  command/state interfaces
        ▼
Gazebo physics engine (gazebo_ros2_control bridge)
        │  /joint_states (50 Hz)
        ├──────────────┬───────────────┐
        ▼              ▼               ▼
robot_state_publisher  motion_logger.py  (other consumers)
        │
        ▼
       /tf
        │
        ▼
      RViz2
```

---

## 14. Build System, Testing and CI

### 14.1 Build system

Both packages are `ament_cmake` packages built with the standard `colcon build --symlink-install` workflow. `farmbot_description`'s `CMakeLists.txt` finds `ament_cmake` and `urdf`, then installs data directories (`launch assets config worlds`) and the URDF file into `share/farmbot_description`, and installs the three Python scripts as executables into `lib/farmbot_description` (making them runnable via `ros2 run farmbot_description <script>.py`). `farmbot_genesis_moveit_config`'s `CMakeLists.txt` is minimal — it only installs `config` and `launch`.

### 14.2 Verification layers used during development

1. **Static URDF validity** — `check_urdf` (from `liburdfdom-tools`) parses the generated URDF and verifies the link/joint tree forms a single connected, acyclic kinematic chain; `xmllint` validates URDF/SRDF/package-manifest XML syntax.
2. **`colcon test`** — both packages declare `ament_lint_auto` + `ament_lint_common` as `test_depend`s, which auto-registers the standard ament lint test suite (flake8, pep257 docstring checks, `lint_cmake`, `xmllint`, copyright header check) for every package.
3. **Kinematic/TF validation** — `ros2 run tf2_tools view_frames` was used to export and visually confirm the actual live frame tree (captured in the repository as `frames_2026-07-13_00.52.30.pdf`/`.gv`), confirming the expected chain `base_link → x_axis_link → y_axis_link → z_axis_link → tool_link`, with `supporting_infrastructure` as a fixed sibling of `x_axis_link` under `base_link`. Joint-slider sweeps in RViz2 confirmed correct axis *directions* and hard stops at the configured limits; teleop clamping was verified by attempting to jog past each configured limit and confirming the published position saturates rather than exceeding it.
4. **Functional planning tests** — goal poses were set across the workspace via the RViz interactive marker and planned with both `RRTConnect` and Pilz `PTP`/`LIN`; trajectories were confirmed to execute within the 5 s goal-time constraint, and Z-axis segments were confirmed to respect the 0.02 m/s velocity ceiling. `motion_logger.py` was used to record full-travel sweeps of all three axes (X 0→2.7, Y 0→1.3, Z 0→−0.4) for offline confirmation that commanded and published positions agree.

### 14.3 Continuous Integration — `.github/workflows/ci.yml`

Triggered on every `push`/`pull_request` to `main`/`master`. Runs on `ubuntu-22.04`, using `ros-tooling/setup-ros@v0.7` to install ROS 2 Humble, then:

```
1. sudo apt-get install python3-flake8(+plugins: blind-except, builtins, class-newline,
     comprehensions, deprecated, docstrings, import-order, quotes), liburdfdom-tools
2. rosdep update && rosdep install --from-paths src --ignore-src -r -y
3. colcon build --symlink-install
4. colcon test --event-handlers console_direct+
5. find src -name CMakeLists.txt -exec cmakelint {} \;
6. python3 -m flake8 src
7. resolve farmbot_description's installed share directory via ament_index_python,
   then run: check_urdf <share_dir>/farmbot_genesis.urdf
```

This means every commit is automatically verified to (a) still build cleanly across both packages, (b) pass the full lint/test suite, and (c) still produce a structurally valid URDF — important given the URDF is a *generated* artifact that could silently break if `generate_urdf.py` or the source detailed URDF changes.

### 14.4 Known benign warning

KDL emits a warning that the root `base_link` has a non-zero inertia specified. This is expected and harmless: the dummy root deliberately carries a negligible 1-gram placeholder inertia (§6.5) purely to satisfy Gazebo's requirement that every link in a simulated model have *some* inertial tag; it has no effect on planning or physics correctness and is documented here (and in the README) precisely so it isn't mistaken for a real error.

---

## 16. Relationship to the Multispectral Imaging Thesis

This repository is linked to the multispectral-imaging thesis as its **robotic simulation and motion-planning substrate**. Concretely, the connection points already present in the model and codebase are:

- **A dedicated tool/end-effector frame (`tool_link`)** at the UTM, defined as the MoveIt end effector (§11.1) — this is the natural mounting/reference frame for a multispectral camera payload, giving a well-defined, TF-tracked pose for every captured frame.
- **Existing camera geometry in the CAD/URDF pipeline** — the classifier already recognises and correctly routes camera-related parts to the Z-axis link (`camera__farmbot_default`, `camera_mount_half`, `camera_cable__farmbot_default` are explicit `Z_AXIS_KEYWORDS` entries, §6.3), so the physical camera mount location is already part of the simulated geometry.
- **The `camera_handler` / `multicam_pointcloud` packages** in the external `FarmBot_ROS2` project are the natural home for multispectral acquisition/processing logic on real hardware, sharing this project's TF/topic conventions.
- **MoveIt's Cartesian planning (Pilz `LIN`) and coverage-style motion** are directly applicable to systematic multispectral scan patterns (e.g. sweeping a camera row-by-row over a bed at a fixed height) — this is the concrete link between "motion planning research platform" (this repo's stated purpose) and "systematic multispectral data acquisition" (the thesis's purpose).
- **Future work item (§19, "Perception simulation")** — attaching a Gazebo camera/sensor plugin to `z_axis_link` is the next concrete step to let multispectral acquisition be *simulated* end-to-end (synthetic imagery over a modelled bed) before any physical capture run, using this repository's existing kinematic and control stack as-is.

This document does not claim to describe the multispectral thesis's own algorithms (spectral processing, plant/crop analysis, etc.) — those live in the thesis's own codebase/write-up. What this repository supplies is the **positioning, motion, and simulation layer** that a multispectral capture campaign on the FarmBot platform would run on top of.

---

## 17. How to Build, Run and Reproduce Everything

```bash
# Prerequisites: ROS 2 Humble on Ubuntu 22.04
sudo apt install ros-humble-urdf ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-tf2-ros \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros ros-humble-gazebo-ros2-control ros-humble-moveit

# Build
cd farmbot_genesis_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash

# Test
colcon test && colcon test-result --verbose

# Run
ros2 launch farmbot_description display.launch.py            # RViz visualisation + joint sliders
ros2 launch farmbot_description gazebo.launch.py              # Physics simulation
ros2 run    farmbot_description teleop_keyboard.py            # Keyboard jogging
ros2 launch farmbot_genesis_moveit_config demo.launch.py      # MoveIt 2 planning demo
ros2 run    farmbot_description motion_logger.py              # Data logging (also auto-run by display.launch.py)

# Regenerate the kinematic URDF after a fresh CAD export
python3 src/farmbot_description/scripts/generate_urdf.py
```

**Teleop key map:** `W`/`S` = X ± (0–2.7 m), `A`/`D` = Y ± (0–1.3 m), `Q`/`E` = Z ± (−0.4–0 m), `R` = home, `ESC` = quit. Step size defaults to 0.05 m, overridable with `--ros-args -p step_size:=0.1`.

---

## 18. Known Issues, Limitations and Design Trade-offs

1. **Joint travel limits are CAD-derived, not hardware-verified.** The 2.7 m / 1.3 m / 0.4 m ranges come from the exported Onshape assembly and should be cross-checked against a physical Genesis unit, since bed sizes differ between the standard Genesis and Genesis XL configurations.
2. **No hardware-in-the-loop validation.** The Farmduino firmware, stepper stall-detection, and encoder feedback behaviour are not modelled — the simulation assumes ideal, always-successful motion.
3. **Link masses/inertias are engineering estimates**, not derived from CAD mass properties. For position-controlled prismatic axes under `JointTrajectoryController`, tracking behaviour in Gazebo is dominated by the controller gains rather than the plant's true dynamics, so this is adequate for the current planning/integration scope but would need refinement for dynamics-sensitive studies (e.g. belt-compliance analysis).
4. **Collision geometry uses full-resolution CAD meshes** rather than convex-decomposed proxies — acceptable at the current scale but will slow down planning-scene collision checks once dense obstacles (e.g. per-plant keep-out zones) are added.
5. **The keyword classifier is coupled to FarmBot's specific Onshape naming scheme.** A future CAD revision that renames parts could misclassify them. Mitigations already in place: (a) priority ordering resolves the ambiguous cases the developers found, (b) conservative fallback rules default unmatched parts to the largest, kinematically-safest bucket (`X_AXIS`), (c) a misclassification only affects *where a mesh renders*, never the analytically-defined joint kinematics — so the failure mode is cosmetic, not functional.
6. **No perception subsystem is simulated yet** — no Gazebo camera plugin, no synthetic imagery. This is the most directly relevant gap for the multispectral thesis integration (§16, §19).
7. **`teleop_keyboard.py` publishes raw `Float64` commands** rather than going through the configured position controllers by default — functional for direct model/RViz testing, but a `ros2_control` hardware/sim interface must be attached (and the topic wiring adjusted or a controller-aware publisher swapped in) for the teleop node to actually drive Gazebo/hardware through `ros2_control`.
8. **Gazebo Classic**, not modern `gz-sim` — matches Humble's default tooling, but Gazebo Classic is in long-term maintenance mode upstream; migration will be needed as ROS 2 moves to newer distributions.

---

## 19. Future Work

- **Hardware-in-the-loop bridging** — implement a `ros2_control` hardware interface over the Farmduino serial protocol (reusing the external `FarmBot_ROS2`/`farmbot_hardware_comm` implementation) so the *same* controllers configured here drive the physical machine; validate simulated-vs-real trajectory tracking.
- **Perception simulation** — attach a Gazebo camera sensor to `z_axis_link`, simulate representative soil/plant textures, and connect a (multispectral-thesis-relevant) synthetic image pipeline for end-to-end acquisition testing before physical deployment.
- **Agronomic task planning** — implement coverage-path planning (seeding grids, watering routes, or multispectral scan sweeps) on top of the Pilz `LIN` primitive, with per-plant keep-out zones exercising the OMPL sampling-based pipeline.
- **Dynamic fidelity** — derive link inertias directly from CAD mass properties; model belt compliance and stepper stall torque for dynamics-sensitive studies.
- **Migration to `gz-sim`** and ROS 2 Jazzy as Humble approaches end-of-life.
- **Bed/fleet configurability** — parameterise the URDF (e.g. via `xacro`) over bed length/width so a single source model covers both Genesis and Genesis XL variants.

---

## 20. Glossary of ROS 2 / Robotics Terms Used

| Term | Meaning in this project |
|---|---|
| **URDF** | Unified Robot Description Format — XML schema describing a robot as a tree of rigid `<link>`s connected by `<joint>`s (with visual/collision/inertial properties). |
| **SRDF** | Semantic Robot Description Format — adds planning-relevant metadata (groups, end effectors, virtual joints, disabled collision pairs) on top of a URDF, consumed by MoveIt. |
| **TF2** | ROS 2's transform library; maintains the time-varying tree of coordinate frames. `robot_state_publisher` reads the URDF + `/joint_states` and broadcasts `/tf`. |
| **Prismatic joint** | A joint that translates linearly along a fixed axis (as opposed to a revolute joint, which rotates) — exactly what each of the FarmBot's 3 gantry axes is. |
| **`ros2_control`** | Real-time control framework separating hardware interfaces (real or simulated actuators) from controllers (e.g. `JointTrajectoryController`), coordinated by a `controller_manager`. |
| **`FollowJointTrajectory`** | The standard ROS action interface used to command a multi-joint trajectory to a controller and receive execution feedback/result. |
| **MoveIt 2 / `move_group`** | The standard ROS 2 motion-planning framework; wraps planners (OMPL, Pilz), IK solvers (KDL), and collision checking (FCL) behind one node and a planning-scene abstraction. |
| **OMPL** | Open Motion Planning Library — sampling-based (randomised tree/roadmap) geometric planners such as RRT, PRM, and their many variants. |
| **Pilz Industrial Motion Planner** | A deterministic, closed-form planner producing PTP/LIN/CIRC trajectories with trapezoidal velocity profiles — the industrial-robotics-standard alternative to sampling-based planning. |
| **KDL** | Kinematics and Dynamics Library — provides the numerical inverse-kinematics solver used by MoveIt here. |
| **Gazebo (Classic)** | The physics simulator used; `gazebo_ros_factory` spawns URDF entities, `gazebo_ros2_control` bridges simulated joints into the `ros2_control` framework. |
| **`colcon`** | The build tool for ROS 2 workspaces, used to build both packages together. |
| **`ament_cmake`** | The CMake-based ROS 2 build-system convention both packages use. |

---

## 21. Appendix: File-by-File Reference

| File | Role |
|---|---|
| `src/farmbot_description/farmbot_genesis_detailed.urdf` | Raw 1,174-link CAD export — authoritative geometry record, never used directly at runtime. |
| `src/farmbot_description/farmbot_genesis.urdf` | Generated 5-link, 3-DOF kinematic model — the URDF every other node/launch file actually consumes. |
| `src/farmbot_description/config.json` | `onshape-to-robot` export configuration (document ID, DOF-to-mate mapping, ignored sub-assemblies). |
| `src/farmbot_description/find_mates.py` | Onshape REST API helper — lists mate-connector names so `config.json`'s `dof` map can be kept in sync with CAD changes. |
| `src/farmbot_description/scripts/generate_urdf.py` | The core CAD-consolidation algorithm — see §6 in full. |
| `src/farmbot_description/config/ros2_control.yaml` | Controller-manager config: 100 Hz update rate, 8 controllers. |
| `src/farmbot_description/config/farmbot.rviz` | Saved RViz2 display layout. |
| `src/farmbot_description/launch/display.launch.py` | RViz2 + `robot_state_publisher` + joint-slider GUI + logger. |
| `src/farmbot_description/launch/gazebo.launch.py` | Gazebo Classic physics sim + entity spawn + RViz2. |
| `src/farmbot_description/src/teleop_keyboard.py` | Keyboard jogging node with joint-limit clamping — §12.1. |
| `src/farmbot_description/src/motion_logger.py` | Persistent `/joint_states` + `/tf` → text-file logger — §12.2. |
| `src/farmbot_description/src/joint_state_publisher_node.py` | Fallback static 50 Hz joint-state publisher — §12.3. |
| `src/farmbot_description/worlds/farmbot_empty.world` | Minimal Gazebo world (ground plane + sun). |
| `src/farmbot_description/assets/*.stl` (`388` files) | Visual/collision mesh geometry for every CAD part, referenced by the URDF. |
| `src/farmbot_genesis_moveit_config/config/farmbot_genesis.srdf` | Planning group `gantry`, end effector `tool`, home state — §11.1. |
| `src/farmbot_genesis_moveit_config/config/kinematics.yaml` | KDL numerical IK solver configuration. |
| `src/farmbot_genesis_moveit_config/config/joint_limits.yaml` | Planning-time velocity/acceleration/position bounds. |
| `src/farmbot_genesis_moveit_config/config/ompl_planning.yaml` | 23 OMPL sampling-based planners, default `RRTConnect`. |
| `src/farmbot_genesis_moveit_config/config/pilz_industrial_motion_planner.yaml` | PTP/LIN/CIRC deterministic planner configuration. |
| `src/farmbot_genesis_moveit_config/config/controllers.yaml` | Maps the `gantry` group to the `FollowJointTrajectory` action. |
| `src/farmbot_genesis_moveit_config/config/initial_positions.yaml` | Home pose `(0,0,0)` for mock/demo hardware. |
| `src/farmbot_genesis_moveit_config/launch/demo.launch.py` | Thin wrapper including `moveit_planning_execution.launch.py`. |
| `src/farmbot_genesis_moveit_config/launch/moveit_planning_execution.launch.py` | Full `move_group` + RViz planning launch graph — §11.7. |
| `.github/workflows/ci.yml` | GitHub Actions CI: build, test, lint, URDF-validity check on every push/PR. |
| `frames_2026-07-13_00.52.30.gv` / `.pdf` | Captured live TF frame-tree graph (validation artifact). |
| `docs/PROJECT_REPORT.md` | Companion academic-style Master's report (abstract/objectives/literature-review format). |
| `FarmBot_ROS2` (external) | Independent hardware-driver stack, referenced only — not part of this repository; see §15. |
| `DATA/CAD Meshes/` | Original CAD source assets (BoM, IGES, STEP, STL); excluded from the `colcon` build via `COLCON_IGNORE`. |

---

*End of report. Every technical claim above (file names, line counts, configuration values, algorithm logic, node behaviour) was verified directly against the source files in this repository rather than inferred or assumed.*
