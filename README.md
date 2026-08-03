# FarmBot Genesis — ROS 2 Digital Twin

[![FarmBot Genesis CI](https://github.com/SATHv1kk/farmbot-genesis-ros2-digital-twin/actions/workflows/ci.yml/badge.svg)](https://github.com/SATHv1kk/farmbot-genesis-ros2-digital-twin/actions/workflows/ci.yml)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-Apache%202.0-blue)

A full ROS 2 (Humble) simulation stack for the **FarmBot Genesis** — an open-source Cartesian precision-farming robot. This workspace turns FarmBot's official (non-ROS, simulator-less) CAD model into a physics-simulated, motion-plannable digital twin: kinematic URDF, Gazebo physics, MoveIt 2 motion planning, keyboard teleop, and a CI pipeline that builds and tests it on every push.

**Full write-ups:** [`docs/ENGINEERING_REPORT.md`](docs/ENGINEERING_REPORT.md) (line-by-line technical reference — every node, algorithm, and data flow) · [`docs/PROJECT_REPORT.md`](docs/PROJECT_REPORT.md) (Master's thesis report — motivation, literature review, results)

---

## Highlights

- **1,174 CAD parts → 5 kinematic links, automatically.** The manufacturer's Onshape export has one link per physical part (screws, cables, brackets — everything). A deterministic classifier (`scripts/generate_urdf.py`) sorts every part by which physical axis it rides on and consolidates them into a 5-link, 3-DOF prismatic model, keeping 100% of the original mesh detail for rendering while cutting the TF tree and planning dimensionality by two orders of magnitude. Full algorithm writeup: [ENGINEERING_REPORT §6](docs/ENGINEERING_REPORT.md#6-the-cad--urdf-pipeline-core-algorithm).
- **Collision geometry sized for real-time physics.** A second pass (`scripts/simplify_collisions.py`) replaces the hundreds of concave per-part collision meshes on each link with a single axis-aligned box derived from the true world-space extent of that link's visuals — concave-trimesh collision is what makes Gazebo/ODE slow or unstable, box primitives aren't.
- **Live teleop, no controller required.** `teleop_keyboard.py` and the fallback `joint_state_publisher_node.py` talk to each other directly over per-axis position topics, so the model moves in RViz/Gazebo from the keyboard even with no `ros2_control` hardware interface attached.
- **Dual motion-planning pipelines.** MoveIt 2 configured with all 23 OMPL sampling-based planners *and* Pilz's deterministic PTP/LIN/CIRC industrial planner — the latter producing the same repeatable trajectory every time, which is what a real seeding/watering pass needs.
- **CI-verified, not just "should work."** Every push builds the full workspace on Ubuntu 22.04 + ROS 2 Humble, runs the test suite, checks URDF validity with `check_urdf`, and lints Python/CMake — see the badge above.

---

## Table of Contents

- [Onshape Assembly (CAD Source)](#onshape-assembly-cad-source)
- [Third-Party Data Notice](#third-party-data-notice)
- [Project Structure](#project-structure)
- [Kinematic Chain](#kinematic-chain)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Configuration](#configuration)
- [Regenerating the URDF](#regenerating-the-urdf)
- [CI Pipeline](#ci-pipeline)
- [License](#license)
- [Notes](#notes)

---

## Onshape Assembly (CAD Source)

The robot geometry originates from the manufacturer's official **FarmBot Genesis** Onshape assembly, not a CAD model authored in this repository:

- **Document ID:** `af074be1790a0f567372e4fd` ([open in Onshape](https://cad.onshape.com/documents/af074be1790a0f567372e4fd/w/59c9afe8f4d523552701f562/e/3b072c92ea0892bc7dc55f17))
- **Export tool:** [`onshape-to-robot`](https://github.com/Rhoban/onshape-to-robot) (Rhoban team), configured via `src/farmbot_description/config.json`
- **Raw export:** `farmbot_genesis_detailed.urdf` — 1,174 individually-named `<link>` elements, one per physical CAD part, each backed by an STL mesh under `assets/`. This file is preserved as-is and never hand-edited; it is the authoritative geometric record.
- **What *is* original to this project:** `scripts/generate_urdf.py`, the algorithm that classifies all 1,174 raw CAD parts by which physical axis they ride on and consolidates them into the 5-link, 3-DOF kinematic model (`farmbot_genesis.urdf`) that everything else in this workspace actually runs against, plus `scripts/simplify_collisions.py`, which reduces each link's collision geometry to a single box primitive — see [`docs/ENGINEERING_REPORT.md`](docs/ENGINEERING_REPORT.md) §6 for the full algorithm writeup.

In short: the mesh geometry is the manufacturer's design, exported via a third-party tool; the kinematic consolidation, control/planning configuration, and tooling built on top of it are this project's own work.

## Third-Party Data Notice

Data in this project that originates from other sources:

| Data | Source | License |
|------|--------|---------|
| `src/farmbot_description/assets/*.stl` (194 mesh files) | [FarmBot Genesis](https://genesis.farm.bot/docs/cad) official Onshape CAD, exported with [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) | [CC0 Public Domain](https://farm.bot/pages/open-source) (FarmBot Inc.) |
| `src/farmbot_description/assets/*.part` | Onshape part metadata written by onshape-to-robot during export (records the exact source document/element of each mesh) | CC0 (derived from the same CAD) |
| `src/farmbot_description/farmbot_genesis_detailed.urdf` | Raw onshape-to-robot export of the full assembly (1,174 parts) | CC0 (geometry) |
| `DATA/` (not in git, local only) | [Unofficial FarmBot Genesis XL CAD export](https://spacecruft.org/spacecruft/farmbot-genesis-xl-cad) — STEP/IGES/STL archives used as offline reference | CC0 (FarmBot Inc. design) |

Everything else (launch files, scripts, MoveIt/controller configuration, documentation) is original to this repository and licensed under Apache-2.0.

## Project Structure

```
farmbot_genesis_ros2/
├── .github/workflows/ci.yml                     # GitHub Actions CI
├── .gitignore
├── README.md
├── docs/
│   ├── ENGINEERING_REPORT.md                     # Deep technical reference
│   └── PROJECT_REPORT.md                          # Master's thesis report
└── src/
    ├── farmbot_description/                      # Main package
    │   ├── farmbot_genesis.urdf                  # Kinematic URDF (prismatic joints)
    │   ├── farmbot_genesis_detailed.urdf         # Full detail (backup, 1174 parts)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config.json                           # Onshape export config
    │   ├── find_mates.py                         # Onshape mate finder
    │   ├── config/
    │   │   ├── farmbot.rviz                      # RViz2 config
    │   │   └── ros2_control.yaml                 # Controller config
    │   ├── launch/
    │   │   ├── display.launch.py                 # RViz visualization (use_gui:=true/false)
    │   │   └── gazebo.launch.py                  # Gazebo simulation
    │   ├── src/
    │   │   ├── teleop_keyboard.py                # W/S X, A/D Y, Q/E Z control
    │   │   ├── joint_state_publisher_node.py     # Fallback joint states, live-driven by teleop
    │   │   └── motion_logger.py                  # Persistent /joint_states + /tf logger
    │   ├── scripts/
    │   │   ├── generate_urdf.py                  # Regenerate URDF from detailed
    │   │   └── simplify_collisions.py            # Reduce collision meshes to boxes per link
    │   ├── worlds/
    │   │   └── farmbot_empty.world               # Gazebo world
    │   └── assets/                               # 194 STL meshes + .part metadata
    └── farmbot_genesis_moveit_config/            # MoveIt2 config package
        ├── config/
        │   ├── farmbot_genesis.srdf              # 3-DOF gantry group
        │   ├── kinematics.yaml                   # KDL solver
        │   ├── joint_limits.yaml                 # Joint limits
        │   ├── controllers.yaml                  # FollowJointTrajectory
        │   ├── ompl_planning.yaml                # 23 planners, default RRTConnect
        │   ├── pilz_industrial_motion_planner.yaml
        │   └── initial_positions.yaml
        └── launch/
            ├── demo.launch.py
            └── moveit_planning_execution.launch.py
```

## Kinematic Chain

```
base_link
├── supporting_infrastructure (fixed)     ← stationary frame/electronics
└── x_axis_joint (prismatic, 0–2.7m)     ← gantry along tracks
    └── x_axis_link
        └── y_axis_joint (prismatic, 0–1.3m)  ← cross-slide along gantry
            └── y_axis_link
                └── z_axis_joint (prismatic, -0.4–0m)  ← tool vertical
                    └── z_axis_link                      ← tools/UTM/camera
                        └── joint_tool (fixed)
                            └── tool_link                ← end-effector frame
```

Joint limits: X=0–2.7m @ 0.1m/s, Y=0–1.3m @ 0.1m/s, Z=-0.4–0m @ 0.02m/s (Z travels downward from the fully-retracted `0` position).

## Quick Start

### Prerequisites

- ROS 2 Humble (desktop-full recommended)
- Gazebo (optional, for simulation)
- MoveIt2 (optional, for motion planning)

```bash
sudo apt install ros-humble-urdf ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-tf2-ros \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros ros-humble-gazebo-ros2-control \
  ros-humble-moveit ros-humble-ament-cmake ros-humble-ament-lint-common
```

### Build

```bash
cd farmbot_genesis_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

### Run Tests

```bash
colcon test
colcon test-result --verbose
```

## Usage

### 1. RViz Visualization

```bash
ros2 launch farmbot_description display.launch.py
```

Opens RViz with the FarmBot model, driven by the **Joint State Publisher GUI** sliders by default. Use `use_gui:=false` to drive it from keyboard teleop instead (see below).

### 2. Gazebo Simulation

```bash
ros2 launch farmbot_description gazebo.launch.py
```

Spawns the robot in Gazebo with physics enabled. The robot starts at home position (0,0,0).

### 3. Keyboard Teleop

```bash
ros2 launch farmbot_description display.launch.py use_gui:=false
ros2 run farmbot_description teleop_keyboard.py
```

`use_gui:=false` swaps the interactive slider GUI for `joint_state_publisher_node.py`, which subscribes to the same per-axis position topics `teleop_keyboard.py` publishes to — so pressing keys actually moves the model, with no `ros2_control` controller required.

| Key | Action |
|-----|--------|
| W/S | X axis forward/backward |
| A/D | Y axis left/right |
| Q/E | Z axis down/up |
| R   | Reset all to home (0,0,0) |
| ESC | Quit |

Default step size: 0.05m. Change with `--ros-args -p step_size:=0.1`.

### 4. MoveIt2 Motion Planning

```bash
ros2 launch farmbot_genesis_moveit_config demo.launch.py
```

Requires `ros-humble-moveit` to be installed. Launches move_group with both the OMPL and Pilz planning pipelines plus RViz. Add the **MotionPlanning** display in RViz to plan trajectories for the 3-DOF gantry (planning group: `gantry`); switch pipelines/planners under Context. Note: trajectory *execution* needs a `FollowJointTrajectory` action server (see `controllers.yaml`), which is not started by this demo — planning and visualization work without it.

### 5. Fallback Joint State Publisher

```bash
ros2 run farmbot_description joint_state_publisher_node.py
```

Publishes joint states when no `ros2_control` controller is running — home position by default, or live-updated from `teleop_keyboard.py`'s command topics if that node is also running. Useful for development, testing, and headless CI checks.

## Configuration

### ros2_control (`config/ros2_control.yaml`)

Prepared controller configuration for future hardware/simulation integration (not loaded by any current launch file — the URDF has no `<ros2_control>` hardware interface yet):

- Controller update rate: 100 Hz
- State publish rate: 50 Hz
- JointTrajectoryController for coordinated 3-axis motion
- Individual position/velocity controllers per joint

The keyboard teleop node (`teleop_keyboard.py`) currently talks to `joint_state_publisher_node.py` directly over plain `Float64` topics rather than through this controller layer — intentional, so teleop works without a running controller manager. `ros2_control.yaml` remains ready for a `gazebo_ros2_control` or hardware-interface plugin to be attached later for closed-loop control.

### MoveIt Planners

- **OMPL**: 23 planners, default RRTConnect, 5mm resolution
- **Pilz**: PTP (point-to-point), LIN (linear), CIRC (circular) planners

## Regenerating the URDF

When you export an updated model from Onshape, regenerate the kinematic URDF:

```bash
python3 src/farmbot_description/scripts/generate_urdf.py
```

This reads `farmbot_genesis_detailed.urdf` and produces `farmbot_genesis.urdf` with:
- Parts categorized by axis (X/Y/Z/Base/Common)
- Prismatic joints with proper limits
- Corrected mesh paths

> **Warning:** the shipped `farmbot_genesis.urdf` contains manual refinements on top of the generator output (corrected part-to-axis assignments for gantry bearings/cable-carrier hardware, and tuned material colors). Rerunning the script overwrites the file and loses those refinements — diff the result against git before keeping it.

Then rebuild collision geometry, since the regenerated visuals invalidate the old boxes:

```bash
pip install numpy   # not a runtime dependency of any ROS node — only this script
python3 src/farmbot_description/scripts/simplify_collisions.py
```

This replaces every link's per-part mesh collisions with a single axis-aligned box sized to that link's true visual extent — see [Highlights](#highlights) above for why.

### Exporting from Onshape

1. Set API keys: `export ONSHAPE_API=... ONSHAPE_ACCESS_KEY=... ONSHAPE_SECRET_KEY=...`
2. Find mate names: `onshape_env/bin/python src/farmbot_description/find_mates.py`
3. Edit `config.json` (set `noDynamics: false`, map `dof` names)
4. Export: `onshape_env/bin/onshape-to-robot src/farmbot_description/config.json`
5. Move output to `farmbot_genesis_detailed.urdf`
6. Run `generate_urdf.py` then `simplify_collisions.py` to rebuild the kinematic version

## CI Pipeline

GitHub Actions workflow (`.github/workflows/ci.yml`):
- Triggers on push/PR to main/master
- Ubuntu 22.04 + ROS 2 Humble
- Builds with colcon, runs all tests, checks URDF validity
- Runs flake8, pep257, lint_cmake, xmllint

## License

- **Code, launch files, configs, and docs** in this repository (everything except `assets/` and `farmbot_genesis_detailed.urdf`) are licensed under the [Apache License 2.0](LICENSE), as declared in `package.xml`.
- **CAD meshes** (`src/farmbot_description/assets/*.stl`, `*.part`) and the raw CAD export (`farmbot_genesis_detailed.urdf`) are the manufacturer's FarmBot Genesis design, released by FarmBot under [CC0 Public Domain](https://farm.bot/pages/open-source) — free to use, copy, modify, and redistribute without attribution. They are **not** original work of this repository; see the [Onshape Assembly](#onshape-assembly-cad-source) section above.

## Notes

- `base_link` is a massless dummy root (KDL does not support inertia on the root link)
- The detailed URDF (`farmbot_genesis_detailed.urdf`) has 1174 individually-named parts — the kinematic version consolidates these into 5 links for performance
- Each of the 4 moving/static links carries a single box collision primitive (see `simplify_collisions.py`); full mesh detail is retained for `<visual>`, only `<collision>` is simplified
- All mesh paths use `package://farmbot_description/assets/` — the generator script auto-fixes this
