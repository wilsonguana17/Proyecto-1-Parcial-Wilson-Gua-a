# F1TENTH Sepang — Reactive Follow the Gap (V3.1)

**English** | [Español](README_ES.md)

> ROS 2 Humble · Ubuntu 22.04 · LiDAR reactive navigation · Ackermann steering

This repository contains a reactive **Follow the Gap** controller for the **Sepang** track in the F1TENTH simulator, a standalone **10-lap timer**, the Sepang occupancy-grid map, and a setup helper. It is written as a **beginner-friendly tutorial** so another student can install the dependencies, configure the map, and reproduce a simulation run on Ubuntu.

**What was observed in the author's simulation:** 10 laps completed, **777.40 s total**, **77.47 s best lap**, with no collisions **observed by the operator**. This is an experimental run, not a guarantee for every machine or run. The lap timer does not detect impacts.

## 🎬 Race demonstration — Sepang (10 laps)

[![▶ Watch the 10-lap Sepang run on YouTube](https://img.youtube.com/vi/9m-QhL65yLw/hqdefault.jpg)](https://youtu.be/9m-QhL65yLw)

**▶ Click the video preview to watch the full 10-lap run on YouTube.** GitHub READMEs display the thumbnail, but do not play YouTube videos directly in the page.

## 1. Project idea: reactive navigation

Unlike a global path planner, the controller **does not read a map or use a prerecorded racing line**. Its control loop processes LiDAR data at runtime:

1. Read the forward LiDAR field (approximately ±90°) and smooth the distance samples.
2. Mask a **safety bubble** around the closest selected obstacle.
3. Detect contiguous free-space regions (**gaps**) and choose the widest one.
4. Choose a target within that gap using a weighted combination of the farthest sample and the gap center.
5. Smooth the steering angle and calculate speed limits from steering, lateral acceleration, and forward clearance.
6. Optionally permit a **bounded frontal speed increase** when a finite-width LiDAR corridor and recent odometry indicate stable straight motion.
7. Publish speed and steering on the ROS 2 `/drive` topic.

**Data flow:** `/scan` (`sensor_msgs/LaserScan`) + `/ego_racecar/odom` (`nav_msgs/Odometry`) → controller → `/drive` (`ackermann_msgs/AckermannDriveStamped`). The timer only subscribes to odometry and does not command the vehicle.

## 2. Repository structure

```text
Proyecto-1-Parcial-Wilson-Gua-a/
├── README.md                           # Full English tutorial (this file)
├── README_ES.md                        # Full Spanish tutorial
├── controllers/
│   └── gap_follow_sepang_v31_en.py     # Reactive controller + English telemetry
├── timing/
│   └── lap_timer_sepang.py             # English 10-lap timer + CSV output
├── maps/
│   ├── Sepang_map.png                  # Occupancy-grid image
│   └── Sepang_map.yaml                 # Map metadata
├── scripts/
│   └── configure_sepang.py             # Copies map; updates simulator sim.yaml
├── results/
│   └── summary.csv                     # Summary of observed runs
└── .gitignore
```

The **simulator is an external dependency** (not copied into this repository). See the upstream projects listed at the end.

## 3. Requirements

- **Ubuntu 22.04 LTS** with desktop GUI/RViz access.
- **ROS 2 Humble Desktop**, installed following the [official ROS guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- Python 3, `git`, `pip`, `numpy`, `colcon`, `rosdep`, and F1TENTH Gym.
- ROS 2 runtime packages for `ackermann_msgs`, Nav2 map server/lifecycle manager, Xacro, and robot-state publisher.

> **Compatibility note:** The controller/timer were run in an existing Ubuntu + ROS Humble workspace. The fresh-PC workflow below follows published simulator instructions but **has not yet been independently verified on a clean computer**; dependency versions may require adjustment. The controller itself uses only local LiDAR and odometry topics, not external cloud services.

> **Repository access:** while **private**, only the owner and invited collaborators can clone the repository. Before sharing it with the instructor, make it **public** or grant access.

## 4. Fresh Ubuntu installation (step by step)

### 4.1 Install ROS 2 Humble

Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), install the **desktop** variant, and verify:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

### 4.2 Install build and ROS dependencies

```bash
sudo apt update
sudo apt install -y git unzip python3-pip python3-numpy python3-colcon-common-extensions python3-rosdep2
sudo apt install -y ros-humble-ackermann-msgs ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager ros-humble-xacro ros-humble-robot-state-publisher
```

If `rosdep` has never been initialized on this system, run `sudo rosdep init` **once**. Then:

```bash
rosdep update
```

### 4.3 Install F1TENTH Gym (Python simulation engine)

```bash
cd ~
git clone https://github.com/f1tenth/f1tenth_gym.git
cd ~/f1tenth_gym
python3 -m pip install -e .
```

This matches the setup approach described by the [F1TENTH Gym project](https://github.com/f1tenth/f1tenth_gym). If Python dependency conflicts occur, reconcile Gym and ROS 2 Python environments before continuing; do not blindly remove ROS packages.

### 4.4 Download the ROS simulator and this project

The [F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository) documents a ROS 2 Humble native workflow and provides `src/f1tenth_gym_ros`. Download it into one workspace:

```bash
cd ~
git clone https://github.com/widegonz/F1Tenth-Repository.git
```

Clone **this** repository separately. Replace `wilsonguana17` with the real owner after the project has been published:

```bash
cd ~
git clone https://github.com/wilsonguana17/Proyecto-1-Parcial-Wilson-Gua-a.git
```

> Do not type the placeholder `wilsonguana17` literally.

### 4.5 Configure Sepang without hard-coded usernames

```bash
python3 ~/Proyecto-1-Parcial-Wilson-Gua-a/scripts/configure_sepang.py \
  --workspace ~/F1Tenth-Repository
```

The script checks that the simulator is present, copies `Sepang_map.png` and `Sepang_map.yaml` into the simulator's `maps/` folder, **backs up** `src/f1tenth_gym_ros/config/sim.yaml`, and updates these parameters:

```yaml
map_path: '<HOME>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_map'
map_img_ext: '.png'
num_agent: 1
sx: -14.14
sy: -1.14
stheta: 0.08
```

`<HOME>` stands for the absolute user-specific home path; the script fills it in automatically. This YAML excerpt shows **only edited keys**, not the entire `sim.yaml`.

### 4.6 Install ROS package dependencies and build

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
rosdep install -i --from-paths src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

If `colcon` fails, read the actual error first. Some older F1TENTH Gym revisions may require compatible Python package versions.

## 5. Run the Sepang race (three terminals)

**Terminal 1 — launch the simulator (RViz):**

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Check that **Sepang** appears in RViz, with a stationary vehicle inside the track.

**Terminal 2 — start the lap timer BEFORE the vehicle moves:**

```bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 timing/lap_timer_sepang.py
```

**Terminal 3 — run the successful V3.1 configuration:**

```bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 controllers/gap_follow_sepang_v31_en.py --ros-args \
  -p max_speed:=12.0 -p front_boost_max:=0.8
```

You can stop the controller with **Ctrl+C**. The lap timer reports up to 10 laps and writes `sepang_lap_results_YYYYMMDD_HHMMSS.csv` in the **current working directory**. It does **not** stop the car automatically. Run only **one controller at a time**.

**If you already have the tested workspace** at `~/FollowTheGap-F1Tenth`, you may instead configure it with `--workspace ~/FollowTheGap-F1Tenth`, build/source that workspace, and run the same two Python nodes from this repository. No need to clone the simulator twice.

## 6. Essential code explained

The core controller is `controllers/gap_follow_sepang_v31_en.py`:

| Function | Responsibility |
|---|---|
| `__init__` | Creates ROS 2 subscribers and publisher, declares parameters and initializes controller state. |
| `odom_callback` | Reads actual planar speed, yaw rate, and X/Y coordinates; tracks freshness of odometry. |
| `preprocess_lidar` | Selects forward LiDAR angles, handles invalid measurements, clips and smooths values. |
| `choose_gap` | Creates a safety bubble, finds the widest free-space gap, and selects a steering target. |
| `front_distance_v2` | Computes the 20th percentile of valid ranges in the ±12° forward sector. |
| `corridor_clearance` | Estimates clearance along a locally curved, finite-width corridor. |
| `confidence_component` | Converts small steering/yaw rates into confidence for a straight-line speed boost. |
| `scan_callback` | Computes steering and speed limits, applies acceleration slew limits, publishes `/drive`, and prints English telemetry. |

**Steering:** For a selected LiDAR ray index `i`, `δ_proposed = angle_min + i × angle_increment`. Steering is clipped to ±0.4189 rad and filtered: `δ = 0.65 δ_proposed + 0.35 δ_previous`.

**Speed constraints** (all in m/s):

```text
v_FTG      = max(1, vmax × (1 - (abs(steer) / steer_max)^1.8))
v_corner   = sqrt(a_lat_max × wheelbase / max(abs(tan(steer)), 1e-5))
v_front_v2 = sqrt(2 × brake_accel × max(0, front_distance - front_margin))
v_target   = min(vmax, v_FTG, v_corner, v_front_v3_1)
```

Defaults include `wheelbase=0.33 m`, `a_lat_max=8 m/s²`, `brake_accel=5 m/s²`, `front_margin=1 m`, `accel_limit=3 m/s²`, and `decel_limit=12 m/s²`. These are **controller settings**, not measured physical guarantees.

**V3.1 improvement:** If steering and yaw rate suggest stable straight travel and odometry is recent, evaluate a finite-width corridor. A nonnegative speed boost of at most `front_boost_max × confidence` can increase the V2 frontal limit. This boost cannot by itself reduce the V2 frontal limit. Final speed still obeys the other constraints.

The lap timer (`timing/lap_timer_sepang.py`) stores the spawn position when it first receives odometry, starts timing when movement exceeds 0.4 m, and counts a lap when the car returns near the start **after** leaving the rearm radius, satisfying minimum distance/time and forward-heading checks. It estimates travel distance from incremental X/Y odometry. It is a proximity-based counter, not a precise geometric start/finish line detector.

## 7. Console telemetry and CSV

| English field | Meaning |
|---|---|
| `ActualSpeed` | Speed magnitude computed from odometry (m/s). |
| `CommandSpeed` | Speed requested via `/drive` (m/s). |
| `TargetSpeed` | Desired speed after constraints (m/s). |
| `CornerLimit` | Curve-based speed limit (m/s). |
| `FrontDistanceV2` | V2 forward LiDAR clearance estimate (m). |
| `CorridorDistance` | Estimated local corridor clearance (m), or `--` if unavailable. |
| `FrontSpeedLimit` | V3.1 forward speed limit (m/s). |
| `V2Target` | Target speed that V2 limits would select (m/s). |
| `SpeedGain` | Target-speed increase versus V2 (m/s). |
| `Confidence` | Straight-driving confidence between 0 and 1. |
| `Mode` | `STRAIGHT`, `BLENDED`, or `V2` mode of the boost. |
| `LimitedBy` | Main active speed constraint (`MAX`, `FTG`, `CORNER`, `FRONT`). |
| `SteeringAngle` | Steering command (degrees). |
| `YawRate` | Measured yaw rate (degrees/s). |
| `X`, `Y` | Map-frame odometry position (m). |

ROS 2 log timestamps/names are generated by ROS. The CSV columns are `lap`, `lap_time_s`, `best_lap_s`, `elapsed_time_s`, `lap_distance_m`.

## 8. Troubleshooting

- **`python3: can't open file`** — run from this repository root, or use the full path to `controllers/` or `timing/`.
- **Old track appears** — stop the simulator, rerun the setup script, rebuild/source the workspace, relaunch; check `ros2 param get /bridge map_path`.
- **Vehicle does not move** — check that `/scan`, `/ego_racecar/odom`, and `/drive` exist (`ros2 topic list`) and that no second controller is publishing.
- **`No module named rclpy/ackermann_msgs`** — source ROS 2, install the ROS packages, then source the built workspace.
- **`No module named f110_gym` or `f110-v0` error** — confirm that the F1TENTH Gym Python package is installed into the Python environment used by ROS.
- **Lap count wrong** — start the timer with the vehicle stationary at the expected spawn, do not reset/teleport mid-run, and inspect timer thresholds.

## 9. Credits and scope

- Simulation backend and ROS bridge: [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym), [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros), and the [ROS 2 Humble simulator workspace guide](https://github.com/widegonz/F1Tenth-Repository). The simulator code is **not redistributed** here.
- The `Sepang_map` files came from the author's supplied `Mapas-F1Tenth-v2.zip`; **verify the map's redistribution/license terms before making the repository public**.
- The controller is intended for **simulation**, not an independently validated physical vehicle safety system.
- The project owner should choose a license for original source files before publishing. Do not add a license to third-party materials without checking their terms.
