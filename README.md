# 🏎️ F1TENTH Sepang | Reactive Follow the Gap V3.1

**English** · [Español](README_ES.md)  
**ROS 2 Humble · Ubuntu 22.04 · LiDAR · Ackermann steering**

A reactive controller that drives the **Sepang** track in the F1TENTH simulator without a predefined racing line. The project includes a **10-lap timer**, Sepang map, and setup script. The controller uses current LiDAR measurements to steer and odometry to monitor vehicle motion.

## 🎥 10-lap demonstration

[![Watch the 10-lap Sepang demonstration](https://img.youtube.com/vi/9m-QhL65yLw/hqdefault.jpg)](https://youtu.be/9m-QhL65yLw)

**[▶ Watch on YouTube](https://youtu.be/9m-QhL65yLw)** — Click the preview to open the video.

## 💡 How Follow the Gap works

1. **Read LiDAR:** `preprocess_lidar()` selects the frontal ±90° scan, handles invalid readings, caps distances at 4.5 m and smooths with a 5-sample window.
2. **Safety bubble:** `choose_gap()` masks a region around the closest obstacle, using a **0.60 m bubble radius**.
3. **Find free space:** It finds contiguous regions with processed distance greater than **1.20 m** and chooses the widest gap.
4. **Choose a target:** The target index blends **60% of the farthest point** in the selected gap and **40% of its center**.
5. **Steer and set speed:** `scan_callback()` limits and smooths steering, then limits speed for turning, lateral acceleration and frontal clearance.
6. **V3.1 straight-line boost:** When steering, yaw rate and recent odometry support a stable straight segment, a LiDAR corridor may raise the **frontal** speed limit by a bounded amount. Otherwise, the controller retains the conservative V2 frontal limit.
7. **Drive:** Publish an `AckermannDriveStamped` command to `/drive`.

**ROS 2 data flow:** `/scan` + `/ego_racecar/odom` → **Follow the Gap V3.1** → `/drive`. The timer reads odometry independently; it never commands the vehicle.

## 🧩 Code explained

Main file: [`controllers/gap_follow_sepang_v31_en.py`](controllers/gap_follow_sepang_v31_en.py)

### ROS interfaces

The node subscribes to LiDAR and odometry and publishes Ackermann commands:

~~~python
self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
~~~

### Key functions

| Function | Responsibility |
|---|---|
| `preprocess_lidar()` | Clean, crop and smooth LiDAR ranges |
| `choose_gap()` | Apply the safety bubble, find the widest gap and select a target |
| `front_distance_v2()` | Estimate frontal clearance from the 20th percentile of valid measurements within ±12° |
| `corridor_clearance()` | Estimate free distance along a finite-width local Ackermann corridor |
| `confidence_component()` | Turn steering/yaw conditions into a confidence value for the straight-line boost |
| `odom_callback()` | Update measured speed, yaw rate and position |
| `scan_callback()` | Compute steering, speed constraints, boost, acceleration limits and the `/drive` message |

The target selection is **not** simply the center of the gap. The relevant code is:

~~~python
best_rel = int(np.argmax(processed[gap_start:gap_end + 1]))
center = (gap_start + gap_end) // 2
return int(0.6 * (gap_start + best_rel) + 0.4 * center)
~~~

The chosen LiDAR angle is clipped to **±0.4189 rad** (approximately 24°). Subsequent steering commands are smoothed using `steer_alpha = 0.65`.

**Speed logic:** The controller compares the configured maximum, steering-dependent limit, curve limit and frontal braking limit; the lowest determines its target. It then applies acceleration/deceleration rate limits. The V3.1 corridor boost can only *increase* the V2 frontal limit, by at most `front_boost_max × confidence`, and does not override the other speed limits.

**Run configuration used for this project:**

~~~bash
python3 controllers/gap_follow_sepang_v31_en.py --ros-args -p max_speed:=12.0 -p front_boost_max:=0.8
~~~

These values are controller settings, not a guarantee of attained vehicle speed.

## 📂 Repository structure

~~~text
Proyecto-1-Parcial-Wilson-Gua-a/
├── README.md                          # English guide
├── README_ES.md                       # Spanish guide
├── controllers/
│   └── gap_follow_sepang_v31_en.py    # Reactive control + English telemetry
├── timing/
│   └── lap_timer_sepang.py            # Independent 10-lap timer
├── maps/
│   ├── Sepang_map.png
│   └── Sepang_map.yaml
├── scripts/
│   └── configure_sepang.py            # Map/spawn configuration helper
├── results/
│   └── summary.csv                    # Previous experimental log
└── .gitignore
~~~

The simulator is an **external dependency** and is not bundled in this repository.

## ⚙️ Install on Ubuntu 22.04

**1. Install ROS 2 Humble Desktop** using the [official Ubuntu instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Check the installation:

~~~bash
source /opt/ros/humble/setup.bash
ros2 --help
~~~

**2. Install system and ROS dependencies:**

~~~bash
sudo apt update
sudo apt install -y git python3-pip python3-numpy python3-colcon-common-extensions python3-rosdep2
sudo apt install -y ros-humble-ackermann-msgs ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager ros-humble-xacro ros-humble-robot-state-publisher
~~~

Initialize `rosdep` once if required (`sudo rosdep init`), then run `rosdep update`.

**3. Install F1TENTH Gym and download the ROS simulator and this project:**

~~~bash
cd ~
git clone https://github.com/f1tenth/f1tenth_gym.git
cd ~/f1tenth_gym
python3 -m pip install -e .

cd ~
git clone https://github.com/widegonz/F1Tenth-Repository.git
git clone https://github.com/wilsonguana17/Proyecto-1-Parcial-Wilson-Gua-a.git
~~~

See the [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym) and [ROS simulator](https://github.com/widegonz/F1Tenth-Repository) repositories for their environment requirements.

**4. Configure the Sepang map and build the simulator:**

~~~bash
python3 ~/Proyecto-1-Parcial-Wilson-Gua-a/scripts/configure_sepang.py \
  --workspace ~/F1Tenth-Repository

cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
rosdep install -i --from-paths src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
~~~

The helper copies both map files and backs up `sim.yaml` before setting `map_path`, `map_img_ext`, `num_agent` and the Sepang start pose (`sx=-14.14`, `sy=-1.14`, `stheta=0.08`).

If you already have a working simulator in `~/FollowTheGap-F1Tenth`, pass that path as `--workspace` instead; use the same workspace when sourcing its `install/setup.bash`.

> **Compatibility note:** The controller was tested in an existing Ubuntu/ROS 2 Humble environment; fresh-machine installation may require adjusting external simulator/Python dependencies.

## 🚀 Run Sepang (3 terminals)

**Terminal 1 — simulator:**

~~~bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
~~~

**Terminal 2 — start the timer while the car is stationary:**

~~~bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 timing/lap_timer_sepang.py
~~~

**Terminal 3 — start the controller last:**

~~~bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 controllers/gap_follow_sepang_v31_en.py --ros-args \
  -p max_speed:=12.0 -p front_boost_max:=0.8
~~~

Use `Ctrl+C` to stop the controller. **Do not run multiple driving controllers at once.** The timer does **not** stop the car automatically.

## ⏱️ Timer, telemetry and troubleshooting

[`timing/lap_timer_sepang.py`](timing/lap_timer_sepang.py) starts timing after the vehicle moves 0.4 m, then counts a lap when it returns near the start after meeting minimum distance/time, re-arming and heading checks. It writes `sepang_lap_results_YYYYMMDD_HHMMSS.csv` to the directory where it is launched. It uses elapsed monotonic time and does not detect collisions.

The controller logs English fields such as `ActualSpeed`, `CommandSpeed`, `TargetSpeed`, `FrontDistanceV2`, `CorridorDistance`, `Confidence`, `Mode` and `LimitedBy`. The `STRAIGHT`, `BLENDED` and `V2` modes describe whether the corridor-based boost is active.

- **No vehicle motion:** Check topics with `ros2 topic list` and make sure only one controller publishes to `/drive`.
- **Wrong map:** Rerun `configure_sepang.py` for the correct workspace, then restart the simulator.
- **Missing ROS imports:** Source both ROS 2 and the built simulator workspace.
- **Incorrect lap count:** Start the timer before moving the car and avoid resetting/teleporting mid-run.

**Sources:** [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym) · [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros) · [ROS 2 Humble simulator setup](https://github.com/widegonz/F1Tenth-Repository). The included Sepang map comes from `Mapas-F1Tenth-v2.zip`; its redistribution terms should be checked. This controller is intended for **simulation**, not validated physical-vehicle operation.
