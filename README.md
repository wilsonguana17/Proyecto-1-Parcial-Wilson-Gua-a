# 🏎️ F1TENTH Sepang | Reactive Follow the Gap — Parts 1 & 2

**English** · [Español](README_ES.md)  
**ROS 2 Humble · Ubuntu 22.04 · LiDAR · Ackermann steering**

This repository contains **Part 1 (single-car Follow the Gap)** and **Part 2 (five obstacles plus two-car dynamic overtaking)** on Sepang, without a predefined global racing line. The project includes a **10-lap timer**, Sepang map, and setup script. The controller uses current LiDAR measurements to steer and odometry to monitor vehicle motion.

**Contents:** [Part 1 — original controller](#-how-follow-the-gap-works) · [Part 2 — competition controller V3.9](#-part-2--five-static-obstacles-and-one-dynamic-opponent)

## 🎥 10-lap demonstration

[![Watch the 10-lap Sepang demonstration](https://img.youtube.com/vi/9m-QhL65yLw/hqdefault.jpg)](https://youtu.be/9m-QhL65yLw)

**[▶ Watch on YouTube](https://youtu.be/9m-QhL65yLw)** — Click the preview to open the video.

### 🎬 Original unedited 10-lap recording (Part 1)

[![Watch the original unedited 10-lap recording](https://img.youtube.com/vi/5wyF8lTBg24/hqdefault.jpg)](https://youtu.be/5wyF8lTBg24)

**[▶ Watch the original unedited recording on YouTube](https://youtu.be/5wyF8lTBg24)** — Click the thumbnail to open the complete original recording.

## 💡 How Follow the Gap works

1. **Read LiDAR:** `preprocess_lidar()` selects the frontal ±90° scan, handles invalid readings, caps distances at 4.5 m and smooths with a 5-sample window.
2. **Safety bubble:** `choose_gap()` masks a region around the closest obstacle, using a **0.60 m bubble radius**.
3. **Find free space:** It finds contiguous regions with processed distance greater than **1.20 m** and chooses the widest gap.
4. **Choose a target:** The target index blends **60% of the farthest point** in the selected gap and **40% of its center**.
5. **Steer and set speed:** `scan_callback()` limits and smooths steering, then limits speed for turning, lateral acceleration and frontal clearance.
6. **Straight-line speed adjustment:** When steering, yaw rate and recent odometry indicate stable movement, the controller checks a local LiDAR corridor and may increase the frontal speed limit by a bounded amount. When confidence or clearance is low, it keeps the conservative braking limit.
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

| Function or stage | Responsibility |
|---|---|
| `preprocess_lidar()` | Clean, crop and smooth LiDAR ranges |
| `choose_gap()` | Apply the safety bubble, find the widest gap and select a target |
| Frontal clearance estimation | Obtain a reference distance from the 20th percentile of valid LiDAR measurements within ±12° |
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

**Speed logic:** The controller compares the configured maximum with limits for steering, cornering and frontal clearance. The lowest sets the target speed. When enough space and straight-line confidence are available, corridor analysis can increase the frontal limit by up to `front_boost_max × confidence`, without overriding the other limits. Acceleration and braking rate limits are applied afterward.

### How does the controller calculate speed?

Inside `scan_callback()`, the controller computes **three independent limits**: `speed_original` reduces speed for larger steering angles, `speed_curve` accounts for the configured lateral acceleration limit, and a frontal braking limit depends on the available distance. Two of the calculations are shown below:

~~~python
speed_original = max(self.MIN_SPEED, self.max_speed * (
    1.0 - (abs(steering) / self.MAX_STEER_ANGLE) ** 1.8
))
speed_curve = math.sqrt(
    self.lateral_accel_max * self.WHEELBASE_M /
    max(abs(math.tan(steering)), 1e-5)
)
~~~

The **frontal braking limit** uses LiDAR clearance after subtracting a safety margin: `v_front = √(2 × braking_deceleration × max(0, frontal_distance − margin))`. Less distance to stop results in a lower permitted speed.

The controller also uses `corridor_clearance()` to sample a short local path, checking its center and lateral edges. If recent odometry, small steering/yaw rate and frontal clearance support stable movement, `straight_confidence` lets it **gradually increase** the frontal speed limit. When confidence drops, the boost deactivates and the conservative limit remains. Steering and cornering limits still apply.

~~~python
target = max(0.0, min(
    self.max_speed, speed_original, speed_curve, speed_front
))
drive.drive.steering_angle = float(steering)
drive.drive.speed = float(command)
self.drive_pub.publish(drive)
~~~

`target` is the desired speed, while `command` is the speed actually sent **after rate-limiting its changes** using `accel_limit` and `decel_limit`. This prevents sudden commanded speed jumps as LiDAR readings change.

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

### How does the lap timer work?

File: [`timing/lap_timer_sepang.py`](timing/lap_timer_sepang.py). This is a **separate ROS 2 node** that subscribes to `/ego_racecar/odom` but never publishes driving commands to `/drive`.

- **Starting reference:** `odom_callback()` stores the first `(x, y)` coordinates and initial heading; `yaw_from_quaternion()` extracts the yaw angle from ROS orientation.
- **Timer start:** it waits for the car to move **0.4 m** from the initial position, then sets `lap_start_time` with `time.monotonic()`.
- **Lap detection:** the car must travel at least **5 m** away to re-arm the finish area, then return within **2 m** of the start. It must also meet **20 s** minimum lap time, **80 m** estimated travel distance and a compatible heading.
- **Recording:** it increments `count`, updates lap/best times and writes a CSV row. Counting finishes after **10 laps**, but it **does not stop the driving controller**.

This code prevents false lap counts:

~~~python
if distance_to_start >= self.REARM_RADIUS_M:
    self.armed = True

if not self.armed or distance_to_start > self.START_RADIUS_M:
    return

elapsed = now - self.lap_start_time
heading_dot = math.cos(yaw - self.start_yaw)
if (
    elapsed < self.MIN_LAP_TIME_S
    or self.lap_distance < self.MIN_LAP_DISTANCE_M
    or heading_dot < self.MIN_HEADING_DOT
):
    return

self.count += 1
~~~

Distance is **estimated by summing position changes from odometry**, ignoring jumps larger than 20 m. Each run writes `sepang_lap_results_YYYYMMDD_HHMMSS.csv` to the working directory. Timing uses elapsed monotonic time (not necessarily simulation time), and **collisions are not detected**.

The controller logs English fields such as `ActualSpeed`, `CommandSpeed`, `TargetSpeed`, `CorridorDistance`, `Confidence`, `Mode` and `LimitedBy`. The `Mode` field indicates whether straight-line speed enhancement is disabled, partially active or fully enabled.

- **No vehicle motion:** Check topics with `ros2 topic list` and make sure only one controller publishes to `/drive`.
- **Wrong map:** Rerun `configure_sepang.py` for the correct workspace, then restart the simulator.
- **Missing ROS imports:** Source both ROS 2 and the built simulator workspace.
- **Incorrect lap count:** Start the timer before moving the car and avoid resetting/teleporting mid-run.

**Sources:** [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym) · [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros) · [ROS 2 Humble simulator setup](https://github.com/widegonz/F1Tenth-Repository). The included Sepang map comes from the map files supplied for this project; its redistribution terms should be checked. This controller is intended for **simulation**, not validated physical-vehicle operation.

---

# 🏁 Part 2 — Five static obstacles and one dynamic opponent

**Final competition controller: [Follow the Gap V3.9](controllers/gap_follow_sepang_v39.py).**
Both cars run **the same Python algorithm**; only ROS topic remappings and the maximum speed setting differ. This section documents the final controller, not the intermediate experiments.

## 🎥 Part 2 — Competition demonstration

[![Watch the Part 2 F1TENTH Sepang competition video](https://img.youtube.com/vi/_IGRx_2CJ-s/hqdefault.jpg)](https://youtu.be/_IGRx_2CJ-s)

**[▶ Watch the Part 2 video on YouTube](https://youtu.be/_IGRx_2CJ-s)** — Click the thumbnail to open the video.

### 🎬 Original unedited recording — Part 2 (Google Drive)

[![Watch the original unedited Part 2 recording on Google Drive](https://drive.google.com/thumbnail?id=1qDRfRShxvAR081CvCkMgt6b2JzihsOHV&sz=w640)](https://drive.google.com/file/d/1qDRfRShxvAR081CvCkMgt6b2JzihsOHV/view?usp=sharing)

**[▶ Watch the complete original, unedited Part 2 recording on Google Drive](https://drive.google.com/file/d/1qDRfRShxvAR081CvCkMgt6b2JzihsOHV/view?usp=sharing)** — Click the preview or link to open the recording.

*If you cannot open the video, the file owner needs to enable **Anyone with the link → Viewer** in Google Drive sharing settings.*

## 🎯 Competition tasks and observed outcome

| Test | Setup | Observed outcome |
|---|---|---|
| Static obstacle avoidance | Sepang with five manually added occupancy-map obstacles | Ten consecutive laps reported without collision |
| Dynamic opponent | Same five-obstacle map, two autonomous cars, opponent initially ~5 m ahead | Ten laps for each car and **one successful overtake** reported without collision |

## 📊 Part 2 — Final two-car race results (22 November 2026)

Both cars ran **Follow the Gap V3.9** on the same Sepang track with five static obstacles. The ego vehicle started about 5 m behind the opponent and performed one observed overtake during the race.

| Metric | Ego vehicle (main car) | Opponent vehicle |
|---|---:|---:|
| Reactive controller | Follow the Gap V3.9 | Follow the Gap V3.9 |
| Configured maximum speed | 12 m/s | 6 m/s |
| **Laps completed** | **10 / 10 ✅** | **10 / 10 ✅** |
| **Best lap time** | **81.24 s 🥇** | **93.43 s 🥈** |
| Fastest lap number | 6 | 6 |
| **Total time (10 laps)** | **847.96 s** | **944.78 s** |

The main car finished its ten laps **96.82 s faster** (difference between the two timer totals). The times above were transcribed from the provided screenshot of the two lap-counter terminals: `sepang_lap_results_ego_20261122_160532.csv` and `sepang_lap_results_opp_20261122_160552.csv` were the CSV filenames shown by those terminals, but the CSV files themselves have not been uploaded to this repository.

**Validation scope:** The two counters each reported `FINISH 10/10`. The author reported no collisions and one successful overtake. The lap timers measure laps and elapsed time but do **not** independently detect collisions or verify overtaking; the Part 2 video above supplies visual context. The assignment requires overtaking the additional robot during the race, not multiple overtakes.

## 🧠 Reactive approach: Follow the Gap V3.9

The controller uses a local reactive policy rather than a predefined global racing line:

1. **Scan preprocessing:** `preprocess_lidar()` takes the front 180° of `/scan`, handles missing returns conservatively, caps range, and applies a 3-ray local minimum to preserve obstacle boundaries.
2. **Safe-gap selection:** `choose_gap()` searches contiguous free regions within ±65°, using a speed-dependent lookahead (`lookahead_base + lookahead_gain × actual_speed`, capped at 12 m). A local bubble discourages steering toward protruding frontal obstacles.
3. **Steering and corridor checks:** `refine_steering()` examines alternate steering angles when the nominal path is too short. `path_clearance()` estimates reachable free distance along an Ackermann arc, checking the path center and both lateral edges.
4. **Adaptive speed:** the chosen speed is the minimum of maximum-speed, steering, lateral-acceleration and path-clearance limits. Acceleration and deceleration commands are rate-limited. **A configured maximum of 12 m/s does not imply that the car always reaches 12 m/s.**
5. **Opportunistic overtaking:** `opp_callback()` receives the moving opponent's odometry. `_relative()` measures relative pose; `_overtake()` evaluates candidate corridors to the left and right and enters `OVERTAKE` only when a feasible corridor is identified. `_pass_path()` checks candidate lateral paths and models the opponent's future occupancy; `_filtered()` separates the opponent's approximate LiDAR silhouette from static returns while retaining a separate dynamic collision check. Once the opponent is behind, the controller resumes `NORMAL`.
6. **ROS output:** `scan_callback()` publishes an `AckermannDriveStamped` command through the car's mapped drive topic.

**Sensor scope:** normal wall/obstacle avoidance uses LiDAR and own odometry. Dynamic overtaking **also uses the opponent's odometry** published by this simulator; it is not a LiDAR-only opponent tracker. The local path checks are estimates, not formal collision guarantees.

### Code organization and ROS interfaces

| Path | Purpose |
|---|---|
| [`controllers/gap_follow_sepang_v39.py`](controllers/gap_follow_sepang_v39.py) | Part 2 final reactive controller, adaptive speed, static obstacle avoidance and overtaking |
| [`timing/lap_timer_sepang_dual.py`](timing/lap_timer_sepang_dual.py) | Two independent 10-lap timers: `vehicle:=ego` and `vehicle:=opp` |
| [`scripts/configure_sepang_part2.py`](scripts/configure_sepang_part2.py) | Copies Part 2 map and sets `num_agent: 2`, initial poses and map location |
| [`maps/Sepang_5obs_map.yaml`](maps/Sepang_5obs_map.yaml) | Occupancy-map metadata; references `Sepang_5obs_map.png` |
| `maps/Sepang_5obs_map.png` | **Required:** definitive five-obstacle PNG created in GIMP; add the exact image from the local simulation workspace |
| `results/` | Optional race evidence (CSV logs/screenshots/video links) |
| [`controllers/gap_follow_sepang_v31_en.py`](controllers/gap_follow_sepang_v31_en.py) | Original Part 1 controller, preserved unchanged |
| [`timing/lap_timer_sepang.py`](timing/lap_timer_sepang.py) | Original Part 1 lap timer, preserved unchanged |

ROS topics for the two cars:

| Signal | Ego / main car | Opponent |
|---|---|---|
| LaserScan | `/scan` | `/opp_scan` |
| Own odometry | `/ego_racecar/odom` | `/opp_racecar/odom` |
| Other car's odometry | `/ego_racecar/opp_odom` | `/opp_racecar/opp_odom` |
| Drive command | `/drive` | `/opp_drive` |

## ⚙️ Part 2 setup (same repository and simulator as Part 1)

Follow the Part 1 dependency instructions above first. The ROS simulator remains an **external** workspace, here assumed to be `~/F1Tenth-Repository`.

**Required map asset:** the edited PNG is unique to the five obstacles placed for this project. Copy the definitive image from your Ubuntu workspace into `maps/Sepang_5obs_map.png` in this repository, and commit that PNG together with this code. A screenshot of RViz is **not** a substitute for the original occupancy map. The existing Part 2 YAML retains the original map resolution (0.05944 m/pixel) and origin.

For an existing local installation that already has the final map inside the simulator:

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
cp ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_5obs_map.png maps/
~~~

Configure a fresh Part 2 run **after the PNG is present**:

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 scripts/configure_sepang_part2.py --workspace ~/F1Tenth-Repository

cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

The setup script backs up `sim.yaml` and configures:

~~~yaml
map_path: '/home/wilson/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_5obs_map'
map_img_ext: '.png'
num_agent: 2
sx: -14.14
sy: -1.14
stheta: 0.08
sx1: -9.16
sy1: -0.74
stheta1: 0.08
~~~

The `map_path` shown above is an **example** for the author's Ubuntu account: the script automatically uses the supplied workspace path on other machines. The two spawn centers are about 5 m apart along the initial heading; **verify both poses are on the road, free from obstacles, in RViz**. Do **not** rerun the Part 1 setup script for this test: it resets `num_agent` to 1.

## 🚀 Run two cars and time ten laps (five terminals)

**Terminal 1 — ROS simulator:**

~~~bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
~~~

Start **both timers while the vehicles are stationary**, before either driving controller.

**Terminal 2 — ego timer:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 timing/lap_timer_sepang_dual.py --ros-args -r __node:=lap_timer_ego -p vehicle:=ego
~~~

**Terminal 3 — opponent timer:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 timing/lap_timer_sepang_dual.py --ros-args -r __node:=lap_timer_opponent -p vehicle:=opp
~~~

**Terminal 4 — opponent, V3.9 at a maximum of 6 m/s:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 controllers/gap_follow_sepang_v39.py --ros-args \
  -r __node:=controller_opponent \
  -r /scan:=/opp_scan \
  -r /ego_racecar/odom:=/opp_racecar/odom \
  -r /ego_racecar/opp_odom:=/opp_racecar/opp_odom \
  -r /drive:=/opp_drive \
  -p max_speed:=6.0 -p accel_limit:=5.0
~~~

**Terminal 5 — ego, the same V3.9 at a maximum of 12 m/s:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 controllers/gap_follow_sepang_v39.py --ros-args \
  -r __node:=controller_ego \
  -p max_speed:=12.0 -p accel_limit:=5.0
~~~

In two-agent mode this version of the gym bridge advances the simulation **only after both drive topics have received commands**. The opponent may stay still until the ego controller starts. Do **not** publish two controllers to the same drive topic.

### Optional: static-obstacles-only run

To test Part 2 Task 1 independently, keep `Sepang_5obs_map` but set `num_agent: 1` in the simulator's `sim.yaml`, rebuild if required and restart. Run **one** V3.9 controller on the ego topics and the original or dual ego lap timer. Restore `num_agent: 2` before the overtaking test.

## ⏱️ Evidence, logs and troubleshooting


### How `lap_timer_sepang_dual.py` is structured

The [dual lap timer](timing/lap_timer_sepang_dual.py) is an **independent ROS 2 node**. The same file runs twice: once with `vehicle:=ego` (subscribing to `/ego_racecar/odom`) and once with `vehicle:=opp` (subscribing to `/opp_racecar/odom`). It never sends steering or speed commands.

| Main component | Role |
|---|---|
| `__init__()` | Selects the vehicle and odometry topic, initializes lap counters, and creates a separate CSV file. |
| `yaw_from_quaternion()` | Converts the orientation quaternion to heading (yaw). |
| `odom_callback()` | Reads positions, starts timing after initial movement, checks each return to the start, and records lap/best/total times until 10 laps. |
| `destroy_node()` | Closes the CSV file cleanly. |
| `main()` | Starts the ROS 2 node and keeps it listening for odometry updates. |

To avoid false counts, a completed lap must satisfy the start-area rearming distance, minimum travel distance and time, and compatible heading. **Collision detection is not part of this timer.**


- `lap_timer_sepang_dual.py` uses each car's own odometry and the **same 10-lap counting criteria** as the original timer: leave the start by 5 m, return within 2 m, satisfy a minimum of 20 s / 80 m, and match the starting heading. Timing begins after 0.4 m initial movement.
- Separate CSV files are produced in the current working directory: `sepang_lap_results_ego_YYYYMMDD_HHMMSS.csv` and `sepang_lap_results_opp_YYYYMMDD_HHMMSS.csv`. They report lap, lap time, best lap, elapsed time and estimated distance.
- The controller prints measured/commanded/target speed, steering, local clearance, current coordinates and `PASS START`, `PASS WAIT`, `PASS ABORT`, `PASS COMPLETE` during overtaking attempts.
- **A `PASS COMPLETE` message is not independent proof of a safe pass.** Use an RViz screen recording to confirm the physical order of the cars changes without contact. The counters do not detect crashes, stop cars or verify overtaking.
- **If the cars do not move**, check `num_agent: 2`, both `/drive` and `/opp_drive`, and both running controllers. Use `ros2 topic list`.
- **If the wrong map appears**, check `map_path`, matching PNG/YAML names and whether the map was copied and the simulator restarted.
- **If counting is incorrect**, restart the simulator and both lap timers from stationary starting poses; do not teleport during a timed run.

**Scope:** F1Tenth simulation only. The original Part 1 demonstration remains available above.
