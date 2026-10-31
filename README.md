# Scout — ROS 2 Jazzy differential-drive mobile robot

A complete, runnable simulation stack for **Scout**, a small indoor
differential-drive robot: URDF/xacro description, a Gazebo Harmonic arena,
`ros_gz` bridging, SLAM, and Nav2 autonomous navigation.

```
map ──▶ odom ──▶ base_footprint ──▶ base_link ──┬──▶ left_wheel_link
                                                ├──▶ right_wheel_link
                                                ├──▶ front/rear caster
                                                ├──▶ lidar_mast_link ──▶ lidar_link
                                                └──▶ camera_link
```

## Hardware in simulation

| Item | Value |
|---|---|
| Chassis | 0.44 × 0.36 × 0.14 m, 3.0 kg |
| Drive wheels | r = 0.085 m, separation 0.38 m, `mu1` 1.2 |
| Casters | 2 fixed spheres r = 0.03 m, `mu` 0.001 (slide, not drag) |
| Max speed | 0.6 m/s linear, 1.6 rad/s angular |
| Lidar | 720 samples, 360°, 0.12–12.0 m, 10 Hz, Gaussian noise σ = 0.01 |
| Camera | RGB-D, 640×480, 1.204 rad HFOV, 15 Hz |
| World | 12 × 9 m walled arena with partitions, crates, pillars |

All three ground contacts (two wheels, two caster balls) meet `z = 0` when
`base_footprint` is on the ground, so the robot spawns level and does not pitch.

## Build

```bash
cd ~/projects/scout-nav2-gazebo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# 1. Just look at the model (RViz, no physics)
ros2 launch scout_description display.launch.py

# 2. Gazebo world + robot (headless: add gui:=false)
ros2 launch scout_description gazebo.launch.py

# 3. Drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. Map the arena — saves maps/scout_map.pgm on Ctrl-C
scripts/map.sh

# 5. Navigate — localizes on the saved map
scripts/nav.sh
```

See [docs/RUNBOOK.md](docs/RUNBOOK.md) for the full operating procedure and
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for how the pieces fit together.

## Layout

```
src/scout_description/
├── urdf/scout.urdf.xacro      # robot model + gz plugins
├── worlds/scout_world.sdf     # 12x9 m arena
├── config/
│   ├── gz_bridge.yaml         # gz <-> ros topic map
│   ├── slam_toolbox.yaml      # async online mapping
│   └── nav2_params.yaml       # Nav2 + AMCL
├── launch/
│   ├── display.launch.py
│   ├── gazebo.launch.py
│   ├── slam.launch.py
│   └── navigation.launch.py
├── rviz/                      # scout.rviz, nav.rviz
└── maps/                      # scout_map.yaml (+ .pgm after mapping)
scripts/                       # map.sh, nav.sh
```