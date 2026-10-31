# Scout — Architecture

## Node and topic graph

```
                     ┌──────────────────┐
  /cmd_vel  ────────▶│  Gazebo Harmonic │
  (Twist)            │  DiffDrive system│
                     └────────┬─────────┘
                              │ /odom, /tf, /joint_states, /scan, /camera/*
                     ┌────────▼─────────┐
                     │  ros_gz_bridge   │  config/gz_bridge.yaml
                     └────────┬─────────┘
                              │ ROS 2 topics
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
┌───────▼────────┐   ┌────────▼────────┐   ┌────────▼────────┐
│ robot_state_   │   │ slam_toolbox    │   │ Nav2            │
│ publisher      │   │  OR             │   │ controller /    │
│ (URDF → TF)    │   │ map_server+AMCL │   │ planner / BT    │
└────────────────┘   └────────┬────────┘   └────────┬────────┘
                              │ map→odom            │ cmd_vel
                              └─────────────────────┘
```

## Who owns which transform

| Transform | Publisher |
|---|---|
| `base_footprint → base_link → wheels/sensors` | `robot_state_publisher` (from URDF + `/joint_states`) |
| `odom → base_footprint` | gz `DiffDrive` plugin, bridged on `/tf` |
| `map → odom` | **exactly one of** slam_toolbox (mapping) or AMCL (localization) |

The last row is the invariant the launch files protect. Two publishers of
`map → odom` produce a TF tree that flips between two answers and every
consumer downstream fails intermittently — the classic "Nav2 works for ten
seconds then dies" symptom.

## Why the Gazebo `DiffDrive` plugin, not `ros2_control`

`gz_ros2_control` + `diff_drive_controller` is the "proper" ROS 2 answer and it
was the first thing built here. It was dropped because it adds a controller
manager, a spawner lifecycle, and a `ros2_control` YAML block for zero
functional gain in this sim: the `gz-sim-diff-drive-system` plugin already
publishes odometry, TF and joint states, and accepts `/cmd_vel` directly. If
you later need real hardware, the swap point is narrow — replace the plugin
with a `ros2_control` block and add the two spawners; nothing else changes.

## Bridge directions

Every mapping in `config/gz_bridge.yaml` is one-directional except where the
topic is genuinely bidirectional. Getting a direction backwards is a silent
failure (the topic exists, nothing flows):

- `/cmd_vel` — **ROS → GZ**. Only inbound command in the system.
- `/odom`, `/tf`, `/joint_states`, `/scan`, `/camera/*`, `/clock` — **GZ → ROS**.
- `/tf` is bridged as `tf2_msgs/TFMessage` ↔ `gz.msgs.Pose_V`; the `DiffDrive`
  plugin's `tf_topic` must match, or odometry flows with no TF.

## Sensor authoring rule

In URDF, a `<sensor>` element inside `<link>` is **valid URDF and invalid for
Gazebo**. The URDF→SDF converter only looks at `<gazebo reference="link">`
blocks. A lidar declared the wrong way produces a robot with no sensor, no
error, and a `/scan` topic that never publishes. Each sensor here carries an
explicit `<gz_frame_id>` so the scan's `frame_id` matches the TF frame.

## Costmap layers

- **Local** — `voxel_layer` (from `/scan`) + `inflation_layer`, `rolling_window`,
  4×4 m at 0.05 m, `global_frame: odom`. Rolling so it follows the robot.
- **Global** — `static_layer` (the SLAM map) + `obstacle_layer` (live scan) +
  `inflation_layer`, `global_frame: map`, `track_unknown_space: true`.

`inflation_radius: 0.45` and `cost_scaling_factor: 3.0` are set for a 0.44 m
chassis in a corridor ~1 m wide — tight enough to use the arena, loose enough
not to declare the whole map lethal.

## Controller choice

`RegulatedPurePursuitController` at 0.45 m/s with `use_rotate_to_heading: true`:
rotate in place first, then drive. For a differential-drive base with no
swerve, that beats a controller that tries to arc from a standstill, and it
keeps the lookahead geometry valid.