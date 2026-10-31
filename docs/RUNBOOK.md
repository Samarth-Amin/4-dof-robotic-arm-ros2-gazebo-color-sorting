# Scout Runbook

Operating procedure, in the order you would actually do it.

## 0. Every shell

```bash
source /opt/ros/jazzy/setup.bash
cd ~/projects/scout-nav2-gazebo
source install/setup.bash
```

If `install/` is missing or you edited `CMakeLists.txt`:

```bash
colcon build --symlink-install
```

Launch files, the xacro, and YAML configs are symlinked into `install/`, so
editing those needs no rebuild. Only `package.xml` / `CMakeLists.txt` changes do.

## 1. Model check — no physics

```bash
ros2 launch scout_description display.launch.py
```

Expect RViz with the robot at the origin and the `joint_state_publisher_gui`
slider panel. Drag the wheel sliders — the wheels should spin. This confirms
the URDF and the visual meshes before Gazebo is involved.

Quick non-GUI equivalent:

```bash
xacro src/scout_description/urdf/scout.urdf.xacro > /tmp/scout.urdf
check_urdf /tmp/scout.urdf
```

## 2. Gazebo

```bash
ros2 launch scout_description gazebo.launch.py
```

`gui:=false` runs the server only (`gz sim -s`) — use that on a headless box or
in CI. The robot spawns at (-4.5, -3.5) facing 0.9 rad, in the arena corner.

### Verify the bridge

In a second shell (with the overlay sourced):

```bash
ros2 topic list | grep -E 'scan|odom|clock|cmd_vel|camera'
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom          # ~50 Hz
ros2 topic echo /clock --once
```

`/clock` matters — every node that uses `use_sim_time: true` will sit frozen
without it.

If `/scan` is silent, check that the sensor actually made it into the SDF:

```bash
gz topic -l | grep scan
```

A missing `/scan` almost always means the `<sensor>` was placed inside `<link>`
rather than inside `<gazebo reference="...">` — the URDF→SDF conversion silently
drops it there.

## 3. Drive

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Watch the robot move. If it drives backwards, the wheel joints in the URDF are
swapped; if it spins instead of driving straight, the wheel separation in the
`DiffDrive` plugin disagrees with the URDF (`2 * wheel_y` = 0.38).

## 4. Map the arena

```bash
scripts/map.sh
```

That launches Gazebo + `slam_toolbox` + RViz. Drive a full loop of the arena
(hug the walls, pass through the partition gaps, come back to the start). Then
**Ctrl-C** — the script's trap runs `map_saver_cli` and writes:

```
src/scout_description/maps/scout_map.pgm
src/scout_description/maps/scout_map.yaml   (already present, gets overwritten)
```

Check it landed before moving on:

```bash
ls -l src/scout_description/maps/
```

If the `.pgm` is absent, `map_saver` could not reach `/map` — slam_toolbox had
already died, or you Ctrl-C'd before it published.

> Tuning note: `config/slam_toolbox.yaml` has `minimum_travel_distance: 0.3`.
> Driving slowly produces fewer, better-spaced nodes; blasting around at full
> speed on a 12 m arena smears the map.

## 5. Navigate

```bash
scripts/nav.sh
```

This runs `slam:=false`, i.e. `map_server` loads `scout_map.yaml` and AMCL
localizes against it. AMCL is seeded with the spawn pose (-4.5, -3.5, yaw 0.9),
so it starts converged; if you moved the spawn point, either update
`initial_pose` in `launch/navigation.launch.py` or use RViz's **2D Pose
Estimate** tool.

In RViz:

1. Set **Fixed Frame** to `map` (it already is in `nav.rviz`).
2. Confirm the laser scan lines up with the map walls. If it is offset by a
   constant amount, AMCL is mis-seeded — use 2D Pose Estimate.
3. Use **2D Goal Pose** to send a goal.

```bash
# headless equivalent
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

### Expected timeline after `scripts/nav.sh`

| t (s) | What comes up |
|---|---|
| 0 | Gazebo server + client, `robot_state_publisher` |
| 2 | robot spawned |
| 3 | `ros_gz` bridge |
| ~8 | Nav2 servers (controller, planner, behaviors, bt_navigator…) |
| ~12 | RViz |

Nav2 servers auto-start via lifecycle managers. To confirm everything is up:

```bash
ros2 lifecycle get /controller_server      # active [3]
ros2 lifecycle get /bt_navigator           # active [3]
ros2 action list | grep navigate
```

## 6. Live mapping + Nav2 (no saved map)

```bash
scripts/nav.sh --slam
```

slam_toolbox publishes `map→odom` and Nav2 plans on the map it is building.
Useful for a quick demo; for repeatable results, map first then localize.

**Never run both.** Exactly one node may publish `map→odom`. `navigation.launch.py`
enforces this with the `slam` argument — `slam_toolbox` and the
`map_server`+AMCL pair are mutually exclusive by construction.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Robot falls through floor / jitters | spawn `z` too low, or inertia nonsense | `z:=0.05`; check inertia in xacro |
| Robot pirouettes on the spot | caster friction too high | `mu1`/`mu2` on casters must be ~0.001 |
| Robot pitches nose-down | caster radius wrong so contact is not at z=0 | caster origin z = `caster_radius - base_z` |
| `/scan` empty in RViz | `Fixed Frame` is `odom` before any TF exists, or sensor not in SDF | set frame `map`/`odom`; `gz topic -l \| grep scan` |
| Nav2 "Timed out waiting for transform" | `/clock` not bridged | `ros2 topic hz /clock` |
| Costmap all unknown | static layer got no `/map` | `ros2 topic echo /map --once`; is map_server active? |
| Goal accepted but robot doesn't move | `cmd_vel` bridge direction wrong | it must be `ROS_TO_GZ`; `gz topic -e -t /cmd_vel` |
| AMCL drifts | seeded pose wrong | 2D Pose Estimate in RViz |