# Vision-Guided Robotic Sorting Cell with Moving-Conveyor Interception

**ROS 2 Jazzy · Gazebo Harmonic · SolidWorks · URDF/Xacro · Python · RGB Colour Segmentation · Constrained-Pose IK · Quintic Trajectories**

**Author:** Samarth Amin  
**Discipline:** B.Tech Mechanical Engineering (Minor in Software Engineering)  
**Repository status:** Simulation-based engineering project

A simulation workcell that combines a SolidWorks-derived **4-DOF revolute manipulator**, a **two-finger parallel gripper**, a continuously circulating stadium conveyor, RGB-based workpiece classification and model-synchronised moving-object interception.

> **Scope note:** The current URDF contains four revolute arm joints and two independently actuated prismatic gripper joints. The gripper is modelled as rigid bodies in Gazebo; elastic Fin Ray deformation and physical contact-based grasp performance are not validated by the current simulation.

---

## Visual Demonstration

**Gazebo Harmonic simulation recording**

https://github.com/user-attachments/assets/de3e2d94-82c2-4213-9ce3-d73b05b5dad1

**Workcell overview**

<img width="1600" alt="Workcell workspace and reachability overview" src="https://github.com/user-attachments/assets/83c5bec9-92e7-4e68-b995-2478a441db5d" />

---

## 1. Project Overview

The project develops a CAD-to-ROS 2 workflow for a compact robotic sorting cell. The manipulator is described using URDF/Xacro and simulated in Gazebo Harmonic. A Python ROS 2 node coordinates the conveyor path, camera image processing, constrained-pose inverse kinematics, joint commands and Gazebo detachable-joint requests.

The stadium-shaped conveyor carries **15 workpieces** around a continuous loop:

- **10 target workpieces:** five red and five blue blocks.
- **5 distractors:** yellow, purple, orange, cyan and green blocks.
- **Two sorting destinations:** a red bucket and a blue bucket.

### Main implemented elements

1. **Mechanical CAD integration:** seven SolidWorks-exported STL subassemblies are referenced by the URDF/Xacro robot description.
2. **Four-axis arm control:** four revolute joints provide turret yaw, shoulder pitch, wrist pitch and wrist roll. Two independent prismatic joints operate the gripper fingers.
3. **Conveyor path tracking:** the controller integrates a model-based loop coordinate and publishes X/Y position commands to 15 two-axis carrier cradles.
4. **RGB image processing:** channel-dominance masks classify red, blue and five distractor colours. Image moments and contour dimensions are used for an upright-versus-fallen pose-state heuristic.
5. **Moving-object interception:** the controller reduces conveyor speed during pickup and updates the arm's constrained-pose target while descending.
6. **Smooth joint references:** point-to-point joint commands use fifth-order quintic time scaling and Gazebo position-controller plugins.

### Important implementation boundaries

- The pickup XY coordinates are taken from the modelled carrier path, not from the camera-derived centroid. The camera-derived image position is calculated but is not currently used as the arm's pickup target.
- The image orientation angle is calculated, but the sorting path does not currently pass it into the wrist-roll command.
- When no camera detection is returned, the controller falls back to the workpiece colour stored in the configuration file. The current benchmark is therefore not an independent camera-only classification test.
- During grasping, Gazebo's DetachableJoint constraint provides idealised transport. The current benchmark does not demonstrate a purely frictional, compliant Fin Ray grasp.

---

## 2. Workcell Architecture

The simulated workcell contains:

- **Manipulator:** four revolute arm joints: waist yaw, shoulder pitch, wrist pitch and wrist roll.
- **Gripper:** two independently commanded prismatic fingers acting as a parallel-jaw gripper.
- **Conveyor:** two straight lanes connected by two 180-degree turnaround sections.
- **Carriers:** 15 independently positioned X/Y cradle assemblies.
- **Camera:** overhead RGB camera publishing to the ROS image topic.
- **Buckets:** separate red and blue open-top destinations.

### Principal configuration values

| Parameter | Configuration value |
|---|---:|
| Turret yaw origin, x0 | 0.0258 m |
| Turret yaw origin, y0 | 0.0000 m |
| Shoulder X offset, j2_x | -0.0581 m |
| Shoulder height, Z_shoulder | 0.2581 m |
| Arm beam length, L2 | 0.2250 m |
| Gripper pad X offset, X_pad | 0.0795 m |
| Gripper pad Z offset, Z_pad | -0.0120 m |
| Nominal horizontal grasp radius, R_grasp | 0.1908 m |
| Upright grasp TCP height | 0.098 m |
| Fallen grasp TCP height | 0.080 m |
| Camera position | (0.2166, 0.0000, 0.72) m |
| Camera horizontal field of view | 0.70 rad |
| Camera resolution / update rate | 320 × 240 px / 15 Hz |
| Conveyor straight length | 1.00 m per lane |
| Conveyor path radius | 0.14 m |
| Modelled conveyor perimeter | 2.8796 m |
| Carrier cradle rim height | 9 mm |

The conveyor's visible drum geometry and the controller's path parameterisation are separate modelling elements. The controller uses the 0.14 m path radius in its loop calculation.

---

## 3. Mechanical Design and Degrees of Freedom

### CAD-to-URDF workflow

The robot description references seven SolidWorks-exported STL subassemblies:

- base_link.stl
- link_turret.stl
- link_upright.stl
- link_arm_beam.stl
- gripper_base.stl
- gripper_finger_left.stl
- gripper_finger_right.stl

The current URDF contains **four active revolute manipulator joints** and **two prismatic gripper joints**. The prismatic finger joints are not counted as additional arm positioning axes.

### Arm joint configuration

| Joint | URDF joint | Parent → child | Axis | Origin XYZ (m) | Limits |
|---|---|---|---|---|---|
| 1 | joint_1 | base_link → link_upright | Z | (0.0258, 0, 0.0800) | -π to +π rad |
| 2 | joint_2 | link_upright → link_arm_beam | Y | (-0.0581, 0, 0.1781) | -1.20 to +1.20 rad |
| 3 | joint_3 | link_arm_beam → link_wrist | Y | (0.2250, 0, 0) | -1.57 to +1.57 rad |
| 4 | joint_4 | link_wrist → gripper_base | X | (0, 0, 0) | -π to +π rad |

The gripper has two additional prismatic joints:

| Joint | Axis | Origin XYZ (m) | Limits |
|---|---|---|---|
| finger_left_joint | +Y | (0.0535, +0.016, -0.0120) | -0.016 to +0.018 m |
| finger_right_joint | -Y | (0.0535, -0.016, -0.0120) | -0.016 to +0.018 m |

### DOF scope decision

The initial project direction considered a fifth revolute positioning axis. The current implementation was scoped to four revolute arm axes. In the sorting controller, the shoulder and wrist pitch commands are constrained by:

\[
q_3=-q_2
\]

This maintains the selected horizontal-tool pose during the pickup motion. The resulting workspace is therefore constrained; it is not an arbitrary six-dimensional end-effector pose solver. A fifth independent positioning joint is not present in the current URDF and is not claimed as implemented.

---

## 4. Constrained-Pose Kinematics

The sorting node implements a closed-form constrained-pose IK routine. For a target point (X, Y, Z_TCP), the turret yaw is:

\[
q_1=\operatorname{atan2}(Y-y_0,\;X-x_0)
\]

The shoulder pitch is calculated using the configured shoulder height, gripper pad offset and arm length:

\[
q_2=
\arcsin\left[
\operatorname{clip}
\left(
\frac{Z_{\mathrm{shoulder}}+Z_{\mathrm{pad}}-Z_{\mathrm{TCP}}}{L_2},
-0.85,\;0.85
\right)
\right]
\]

The wrist pitch is constrained to:

\[
q_3=-q_2
\]

The wrist-roll command is an input to the IK function. In the current sorting sequence it defaults to zero; the image-derived orientation angle is not yet connected to this command.

For the constrained horizontal-tool pose, the radial reach relation is:

\[
R(q_2)=j_{2x}+L_2\cos(q_2)+X_{\mathrm{pad}}
\]

The modelled pickup lane is positioned using the configured nominal grasp radius. The forward-kinematics helper is also used by the controller when calculating a tool-centre-point position for optional attached-object pose synchronisation.

---

## 5. Quintic Trajectory Generation and Joint Control

Each point-to-point arm command uses fifth-order quintic time scaling:

\[
s(\tau)=10\tau^3-15\tau^4+6\tau^5,
\qquad \tau=\frac{t}{T}\in[0,1]
\]

The velocity and acceleration profiles are:

\[
\dot{s}(\tau)=
\frac{30\tau^2-60\tau^3+30\tau^4}{T}
\]

\[
\ddot{s}(\tau)=
\frac{60\tau-180\tau^2+120\tau^3}{T^2}
\]

This gives zero velocity and acceleration at the start and end of each rest-to-rest segment.

<img width="1600" alt="Quintic trajectory profiles" src="media/quintic_trajectories.png" />

The Python node publishes joint-position setpoints at 60 Hz. Gazebo's JointPositionController plugins use the following gains:

| Joint | Kp | Ki | Kd | URDF passive damping |
|---|---:|---:|---:|---:|
| joint_1 | 200 | 0 | 28 | 3.5 |
| joint_2 | 240 | 0 | 32 | 3.5 |
| joint_3 | 140 | 0 | 16 | 2.0 |
| joint_4 | 45 | 0 | 6 | 1.5 |

The gains are simulation settings. They have not been validated on a physical actuator or supported by a measured damping-ratio study. The current implementation should not be described as experimentally proven zero-oscillation control.

---

## 6. Mechanical Torque Sizing

The README's shoulder-joint sizing calculation uses the masses and inertia values specified in the URDF, a 250 g reference payload, a peak angular acceleration of 4 rad/s² and a design factor of 2.0.

| Quantity | Current calculation |
|---|---:|
| Distal assembly mass, including reference payload | 2.640 kg |
| Static gravity torque | 4.7071 N·m |
| Effective reflected inertia | 0.1265 kg·m² |
| Dynamic torque at 4 rad/s² | 0.5060 N·m |
| Design torque with factor 2.0 | 10.43 N·m |

\[
\tau_{\mathrm{design}}=
2.0\left(4.7071+0.1265\times4.0\right)
\approx10.43\ \mathrm{N\,m}
\]

### Reference motor and transmission estimate

The documented reference selection is a StepperOnline 23HS22-2804S NEMA 23 stepper motor with 1.26 N·m holding torque, a 5:1 planetary stage and a 2:1 timing-belt stage. With an assumed combined efficiency of 0.85, the holding-torque-based estimate is:

\[
\tau_{\mathrm{out,estimate}}=1.26\times10\times0.85
=10.71\ \mathrm{N\,m}
\]

This is only approximately 2.7% above the calculated design torque. It is a preliminary estimate based on holding torque, not a verified operating torque at the required speed. A motor torque-speed curve, gearbox rating and transmission loss validation are still required.

**Simulation limitation:** the current URDF effort limits are 90 N·m for joints 1 and 2, 50 N·m for joint 3 and 25 N·m for joint 4. These do not enforce the 10.71 N·m reference output estimate. Consequently, the present Gazebo controller has not been validated against the selected motor's torque-speed or saturation limits.

<img width="1600" alt="Mechanical joint torque sizing" src="media/torque_analysis.png" />

---

## 7. Stadium Conveyor and Moving-Object Interception

The conveyor controller uses a loop coordinate \(s\) and a piecewise path function:

1. Active pickup lane: straight translation along the active lane.
2. Front turnaround: 180-degree arc.
3. Return lane: straight translation in the opposite direction.
4. Rear turnaround: 180-degree arc back to the active lane.

The configured path perimeter is 2.8796 m. Each carrier is represented by two prismatic stages, one for X and one for Y. The Python node publishes the carrier's X/Y offsets while advancing the shared loop coordinate.

### Conveyor speed settings

| Operating phase | Controller target |
|---|---:|
| Cruise | 42 mm/s |
| Precision pickup synchronisation | 6 mm/s |
| Post-lift transport phase | 16 mm/s |
| End-of-run circulation | 55 mm/s |

A belt-speed slew limit is applied in the conveyor update loop. During pickup, the arm target is recalculated from the modelled carrier path as the conveyor advances.

<img width="1600" alt="Conveyor tracking and carrier path" src="https://github.com/user-attachments/assets/6b38ee91-055f-4a8a-8e0c-83bfc986ce25" />

The carrier pad uses a friction coefficient of 2.0 in the SDF. This is a simulation parameter, not an experimentally measured material coefficient.

---

## 8. Gripper and Gazebo Attachment Workflow

The current gripper is a two-finger parallel-jaw mechanism with independently commanded prismatic joints. The visual finger meshes may be described as Fin Ray-inspired only in terms of their geometry; the current Gazebo collision bodies are rigid and do not model elastic finger deformation.

### Current grasp sequence

1. The arm tracks the carrier and descends with the fingers open.
2. The controller requests a Gazebo DetachableJoint attachment.
3. The controller waits for Gazebo's detachable-joint state topic to confirm attachment.
4. If attachment is confirmed, the fingers close.
5. The controller samples finger joint positions.
6. The lift and bucket transport use the confirmed detachable-joint state as the success condition.

**Important:** the current script calculates the finger-position check, but then overwrites the result with the detachable-joint confirmation. Finger displacement therefore does not independently verify a physical grasp. The DetachableJoint provides an idealised transport constraint.

The optional attached-object pose synchronisation helper is present in the script, but the current sorting path passes no held-model name to the movement functions; that helper is not active in the benchmark sequence.

The Xacro currently assigns friction values of 250 to the gripper finger contact surfaces. These are simulation-tuned values and have not been calibrated against a physical rubber/Fin Ray contact pair. The current sorting result should not be interpreted as proof of frictional force closure.

<img width="1600" alt="Gripper and Gazebo attachment workflow" src="https://github.com/user-attachments/assets/d6e3c4d7-a5aa-4b2c-9626-2ee6d67b8ef1" />

---

## 9. RGB Vision and Workpiece Classification

The overhead camera publishes 320 × 240 RGB images at 15 Hz. The Python node applies channel-dominance colour masks for seven classes:

- Target colours: RED and BLUE.
- Distractor colours: YELLOW, PURPLE, ORANGE, CYAN and GREEN.

The controller also calculates the image centroid, contour spans, second-order central moments and an image-plane principal-axis angle:

\[
\theta_{\mathrm{img}}=
\frac{1}{2}\operatorname{atan2}
(2\mu_{11},\mu_{20}-\mu_{02})
\]

An aspect-ratio / pixel-area heuristic distinguishes the current upright and tilted/fallen states. That state selects a different grasp height and jaw opening.

### Perception limitations

- If no camera detection is available, the controller uses the configured expected colour for that carrier.
- A detected colour is cross-checked against the configured carrier metadata.
- The camera-derived centroid is converted to an approximate position using a pinhole projection, but that position is not used as the pickup XY target.
- The projection uses an approximate focal-length calculation from the configured horizontal field of view; a calibrated camera matrix has not been documented.
- The principal-axis angle is computed but is not currently used to command wrist roll.

The current pipeline is therefore **RGB colour segmentation with image-moment pose-state heuristics, combined with model-based carrier tracking**. It is not a neural-network detector or a fully vision-localised manipulation system.

---

## 10. Simulation Benchmark and Results

The scenario contains 15 circulating workpieces: five red targets, five blue targets and five distractors.

The current script reports the following counters for a single run:

| Controller-log metric | Reported result | Interpretation |
|---|---:|---|
| Red target workflows marked sorted | 5 / 5 | Reached the end-of-sort controller path |
| Blue target workflows marked sorted | 5 / 5 | Reached the end-of-sort controller path |
| Distractors skipped | 5 / 5 | Classified or metadata-fallback path skipped them |
| Gazebo maximum step size | 0.002 s | Configured physics step |
| Gazebo update-rate target | 500 Hz | Configured target, not an independently measured rate |
| Real-time factor | Not reported | Depends on host simulation performance |

**Benchmark interpretation:** the current script records a target as sorted when the sorting sequence reaches its completion path. The outcome is not independently verified by checking that the block is inside the correct bucket, and the release confirmation result does not gate the final sorted counter. The 5/5 figures should therefore be treated as single-run controller-flow results, not as a validated physical sorting success rate.

The script records a per-target elapsed time from the start of manipulation to the return-to-ready sequence. The previous 4.15 s figure is not retained here because it needs to be reproduced from a fresh benchmark log using that full timing scope.

<img width="1600" alt="Single-run sorting benchmark summary" src="https://github.com/user-attachments/assets/456f5632-7378-4df3-b487-ee3a8dce0f45" />

---

## 11. Software Stack

| Layer | Technology |
|---|---|
| Mechanical CAD | SolidWorks |
| Robot description | URDF / Xacro |
| Robotics middleware | ROS 2 Jazzy |
| Physics simulation | Gazebo Harmonic |
| Programming | Python |
| Image processing | NumPy-based RGB masks and image moments |
| Joint feedback | sensor_msgs/JointState |
| Joint commands | std_msgs/Float64 |
| Simulation bridge | ros_gz_bridge |
| Visualisation | RViz2 |

---

## 12. Repository Structure

~~~text
repository-root/
├── README.md
└── src/
    └── jork_arm_description/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── kinematics_config.json
        ├── launch/
        │   ├── display.launch.py
        │   └── gazebo.launch.py
        ├── meshes/
        │   ├── base_link.stl
        │   ├── link_turret.stl
        │   ├── link_upright.stl
        │   ├── link_arm_beam.stl
        │   ├── gripper_base.stl
        │   ├── gripper_finger_left.stl
        │   └── gripper_finger_right.stl
        ├── rviz/
        │   └── view_arm.rviz
        ├── scripts/
        │   ├── pick_and_place_demo.py
        │   └── vision_sorting_cell.py
        ├── urdf/
        │   └── jork_arm.urdf.xacro
        └── worlds/
            └── sorting_cell.sdf
~~~

---

## 13. Build and Run

From a ROS 2 Jazzy workspace containing this repository:

~~~bash
cd ~/projects/scout-nav2-gazebo
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jork_arm_description
source install/setup.bash
~~~

Launch Gazebo:

~~~bash
ros2 launch jork_arm_description gazebo.launch.py
~~~

In a second terminal, source the workspace and run the sorting node:

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/projects/scout-nav2-gazebo/install/setup.bash
ros2 run jork_arm_description vision_sorting_cell.py
~~~

To inspect the robot description in RViz, use the display launch file:

~~~bash
ros2 launch jork_arm_description display.launch.py
~~~

---

## 14. Known Limitations and Next Engineering Steps

The following items are deliberately separated from completed implementation:

1. **Actuator realism:** reconcile Gazebo joint effort limits with motor torque-speed curves, gearbox ratings and the reference torque calculation.
2. **Contact-based grasping:** replace idealised detachable-joint transport with contact-based grasp verification and physically defensible friction/contact parameters.
3. **Vision localisation:** use calibrated camera intrinsics and camera-derived object coordinates for the actual pickup target; remove the expected-colour fallback from evaluation runs.
4. **Pose alignment:** connect the image-derived object orientation to wrist-roll control and test it with rotated workpieces.
5. **Sorting validation:** verify release success and bucket occupancy independently; repeat runs with randomised carrier positions and report success rate and cycle-time statistics.
6. **Trajectory evaluation:** regenerate position, velocity, acceleration and jerk plots from the exact trajectory parameters used in the simulation.
7. **Mechanical transmission documentation:** document the actual belt/gear arrangement from the CAD assembly and include gear ratios, materials and dimensions only where they are supported by the design.

The current repository is a simulation-based design and integration project. Physical robot construction, real-world sorting accuracy and hardware performance are not claimed.


---

## 15. Reproducible Simulation Validation

The repository includes a static cross-file checker and runtime telemetry recorder. Use both before publishing updated performance figures.

### 15.1 Static consistency check

From the repository root:

~~~bash
python3 src/jork_arm_description/scripts/check_project_consistency.py
~~~

The checker compares the current JSON configuration against the URDF/Xacro and SDF, including:

- Four revolute arm joints and two prismatic gripper joints
- Joint origin values used by the constrained-pose IK
- Finger mounting origins
- Camera pose, field of view and image configuration
- Conveyor origin and stadium-loop perimeter
- Fifteen carrier/workpiece identifiers and target/distractor colour counts
- Detachable-joint plugin inventory
- ROS command-topic templates and launch bridges
- Package runtime dependencies and CMake installation

[PASS] means the checked files agree on that item. [WARN] identifies a known engineering limitation that needs review, not necessarily a broken build. [FAIL] means a cross-file mismatch must be resolved before trusting the run.

This is a static check. It does not launch Gazebo or prove that the robot moves correctly.

### 15.2 Build and launch

Build the package after pulling the validation branch:

~~~bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jork_arm_description
source install/setup.bash
~~~

Terminal 1 — start the workcell:

~~~bash
ros2 launch jork_arm_description gazebo.launch.py
~~~

Terminal 2 — run the complete 15-workpiece scenario and save telemetry:

~~~bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run jork_arm_description vision_sorting_cell.py --ros-args -p audit_dir:=$HOME/.ros/robot_sorting_audits
~~~

Do not set dry_run:=true for the complete sorting benchmark. The default is false.

### 15.3 Motion sequence recorded

| Phase label | Motion / operation | What is recorded |
|---|---|---|
| startup_to_ready | Move from initial pose to side-ready posture | Six commanded and measured joint positions |
| approach_pre_descent | Track carrier and move above pickup | Joint tracking and conveyor path |
| approach_final_descent | Descend to configured grasp height | Joint tracking and grasp height |
| hold_during_attach | Maintain pickup pose while requesting Gazebo attachment | Attachment wait duration and joint tracking |
| gripper_close_after_attach | Close both prismatic fingers after attachment confirmation | Finger commands and measured positions |
| lift_clear_of_cradle | Raise the workpiece from the cradle | Joint tracking and lift duration |
| transport_to_bucket_hover | Move to the selected colour bucket | Joint tracking and transport duration |
| bucket_final_descent | Descend to the bucket drop pose | Joint tracking and arrival check |
| open_jaws_at_drop_pose | Open the fingers | Finger commands and measured positions |
| retract_from_bucket | Retract from the drop pose | Joint tracking |
| return_to_ready | Return to the side-ready posture | Joint tracking and return duration |
| end_of_run_park | Move to the final park pose | Joint tracking |

The event log also records classification results, whether camera metadata fallback was used, attach/detach requests and confirmations, finger-position checks, bucket-pose arrival checks, skipped distractors and conveyor speed targets.

### 15.4 Audit files

Each run creates a timestamped folder:

~~~text
~/.ros/robot_sorting_audits/<UTC-run-id>/
├── run_summary.json
├── phase_events.csv
├── motion_tracking.csv
├── joint_state_telemetry.csv
└── conveyor_path_commands.csv
~~~

- run_summary.json: run status, configuration snapshot, part outcomes, sample counts and maximum observed absolute tracking error.
- phase_events.csv: timestamped phase transitions, classification decisions, grasp/release confirmations and skip reasons.
- motion_tracking.csv: commanded versus measured positions for all four revolute joints and both prismatic fingers during every programmed movement.
- joint_state_telemetry.csv: measured joint-state samples from ROS 2.
- conveyor_path_commands.csv: loop coordinate, commanded belt-speed target and rate-limited slewed belt-speed state, each carrier's path coordinates and commanded X/Y offsets. These are commanded carrier targets, not measured carrier positions.

Joint angles are in radians; finger positions and conveyor coordinates are in metres; belt speeds are in metres per second. Wall-clock and simulation-clock timestamps are both included.

The recorder saves its files on normal completion and on Ctrl+C. If the process exits forcibly or the computer loses power, the last run may be incomplete.

### 15.5 Interpreting the benchmark correctly

The target cycle timer starts immediately before the first pickup approach and ends after the robot returns to the ready posture. It includes descent, attachment wait, finger closure, lift, bucket transport, bucket descent, release wait and return motion. It does not include the time spent waiting for the workpiece to arrive at the pickup window.

A Gazebo detachable-joint confirmation proves only that the simulated constraint was attached or detached. It does not prove frictional grasp force closure or independently verify that a released block settled inside the intended bucket. The run summary therefore distinguishes attachment/release sequence completion from verified bucket occupancy.

The camera pipeline currently has a configured-colour fallback when no image detection is available. Pickup XY is derived from the modelled carrier path, and the calculated image centroid and principal-axis angle are not yet used to command pickup XY or wrist roll. These limitations are included in the audit output so the run cannot be misreported as camera-only autonomous localisation.

### 15.6 Evidence required before updating project claims

For a defensible result, retain:

1. The full audit folder from each run.
2. A screen recording showing the complete conveyor and manipulator sequence.
3. The static checker output.
4. At least three complete runs with the same configuration before reporting an average cycle time or repeatability statistic.
5. Separate counts for camera detections, metadata fallbacks, attach confirmations, release confirmations, arm-arrival checks and independently verified bucket placements.

Do not report a 100% sorting success rate from controller-flow counters alone. Bucket occupancy must be checked independently before calling a block successfully sorted.
