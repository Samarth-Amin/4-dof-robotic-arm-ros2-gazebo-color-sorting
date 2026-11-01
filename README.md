
This project demonstrates the integration of mechanical CAD design, robot kinematic and dynamic modelling, motion control, machine vision, and physics simulation into an autonomous manufacturing sorting workcell.
=======
# Vision-Guided Robotic Sorting Cell with Moving-Conveyor Interception

**ROS 2 Jazzy · Gazebo Harmonic · SolidWorks · URDF/Xacro · Python · Machine Vision · Analytical Inverse Kinematics · Quintic Trajectory Planning**

**Author:** Samarth Amin  
**Contact:** `23bme005@nirmauni.ac.in`  
**Discipline:** B.Tech Mechanical Engineering (Minor in Software Engineering)

A simulation-based robotic manufacturing workcell integrating a custom SolidWorks-derived **4-DOF belt-and-bevel robotic manipulator**, a **2-axis compliant Fin Ray parallel gripper**, a **closed-loop stadium conveyor**, **RGB machine-vision classification**, and **co-moving lead-pursuit interception**.

---

## Visual Demonstration

* **Simulation Recording:** 

https://github.com/user-attachments/assets/de3e2d94-82c2-4213-9ce3-d73b05b5dad1


* **Workcell Preview:**

<img width="3900" height="1650" alt="workspace_envelope" src="https://github.com/user-attachments/assets/83c5bec9-92e7-4e68-b995-2478a441db5d" />


---

## 1. Project Overview

This project develops an end-to-end mechanical CAD-to-ROS 2 simulation workflow for a belt-and-bevel-driven robotic manipulator and a compliant Fin Ray parallel gripper. The workcell is modelled in URDF/Xacro and evaluated in **Gazebo Harmonic**, with ROS 2 nodes coordinating camera-based workpiece classification, closed-loop conveyor tracking, quintic trajectory generation, and grasp verification.

The system operates around a continuously circulating stadium-shaped conveyor carrying **15 workpieces**:
* **10 Target Workpieces:** 5 Red and 5 Blue blocks arranged in uneven multi-object clusters (`2 RED`, `1 BLUE`, `3 BLUE`, `1 RED`, `2 RED`, `1 BLUE`).
* **5 Distractor Workpieces:** Yellow, Purple, Orange, Cyan, and Green blocks interspersed between the target clusters.

### Main Capabilities

1. **Hybrid Vision Classification & Pose-State Estimation:** An overhead RGB camera uses channel-dominance segmentation to distinguish red and blue targets from the five distractor colours, while second-order central image moments ($\\mu_{20}, \\mu_{02}, \\mu_{11}$) classify whether an incoming workpiece is **upright (vertical)** or **tilted/fallen (horizontal)** and estimate its image-plane orientation angle for wrist-roll alignment (`joint_4`).
2. **Model-Synchronized Moving-Conveyor Interception:** Rather than commanding the arm to a static coordinate, the controller tracks the carrier's parametric path coordinate $s(t)$ around the loop, reduces belt speed from $42\\text{ mm/s}$ (cruise) to $6\\text{ mm/s}$ (precision grasp window), and continuously updates the inverse-kinematics target during descent and finger closure.
3. **Finger-Closure Verification & `DetachableJoint` Transport:** The controller closes the prismatic Fin Ray fingers first, evaluates `/joint_states` displacement against a fault threshold ($q_{\\mathrm{th}} = -7.5\\text{ mm}$), and engages Gazebo's `DetachableJoint` constraint for rigid transport once grasp readiness is confirmed. Failed grasps trigger a non-blocking $0.3\\text{ s}$ recovery/skip transition so missed targets circulate for a subsequent pass.
4. **Smooth $C^2$ Joint Motion & PD Control:** Joint references are interpolated at $60\\text{ Hz}$ using fifth-order quintic time scaling with proportional-derivative control ($K_i = 0$) and passive joint damping ($b = 3.5\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad}$ on primary axes) to prevent integral limit-cycle oscillation.
5. **Arrival-Order Simulation Benchmarking:** Evaluates sorting completion, distractor rejection, per-stage manipulation cycle time, and simulation execution rate across the 15-workpiece loop scenario.

---

## 2. Workcell Architecture

The simulated workcell consists of:
* **4-DOF Revolute Manipulator (`jork_arm.urdf.xacro`):** Waist turret yaw (`joint_1`), shoulder pitch (`joint_2`), wrist pitch (`joint_3`), and wrist differential roll (`joint_4`).
* **2-Axis Compliant Fin Ray Parallel Gripper:** Symmetrical prismatic finger joints (`finger_left_joint` and `finger_right_joint`).
* **Closed-Loop Stadium Conveyor (`sorting_cell.sdf`):** Two straight transport lanes connected by two $180^\\circ$ turnaround drums with 15 circulating 2-DOF (`X, Y`) carrier cradles.
* **Overhead RGB Vision Sensor (`overhead_camera`):** Mounted at $(X, Y, Z) = (0.2166, -0.0000, 0.72)\\text{ m}$ looking vertically downward ($\\text{FOV}_h = 0.70\\text{ rad}$, $320 \\times 240\\text{ px}$ at $15\\text{ Hz}$).
* **Dual Open-Top Sorting Containers:** Red Bucket at $(-0.0235, 0.1843)\\text{ m}$ and Blue Bucket at $(-0.0235, -0.1843)\\text{ m}$.

### Calibrated Workcell & Conveyor Parameters

| Parameter | Config / World Key | Calibrated Value |
| :--- | :--- | :--- |
| **Turret Yaw Origin $(x_0, y_0)$** | `x0`, `y0` | $(0.0258,\\; -0.0000)\\text{ m}$ |
| **Shoulder Pitch Offset $(j_{2x}, Z_{\\mathrm{shoulder}})$** | `j2_x`, `Z_shoulder` | $(-0.0581,\\; 0.2581)\\text{ m}$ |
| **Arm Beam Length ($L_2$)** | `L2` | $0.2250\\text{ m}$ |
| **Gripper Pad Offset $(X_{\\mathrm{pad}}, Z_{\\mathrm{pad}})$** | `X_pad`, `Z_pad` | $(0.0795,\\; -0.0120)\\text{ m}$ |
| **Nominal Horizontal Grasp Reach ($R_{\\mathrm{grasp}}$)** | `R_grasp` | $0.1908\\text{ m}$ |
| **Overhead Camera Center $(X_{\\mathrm{cam}}, Y_{\\mathrm{cam}}, Z_{\\mathrm{cam}})$** | `cam_x`, `cam_y`, `cam_z` | $(0.2166,\\; -0.0000,\\; 0.72)\\text{ m}$ |
| **Straight Conveyor Lane Length ($L_{\\mathrm{straight}}$)** | `y_front - y_rear` | $1.00\\text{ m each}$ ($Y_{\\mathrm{rel}} \\in [-0.55, +0.45]\\text{ m}$) |
| **Turnaround Drum Radius ($R_{\\mathrm{loop}}$)** | `R_loop` | $0.14\\text{ m}$ |
| **Total Conveyor Loop Perimeter ($P_{\\mathrm{loop}}$)** | `P_loop` | $2.8796\\text{ m}$ |
| **Carrier Cradle Retaining-Rim Height** | `rim_*` | $9.0\\text{ mm}$ (visual pocket + high-friction $\\mu=500$ base pad) |
| **Workpieces in Benchmark Scenario** | `parts` | $15$ ($5$ Red targets, $5$ Blue targets, $5$ Distractors) |

---

## 3. Mechanical Design and Kinematics

### CAD-to-URDF Workflow & DOF Accounting

The manipulator geometry is exported from SolidWorks as seven rigid-body STL subassemblies (`base_link.stl`, `link_turret.stl`, `link_upright.stl`, `link_arm_beam.stl`, `gripper_base.stl`, `gripper_finger_left.stl`, `gripper_finger_right.stl`).

In the current URDF implementation (`urdf/jork_arm.urdf.xacro`), `base_link.stl` and `link_turret.stl` form the stationary base assembly, while four active revolute joints (`joint_1` through `joint_4`) provide **4 spatial arm degrees of freedom** ($X, Y, Z$ positioning with constrained tool pitch plus wrist roll). The two Fin Ray finger joints (`finger_left_joint`, `finger_right_joint`) are independent prismatic end-effector joints and are accounted for separately from the arm's positioning DOF.

### Calibrated Joint & Modified Denavit–Hartenberg Table

| Joint $i$ | URDF Joint Name | Parent $\\rightarrow$ Child Link | Axis | Origin $(X, Y, Z)$ (m) | $\\theta_i$ (rad) | $d_i$ (m) | $a_i$ (m) | $\\alpha_i$ (rad) | Joint Limits |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **1** | `joint_1` | `base_link` $\\rightarrow$ `link_upright` | $Z_0$ (`0 0 1`) | $(0.0258, -0.0000, 0.0800)$ | $q_1$ | $0.0800$ | $-0.0581$ | $-\\pi/2$ | $[-\\pi, +\\pi]\\text{ rad}$ |
| **2** | `joint_2` | `link_upright` $\\rightarrow$ `link_arm_beam` | $Y_1$ (`0 1 0`) | $(-0.0581, 0.0000, 0.1781)$ | $q_2$ | $0.1781$ | $0.2250$ | $0.0000$ | $[-1.20, +1.20]\\text{ rad}$ |
| **3** | `joint_3` | `link_arm_beam` $\\rightarrow$ `link_wrist` | $Y_2$ (`0 1 0`) | $(0.2250, 0.0000, 0.0000)$ | $q_3$ | $0.0000$ | $0.0795$ | $-\\pi/2$ | $[-1.57, +1.57]\\text{ rad}$ |
| **4** | `joint_4` | `link_wrist` $\\rightarrow$ `gripper_base` | $X_3$ (`1 0 0`) | $(0.0000, 0.0000, 0.0000)$ | $q_4$ | $0.0000$ | $0.0000$ | $0.0000$ | $[-\\pi, +\\pi]\\text{ rad}$ |
| **Gripper L** | `finger_left_joint` | `gripper_base` $\\rightarrow$ `gripper_finger_left` | $+Y_4$ (`0 1 0`) | $(0.0500, +0.0160, -0.0120)$ | $0.0$ | $q_{f,L}$ | $0.0260$ | $0.0000$ | $[-0.016, +0.018]\\text{ m}$ |
| **Gripper R** | `finger_right_joint` | `gripper_base` $\\rightarrow$ `gripper_finger_right` | $-Y_4$ (`0 -1 0`) | $(0.0500, -0.0160, -0.0120)$ | $0.0$ | $q_{f,R}$ | $0.0260$ | $0.0000$ | $[-0.016, +0.018]\\text{ m}$ |

### Constrained-Pose Analytical Inverse Kinematics

Because the 4-DOF arm has one radial pitch link (`link_arm_beam` of length $L_2 = 0.2250\\text{ m}$) between the shoulder pivot and the wrist pitch pivot, keeping the gripper fingers horizontal ($q_2 + q_3 = 0$) yields the constrained-pose inverse kinematics relation used by `solve_ik()` in `scripts/vision_sorting_cell.py`:

$$q_1 = \\operatorname{atan2}(Y - y_0,\\; X - x_0)$$

$$\\sin q_2 = \\operatorname{clip}\\!\\left(\\frac{Z_{\\mathrm{shoulder}} + Z_{\\mathrm{pad}} - Z_{\\mathrm{TCP}}}{L_2},\\; -0.85,\\; +0.85\\right), \\qquad q_2 = \\arcsin(\\sin q_2)$$

$$q_3 = -q_2, \\qquad q_4 = \\psi_{\\mathrm{roll}}$$

Under this horizontal-tool constraint, the radial distance from the turret yaw axis is determined by $q_2$:

$$R(q_2) = j_{2x} + L_2 \\cos q_2 + X_{\\mathrm{pad}}$$

At the upright grasp height $Z_{\\mathrm{TCP}} = 0.098\\text{ m}$, the resulting grasp radius is $R_{\\mathrm{grasp}} = 0.1908\\text{ m}$, which defines the placement radius of both the active conveyor pickup lane ($X_{\\mathrm{belt}} = x_0 + R_{\\mathrm{grasp}} = 0.2166\\text{ m}$) and the two sorting buckets. Achieving independent radial reach $R$ and vertical height $Z$ simultaneously at fixed tool pitch requires a fifth revolute joint (see Section 13).

---

## 4. Trajectory Generation and Joint Control

Each point-to-point joint move uses fifth-order quintic time scaling so that velocity and acceleration are zero at the start and end of every trajectory segment:

$$s(\\tau) = 10\\tau^3 - 15\\tau^4 + 6\\tau^5, \\qquad \\tau = \\frac{t}{T} \\in [0, 1]$$

$$\\dot{s}(\\tau) = \\frac{30\\tau^2 - 60\\tau^3 + 30\\tau^4}{T}, \\qquad \\ddot{s}(\\tau) = \\frac{60\\tau - 180\\tau^2 + 120\\tau^3}{T^2}$$

![Quintic Trajectory Profiles](media/quintic_trajectories.png)

Joint position setpoints are published at $60\\text{ Hz}$ to Gazebo Harmonic's `JointPositionController` plugins with integral gain set to zero ($K_i = 0$) to eliminate steady-state integral windup tremor:
* `joint_1` (Waist Yaw): $K_p = 200,\\; K_i = 0,\\; K_d = 28$, passive damping $b = 3.5\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad}$.
* `joint_2` (Shoulder Pitch): $K_p = 240,\\; K_i = 0,\\; K_d = 32$, passive damping $b = 3.5\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad}$.
* `joint_3` (Wrist Pitch): $K_p = 140,\\; K_i = 0,\\; K_d = 16$, passive damping $b = 2.0\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad}$.
* `joint_4` (Wrist Roll): $K_p = 45,\\; K_i = 0,\\; K_d = 6$, passive damping $b = 1.5\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad}$.

---

## 5. Mechanical Torque Sizing

The worst-case shoulder pitch (`joint_2`) torque requirement is computed directly from the mass and inertia properties defined in `urdf/jork_arm.urdf.xacro` at full horizontal extension ($\cos q_2 = 1$), peak angular acceleration $\\alpha_{\\max} = 4.0\\text{ rad/s}^2$, and safety factor $FS = 2.0$:

| Component | URDF Link Name | Mass $m_i$ (kg) | Distal COM $r_{ci}$ from `joint_2` (m) | Local Inertia $I_{yy,i}$ ($\\text{kg}\\cdot\\text{m}^2$) | Gravity Torque $\\tau_{g,i} = m_i g r_{ci}$ ($\\text{N}\\cdot\\text{m}$) | Reflected Inertia $I_{yy,i} + m_i r_{ci}^2$ ($\\text{kg}\\cdot\\text{m}^2$) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Upper Arm Beam** | `link_arm_beam` | $1.400$ | $0.1125$ | $0.0150$ | $1.5451$ | $0.0327$ |
| **Wrist Pitch Hub** | `link_wrist` | $0.400$ | $0.2250$ | $0.0040$ | $0.8829$ | $0.0243$ |
| **Gripper Assembly** | `gripper_base` + 2 fingers | $0.590$ | $0.2647$ | $0.0050$ | $1.5323$ | $0.0464$ |
| **Rated Design Payload** | Workpiece ($250\\text{ g}$ max rated) | $0.250$ | $0.3045$ | $0.0000$ | $0.7468$ | $0.0232$ |
| **Total Distal Assembly** | — | **$2.640\\text{ kg}$** | — | — | **$\\tau_{\\mathrm{static}} = 4.7071\\text{ N}\\cdot\\text{m}$** | **$J_{\\mathrm{eff}} = 0.1265\\text{ kg}\\cdot\\text{m}^2$** |

With peak angular acceleration $\\alpha_{\\max} = 4.0\\text{ rad/s}^2$:

$$\\tau_{\\mathrm{dynamic}} = J_{\\mathrm{eff}} \\alpha_{\\max} = 0.1265 \\times 4.0 = 0.5060\\text{ N}\\cdot\\text{m}$$

$$\\tau_{\\mathrm{design}} = FS \\left( \\tau_{\\mathrm{static}} + \\tau_{\\mathrm{dynamic}} \\right) = 2.0 \\left( 4.7071 + 0.5060 \\right) = 10.43\\text{ N}\\cdot\\text{m}$$

### Reference Actuator & Transmission Selection
* **Motor:** StepperOnline `23HS22-2804S` NEMA 23 Bipolar Stepper ($1.26\\text{ N}\\cdot\\text{m}$ holding torque, $2.8\\text{ A}$/phase, rotor inertia $300\\text{ g}\\cdot\\text{cm}^2$).
* **Reduction Stage:** $5:1$ planetary gearbox coupled with a $2:1$ HTD-3M timing-belt pulley stage ($i_{\\mathrm{total}} = 10:1$, combined transmission efficiency $\\eta \\approx 0.85$).
* **Available Output Torque:** $\\tau_{\\mathrm{out,max}} = 1.26 \\times 10 \\times 0.85 = 10.71\\text{ N}\\cdot\\text{m} > \\tau_{\\mathrm{design}} (10.43\\text{ N}\\cdot\\text{m})$.

![Mechanical Joint Torque Sizing](media/torque_analysis.png)

---

## 6. Conveyor Tracking and Moving-Object Interception

The stadium conveyor track consists of four piecewise segments parameterized by arc length $s \\in [0, 2.8796)\\text{ m}$:
1. **Active Pickup Lane ($s \\in [0, 1.00]\\text{ m}$):** Straight $+Y$ translation along $X = 0.2166\\text{ m}$.
2. **Front $180^\\circ$ Turnaround Drum ($s \\in [1.00, 1.44]\\text{ m}$):** Semicircular arc of radius $R_{\\mathrm{loop}} = 0.14\\text{ m}$.
3. **Return Lane ($s \\in [1.44, 2.44]\\text{ m}$):** Straight $-Y$ translation along $X = 0.4966\\text{ m}$.
4. **Rear $180^\\circ$ Turnaround Drum ($s \\in [2.44, 2.88]\\text{ m}$):** Semicircular arc returning to the active lane.

During sorting, `vision_sorting_cell.py` monitors the arrival of any pending carrier puck entering the pickup window $s \\in [s_{\\mathrm{pickup}} - 0.018\\text{ m},\\; s_{\\mathrm{pickup}} + 0.025\\text{ m}]$ in arrival order. When a target puck enters the window, `move_puck_tracked()` reduces belt velocity from $42\\text{ mm/s}$ to $6\\text{ mm/s}$ and updates the inverse-kinematics target $(X_{\\mathrm{puck}}(t), Y_{\\mathrm{puck}}(t), Z(t))$ at $60\\text{ Hz}$ during descent and finger closure.

<img width="3900" height="1560" alt="conveyor_tracking" src="https://github.com/user-attachments/assets/6b38ee91-055f-4a8a-8e0c-83bfc986ce25" />


---

## 7. Fin Ray Gripper, Grasp Verification, and Simulation Attachment Mechanics

The gripper models two symmetrical prismatic Fin Ray fingers (`finger_left_joint`, `finger_right_joint`) with stroke limits $[-16\\text{ mm}, +18\\text{ mm}]$.

### Important Distinction: Contact Verification vs. `DetachableJoint` Transport
In multibody physics engines, transporting small rigid bodies at high slewing accelerations using pure penalty-based finger contact can introduce solver jitter or contact penetration. Therefore, the current simulation uses a **two-stage hybrid grasp architecture**:
1. **Prismatic Finger Closure & Encoder Check:** The fingers close around the workpiece (`cmd = -12 mm`), and `/joint_states` feedback is checked against the threshold $q_{\\mathrm{th}} = -7.5\\text{ mm}$ (a $22\\text{ mm}$ upright block halts the fingers near $-2.8\\text{ mm}$; closing past $-7.5\\text{ mm}$ indicates an empty or slipped grasp).
2. **Idealized `DetachableJoint` Transport Constraint:** To ensure deterministic transport during high-speed slews to the buckets, the controller engages Gazebo Harmonic's `DetachableJoint` plugin (`/jork_arm/grasp/part_XX/attach`) during lift and transport, and publishes `/detach` over the target bucket.

Accordingly, the benchmark results represent **workflow validation using Gazebo's `DetachableJoint` attachment constraint**, rather than pure frictional Fin Ray deformation under load.

<img width="3900" height="1560" alt="gripper_verification" src="https://github.com/user-attachments/assets/d6e3c4d7-a5aa-4b2c-9626-2ee6d67b8ef1" />


---

## 8. Vision-Guided Sorting Logic

### Hybrid Perception Architecture
The perception pipeline in `scan_pickup_window_camera()` combines **camera-based colour and pose-state classification** with **model-based conveyor position tracking**:

1. **RGB Channel-Dominance Colour Segmentation:** Rather than relying on a neural network, incoming $320 \\times 240$ RGB frames from `/camera/image_raw` are segmented within the conveyor strip using explicit channel-dominance masks:
   * `RED`: $(R > 155) \\land (R > G + 75) \\land (R > B + 75)$
   * `BLUE`: $(B > 135) \\land (B > R + 65) \\land (B > G + 40)$
   * `YELLOW`: $(R > 165) \\land (G > 165) \\land (B < 80)$
   * `PURPLE`: $(R > 115) \\land (B > 145) \\land (G < 80)$
   * `ORANGE`: $(R > 170) \\land (65 < G < 155) \\land (B < 65)$
   * `CYAN`: $(G > 155) \\land (B > 155) \\land (R < 80)$
   * `GREEN`: $(G > 145) \\land (G > R + 60) \\land (G > B + 50)$
2. **Central Image Moments & Pose-State Classification:** From the segmented pixel coordinates $(u_k, v_k)$, the controller computes the centroid $(\\bar{u}, \\bar{v})$, bounding-box aspect ratio, and second-order central moments $(\\mu_{20}, \\mu_{02}, \\mu_{11})$:
   $$\\theta_{\\mathrm{img}} = \\frac{1}{2} \\operatorname{atan2}(2\\mu_{11},\\; \\mu_{20} - \\mu_{02})$$
   Objects with aspect ratio $> 1.38$ or pixel area $> 150\\text{ px}$ are classified as `Tilted/Fallen` (lowering the grasp height to $Z = 0.080\\text{ m}$, widening jaw opening to $+18\\text{ mm}$, and passing $\\theta_{\\mathrm{img}}$ to wrist roll `joint_4`), whereas compact contours are classified as `Upright` ($Z = 0.098\\text{ m}$, jaw opening $+15\\text{ mm}$).
3. **Cross-Check Guard & Model-Tracked Pickup Coordinates:** To prevent stray debris in the pickup strip from causing a mis-sort, the controller cross-checks `cam_det['color']` against the carrier puck's expected metadata and uses the puck's parametric loop position $(X_{\\mathrm{puck}}, Y_{\\mathrm{puck}})$ for horizontal trajectory tracking during the pinch.

---

## 9. Simulation 

The benchmark scenario evaluates a single complete pass of the 15-workpiece loop (5 Red targets, 5 Blue targets, 5 distractors) at a cruise conveyor speed of $42\\text{ mm/s}$ ($6\\text{ mm/s}$ during grasp synchronization).

![15-Block Sorting Benchmark Summary]<img width="3900" height="1560" alt="benchmark_summary" src="https://github.com/user-attachments/assets/456f5632-7378-4df3-b487-ee3a8dce0f45" />





https://github.com/user-attachments/assets/711da63c-7861-4514-9344-739f3f627aa7



| Metric | Single-Run Simulation Result | Scope / Condition |
| :--- | :--- | :--- |
| **Red Target Blocks Sorted** | `5 / 5 (100%)` | Placed inside Red Bucket via arrival-order loop |
| **Blue Target Blocks Sorted** | `5 / 5 (100%)` | Placed inside Blue Bucket via arrival-order loop |
| **Distractor Blocks Rejected** | `5 / 5 (100%)` | Yellow, Purple, Orange, Cyan, Green skipped on loop |
| **Mean Manipulation Cycle Time** | `4.15 s / target workpiece` | Descent + Co-Moving Pinch + Lift + Bucket Slew + Return |
| **Gazebo Physics Configuration** | `500 Hz (max_step_size = 0.002 s)` | Gazebo Harmonic `gz-sim-physics-system` (default DART) |
| **Target Real-Time Factor (RTF)** | `1.0 (configured target)` | Actual runtime RTF depends on host GPU / WSL2 load |

---

## 10. Software Stack

| Layer | Technology / Interface |
| :--- | :--- |
| **Mechanical CAD** | SolidWorks (7 exported binary STL subassemblies) |
| **Robot Description** | URDF / Xacro (`jork_arm.urdf.xacro`, 4 revolute + 2 prismatic joints) |
| **Robotics Middleware** | ROS 2 Jazzy (`rclpy`, `robot_state_publisher`, `ros_gz_bridge`) |
| **Physics Simulation** | Gazebo Harmonic (`gz-sim-physics-system`, `2 ms` step, `500 Hz`) |
| **Control & Orchestration** | Python ROS 2 node (`vision_sorting_cell.py`), $60\\text{ Hz}$ PD position control |
| **Joint-State Feedback** | `/joint_states` (`sensor_msgs/msg/JointState`) |
| **Perception** | Overhead RGB camera (`/camera/image_raw`), NumPy channel dominance & central moments |
| **Manipulation** | Constrained-pose analytical IK, 5th-order quintic interpolation |

---

## 11. Repository Structure

```text
scout-nav2-gazebo/
├── README.md
├── media/
│   ├── workspace_envelope.png
│   ├── quintic_trajectories.png
│   ├── torque_analysis.png
│   ├── conveyor_tracking.png
│   ├── gripper_verification.png
│   └── benchmark_summary.png
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

>>>>>>> 0b0ac8a (Fix: Reconcile 4-DOF URDF docs, torque math, camera parameters, and package.xml metadata)
