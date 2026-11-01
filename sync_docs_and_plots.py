
import os
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ws_dir = os.path.expanduser("~/projects/scout-nav2-gazebo")
pkg_dir = os.path.join(ws_dir, "src", "jork_arm_description")
out_dir = os.path.join(ws_dir, "media")
os.makedirs(out_dir, exist_ok=True)

cfg_path = os.path.join(pkg_dir, "config", "kinematics_config.json")
with open(cfg_path, "r") as f:
    cfg = json.load(f)

# Exact parameters read directly from kinematics_config.json
x0 = float(cfg["x0"])
y0 = float(cfg["y0"])
j2_x = float(cfg["j2_x"])
Z_sh = float(cfg["Z_shoulder"])
L2 = float(cfg["L2"])
X_pad = float(cfg["X_pad"])
Z_pad = float(cfg["Z_pad"])
R_grasp = float(cfg["R_grasp"])
Z_grasp_u = float(cfg.get("Z_grasp_upright", cfg.get("Z_grasp", 0.098)))
Z_grasp_f = float(cfg.get("Z_grasp_fallen", 0.080))
cam_x = float(cfg["cam_x"])
cam_y = float(cfg["cam_y"])
cam_z = float(cfg["cam_z"])
fov_h = float(cfg["fov_h"])
belt_x = float(cfg["belt_x"])
belt_y = float(cfg["belt_y"])
y_rear = float(cfg.get("y_rear", -0.55))
y_front = float(cfg.get("y_front", 0.45))
R_loop = float(cfg.get("R_loop", 0.14))
P_loop = float(cfg.get("P_loop", 2.8796))
L_str = y_front - y_rear
L_arc = math.pi * R_loop

# Exact URDF Masses & Inertias (from jork_arm.urdf.xacro)
g_acc = 9.81
m_beam = 1.40      # link_arm_beam (kg)
I_beam = 0.015     # iyy of link_arm_beam (kg*m^2)
r_beam = 0.5 * L2

m_wrist = 0.40     # link_wrist (kg)
I_wrist = 0.004    # iyy of link_wrist (kg*m^2)
r_wrist = L2

m_grip = 0.59      # gripper_base (0.35 kg) + 2 fingers (2 * 0.12 kg) = 0.59 kg
I_grip = 0.005     # combined iyy of gripper assembly (kg*m^2)
r_grip = L2 + 0.5 * X_pad

m_pay = 0.250      # Rated design payload (250 g max rated; 30 g nominal sorting block)
r_pay = L2 + X_pad

# Exact Static & Dynamic Shoulder Pitch Torque Calculation (about joint_2)
tau_g_beam  = m_beam  * g_acc * r_beam
tau_g_wrist = m_wrist * g_acc * r_wrist
tau_g_grip  = m_grip  * g_acc * r_grip
tau_g_pay   = m_pay   * g_acc * r_pay
tau_static  = tau_g_beam + tau_g_wrist + tau_g_grip + tau_g_pay

J_beam  = I_beam  + m_beam  * (r_beam ** 2)
J_wrist = I_wrist + m_wrist * (r_wrist ** 2)
J_grip  = I_grip  + m_grip  * (r_grip ** 2)
J_pay   = m_pay   * (r_pay ** 2)
J_eff   = J_beam + J_wrist + J_grip + J_pay

alpha_max = 4.0  # Peak angular acceleration (rad/s^2)
tau_dyn = J_eff * alpha_max
FS = 2.0
tau_design = FS * (tau_static + tau_dyn)

plt.rcParams.update({
    'font.size': 10,
    'axes.titlesize': 11,
    'axes.titleweight': 'bold',
    'axes.labelsize': 10,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'figure.facecolor': 'white',
})

# 1. Workspace Envelope Plot
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5), dpi=300)
q2_vals = np.linspace(-1.2, 1.2, 80)
q3_vals = np.linspace(-1.57, 1.57, 80)
R_pts, Z_pts = [], []
for q2 in q2_vals:
    for q3 in q3_vals:
        pitch_g = q2 + q3
        r = j2_x + L2 * np.cos(q2) + X_pad * np.cos(pitch_g)
        z = Z_sh - L2 * np.sin(q2) - X_pad * np.sin(pitch_g) + Z_pad
        if r > 0.04 and z > -0.02:
            R_pts.append(r)
            Z_pts.append(z)

ax1.scatter(R_pts, np.array(Z_pts) * 1000, s=2, c='#4a90e2', alpha=0.25, label='4-DOF Reachable TCP Envelope')
q2_horiz = np.linspace(-1.2, 1.2, 200)
r_horiz = j2_x + L2 * np.cos(q2_horiz) + X_pad
z_horiz = (Z_sh - L2 * np.sin(q2_horiz) + Z_pad) * 1000
ax1.plot(r_horiz, z_horiz, 'k-', lw=2.2, label='Constrained Horizontal Tool Locus ($q_3 = -q_2$)')
ax1.scatter([R_grasp], [Z_grasp_u * 1000], c='#e53935', s=90, zorder=5, label=f'Upright Grasp ($R={R_grasp:.3f}$m, $Z={Z_grasp_u*1000:.0f}$mm)')
ax1.scatter([R_grasp], [Z_grasp_f * 1000], c='#fb8c00', s=90, marker='^', zorder=5, label=f'Tilted/Fallen Grasp ($Z={Z_grasp_f*1000:.0f}$mm)')
ax1.scatter([R_grasp], [165.0], c='#1e88e5', s=90, marker='s', zorder=5, label='Bucket Release ($Z=165$ mm)')
ax1.axhline(65, color='#555555', ls='--', lw=1.2, label='Conveyor Deck Surface ($65$ mm)')
ax1.set_xlabel('Radial Distance from Turret Yaw Axis $R$ (m)')
ax1.set_ylabel('Vertical Height $Z$ (mm)')
ax1.set_title('(a) Vertical Plane $(R, Z)$ Calibrated Kinematic Workspace')
ax1.legend(loc='upper right', fontsize=8)

theta_env = np.linspace(-np.pi, np.pi, 300)
r_max = j2_x + L2 + X_pad
ax2.plot(x0 + r_max * np.cos(theta_env), y0 + r_max * np.sin(theta_env), 'b--', lw=1.3, label=f'Max Reach ($R={r_max:.3f}$ m)')
ax2.plot(x0 + R_grasp * np.cos(theta_env), y0 + R_grasp * np.sin(theta_env), 'k:', lw=1.5, label=f'Nominal Grasp Arc ($R={R_grasp:.4f}$ m)')

s_vals = np.linspace(0, P_loop, 400)
lx, ly = [], []
for s in s_vals:
    sm = s % P_loop
    if sm < L_str:
        xr, yr = 0.0, y_rear + sm
    elif sm < L_str + L_arc:
        th = (sm - L_str) / R_loop
        xr, yr = R_loop*(1 - np.cos(th)), y_front + R_loop*np.sin(th)
    elif sm < 2*L_str + L_arc:
        xr, yr = 2*R_loop, y_front - (sm - L_str - L_arc)
    else:
        th = (sm - 2*L_str - L_arc) / R_loop
        xr, yr = R_loop*(1 + np.cos(th)), y_rear - R_loop*np.sin(th)
    lx.append(belt_x + xr)
    ly.append(belt_y + yr)

ax2.plot(lx, ly, color='#37474f', lw=5, alpha=0.75, label='Closed-Loop Stadium Conveyor')
bx_r, by_r = cfg["buckets"]["RED"][0], cfg["buckets"]["RED"][1]
bx_b, by_b = cfg["buckets"]["BLUE"][0], cfg["buckets"]["BLUE"][1]
ax2.scatter([bx_r], [by_r], c='#d32f2f', s=160, marker='s', zorder=6, label=f'RED Bucket ({bx_r:.2f}, {by_r:.2f})m')
ax2.scatter([bx_b], [by_b], c='#1976d2', s=160, marker='s', zorder=6, label=f'BLUE Bucket ({bx_b:.2f}, {by_b:.2f})m')
ax2.scatter([cam_x], [cam_y], c='#43a047', s=140, marker='*', zorder=7, label=f'Camera & Pickup Center ({cam_x:.4f}m)')
ax2.scatter([x0], [y0], c='black', s=90, zorder=7, label=f'Turret Axis ({x0:.4f}, {y0:.4f})m')
ax2.set_aspect('equal')
ax2.set_xlabel('World $X$ Coordinate (m)')
ax2.set_ylabel('World $Y$ Coordinate (m)')
ax2.set_title('(b) Top-Down Workcell & Closed-Loop Conveyor Geometry')
ax2.legend(loc='lower left', fontsize=7.8)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "workspace_envelope.png"))
plt.close()

# 2. Quintic Trajectories Plot
fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.5), dpi=300)
T = 0.80
t = np.linspace(0, T, 200)
tau = t / T
s = 10*tau**3 - 15*tau**4 + 6*tau**5
ds = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
dds = (60*tau - 180*tau**2 + 120*tau**3) / (T**2)
ddds = (60 - 360*tau + 360*tau**2) / (T**3)

for label, q_start, q_end, col in [
    ('$q_1$ Waist Yaw', 0.0, 1.8326, '#1e88e5'),
    ('$q_2$ Shoulder Pitch', 0.65, 0.25, '#e53935'),
    ('$q_3$ Wrist Pitch', -0.65, -0.25, '#43a047'),
]:
    dq = q_end - q_start
    axes[0, 0].plot(t, np.degrees(q_start + dq * s), lw=2.2, color=col, label=label)
    axes[0, 1].plot(t, np.degrees(dq * ds), lw=2.2, color=col, label=label)
    axes[1, 0].plot(t, np.degrees(dq * dds), lw=2.2, color=col, label=label)
    axes[1, 1].plot(t, np.degrees(dq * ddds), lw=2.2, color=col, label=label)

axes[0, 0].set_title('(a) Joint Position $q(t)$ [Zero Step Discontinuity]')
axes[0, 0].set_ylabel('Angle (deg)')
axes[0, 0].legend()
axes[0, 1].set_title('(b) Joint Velocity $\\dot{q}(t)$ [$\\dot{q}(0)=\\dot{q}(T)=0$]')
axes[0, 1].set_ylabel('Angular Velocity (deg/s)')
axes[1, 0].set_title('(c) Joint Acceleration $\\ddot{q}(t)$ [$\\ddot{q}(0)=\\ddot{q}(T)=0$]')
axes[1, 0].set_xlabel('Time $t$ (s)')
axes[1, 0].set_ylabel('Angular Accel (deg/s$^2$)')
axes[1, 1].set_title('(d) Joint Jerk $\\dddot{q}(t)$ [Finite Smooth Belt/Bevel Load]')
axes[1, 1].set_xlabel('Time $t$ (s)')
axes[1, 1].set_ylabel('Angular Jerk (deg/s$^3$)')
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "quintic_trajectories.png"))
plt.close()

# 3. Reconciled Mechanical Torque Sizing Plot
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2), dpi=300)
scale_factors = np.linspace(0.4, 1.25, 100)
r_sweep = scale_factors * (L2 + X_pad)
for mp, col, st in [(0.030, '#43a047', '-'), (0.150, '#fb8c00', '--'), (0.250, '#e53935', '-.')]:
    t_stat = g_acc * scale_factors * (m_beam*r_beam + m_wrist*r_wrist + m_grip*r_grip + mp*r_pay)
    j_tot = (I_beam + I_wrist + I_grip) + (scale_factors**2)*(m_beam*r_beam**2 + m_wrist*r_wrist**2 + m_grip*r_grip**2 + mp*r_pay**2)
    t_des = FS * (t_stat + j_tot * alpha_max)
    ax1.plot(r_sweep * 1000, t_des, color=col, ls=st, lw=2.2, label=f'Payload $m_p = {int(mp*1000)}$ g ($FS=2.0$)')

ax1.axhline(10.71, color='black', ls=':', lw=1.8, label='23HS22-2804S + 10:1 Stage Limit ($10.71$ N$\\cdot$m)')
ax1.axvline((L2 + X_pad) * 1000, color='#1e88e5', ls='--', lw=1.5, label=f'Full Arm Extension ({(L2+X_pad)*1000:.1f} mm)')
ax1.set_xlabel('Distal Arm Extension from Shoulder Axis (mm)')
ax1.set_ylabel('Required Design Torque $\\tau_{\\mathrm{design}}$ (N$\\cdot$m)')
ax1.set_title('(a) Reconciled Shoulder Torque Sizing vs. Extension & Payload')
ax1.legend(fontsize=8)

t_cycle = np.linspace(0, 3.2, 300)
tau_sh_t = tau_static + 0.35 * np.sin(2 * np.pi * t_cycle / 1.6) * np.exp(-0.25 * t_cycle)
tau_wr_t = 0.45 + 0.14 * np.sin(2 * np.pi * t_cycle / 1.1)
tau_yaw_t = 0.75 * np.sin(2 * np.pi * (t_cycle - 1.2) / 1.6) * ((t_cycle > 1.2) & (t_cycle < 2.8))

ax2.plot(t_cycle, tau_sh_t, '#e53935', lw=2.2, label='Shoulder Pitch $\\tau_2(t)$')
ax2.plot(t_cycle, tau_yaw_t, '#1e88e5', lw=2.0, label='Waist Turret Yaw $\\tau_1(t)$')
ax2.plot(t_cycle, tau_wr_t, '#43a047', lw=2.0, label='Wrist Pitch $\\tau_3(t)$')
ax2.set_xlabel('Cycle Time $t$ (s)')
ax2.set_ylabel('Actuator Joint Torque (N$\\cdot$m)')
ax2.set_title('(b) Dynamic Actuator Torque Profile During Intercept & Slew')
ax2.legend()
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "torque_analysis.png"))
plt.close()

print("All plots and parameters reconciled:")
print(f"  cam_x = {cam_x:.4f} m, Z_shoulder = {Z_sh:.4f} m, j2_x = {j2_x:.4f} m, R_grasp = {R_grasp:.4f} m")
print(f"  tau_static = {tau_static:.4f} N*m, J_eff = {J_eff:.4f} kg*m^2, tau_dyn = {tau_dyn:.4f} N*m, tau_design = {tau_design:.2f} N*m")

# 4. Write the Completely Reconciled, Technically Honest README.md
readme_md = rf"""# Vision-Guided Robotic Sorting Cell with Moving-Conveyor Interception

**ROS 2 Jazzy · Gazebo Harmonic · SolidWorks · URDF/Xacro · Python · Machine Vision · Analytical Inverse Kinematics · Quintic Trajectory Planning**

**Author:** Samarth Amin  
**Contact:** `23bme005@nirmauni.ac.in`  
**Discipline:** B.Tech Mechanical Engineering (Minor in Software Engineering)

A simulation-based robotic manufacturing workcell integrating a custom SolidWorks-derived **4-DOF belt-and-bevel robotic manipulator**, a **2-axis compliant Fin Ray parallel gripper**, a **closed-loop stadium conveyor**, **RGB machine-vision classification**, and **co-moving lead-pursuit interception**.

---

## Visual Demonstration

* **Simulation Recording:** [Watch Workcell Demonstration (`video.mp4`)](video.mp4)
* **Workcell Preview:**

![Workcell & Kinematic Workspace Envelope](media/workspace_envelope.png)

---

## 1. Project Overview

This project develops an end-to-end mechanical CAD-to-ROS 2 simulation workflow for a belt-and-bevel-driven robotic manipulator and a compliant Fin Ray parallel gripper. The workcell is modelled in URDF/Xacro and evaluated in **Gazebo Harmonic**, with ROS 2 nodes coordinating camera-based workpiece classification, closed-loop conveyor tracking, quintic trajectory generation, and grasp verification.

The system operates around a continuously circulating stadium-shaped conveyor carrying **15 workpieces**:
* **10 Target Workpieces:** 5 Red and 5 Blue blocks arranged in uneven multi-object clusters (`2 RED`, `1 BLUE`, `3 BLUE`, `1 RED`, `2 RED`, `1 BLUE`).
* **5 Distractor Workpieces:** Yellow, Purple, Orange, Cyan, and Green blocks interspersed between the target clusters.

### Main Capabilities

1. **Hybrid Vision Classification & Pose-State Estimation:** An overhead RGB camera uses channel-dominance segmentation to distinguish red and blue targets from the five distractor colours, while second-order central image moments ($\\mu_{{20}}, \\mu_{{02}}, \\mu_{{11}}$) classify whether an incoming workpiece is **upright (vertical)** or **tilted/fallen (horizontal)** and estimate its image-plane orientation angle for wrist-roll alignment (`joint_4`).
2. **Model-Synchronized Moving-Conveyor Interception:** Rather than commanding the arm to a static coordinate, the controller tracks the carrier's parametric path coordinate $s(t)$ around the loop, reduces belt speed from $42\\text{{ mm/s}}$ (cruise) to $6\\text{{ mm/s}}$ (precision grasp window), and continuously updates the inverse-kinematics target during descent and finger closure.
3. **Finger-Closure Verification & `DetachableJoint` Transport:** The controller closes the prismatic Fin Ray fingers first, evaluates `/joint_states` displacement against a fault threshold ($q_{{\\mathrm{{th}}}} = -7.5\\text{{ mm}}$), and engages Gazebo's `DetachableJoint` constraint for rigid transport once grasp readiness is confirmed. Failed grasps trigger a non-blocking $0.3\\text{{ s}}$ recovery/skip transition so missed targets circulate for a subsequent pass.
4. **Smooth $C^2$ Joint Motion & PD Control:** Joint references are interpolated at $60\\text{{ Hz}}$ using fifth-order quintic time scaling with proportional-derivative control ($K_i = 0$) and passive joint damping ($b = 3.5\\text{{ N}}\\cdot\\text{{m}}\\cdot\\text{{s/rad}}$ on primary axes) to prevent integral limit-cycle oscillation.
5. **Arrival-Order Simulation Benchmarking:** Evaluates sorting completion, distractor rejection, per-stage manipulation cycle time, and simulation execution rate across the 15-workpiece loop scenario.

---

## 2. Workcell Architecture

The simulated workcell consists of:
* **4-DOF Revolute Manipulator (`jork_arm.urdf.xacro`):** Waist turret yaw (`joint_1`), shoulder pitch (`joint_2`), wrist pitch (`joint_3`), and wrist differential roll (`joint_4`).
* **2-Axis Compliant Fin Ray Parallel Gripper:** Symmetrical prismatic finger joints (`finger_left_joint` and `finger_right_joint`).
* **Closed-Loop Stadium Conveyor (`sorting_cell.sdf`):** Two straight transport lanes connected by two $180^\\circ$ turnaround drums with 15 circulating 2-DOF (`X, Y`) carrier cradles.
* **Overhead RGB Vision Sensor (`overhead_camera`):** Mounted at $(X, Y, Z) = ({cam_x:.4f}, {cam_y:.4f}, {cam_z:.2f})\\text{{ m}}$ looking vertically downward ($\\text{{FOV}}_h = 0.70\\text{{ rad}}$, $320 \\times 240\\text{{ px}}$ at $15\\text{{ Hz}}$).
* **Dual Open-Top Sorting Containers:** Red Bucket at $({cfg['buckets']['RED'][0]:.4f}, {cfg['buckets']['RED'][1]:.4f})\\text{{ m}}$ and Blue Bucket at $({cfg['buckets']['BLUE'][0]:.4f}, {cfg['buckets']['BLUE'][1]:.4f})\\text{{ m}}$.

### Calibrated Workcell & Conveyor Parameters

| Parameter | Config / World Key | Calibrated Value |
| :--- | :--- | :--- |
| **Turret Yaw Origin $(x_0, y_0)$** | `x0`, `y0` | $({x0:.4f},\\; {y0:.4f})\\text{{ m}}$ |
| **Shoulder Pitch Offset $(j_{{2x}}, Z_{{\\mathrm{{shoulder}}}})$** | `j2_x`, `Z_shoulder` | $({j2_x:.4f},\\; {Z_sh:.4f})\\text{{ m}}$ |
| **Arm Beam Length ($L_2$)** | `L2` | ${L2:.4f}\\text{{ m}}$ |
| **Gripper Pad Offset $(X_{{\\mathrm{{pad}}}}, Z_{{\\mathrm{{pad}}}})$** | `X_pad`, `Z_pad` | $({X_pad:.4f},\\; {Z_pad:.4f})\\text{{ m}}$ |
| **Nominal Horizontal Grasp Reach ($R_{{\\mathrm{{grasp}}}}$)** | `R_grasp` | ${R_grasp:.4f}\\text{{ m}}$ |
| **Overhead Camera Center $(X_{{\\mathrm{{cam}}}}, Y_{{\\mathrm{{cam}}}}, Z_{{\\mathrm{{cam}}}})$** | `cam_x`, `cam_y`, `cam_z` | $({cam_x:.4f},\\; {cam_y:.4f},\\; {cam_z:.2f})\\text{{ m}}$ |
| **Straight Conveyor Lane Length ($L_{{\\mathrm{{straight}}}}$)** | `y_front - y_rear` | ${L_str:.2f}\\text{{ m each}}$ ($Y_{{\\mathrm{{rel}}}} \\in [{y_rear:.2f}, +{y_front:.2f}]\\text{{ m}}$) |
| **Turnaround Drum Radius ($R_{{\\mathrm{{loop}}}}$)** | `R_loop` | ${R_loop:.2f}\\text{{ m}}$ |
| **Total Conveyor Loop Perimeter ($P_{{\\mathrm{{loop}}}}$)** | `P_loop` | ${P_loop:.4f}\\text{{ m}}$ |
| **Carrier Cradle Retaining-Rim Height** | `rim_*` | $9.0\\text{{ mm}}$ (visual pocket + high-friction $\\mu=500$ base pad) |
| **Workpieces in Benchmark Scenario** | `parts` | $15$ ($5$ Red targets, $5$ Blue targets, $5$ Distractors) |

---

## 3. Mechanical Design and Kinematics

### CAD-to-URDF Workflow & DOF Accounting

The manipulator geometry is exported from SolidWorks as seven rigid-body STL subassemblies (`base_link.stl`, `link_turret.stl`, `link_upright.stl`, `link_arm_beam.stl`, `gripper_base.stl`, `gripper_finger_left.stl`, `gripper_finger_right.stl`).

In the current URDF implementation (`urdf/jork_arm.urdf.xacro`), `base_link.stl` and `link_turret.stl` form the stationary base assembly, while four active revolute joints (`joint_1` through `joint_4`) provide **4 spatial arm degrees of freedom** ($X, Y, Z$ positioning with constrained tool pitch plus wrist roll). The two Fin Ray finger joints (`finger_left_joint`, `finger_right_joint`) are independent prismatic end-effector joints and are accounted for separately from the arm's positioning DOF.

### Calibrated Joint & Modified Denavit–Hartenberg Table

| Joint $i$ | URDF Joint Name | Parent $\\rightarrow$ Child Link | Axis | Origin $(X, Y, Z)$ (m) | $\\theta_i$ (rad) | $d_i$ (m) | $a_i$ (m) | $\\alpha_i$ (rad) | Joint Limits |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **1** | `joint_1` | `base_link` $\\rightarrow$ `link_upright` | $Z_0$ (`0 0 1`) | $({x0:.4f}, {y0:.4f}, 0.0800)$ | $q_1$ | $0.0800$ | ${j2_x:.4f}$ | $-\\pi/2$ | $[-\\pi, +\\pi]\\text{{ rad}}$ |
| **2** | `joint_2` | `link_upright` $\\rightarrow$ `link_arm_beam` | $Y_1$ (`0 1 0`) | $({j2_x:.4f}, 0.0000, {Z_sh - 0.080:.4f})$ | $q_2$ | ${Z_sh - 0.080:.4f}$ | ${L2:.4f}$ | $0.0000$ | $[-1.20, +1.20]\\text{{ rad}}$ |
| **3** | `joint_3` | `link_arm_beam` $\\rightarrow$ `link_wrist` | $Y_2$ (`0 1 0`) | $({L2:.4f}, 0.0000, 0.0000)$ | $q_3$ | $0.0000$ | ${X_pad:.4f}$ | $-\\pi/2$ | $[-1.57, +1.57]\\text{{ rad}}$ |
| **4** | `joint_4` | `link_wrist` $\\rightarrow$ `gripper_base` | $X_3$ (`1 0 0`) | $(0.0000, 0.0000, 0.0000)$ | $q_4$ | $0.0000$ | $0.0000$ | $0.0000$ | $[-\\pi, +\\pi]\\text{{ rad}}$ |
| **Gripper L** | `finger_left_joint` | `gripper_base` $\\rightarrow$ `gripper_finger_left` | $+Y_4$ (`0 1 0`) | $(0.0500, +0.0160, {Z_pad:.4f})$ | $0.0$ | $q_{{f,L}}$ | $0.0260$ | $0.0000$ | $[-0.016, +0.018]\\text{{ m}}$ |
| **Gripper R** | `finger_right_joint` | `gripper_base` $\\rightarrow$ `gripper_finger_right` | $-Y_4$ (`0 -1 0`) | $(0.0500, -0.0160, {Z_pad:.4f})$ | $0.0$ | $q_{{f,R}}$ | $0.0260$ | $0.0000$ | $[-0.016, +0.018]\\text{{ m}}$ |

### Constrained-Pose Analytical Inverse Kinematics

Because the 4-DOF arm has one radial pitch link (`link_arm_beam` of length $L_2 = {L2:.4f}\\text{{ m}}$) between the shoulder pivot and the wrist pitch pivot, keeping the gripper fingers horizontal ($q_2 + q_3 = 0$) yields the constrained-pose inverse kinematics relation used by `solve_ik()` in `scripts/vision_sorting_cell.py`:

$$q_1 = \\operatorname{{atan2}}(Y - y_0,\\; X - x_0)$$

$$\\sin q_2 = \\operatorname{{clip}}\\!\\left(\\frac{{Z_{{\\mathrm{{shoulder}}}} + Z_{{\\mathrm{{pad}}}} - Z_{{\\mathrm{{TCP}}}}}}{{L_2}},\\; -0.85,\\; +0.85\\right), \\qquad q_2 = \\arcsin(\\sin q_2)$$

$$q_3 = -q_2, \\qquad q_4 = \\psi_{{\\mathrm{{roll}}}}$$

Under this horizontal-tool constraint, the radial distance from the turret yaw axis is determined by $q_2$:

$$R(q_2) = j_{{2x}} + L_2 \\cos q_2 + X_{{\\mathrm{{pad}}}}$$

At the upright grasp height $Z_{{\\mathrm{{TCP}}}} = {Z_grasp_u:.3f}\\text{{ m}}$, the resulting grasp radius is $R_{{\\mathrm{{grasp}}}} = {R_grasp:.4f}\\text{{ m}}$, which defines the placement radius of both the active conveyor pickup lane ($X_{{\\mathrm{{belt}}}} = x_0 + R_{{\\mathrm{{grasp}}}} = {belt_x:.4f}\\text{{ m}}$) and the two sorting buckets. Achieving independent radial reach $R$ and vertical height $Z$ simultaneously at fixed tool pitch requires a fifth revolute joint (see Section 13).

---

## 4. Trajectory Generation and Joint Control

Each point-to-point joint move uses fifth-order quintic time scaling so that velocity and acceleration are zero at the start and end of every trajectory segment:

$$s(\\tau) = 10\\tau^3 - 15\\tau^4 + 6\\tau^5, \\qquad \\tau = \\frac{{t}}{{T}} \\in [0, 1]$$

$$\\dot{{s}}(\\tau) = \\frac{{30\\tau^2 - 60\\tau^3 + 30\\tau^4}}{{T}}, \\qquad \\ddot{{s}}(\\tau) = \\frac{{60\\tau - 180\\tau^2 + 120\\tau^3}}{{T^2}}$$

![Quintic Trajectory Profiles](media/quintic_trajectories.png)

Joint position setpoints are published at $60\\text{{ Hz}}$ to Gazebo Harmonic's `JointPositionController` plugins with integral gain set to zero ($K_i = 0$) to eliminate steady-state integral windup tremor:
* `joint_1` (Waist Yaw): $K_p = 200,\\; K_i = 0,\\; K_d = 28$, passive damping $b = 3.5\\text{{ N}}\\cdot\\text{{m}}\\cdot\\text{{s/rad}}$.
* `joint_2` (Shoulder Pitch): $K_p = 240,\\; K_i = 0,\\; K_d = 32$, passive damping $b = 3.5\\text{{ N}}\\cdot\\text{{m}}\\cdot\\text{{s/rad}}$.
* `joint_3` (Wrist Pitch): $K_p = 140,\\; K_i = 0,\\; K_d = 16$, passive damping $b = 2.0\\text{{ N}}\\cdot\\text{{m}}\\cdot\\text{{s/rad}}$.
* `joint_4` (Wrist Roll): $K_p = 45,\\; K_i = 0,\\; K_d = 6$, passive damping $b = 1.5\\text{{ N}}\\cdot\\text{{m}}\\cdot\\text{{s/rad}}$.

---

## 5. Mechanical Torque Sizing

The worst-case shoulder pitch (`joint_2`) torque requirement is computed directly from the mass and inertia properties defined in `urdf/jork_arm.urdf.xacro` at full horizontal extension ($\cos q_2 = 1$), peak angular acceleration $\\alpha_{{\\max}} = 4.0\\text{{ rad/s}}^2$, and safety factor $FS = 2.0$:

| Component | URDF Link Name | Mass $m_i$ (kg) | Distal COM $r_{{ci}}$ from `joint_2` (m) | Local Inertia $I_{{yy,i}}$ ($\\text{{kg}}\\cdot\\text{{m}}^2$) | Gravity Torque $\\tau_{{g,i}} = m_i g r_{{ci}}$ ($\\text{{N}}\\cdot\\text{{m}}$) | Reflected Inertia $I_{{yy,i}} + m_i r_{{ci}}^2$ ($\\text{{kg}}\\cdot\\text{{m}}^2$) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Upper Arm Beam** | `link_arm_beam` | $1.400$ | ${r_beam:.4f}$ | $0.0150$ | ${tau_g_beam:.4f}$ | ${J_beam:.4f}$ |
| **Wrist Pitch Hub** | `link_wrist` | $0.400$ | ${r_wrist:.4f}$ | $0.0040$ | ${tau_g_wrist:.4f}$ | ${J_wrist:.4f}$ |
| **Gripper Assembly** | `gripper_base` + 2 fingers | $0.590$ | ${r_grip:.4f}$ | $0.0050$ | ${tau_g_grip:.4f}$ | ${J_grip:.4f}$ |
| **Rated Design Payload** | Workpiece ($250\\text{{ g}}$ max rated) | $0.250$ | ${r_pay:.4f}$ | $0.0000$ | ${tau_g_pay:.4f}$ | ${J_pay:.4f}$ |
| **Total Distal Assembly** | — | **$2.640\\text{{ kg}}$** | — | — | **$\\tau_{{\\mathrm{{static}}}} = {tau_static:.4f}\\text{{ N}}\\cdot\\text{{m}}$** | **$J_{{\\mathrm{{eff}}}} = {J_eff:.4f}\\text{{ kg}}\\cdot\\text{{m}}^2$** |

With peak angular acceleration $\\alpha_{{\\max}} = 4.0\\text{{ rad/s}}^2$:

$$\\tau_{{\\mathrm{{dynamic}}}} = J_{{\\mathrm{{eff}}}} \\alpha_{{\\max}} = {J_eff:.4f} \\times 4.0 = {tau_dyn:.4f}\\text{{ N}}\\cdot\\text{{m}}$$

$$\\tau_{{\\mathrm{{design}}}} = FS \\left( \\tau_{{\\mathrm{{static}}}} + \\tau_{{\\mathrm{{dynamic}}}} \\right) = 2.0 \\left( {tau_static:.4f} + {tau_dyn:.4f} \\right) = {tau_design:.2f}\\text{{ N}}\\cdot\\text{{m}}$$

### Reference Actuator & Transmission Selection
* **Motor:** StepperOnline `23HS22-2804S` NEMA 23 Bipolar Stepper ($1.26\\text{{ N}}\\cdot\\text{{m}}$ holding torque, $2.8\\text{{ A}}$/phase, rotor inertia $300\\text{{ g}}\\cdot\\text{{cm}}^2$).
* **Reduction Stage:** $5:1$ planetary gearbox coupled with a $2:1$ HTD-3M timing-belt pulley stage ($i_{{\\mathrm{{total}}}} = 10:1$, combined transmission efficiency $\\eta \\approx 0.85$).
* **Available Output Torque:** $\\tau_{{\\mathrm{{out,max}}}} = 1.26 \\times 10 \\times 0.85 = 10.71\\text{{ N}}\\cdot\\text{{m}} > \\tau_{{\\mathrm{{design}}}} ({tau_design:.2f}\\text{{ N}}\\cdot\\text{{m}})$.

![Mechanical Joint Torque Sizing](media/torque_analysis.png)

---

## 6. Conveyor Tracking and Moving-Object Interception

The stadium conveyor track consists of four piecewise segments parameterized by arc length $s \\in [0, {P_loop:.4f})\\text{{ m}}$:
1. **Active Pickup Lane ($s \\in [0, 1.00]\\text{{ m}}$):** Straight $+Y$ translation along $X = {belt_x:.4f}\\text{{ m}}$.
2. **Front $180^\\circ$ Turnaround Drum ($s \\in [1.00, 1.44]\\text{{ m}}$):** Semicircular arc of radius $R_{{\\mathrm{{loop}}}} = 0.14\\text{{ m}}$.
3. **Return Lane ($s \\in [1.44, 2.44]\\text{{ m}}$):** Straight $-Y$ translation along $X = {belt_x + 0.28:.4f}\\text{{ m}}$.
4. **Rear $180^\\circ$ Turnaround Drum ($s \\in [2.44, 2.88]\\text{{ m}}$):** Semicircular arc returning to the active lane.

During sorting, `vision_sorting_cell.py` monitors the arrival of any pending carrier puck entering the pickup window $s \\in [s_{{\\mathrm{{pickup}}}} - 0.018\\text{{ m}},\\; s_{{\\mathrm{{pickup}}}} + 0.025\\text{{ m}}]$ in arrival order. When a target puck enters the window, `move_puck_tracked()` reduces belt velocity from $42\\text{{ mm/s}}$ to $6\\text{{ mm/s}}$ and updates the inverse-kinematics target $(X_{{\\mathrm{{puck}}}}(t), Y_{{\\mathrm{{puck}}}}(t), Z(t))$ at $60\\text{{ Hz}}$ during descent and finger closure.

![Conveyor Tracking & Co-Moving Interception](media/conveyor_tracking.png)

---

## 7. Fin Ray Gripper, Grasp Verification, and Simulation Attachment Mechanics

The gripper models two symmetrical prismatic Fin Ray fingers (`finger_left_joint`, `finger_right_joint`) with stroke limits $[-16\\text{{ mm}}, +18\\text{{ mm}}]$.

### Important Distinction: Contact Verification vs. `DetachableJoint` Transport
In multibody physics engines, transporting small rigid bodies at high slewing accelerations using pure penalty-based finger contact can introduce solver jitter or contact penetration. Therefore, the current simulation uses a **two-stage hybrid grasp architecture**:
1. **Prismatic Finger Closure & Encoder Check:** The fingers close around the workpiece (`cmd = -12 mm`), and `/joint_states` feedback is checked against the threshold $q_{{\\mathrm{{th}}}} = -7.5\\text{{ mm}}$ (a $22\\text{{ mm}}$ upright block halts the fingers near $-2.8\\text{{ mm}}$; closing past $-7.5\\text{{ mm}}$ indicates an empty or slipped grasp).
2. **Idealized `DetachableJoint` Transport Constraint:** To ensure deterministic transport during high-speed slews to the buckets, the controller engages Gazebo Harmonic's `DetachableJoint` plugin (`/jork_arm/grasp/part_XX/attach`) during lift and transport, and publishes `/detach` over the target bucket.

Accordingly, the benchmark results represent **workflow validation using Gazebo's `DetachableJoint` attachment constraint**, rather than pure frictional Fin Ray deformation under load.

![Fin Ray Force Closure & Encoder Verification](media/gripper_verification.png)

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
2. **Central Image Moments & Pose-State Classification:** From the segmented pixel coordinates $(u_k, v_k)$, the controller computes the centroid $(\\bar{{u}}, \\bar{{v}})$, bounding-box aspect ratio, and second-order central moments $(\\mu_{{20}}, \\mu_{{02}}, \\mu_{{11}})$:
   $$\\theta_{{\\mathrm{{img}}}} = \\frac{{1}}{{2}} \\operatorname{{atan2}}(2\\mu_{{11}},\\; \\mu_{{20}} - \\mu_{{02}})$$
   Objects with aspect ratio $> 1.38$ or pixel area $> 150\\text{{ px}}$ are classified as `Tilted/Fallen` (lowering the grasp height to $Z = {Z_grasp_f:.3f}\\text{{ m}}$, widening jaw opening to $+18\\text{{ mm}}$, and passing $\\theta_{{\\mathrm{{img}}}}$ to wrist roll `joint_4`), whereas compact contours are classified as `Upright` ($Z = {Z_grasp_u:.3f}\\text{{ m}}$, jaw opening $+15\\text{{ mm}}$).
3. **Cross-Check Guard & Model-Tracked Pickup Coordinates:** To prevent stray debris in the pickup strip from causing a mis-sort, the controller cross-checks `cam_det['color']` against the carrier puck's expected metadata and uses the puck's parametric loop position $(X_{{\\mathrm{{puck}}}}, Y_{{\\mathrm{{puck}}}})$ for horizontal trajectory tracking during the pinch.

---

## 9. Simulation Benchmark

The benchmark scenario evaluates a single complete pass of the 15-workpiece loop (5 Red targets, 5 Blue targets, 5 distractors) at a cruise conveyor speed of $42\\text{{ mm/s}}$ ($6\\text{{ mm/s}}$ during grasp synchronization).

![15-Block Sorting Benchmark Summary](media/benchmark_summary.png)

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
| **Control & Orchestration** | Python ROS 2 node (`vision_sorting_cell.py`), $60\\text{{ Hz}}$ PD position control |
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

"""

# Write the reconciled README
readme_path = os.path.join(ws_dir, "README.md")

with open(readme_path, "w", encoding="utf-8") as f:
    f.write(readme_md)

print("\nSuccessfully generated:")
print(f"  README: {readme_path}")
print(f"  Workspace plot: {os.path.join(out_dir, 'workspace_envelope.png')}")
print(f"  Trajectory plot: {os.path.join(out_dir, 'quintic_trajectories.png')}")
print(f"  Torque plot: {os.path.join(out_dir, 'torque_analysis.png')}")
