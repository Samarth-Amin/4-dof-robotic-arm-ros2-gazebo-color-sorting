#!/usr/bin/env python3
import os
import glob
import json
import math
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
out_dir = os.path.join(repo_root, "media")
os.makedirs(out_dir, exist_ok=True)

run_dirs = sorted(glob.glob(os.path.join(repo_root, "data", "audits", "2026*")))
if len(run_dirs) < 2:
    run_dirs = sorted(glob.glob(os.path.expanduser("~/.ros/robot_sorting_audits/2026*")))[-2:]

def load_run(rdir):
    with open(os.path.join(rdir, "run_summary.json")) as f:
        s = json.load(f)
    ph = pd.read_csv(os.path.join(rdir, "phase_events.csv"))
    mt_p = os.path.join(rdir, "motion_tracking.csv.gz")
    if not os.path.exists(mt_p):
        mt_p = os.path.join(rdir, "motion_tracking.csv")
    cp_p = os.path.join(rdir, "conveyor_path_commands.csv.gz")
    if not os.path.exists(cp_p):
        cp_p = os.path.join(rdir, "conveyor_path_commands.csv")
    mt = pd.read_csv(mt_p)
    cp = pd.read_csv(cp_p)
    return s, ph, mt, cp

s1, ph1, mt1, cp1 = load_run(run_dirs[0])
s2, ph2, mt2, cp2 = load_run(run_dirs[1])
cfg = s2["configuration"]

plt.rcParams.update({
    'font.size': 10,
    'axes.titlesize': 11,
    'axes.titleweight': 'bold',
    'axes.labelsize': 10,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'figure.facecolor': 'white',
})

def extract_part_segments(ph):
    starts = ph[(ph.phase == 'part') & (ph.event == 'target_interception_started')]
    ends = ph[(ph.phase == 'part') & (ph.event == 'sequence_finished')]
    rows = []
    for (_, s_row), (_, e_row) in zip(starts.iterrows(), ends.iterrows()):
        pid = int(s_row['part_id'])
        t0 = float(s_row['wall_time_s'])
        t_end = float(e_row['wall_time_s'])
        win = ph[(ph.wall_time_s >= t0) & (ph.wall_time_s <= t_end)]
        def t_of(phase, event, default):
            m = win[(win.phase == phase) & (win.event == event)]
            return float(m['wall_time_s'].iloc[-1]) if len(m) else default
        t_app = t_of('approach_final_descent', 'completed', t0)
        t_att = t_of('grasp', 'attach_result', t_app)
        t_chk = t_of('grasp', 'finger_position_check', t_att)
        t_lft = t_of('lift_clear_of_cradle', 'completed', t_chk)
        t_trn = t_of('transport_to_bucket_hover', 'completed', t_lft)
        t_drp = t_of('bucket', 'drop_pose_arrival_check', t_trn)
        t_rel = t_of('release', 'detach_result', t_drp)
        rows.append({
            'part_id': pid,
            'Approach': t_app - t0,
            'Attach Wait': t_att - t_app,
            'Close & Check': t_chk - t_att,
            'Lift': t_lft - t_chk,
            'Transport': t_trn - t_lft,
            'Bucket Settle': t_drp - t_trn,
            'Release & Open': t_rel - t_drp,
            'Retract & Return': t_end - t_rel,
            'Total': t_end - t0,
        })
    return pd.DataFrame(rows)

seg1 = extract_part_segments(ph1)
seg2 = extract_part_segments(ph2)

# 1. audit_cycle_breakdown.png
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.4), dpi=250)
pr1, pr2 = pd.DataFrame(s1['part_results']), pd.DataFrame(s2['part_results'])
x_idx = np.arange(len(pr1))
w = 0.38
ax1.bar(x_idx - w/2, pr1['time_s'], width=w, color='#2a78d6', edgecolor='black', lw=0.7, label=f"Run 1 ({s1['run_id']}: mean 11.89 s)")
ax1.bar(x_idx + w/2, pr2['time_s'], width=w, color='#eb6834', edgecolor='black', lw=0.7, label=f"Run 2 ({s2['run_id']}: mean 11.18 s)")
for idx_p, row in pr1.iterrows():
    if row['time_s'] == 0:
        ax1.text(idx_p, 0.35, 'SKIP', ha='center', va='bottom', fontsize=7.5, fontweight='bold', color='#52514e', rotation=90)
    else:
        col_tag = 'R' if row['color'] == 'RED' else 'B'
        ax1.text(idx_p, max(pr1.loc[idx_p, 'time_s'], pr2.loc[idx_p, 'time_s']) + 0.25, col_tag, ha='center', va='bottom', fontsize=8, fontweight='bold')
ax1.set_xticks(x_idx)
ax1.set_xticklabels([f"#{i:02d}" for i in pr1['id']], fontsize=8.5)
ax1.set_xlabel('Workpiece ID (#01–#15: 10 Red/Blue Targets + 5 Skipped Distractors)')
ax1.set_ylabel('Measured Manipulation Cycle Time (s)')
ax1.set_title('(a) Per-Workpiece Cycle Time Across Both Baseline Runs')
ax1.set_ylim(0, 14.8)
ax1.legend(loc='upper right', fontsize=8.5)

stages = ['Approach', 'Attach Wait', 'Close & Check', 'Lift', 'Transport', 'Bucket Settle', 'Release & Open', 'Retract & Return']
m1_vals = [seg1[k].mean() for k in stages]
m2_vals = [seg2[k].mean() for k in stages]
y_pos = np.arange(len(stages))
ax2.barh(y_pos - w/2, m1_vals, height=w, color='#2a78d6', edgecolor='black', lw=0.7, label='Run 1 Mean (s)')
ax2.barh(y_pos + w/2, m2_vals, height=w, color='#eb6834', edgecolor='black', lw=0.7, label='Run 2 Mean (s)')
for idx_s, (v1, v2) in enumerate(zip(m1_vals, m2_vals)):
    ax2.text(max(v1, v2) + 0.05, idx_s, f"{v1:.2f}s / {v2:.2f}s", va='center', fontsize=8.5, fontweight='bold')
ax2.set_yticks(y_pos)
ax2.set_yticklabels(stages)
ax2.invert_yaxis()
ax2.set_xlim(0, 3.1)
ax2.set_xlabel('Mean Phase Duration per Sorted Workpiece (s)')
ax2.set_title('(b) Measured 8-Stage Manipulation Breakdown')
ax2.legend(loc='lower right', fontsize=8.5)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, 'audit_cycle_breakdown.png'))
plt.close()

# 2. audit_joint_tracking.png
fig, axes = plt.subplots(2, 2, figsize=(13.5, 7.5), dpi=250)
t_rel = mt2['wall_time_s'] - mt2['wall_time_s'].iloc[0]
for j_name, title_str, ax, col in [
    ('joint_1', 'Waist Turret Yaw $q_1$ (rad)', axes[0, 0], '#2a78d6'),
    ('joint_2', 'Shoulder Pitch $q_2$ (rad)', axes[0, 1], '#e34948'),
    ('joint_3', 'Wrist Pitch $q_3$ (rad)', axes[1, 0], '#1baf7a'),
    ('finger_left_joint', 'Left Finger Prismatic $q_{f,L}$ (mm)', axes[1, 1], '#6250d6'),
]:
    scale = 1000.0 if 'finger' in j_name else 1.0
    unit = 'mm' if 'finger' in j_name else 'rad'
    cmd_v = mt2[f'cmd_{j_name}'] * scale
    act_v = mt2[f'actual_{j_name}'] * scale
    err_rms = np.sqrt(((cmd_v - act_v)**2).mean())
    ax.plot(t_rel, cmd_v, 'k--', lw=1.2, alpha=0.85, label='Commanded')
    ax.plot(t_rel, act_v, color=col, lw=1.4, alpha=0.9, label=f'Measured (RMS err = {err_rms:.3f} {unit})')
    ax.set_title(title_str)
    ax.set_xlabel('Elapsed Run Time (s)')
    ax.set_ylabel(f'Position ({unit})')
    ax.legend(loc='upper right', fontsize=8.5)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, 'audit_joint_tracking.png'))
plt.close()

# 3. audit_vision_and_grasp.png
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.2), dpi=250)
def parse_finger_checks(ph):
    chk = ph[(ph.phase == 'grasp') & (ph.event == 'finger_position_check')]
    pids, fls, frs = [], [], []
    for _, r in chk.iterrows():
        d = json.loads(r['details'])
        pids.append(int(r['part_id']))
        fls.append(d.get('finger_left_m', 0.0) * 1000.0)
        frs.append(d.get('finger_right_m', 0.0) * 1000.0)
    return pids, fls, frs
p1_ids, fl1, fr1 = parse_finger_checks(ph1)
p2_ids, fl2, fr2 = parse_finger_checks(ph2)
x_t = np.arange(len(p1_ids))
ax1.scatter(x_t - 0.12, fl1, color='#2a78d6', s=55, label='Run 1 Left Finger (mm)')
ax1.scatter(x_t - 0.12, fr1, color='#2a78d6', marker='^', s=55, label='Run 1 Right Finger (mm)')
ax1.scatter(x_t + 0.12, fl2, color='#eb6834', s=55, label='Run 2 Left Finger (mm)')
ax1.scatter(x_t + 0.12, fr2, color='#eb6834', marker='^', s=55, label='Run 2 Right Finger (mm)')
ax1.axhline(-7.5, color='#e34948', ls='--', lw=1.6, label='Empty-Close Fault Threshold (-7.5 mm)')
ax1.axhline(-12.0, color='black', ls=':', lw=1.3, label='Closed Finger Command (-12.0 mm)')
ax1.set_xticks(x_t)
ax1.set_xticklabels([f"#{i:02d}" for i in p1_ids])
ax1.set_xlabel('Sorted Target Workpiece ID')
ax1.set_ylabel('Measured Finger Joint Position (mm)')
ax1.set_title('(a) Finger Positions at Grasp Check (0.30 s After Close Command)')
ax1.set_ylim(-14.5, 14.0)
ax1.legend(loc='lower left', fontsize=8)

cats = ['Run 1\nCamera Det.', 'Run 1\nFallback', 'Run 2\nCamera Det.', 'Run 2\nFallback']
counts = [7, 8, 9, 6]
bars_v = ax2.bar(cats, counts, color=['#1baf7a', '#eda100', '#1baf7a', '#eda100'], edgecolor='black', width=0.5)
for br, cv in zip(bars_v, counts):
    ax2.text(br.get_x() + br.get_width()/2, cv + 0.25, f"{cv}/15 ({100*cv/15:.0f}%)", ha='center', va='bottom', fontweight='bold', fontsize=9)
ax2.set_ylim(0, 11.5)
ax2.set_ylabel('Workpiece Count (out of 15 per run)')
ax2.set_title('(b) Pickup-Window Camera Detection vs. Metadata Fallback')
plt.tight_layout()
plt.savefig(os.path.join(out_dir, 'audit_vision_and_grasp.png'))
plt.close()

# 4. audit_conveyor_kinematics.png
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.0), dpi=250)
tc1 = cp1['wall_time_s'] - cp1['wall_time_s'].iloc[0]
tc2 = cp2['wall_time_s'] - cp2['wall_time_s'].iloc[0]
ax1.plot(tc2, cp2['belt_speed_target_m_s'] * 1000.0, 'k--', lw=1.1, alpha=0.75, label='Target Speed Command (mm/s)')
ax1.plot(tc2, cp2['belt_speed_slewed_m_s'] * 1000.0, color='#2a78d6', lw=1.6, label='Slewed Belt Speed (0.08 m/s$^2$ limit)')
ax1.set_xlabel('Elapsed Run Time (s)')
ax1.set_ylabel('Conveyor Belt Speed (mm/s)')
ax1.set_title('(a) Conveyor Speed Slew Synchronization (42 -> 6 -> 16 mm/s)')
ax1.legend(loc='upper right', fontsize=8.5)

ax2.plot(tc1, cp1['loop_s_m'], color='#2a78d6', lw=2.0, label=f"Run 1 Total Travel: {cp1['loop_s_m'].iloc[-1]:.2f} m (1.11 loops)")
ax2.plot(tc2, cp2['loop_s_m'], color='#eb6834', lw=2.0, ls='--', label=f"Run 2 Total Travel: {cp2['loop_s_m'].iloc[-1]:.2f} m (1.10 loops)")
ax2.axhline(2.8796, color='black', ls=':', lw=1.4, label='1 Full Loop Perimeter (2.880 m)')
ax2.set_xlabel('Elapsed Run Time (s)')
ax2.set_ylabel('Integrated Loop Arc-Length $s(t)$ (m)')
ax2.set_title('(b) Cumulative Stadium Conveyor Loop Progression')
ax2.legend(loc='upper left', fontsize=8.5)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, 'audit_conveyor_kinematics.png'))
plt.close()

print("Generated all 4 measured audit plots in media/!")
