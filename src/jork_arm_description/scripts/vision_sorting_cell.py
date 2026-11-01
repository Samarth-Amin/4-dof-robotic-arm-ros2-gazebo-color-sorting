#!/usr/bin/env python3
import os
import json
import math
import time
import csv
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState, Image


BELT_ACCEL = 0.08  # m/s^2 belt speed slew limit


class ClosedLoopStadiumConveyorCell(Node):
    """
    Closed-Loop Stadium Conveyor Sorting Cell (15 Pre-Positioned Cradle Pucks & Blocks):
    - Each carrier puck starts directly at its (xr0, yr0) loop coordinate under its block (zero startup snap).
    - Blocks glide smoothly around both 180-deg turnaround drums inside 9mm self-centering puck cradles.
    - Multi-Pose Vision Grasp Adaptation:
      * If block is Upright (vertical): grasps at Z = 0.098 m with +0.015 m jaw opening.
      * If block is Tilted / Fallen (horizontal): lowers grasp to Z = 0.080 m with wide +0.018 m jaw opening
        and aligns wrist roll so the Fin Ray fingers scoop and clamp the horizontal block cleanly!
    """
    def __init__(self):
        super().__init__('vision_sorting_cell')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('audit_dir', '~/.ros/robot_sorting_audits')
        pkg_share = get_package_share_directory('jork_arm_description')
        cfg_path = os.path.join(pkg_share, 'config', 'kinematics_config.json')
        with open(cfg_path, 'r') as f:
            self.cfg = json.load(f)

        self.audit_run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.audit_dir = Path(os.path.expanduser(str(self.get_parameter('audit_dir').value))) / self.audit_run_id
        self.audit_dir.mkdir(parents=True, exist_ok=True)
        self.audit_wall_start = time.time()
        self.audit_sim_start = None
        self.audit_events, self.motion_samples, self.joint_samples = [], [], []
        self.conveyor_samples = []
        self.audit_part_results = []
        self.audit_pub = self.create_publisher(String, '/sorting/audit_event', 50)

        self.pubs = {
            'j1': self.create_publisher(Float64, '/jork_arm/joint_1/cmd_pos', 10),
            'j2': self.create_publisher(Float64, '/jork_arm/joint_2/cmd_pos', 10),
            'j3': self.create_publisher(Float64, '/jork_arm/joint_3/cmd_pos', 10),
            'j4': self.create_publisher(Float64, '/jork_arm/joint_4/cmd_pos', 10),
            'fl': self.create_publisher(Float64, '/jork_arm/finger_left/cmd_pos', 10),
            'fr': self.create_publisher(Float64, '/jork_arm/finger_right/cmd_pos', 10),
        }
        self.puck_pubs = {}
        for pid in range(1, 16):
            self.puck_pubs[pid] = (
                self.create_publisher(Float64, f'/conveyor/puck_{pid}/cmd_x', 10),
                self.create_publisher(Float64, f'/conveyor/puck_{pid}/cmd_y', 10),
            )
        self.annotated_pub = self.create_publisher(Image, '/vision/annotated_image', 10)

        self.joint_pos = {}
        self.latest_image = None
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)

        self.current = [0.0, 0.0, 0.0, 0.0, 0.015, 0.015]

        self.x0 = float(self.cfg['x0'])
        self.y0 = float(self.cfg['y0'])
        self.j2_x = float(self.cfg['j2_x'])
        self.Z_shoulder = float(self.cfg['Z_shoulder'])
        self.L2 = float(self.cfg['L2'])
        self.X_pad = float(self.cfg['X_pad'])
        self.Z_pad = float(self.cfg['Z_pad'])
        self.Z_grasp_upright = float(self.cfg['Z_grasp_upright'])
        self.Z_grasp_fallen  = float(self.cfg['Z_grasp_fallen'])

        self.cam_x = float(self.cfg['cam_x'])
        self.cam_y = float(self.cfg['cam_y'])
        self.cam_z = float(self.cfg['cam_z'])
        self.fov_h = float(self.cfg['fov_h'])

        self.belt_x = float(self.cfg['belt_x'])
        self.belt_y = float(self.cfg['belt_y'])
        self.y_rear = float(self.cfg['y_rear'])
        self.y_front = float(self.cfg['y_front'])
        self.R_loop = float(self.cfg['R_loop'])
        self.L_straight = self.y_front - self.y_rear
        self.L_arc = math.pi * self.R_loop
        self.P_loop = float(self.cfg['P_loop'])

        self.buckets = {k: tuple(v) for k, v in self.cfg['buckets'].items()}
        self.parts = self.cfg['parts']

        self.loop_s = 0.0
        self.last_belt_tick = None
        self._belt_lock = threading.RLock()
        self._belt_v = 0.0
        self._belt_v_target = 0.0
        self._belt_run = False
        self._last_pub = -1e9
        self.belt_speed = 0.0  # slew-limited target; run_sorting_cell sets the real speeds

    # belt_speed is a slew-limited TARGET: assigning it never makes the belt jump
    @property
    def belt_speed(self):
        return self._belt_v_target

    @belt_speed.setter
    def belt_speed(self, v):
        self._belt_v_target = float(v)

    def start_belt_thread(self):
        self._belt_run = True
        threading.Thread(target=self._belt_loop, daemon=True).start()

    def _belt_loop(self):
        while self._belt_run and rclpy.ok():
            try:
                self.step_loop_conveyor()
            except Exception:
                pass
            time.sleep(0.01)

    def _gz_empty(self, topic):
        cmd = "gz topic -t %s -m gz.msgs.Empty -p ' '; sleep 0.25; gz topic -t %s -m gz.msgs.Empty -p ' '" % (topic, topic)
        return subprocess.Popen(["sh", "-c", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def set_weld(self, name, attach, wait_fn, timeout=2.5):
        """Send an attach/detach request and wait until Gazebo's DetachableJoint state topic confirms it.
        wait_fn() keeps the arm tracking the puck while we wait. Returns True only if confirmed."""
        want = b'"attached"' if attach else b'"detached"'
        topic = '/jork_arm/grasp/%s/%s' % (name, 'attach' if attach else 'detach')
        echo = subprocess.Popen(['gz', 'topic', '-e', '-t', '/model/%s/detachable_joint/state' % name],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        fd = echo.stdout.fileno()
        os.set_blocking(fd, False)
        t0 = time.time()
        sends = [0.3, 1.2]  # first send waits for the echo to subscribe; one resend if still silent
        ok = False
        while time.time() - t0 < timeout:
            wait_fn()
            if sends and time.time() - t0 >= sends[0]:
                sends.pop(0)
                subprocess.Popen(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ' '],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                data = b''
            if want in data:
                ok = True
                break
        echo.terminate()
        try:
            echo.wait(timeout=1.0)
        except Exception:
            pass
        return ok

    def wait_arrived(self, target, tol=0.05, settle=0.25, timeout=2.5):
        """Block until the MEASURED joint_1..3 are within tol rad of target and stay there for `settle` s."""
        names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        t0 = time.time()
        ok_since = None
        err = 9.9
        while time.time() - t0 < timeout:
            self.sleep_with_belt(0.02)
            errs = [abs(self.joint_pos.get(n, 1e9) - target[i]) for i, n in enumerate(names)]
            err = max(errs)
            if err < tol:
                if ok_since is None:
                    ok_since = time.time()
                if time.time() - ok_since >= settle:
                    return True, err
            else:
                ok_since = None
        return False, err

    def attach_block(self, name):
        self._gz_empty('/jork_arm/grasp/%s/attach' % name)

    def detach_block(self, name):
        self._gz_empty('/jork_arm/grasp/%s/detach' % name)

    def detach_all_blocks(self):
        # DetachableJoint welds every child at spawn; release them with at most 4 gz CLIs at a time
        topics = ["/jork_arm/grasp/%s/detach" % p['name'] for p in self.parts]
        one = "printf '%s\\n' " + ' '.join(topics) + " | xargs -P 4 -I{} gz topic -t {} -m gz.msgs.Empty -p ' '"
        t0 = time.time()
        try:
            subprocess.run(["sh", "-c", one + "; sleep 0.5; " + one],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Start-up release timed out, continuing anyway")
        self.get_logger().info("Start-up release sent for %d blocks in %.1f s" % (len(topics), time.time() - t0))

    def _sim_time_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def audit_event(self, phase, event, part=None, details=None):
        sim_t = self._sim_time_s()
        if self.audit_sim_start is None:
            self.audit_sim_start = sim_t
        row = {'wall_time_s': time.time()-self.audit_wall_start, 'sim_time_s': sim_t,
               'phase': phase, 'event': event,
               'part_id': (part.get('id') if isinstance(part, dict) else part),
               'details': details or {}}
        self.audit_events.append(row)
        msg = String(); msg.data = json.dumps(row, sort_keys=True); self.audit_pub.publish(msg)

    def audit_motion_sample(self, phase, command):
        names = ['joint_1','joint_2','joint_3','joint_4','finger_left_joint','finger_right_joint']
        row = {'wall_time_s': time.time()-self.audit_wall_start, 'sim_time_s': self._sim_time_s(), 'phase': phase}
        for i, name in enumerate(names):
            actual = self.joint_pos.get(name)
            row['cmd_'+name] = float(command[i])
            row['actual_'+name] = (float(actual) if actual is not None else None)
            row['error_'+name] = (float(actual)-float(command[i]) if actual is not None else None)
        self.motion_samples.append(row)

    def save_audit(self, status='completed'):
        names = ['joint_1','joint_2','joint_3','joint_4','finger_left_joint','finger_right_joint']
        def write_csv(filename, rows, fields):
            with (self.audit_dir / filename).open('w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)
        event_rows = []
        for row in self.audit_events:
            x = dict(row); x['details'] = json.dumps(x['details'], sort_keys=True); event_rows.append(x)
        write_csv('phase_events.csv', event_rows, ['wall_time_s','sim_time_s','phase','event','part_id','details'])
        motion_fields = ['wall_time_s','sim_time_s','phase'] + [k+'_'+n for n in names for k in ('cmd','actual','error')]
        write_csv('motion_tracking.csv', self.motion_samples, motion_fields)
        write_csv('joint_state_telemetry.csv', self.joint_samples, ['wall_time_s','sim_time_s']+names)
        if self.conveyor_samples:
            write_csv('conveyor_path_commands.csv', self.conveyor_samples, list(self.conveyor_samples[0].keys()))
        max_err = {}
        for name in names:
            vals = [abs(r['error_'+name]) for r in self.motion_samples if r.get('error_'+name) is not None]
            max_err[name] = max(vals) if vals else None
        summary = {
          'run_id': self.audit_run_id, 'status': status,
          'wall_elapsed_s': time.time()-self.audit_wall_start,
          'sim_elapsed_s': (self._sim_time_s()-self.audit_sim_start) if self.audit_sim_start is not None else None,
          'part_results': self.audit_part_results, 'phase_event_count': len(self.audit_events),
          'motion_sample_count': len(self.motion_samples), 'joint_state_sample_count': len(self.joint_samples),
          'conveyor_path_sample_count': len(self.conveyor_samples),
          'max_absolute_tracking_error': max_err,
          'configuration': {k:self.cfg.get(k) for k in ['x0','y0','j2_x','Z_shoulder','L2','X_pad','Z_pad','R_grasp','Z_grasp_upright','Z_grasp_fallen','cam_x','cam_y','cam_z','fov_h','belt_x','belt_y','y_rear','y_front','R_loop','P_loop']},
          'limits': ['DetachableJoint is an idealized grasp, not frictional force-closure validation.',
                     'Release confirmation does not independently verify bucket occupancy.',
                     'Camera metadata fallback and model-derived pickup coordinates remain in use.',
                     'Tracking errors are command-versus-joint-state differences, not measured motor torque.']
        }
        with (self.audit_dir/'run_summary.json').open('w',encoding='utf-8') as f: json.dump(summary,f,indent=2,sort_keys=True)
        self.get_logger().info('SIMULATION AUDIT SAVED: %s' % str(self.audit_dir))

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos
        names = ['joint_1','joint_2','joint_3','joint_4','finger_left_joint','finger_right_joint']
        if all(n in self.joint_pos for n in names):
            self.joint_samples.append({'wall_time_s':time.time()-self.audit_wall_start,
                'sim_time_s':self._sim_time_s(), **{n:float(self.joint_pos[n]) for n in names}})

    def image_cb(self, msg):
        self.latest_image = msg

    def loop_xy(self, s):
        s_mod = s % self.P_loop
        if s_mod < self.L_straight:
            return 0.0, self.y_rear + s_mod
        s_mod -= self.L_straight
        if s_mod < self.L_arc:
            theta = s_mod / self.R_loop
            return self.R_loop * (1.0 - math.cos(theta)), self.y_front + self.R_loop * math.sin(theta)
        s_mod -= self.L_arc
        if s_mod < self.L_straight:
            return 2.0 * self.R_loop, self.y_front - s_mod
        s_mod -= self.L_straight
        theta = s_mod / self.R_loop
        return self.R_loop * (1.0 + math.cos(theta)), self.y_rear - self.R_loop * math.sin(theta)

    def step_loop_conveyor(self):
        """Integrates loop position on the node clock (sim time with use_sim_time:=true), slews belt speed
        toward its target, publishes puck targets at <= ~60 Hz. Thread-safe."""
        with self._belt_lock:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.last_belt_tick is None:
                self.last_belt_tick = now
            else:
                dt = max(0.0, min(0.1, now - self.last_belt_tick))
                self.last_belt_tick = now
                lim = BELT_ACCEL * dt
                dv = self._belt_v_target - self._belt_v
                self._belt_v += max(-lim, min(lim, dv))
                self.loop_s += self._belt_v * dt
            if 0.0 <= now - self._last_pub < 0.015:
                return self.loop_s
            self._last_pub = now
            loop_s = self.loop_s

        conveyor_row = {'wall_time_s':time.time()-self.audit_wall_start, 'sim_time_s':now,
                        'loop_s_m':float(loop_s), 'belt_speed_slewed_m_s':float(self._belt_v),
                        'belt_speed_target_m_s':float(self._belt_v_target)}
        for p in self.parts:
            pid = p['puck_id']
            xr, yr = self.loop_xy(float(p['s0']) + loop_s)
            conveyor_row[f'puck_{pid:02d}_path_x_m'] = float(xr)
            conveyor_row[f'puck_{pid:02d}_path_y_m'] = float(yr)
            mx = Float64()
            my = Float64()
            mx.data = float(xr - float(p['xr0']))
            my.data = float(yr - float(p['yr0']))
            conveyor_row[f'puck_{pid:02d}_cmd_offset_x_m'] = float(mx.data)
            conveyor_row[f'puck_{pid:02d}_cmd_offset_y_m'] = float(my.data)
            pub_x, pub_y = self.puck_pubs[pid]
            pub_x.publish(mx)
            pub_y.publish(my)
        self.conveyor_samples.append(conveyor_row)
        return loop_s

    def sleep_with_belt(self, duration, rate_hz=60):
        steps = max(1, int(duration * rate_hz))
        dt = duration / float(steps)
        for _ in range(steps):
            self.step_loop_conveyor()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def solve_ik(self, x, y, z_tcp, roll=0.0):
        dx = x - self.x0
        dy = y - self.y0
        q1 = math.atan2(dy, dx)
        sin_q2 = max(-0.85, min(0.85, (self.Z_shoulder + self.Z_pad - z_tcp) / self.L2))
        q2 = math.asin(sin_q2)
        q3 = -q2
        return [q1, q2, q3, roll]

    def forward_kinematics(self, q1, q2, q3):
        pitch_g = q2 + q3
        r = self.j2_x + self.L2 * math.cos(q2) + self.X_pad * math.cos(pitch_g)
        z = self.Z_shoulder - self.L2 * math.sin(q2) - self.X_pad * math.sin(pitch_g) + self.Z_pad
        return (self.x0 + r * math.cos(q1), self.y0 + r * math.sin(q1), z)

    def sync_held_part_pose(self, model_name, x, y, z, yaw=0.0):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        req = 'name: "%s", position: {x: %.4f, y: %.4f, z: %.4f}, orientation: {z: %.4f, w: %.4f}' % (
            model_name, x, y, z, qz, qw
        )
        subprocess.run(
            ["gz", "service", "-s", "/world/sorting_cell_world/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "150", "-r", req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def move_quintic(self, target_arm, duration=0.70, rate_hz=60, held_model=None, phase='joint_move'):
        self.audit_event(phase, 'started', details={'duration_s':duration, 'target':list(target_arm)})
        move_t0 = time.time()
        steps = max(8, int(duration * rate_hz))
        start_arm = list(self.current)
        dt = duration / float(steps)
        keys = ['j1', 'j2', 'j3', 'j4', 'fl', 'fr']
        for i in range(1, steps + 1):
            tau = i / float(steps)
            s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
            cmd = [q0 + s * (qf - q0) for q0, qf in zip(start_arm, target_arm)]
            for key, val in zip(keys, cmd):
                msg = Float64()
                msg.data = float(val)
                self.pubs[key].publish(msg)
            self.audit_motion_sample(phase, cmd)
            self.step_loop_conveyor()
            if held_model is not None and (i % 12 == 0 or i == steps):
                tx, ty, tz = self.forward_kinematics(cmd[0], cmd[1], cmd[2])
                self.sync_held_part_pose(held_model, tx, ty, tz, yaw=cmd[0])
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
        self.current = list(target_arm)
        self.audit_event(phase, 'completed', details={'elapsed_wall_s':time.time()-move_t0})

    def move_puck_tracked(self, part, z_start, z_end, f_start, f_end, roll=0.0, duration=0.50, rate_hz=60, held_model=None, phase='tracked_intercept'):
        self.audit_event(phase, 'started', part, {'z_start_m':z_start,'z_end_m':z_end,'finger_start_m':f_start,'finger_end_m':f_end,'duration_s':duration})
        move_t0 = time.time()
        steps = max(8, int(duration * rate_hz))
        dt = duration / float(steps)
        keys = ['j1', 'j2', 'j3', 'j4', 'fl', 'fr']
        cmd = list(self.current)
        for i in range(1, steps + 1):
            tau = i / float(steps)
            s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
            self.step_loop_conveyor()
            rclpy.spin_once(self, timeout_sec=0.0)
            xr, yr = self.loop_xy(float(part['s0']) + self.loop_s)
            px = self.belt_x + xr
            py = self.belt_y + yr
            z_now = z_start + s * (z_end - z_start)
            f_now = f_start + s * (f_end - f_start)
            ik_now = self.solve_ik(px, py, z_now, roll=roll)
            cmd = ik_now + [f_now, f_now]
            for key, val in zip(keys, cmd):
                msg = Float64()
                msg.data = float(val)
                self.pubs[key].publish(msg)
            self.audit_motion_sample(phase, cmd)
            if held_model is not None and (i == steps // 2 or i == steps):
                tx, ty, tz = self.forward_kinematics(cmd[0], cmd[1], cmd[2])
                self.sync_held_part_pose(held_model, tx, ty, tz, yaw=cmd[0])
            time.sleep(dt)
        self.current = list(cmd)
        self.audit_event(phase, 'completed', part, {'elapsed_wall_s':time.time()-move_t0})

    def scan_pickup_window_camera(self):
        """
        Detects RED, BLUE, and all 5 Random Distractor colors in the pickup window AND measures
        whether the block is Upright (vertical) or Tilted/Fallen (horizontal) plus its orientation angle!
        """
        rclpy.spin_once(self, timeout_sec=0.01)
        if self.latest_image is None or len(self.latest_image.data) == 0:
            return None

        w, h = self.latest_image.width, self.latest_image.height
        img = np.frombuffer(self.latest_image.data, dtype=np.uint8).reshape((h, w, 3)).copy()
        vis = img.copy()

        R = img[:, :, 0].astype(np.int16)
        G = img[:, :, 1].astype(np.int16)
        B = img[:, :, 2].astype(np.int16)

        masks = {
            'RED':    (R > 155) & (R > G + 75) & (R > B + 75),
            'BLUE':   (B > 135) & (B > R + 65) & (B > G + 40),
            'YELLOW': (R > 165) & (G > 165) & (B < 80),
            'PURPLE': (R > 115) & (B > 145) & (G < 80),
            'ORANGE': (R > 170) & (G > 65)  & (G < 155) & (B < 65),
            'CYAN':   (G > 155) & (B > 155) & (R < 80),
            'GREEN':  (G > 145) & (G > R + 60) & (G > B + 50),
        }

        fx = (w / 2.0) / math.tan(self.fov_h / 2.0)
        fy = fx
        cx = (w - 1) / 2.0
        cy = (h - 1) / 2.0

        belt_strip = np.zeros((h, w), dtype=bool)
        belt_strip[int(h * 0.25):int(h * 0.75), int(w * 0.32):int(w * 0.68)] = True

        best_det = None
        best_dist = 999.0
        for color_name, mask in masks.items():
            ys, xs = np.nonzero(mask & belt_strip)
            if len(xs) >= 6:
                u = float(np.mean(xs))
                v = float(np.mean(ys))
                du = xs - u
                dv = ys - v
                mu20 = float(np.mean(du * du))
                mu02 = float(np.mean(dv * dv))
                mu11 = float(np.mean(du * dv))
                angle_rad = 0.5 * math.atan2(2.0 * mu11, mu20 - mu02)

                span_u = float(max(xs) - min(xs) + 1)
                span_v = float(max(ys) - min(ys) + 1)
                aspect = max(span_u, span_v) / max(1.0, min(span_u, span_v))

                # Detect if block is non-vertical (Tilted / Fallen on its side) vs Upright
                if aspect > 1.38 or len(xs) > 150:
                    pose_state = "Tilted/Fallen"
                    z_target = self.Z_grasp_fallen
                    z_eff = 0.082
                else:
                    pose_state = "Upright"
                    z_target = self.Z_grasp_upright
                    z_eff = 0.120

                depth = self.cam_z - z_eff
                rx = self.cam_x - ((v - cy) / fy) * depth
                ry = self.cam_y - ((u - cx) / fx) * depth

                if (self.belt_y - 0.045) <= ry <= (self.belt_y + 0.040):
                    dist = abs(ry - self.belt_y)
                    if dist < best_dist:
                        best_dist = dist
                        best_det = {
                            'color': color_name,
                            'pos': (rx, ry),
                            'z_grasp': z_target,
                            'state': pose_state,
                            'angle_rad': angle_rad,
                        }
                    x0_b, x1_b = max(0, int(min(xs)) - 2), min(w - 1, int(max(xs)) + 2)
                    y0_b, y1_b = max(0, int(min(ys)) - 2), min(h - 1, int(max(ys)) + 2)
                    box_col = [0, 255, 0] if color_name in ('RED', 'BLUE') else [255, 165, 0]
                    vis[y0_b:y1_b+1, [x0_b, x1_b]] = box_col
                    vis[[y0_b, y1_b], x0_b:x1_b+1] = box_col

        out_msg = Image()
        out_msg.header = self.latest_image.header
        out_msg.height, out_msg.width = h, w
        out_msg.encoding = 'rgb8'
        out_msg.step = w * 3
        out_msg.data = vis.tobytes()
        self.annotated_pub.publish(out_msg)
        return best_det

    def is_object_held(self):
        for _ in range(8):
            self.step_loop_conveyor()
            rclpy.spin_once(self, timeout_sec=0.02)
        fl = self.joint_pos.get('finger_left_joint', 0.0)
        fr = self.joint_pos.get('finger_right_joint', 0.0)
        held = (fl > -0.0075) and (fr > -0.0075)
        return held, (fl, fr)

    def run_sorting_cell(self):
        self.audit_event('run', 'started', details={'part_count':len(self.parts), 'dry_run':bool(self.get_parameter('dry_run').value)})
        self.get_logger().info("==================================================================================")
        self.get_logger().info(" CLOSED-LOOP STADIUM CONVEYOR SORTING LINE (15 CRADLED BLOCKS: RED/BLUE/RANDOM)   ")
        self.get_logger().info("==================================================================================")

        # Hold pucks at 0.0 offset for 0.5s so blocks settle motionless inside their cradles
        self.get_logger().info("Releasing start-up welds on all blocks...")
        self.detach_all_blocks()
        self.last_belt_tick = None
        self.belt_speed = 0.0
        self.audit_event('conveyor','speed_target_changed',details={'target_m_s':0.0})
        self.sleep_with_belt(0.5)

        if self.get_parameter('dry_run').value:
            self.get_logger().info("DRY RUN: belt only. Watch the blocks round both turns (Ctrl+C to stop).")
            self.move_quintic([-0.55, -0.30, 0.30, 0.0, 0.016, 0.016], duration=1.0, phase='dry_run_park')
            self.belt_speed = 0.042
            while rclpy.ok():
                self.sleep_with_belt(1.0)

        # Park robot in Side Ready Posture (-0.55 rad) so overhead camera has an unobstructed view
        wp_ready = [-0.55, -0.30, 0.30, 0.0, 0.016, 0.016]
        self.move_quintic(wp_ready, duration=1.0, phase='startup_to_ready')

        run_start_time = time.time()
        self.last_belt_tick = None
        benchmarks = self.audit_part_results  # preserve partial results if the run is interrupted
        s_pickup = -self.y_rear  # 0.55 m along Active Lane corresponds to y_rel = 0.0 (belt_y)

        pending = list(self.parts)  # each puck gets one look; missed targets stay pending and come round again
        while pending:
            self.belt_speed = 0.042
            self.audit_event('conveyor','speed_target_changed',details={'target_m_s':0.042})
            part = None
            while part is None:
                self.step_loop_conveyor()
                rclpy.spin_once(self, timeout_sec=0.005)
                for cand in pending:
                    s_c = (float(cand['s0']) + self.loop_s) % self.P_loop
                    if (s_pickup - 0.018) <= s_c <= (s_pickup + 0.025):
                        part = cand
                        break
                if part is None:
                    time.sleep(0.015)
            pending.remove(part)
            pid = part['id']
            expected_color = part['color']
            self.audit_event('part','selected',part,{'expected_color':expected_color,'loop_s':self.loop_s})
            s0 = float(part['s0'])

            # Inspect the pickup window with the overhead camera
            cam_det = self.scan_pickup_window_camera()
            detected_color = cam_det['color'] if (cam_det is not None) else expected_color
            self.audit_event('vision','classification',part,{'detected_color':detected_color,'expected_color':expected_color,'camera_detection_available':cam_det is not None,'metadata_fallback_used':cam_det is None,'camera_xy_m':(list(cam_det['pos']) if cam_det is not None else None),'image_angle_rad':(cam_det['angle_rad'] if cam_det is not None else None),'pose_state':(cam_det['state'] if cam_det is not None else 'Upright')})
            if cam_det is not None and detected_color != expected_color:
                self.get_logger().warn("[BLOCK #%02d] Camera sees %s but this puck carries %s (stray block in the pickup strip?) -> SKIPPING" % (
                    pid, detected_color, expected_color))
                benchmarks.append({'id': pid, 'color': expected_color, 'state': 'n/a', 'action': 'SKIP (Mismatch)',
                                   'outcome': 'MISMATCH SKIP', 'time_s': 0.0})
                self.audit_event('part','skipped_color_mismatch',part,{'detected_color':detected_color,'expected_color':expected_color})
                continue
            pose_state = cam_det['state'] if (cam_det is not None) else 'Upright'
            z_grasp = cam_det['z_grasp'] if (cam_det is not None) else self.Z_grasp_upright
            jaw_open = 0.018 if pose_state == 'Tilted/Fallen' else 0.015

            xr, yr = self.loop_xy(s0 + self.loop_s)
            px = self.belt_x + xr
            py = self.belt_y + yr

            # CASE 1: Random Distractor Color (YELLOW, PURPLE, ORANGE, CYAN, GREEN) -> SKIP!
            if detected_color not in ('RED', 'BLUE'):
                self.get_logger().info(
                    "[BLOCK #%02d / 15] Camera Detected RANDOM Color: %-6s [%s] at (%.3f, %.3f)m -> SKIPPING (Rounding loop turn!)" % (
                        pid, detected_color, pose_state, px, py
                    )
                )
                benchmarks.append({
                    'id': pid, 'color': detected_color, 'state': pose_state,
                    'action': 'SKIP (Random)', 'outcome': 'CIRCULATED LOOP', 'time_s': 0.0
                })
                self.audit_event('part','non_target_skipped',part,{'color':detected_color,'camera_detection_available':cam_det is not None})
                continue

            # CASE 2: Target RED or BLUE Block -> Adaptive Multi-Pose Co-Moving Intercept!
            t_cycle = time.time()
            self.audit_event('part','target_interception_started',part,{'color':detected_color,'pose_state':pose_state,'z_grasp_m':z_grasp,'model_pickup_xy_m':[px,py],'camera_xy_m':(list(cam_det['pos']) if cam_det is not None else None)})
            self.get_logger().info(
                "\n[BLOCK #%02d / 15] Camera Detected TARGET: %-4s [%s, Z_grasp=%.3fm] at (%.3f, %.3f)m -> INTERCEPTING!" % (
                    pid, detected_color, pose_state, z_grasp, px, py
                )
            )

            # Micro-synchronize loop speed to 6 mm/s while tracking puck during descent & jaw pinch
            self.belt_speed = 0.006
            self.audit_event('conveyor','speed_target_changed',part,{'target_m_s':0.006})
            self.move_puck_tracked(part, z_start=0.185, z_end=0.175, f_start=jaw_open, f_end=jaw_open, duration=0.45, phase='approach_pre_descent')
            self.move_puck_tracked(part, z_start=0.175, z_end=z_grasp, f_start=jaw_open, f_end=jaw_open, duration=0.55, phase='approach_final_descent')
            # Weld the block to the gripper with the jaws open; only close the jaws once Gazebo CONFIRMS the weld
            self.audit_event('grasp','attach_requested',part,{'model':part['name']})
            welded = self.set_weld(part['name'], True, lambda: self.move_puck_tracked(
                part, z_start=z_grasp, z_end=z_grasp, f_start=jaw_open, f_end=jaw_open, duration=0.10, phase='hold_during_attach'))
            self.audit_event('grasp','attach_result',part,{'confirmed_by_gazebo':bool(welded)})
            self.get_logger().info("  -> weld %s" % ("CONFIRMED by Gazebo" if welded else "NOT confirmed"))
            if welded:
                self.move_puck_tracked(part, z_start=z_grasp, z_end=z_grasp, f_start=jaw_open, f_end=-0.012, duration=0.50, phase='gripper_close_after_attach')
            self.sleep_with_belt(0.15)

            held, f_state = self.is_object_held()
            self.audit_event('grasp','finger_position_check',part,{'finger_left_m':f_state[0],'finger_right_m':f_state[1],'check_passed':bool(held),'overridden_by_detachable_joint':True})
            # Lift block cleanly out of the 9mm puck cradle (with TCP pose sync if grasped so it never snags on cradle rim)
            # Welded children don't collide with the arm, so finger closure can't prove a grasp any more
            held = welded
            held_model = None
            self.move_puck_tracked(
                part, z_start=z_grasp, z_end=0.205, f_start=(-0.012 if held else jaw_open), f_end=(-0.012 if held else jaw_open), duration=0.50, held_model=held_model, phase='lift_clear_of_cradle'
            )
            self.belt_speed = 0.016
            self.audit_event('conveyor','speed_target_changed',part,{'target_m_s':0.016})

            if not held:
                self.get_logger().warn(
                    "  -> [NO WELD] %s: grasp weld not confirmed (jaws L=%.1fmm, R=%.1fmm). Leaving block on the conveyor..." % (
                        detected_color, f_state[0] * 1000.0, f_state[1] * 1000.0
                    )
                )
                self.sleep_with_belt(0.3)
                self.move_quintic(wp_ready, duration=0.65, phase='return_to_ready')
                benchmarks.append({
                    'id': pid, 'color': detected_color, 'state': pose_state,
                    'action': 'SORT -> ' + detected_color,
                    'outcome': 'ATTACH_FAILED', 'time_s': time.time() - t_cycle
                })
                self.audit_event('part','attach_failed',part,{'finger_left_m':f_state[0],'finger_right_m':f_state[1]})
                continue

            # Slew to Open RED or BLUE Bucket & Release Inside
            bx, by, _ = self.buckets[detected_color]
            wp_bucket_hover = self.solve_ik(bx, by, 0.205)
            wp_bucket_drop  = self.solve_ik(bx, by, 0.165)

            self.get_logger().info(
                "  -> Block welded in gripper (jaw L=%.1fmm, R=%.1fmm) -> Slewing to %s BUCKET!" % (
                    f_state[0] * 1000.0, f_state[1] * 1000.0, detected_color
                )
            )
            self.move_quintic(wp_bucket_hover + [-0.012, -0.012], duration=1.80, held_model=held_model, phase='transport_to_bucket_hover')
            self.move_quintic(wp_bucket_drop  + [-0.012, -0.012], duration=0.80, held_model=held_model, phase='bucket_final_descent')
            arrived, a_err = self.wait_arrived(wp_bucket_drop)
            self.audit_event('bucket','drop_pose_arrival_check',part,{'arrived':bool(arrived),'max_joint_error_rad':float(a_err),'bucket':detected_color})
            self.get_logger().info("  -> Over %s bucket: arm %s (max joint error %.3f rad). Pausing 0.3 s..." % (
                detected_color, "ARRIVED" if arrived else "NOT settled (timeout)", a_err))
            self.sleep_with_belt(0.3)

            # Position block inside bucket opening and open jaws so it drops naturally onto the bucket floor
            self.get_logger().info("  -> Dropping %s Block Inside %s BUCKET!" % (detected_color, detected_color))
            self.move_quintic(wp_bucket_drop  + [0.016, 0.016], duration=0.35, phase='open_jaws_at_drop_pose')
            self.audit_event('release','detach_requested',part,{'model':part['name']})
            released = self.set_weld(part['name'], False, lambda: self.sleep_with_belt(0.05))
            self.audit_event('release','detach_result',part,{'confirmed_by_gazebo':bool(released),'arm_arrived':bool(arrived),'bucket_occupancy_verified':False})
            self.get_logger().info("  -> release %s" % ("CONFIRMED by Gazebo" if released else "NOT confirmed"))
            self.sleep_with_belt(0.4)
            self.move_quintic(wp_bucket_hover + [0.016, 0.016], duration=0.40, phase='retract_from_bucket')

            # Return to Side Ready Posture so camera is clear for the next incoming block
            self.move_quintic(wp_ready, duration=0.65)

            outcome = 'RELEASE_CONFIRMED_AND_ARM_SETTLED' if (released and arrived) else ('RELEASE_CONFIRMED_ARM_NOT_SETTLED' if released else 'RELEASE_NOT_CONFIRMED')
            benchmarks.append({
                'id': pid, 'color': detected_color, 'state': pose_state,
                'action': 'SORT -> ' + detected_color,
                'outcome': outcome, 'time_s': time.time() - t_cycle
            })
            self.audit_event('part','sequence_finished',part,{'outcome':outcome,'release_confirmed':bool(released),'arm_arrived':bool(arrived),'bucket_occupancy_verified':False})

        # Keep circulating the skipped random blocks around the radial turns for 3 seconds at the end
        self.belt_speed = 0.055
        self.audit_event('conveyor','speed_target_changed',details={'target_m_s':0.055})
        self.move_quintic([0.0, 0.0, 0.0, 0.0, 0.015, 0.015], duration=1.2, phase='end_of_run_park')
        self.sleep_with_belt(2.5)
        total_time = time.time() - run_start_time

        self.get_logger().info("\n==========================================================================================")
        self.get_logger().info("       CLOSED-LOOP STADIUM CONVEYOR 15-BLOCK SORTING CELL BENCHMARK REPORT                ")
        self.get_logger().info("==========================================================================================")
        self.get_logger().info("Block # | Camera Color | Pose State    | Assigned Action | Outcome          | Cycle Time")
        self.get_logger().info("--------+--------------+---------------+-----------------+------------------+-----------")
        sorted_red = 0
        sorted_blue = 0
        skipped_random = 0
        for b in benchmarks:
            if b['outcome'] == 'RELEASE_CONFIRMED_AND_ARM_SETTLED':
                if b['color'] == 'RED':
                    sorted_red += 1
                elif b['color'] == 'BLUE':
                    sorted_blue += 1
            if b['outcome'] == 'CIRCULATED LOOP':
                skipped_random += 1
            t_str = ("%5.2f s" % b['time_s']) if b['time_s'] > 0.0 else "  -- (Loop)"
            self.get_logger().info(
                "  #%02d   | %-12s | %-13s | %-15s | %-16s | %s" % (
                    b['id'], b['color'], b['state'], b['action'], b['outcome'], t_str
                )
            )
        self.get_logger().info("------------------------------------------------------------------------------------------")
        self.audit_part_results = benchmarks
        self.audit_event('run','completed',details={'release_confirmed_and_arm_settled_red':sorted_red,'release_confirmed_and_arm_settled_blue':sorted_blue,'non_targets_skipped':skipped_random,'bucket_occupancy_verified':False})
        self.get_logger().info("RED release sequences completed : %d / 5 (bucket occupancy not independently checked)" % sorted_red)
        self.get_logger().info("BLUE release sequences completed: %d / 5 (bucket occupancy not independently checked)" % sorted_blue)
        self.get_logger().info("Non-target workpieces skipped    : %d / 5" % skipped_random)
        self.get_logger().info("Total Run Time         : %.2f s" % total_time)
        self.get_logger().info("==========================================================================================")


def main():
    rclpy.init()
    node = ClosedLoopStadiumConveyorCell()
    status = 'completed'
    try:
        time.sleep(1.0)
        node.start_belt_thread()
        node.run_sorting_cell()
    except KeyboardInterrupt:
        status = 'interrupted'
        node.audit_event('run','interrupted')
    except Exception as exc:
        status = 'error'
        node.audit_event('run','error',details={'exception':repr(exc)})
        raise
    finally:
        node._belt_run = False
        time.sleep(0.05)
        node.save_audit(status)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
