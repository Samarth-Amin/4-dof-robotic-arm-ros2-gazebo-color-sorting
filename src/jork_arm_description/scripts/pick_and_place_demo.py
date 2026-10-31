#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class QuinticIKPickAndPlace(Node):
    """
    Task-Space Analytical Inverse Kinematics (IK) & Quintic Polynomial (C2)
    Trajectory Planner for the JORK 6-DOF Robotic Arm with Fin Ray Gripper.
    """
    def __init__(self):
        super().__init__('quintic_ik_pick_and_place')
        self.pubs = {
            'j1': self.create_publisher(Float64, '/jork_arm/joint_1/cmd_pos', 10),
            'j2': self.create_publisher(Float64, '/jork_arm/joint_2/cmd_pos', 10),
            'j3': self.create_publisher(Float64, '/jork_arm/joint_3/cmd_pos', 10),
            'j4': self.create_publisher(Float64, '/jork_arm/joint_4/cmd_pos', 10),
            'fl': self.create_publisher(Float64, '/jork_arm/finger_left/cmd_pos', 10),
            'fr': self.create_publisher(Float64, '/jork_arm/finger_right/cmd_pos', 10),
        }
        self.measured = {}
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Kinematic constants from calibrated CAD (in meters & radians)
        self.x0, self.y0, self.z0 = 0.075, 0.045, 0.135
        self.L2 = math.hypot(0.265, 0.030)          # 0.2667 m
        self.alpha2 = math.atan2(0.030, 0.265)      # 0.1127 rad
        self.L_tool = 0.080                         # Tool center point (TCP) offset

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.measured[name] = pos

    def solve_analytical_ik(self, x, y, z_tcp, pitch=0.0, roll=0.0):
        """Solves closed-form Analytical IK for target TCP (x, y, z, pitch, roll)."""
        dx = x - self.x0
        dy = y - self.y0
        q1 = math.atan2(dy, dx)

        # Wrist center height relative to shoulder pivot
        dz = (z_tcp - self.z0) + self.L_tool * math.sin(pitch)
        sin_arg = max(-1.0, min(1.0, dz / self.L2))
        q2 = self.alpha2 - math.asin(sin_arg)
        q3 = pitch - q2
        q4 = roll
        return [q1, q2, q3, q4]

    def move_quintic(self, target, duration=2.2, rate_hz=100):
        """
        5th-Order Quintic Polynomial Time-Scaling:
        s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        Guarantees zero velocity and zero acceleration at trajectory endpoints.
        """
        steps = max(10, int(duration * rate_hz))
        start = list(self.current)
        dt = duration / float(steps)

        for i in range(1, steps + 1):
            tau = i / float(steps)
            s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
            cmd = [q0 + s * (qf - q0) for q0, qf in zip(start, target)]
            for key, val in zip(['j1', 'j2', 'j3', 'j4', 'fl', 'fr'], cmd):
                msg = Float64()
                msg.data = float(val)
                self.pubs[key].publish(msg)
            time.sleep(dt)
        self.current = list(target)

    def execute_workcell_cycle(self):
        # Solve IK for Cartesian Workcell Coordinates (x, y, z)
        wp_hover_pick = self.solve_analytical_ik(0.421, 0.045, 0.225, pitch=0.0, roll=0.0)
        wp_grasp_pick = self.solve_analytical_ik(0.421, 0.045, 0.112, pitch=0.0, roll=0.0)
        wp_lift_pick  = self.solve_analytical_ik(0.421, 0.045, 0.250, pitch=0.0, roll=0.0)
        wp_hover_drop = self.solve_analytical_ik(0.075, 0.391, 0.250, pitch=0.0, roll=0.0)
        wp_place_drop = self.solve_analytical_ik(0.075, 0.391, 0.117, pitch=0.0, roll=0.0)

        self.get_logger().info(
            f"[IK SOLVER] Pick Grasp Joint Angles (rad): "
            f"J1={wp_grasp_pick[0]:.3f}, J2={wp_grasp_pick[1]:.3f}, J3={wp_grasp_pick[2]:.3f}"
        )

        self.get_logger().info("Step 1/8: Quintic Trajectory -> Pre-Grasp Hover + Open Fin Ray Gripper")
        self.move_quintic(wp_hover_pick + [0.015, 0.015], duration=2.0)

        self.get_logger().info("Step 2/8: Quintic Trajectory -> Descend to Target Payload (0.421, 0.045, 0.112 m)")
        self.move_quintic(wp_grasp_pick + [0.015, 0.015], duration=2.2)

        self.get_logger().info("Step 3/8: Actuating Compliant Fin Ray Fingers (Force-Closure Grasp)")
        self.move_quintic(wp_grasp_pick + [-0.005, -0.005], duration=1.2)
        time.sleep(0.4)

        self.get_logger().info("Step 4/8: Quintic Trajectory -> Vertical Payload Extraction")
        self.move_quintic(wp_lift_pick + [-0.005, -0.005], duration=2.2)

        self.get_logger().info("Step 5/8: Quintic Trajectory -> Slew Turret 90 deg to Drop-off Station (0.075, 0.391 m)")
        self.move_quintic(wp_hover_drop + [-0.005, -0.005], duration=2.8)

        self.get_logger().info("Step 6/8: Quintic Trajectory -> Precision Placement Descent")
        self.move_quintic(wp_place_drop + [-0.005, -0.005], duration=2.2)

        self.get_logger().info("Step 7/8: Releasing Payload on Drop-off Pedestal")
        self.move_quintic(wp_place_drop + [0.015, 0.015], duration=1.2)

        self.get_logger().info("Step 8/8: Retracting & Returning to Zero-Pose Home")
        self.move_quintic(wp_hover_drop + [0.015, 0.015], duration=1.8)
        self.move_quintic([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=2.2)
        self.get_logger().info("[SUCCESS] Autonomous IK Pick-and-Place Cycle Completed!")


def main():
    rclpy.init()
    node = QuinticIKPickAndPlace()
    time.sleep(1.0)
    node.execute_workcell_cycle()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
