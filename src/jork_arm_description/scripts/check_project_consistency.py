#!/usr/bin/env python3
"""Static consistency checks for the ROS 2 / Gazebo sorting-cell repository.

Run from the repository root:
    python3 src/jork_arm_description/scripts/check_project_consistency.py

This checks duplicated configuration across JSON, URDF/Xacro, SDF, launch,
CMake and package metadata. It does not replace a Gazebo runtime test.
"""
from __future__ import annotations

import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "jork_arm_description"
CFG_PATH = PKG / "config" / "kinematics_config.json"
XACRO_PATH = PKG / "urdf" / "jork_arm.urdf.xacro"
WORLD_PATH = PKG / "worlds" / "sorting_cell.sdf"
LAUNCH_PATH = PKG / "launch" / "gazebo.launch.py"
CMAKE_PATH = PKG / "CMakeLists.txt"
PACKAGE_PATH = PKG / "package.xml"
SCRIPT_PATH = PKG / "scripts" / "vision_sorting_cell.py"

errors = []
warnings = []
passed = []


def ok(message):
    passed.append(message)


def error(message):
    errors.append(message)


def warn(message):
    warnings.append(message)


def close(a, b, tol=1e-5):
    return abs(float(a) - float(b)) <= tol


def child_text(parent, path, default=None):
    node = parent.find(path)
    return node.text.strip() if node is not None and node.text else default


def xyz_from_joint(joint):
    origin = joint.find("origin")
    if origin is None:
        return None
    vals = origin.attrib.get("xyz", "0 0 0").split()
    return tuple(float(v) for v in vals)


def main():
    for p in (CFG_PATH, XACRO_PATH, WORLD_PATH, LAUNCH_PATH, CMAKE_PATH, PACKAGE_PATH, SCRIPT_PATH):
        if not p.exists():
            error(f"Missing required project file: {p.relative_to(ROOT)}")
    if errors:
        return finish()

    cfg = json.loads(CFG_PATH.read_text(encoding="utf-8"))
    robot = ET.parse(XACRO_PATH).getroot()
    world = ET.parse(WORLD_PATH).getroot()
    package = ET.parse(PACKAGE_PATH).getroot()
    xacro_text = XACRO_PATH.read_text(encoding="utf-8")
    script_text = SCRIPT_PATH.read_text(encoding="utf-8")
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
    cmake_text = CMAKE_PATH.read_text(encoding="utf-8")
    world_text = WORLD_PATH.read_text(encoding="utf-8")

    # 1. Arm and gripper joint inventory.
    joints = {j.attrib.get("name"): j for j in robot.findall("joint")}
    revolute = sorted(n for n, j in joints.items() if j.attrib.get("type") == "revolute")
    prismatic = sorted(n for n, j in joints.items() if j.attrib.get("type") == "prismatic")
    expected_rev = ["joint_1", "joint_2", "joint_3", "joint_4"]
    expected_pris = ["finger_left_joint", "finger_right_joint"]
    if revolute == expected_rev:
        ok("URDF contains exactly four revolute arm joints.")
    else:
        error(f"Revolute joint inventory differs: {revolute}")
    if prismatic == expected_pris:
        ok("URDF contains exactly two prismatic gripper joints.")
    else:
        error(f"Prismatic joint inventory differs: {prismatic}")

    # 2. Compare the geometric values used by the IK code with URDF origins.
    j1 = joints.get("joint_1")
    j2 = joints.get("joint_2")
    j3 = joints.get("joint_3")
    if all((j1, j2, j3)):
        o1, o2, o3 = xyz_from_joint(j1), xyz_from_joint(j2), xyz_from_joint(j3)
        checks = [
            ("x0", cfg["x0"], o1[0]),
            ("y0", cfg["y0"], o1[1]),
            ("j2_x", cfg["j2_x"], o2[0]),
            ("Z_shoulder", cfg["Z_shoulder"], o1[2] + o2[2]),
            ("L2", cfg["L2"], o3[0]),
        ]
        for name, configured, urdf_value in checks:
            if close(configured, urdf_value):
                ok(f"{name} matches URDF.")
            else:
                error(f"{name} mismatch: config={configured}, URDF-derived={urdf_value}")

    for joint_name, expected_y in (("finger_left_joint", 0.016), ("finger_right_joint", -0.016)):
        joint = joints.get(joint_name)
        if joint is None:
            continue
        origin = xyz_from_joint(joint)
        if close(origin[0], 0.0535) and close(origin[1], expected_y) and close(origin[2], -0.012):
            ok(f"{joint_name} origin matches the documented finger mount.")
        else:
            error(f"{joint_name} origin mismatch: found {origin}")

    # 3. Conveyor loop arithmetic and workpiece configuration.
    parts = cfg.get("parts", [])
    ids = [p.get("id") for p in parts]
    puck_ids = [p.get("puck_id") for p in parts]
    names = [p.get("name") for p in parts]
    if len(parts) == 15 and sorted(ids) == list(range(1, 16)):
        ok("Configuration contains 15 uniquely numbered workpieces.")
    else:
        error(f"Expected 15 workpieces with IDs 1..15; found {ids}")
    if sorted(puck_ids) == list(range(1, 16)) and len(set(puck_ids)) == 15:
        ok("All 15 workpieces map to unique carrier pucks.")
    else:
        error("Workpiece-to-puck mapping is not one-to-one for IDs 1..15.")
    if len(set(names)) == 15 and all(n == f"part_{i:02d}" for i, n in enumerate(names, 1)):
        ok("Workpiece model names follow part_01..part_15.")
    else:
        error(f"Workpiece model names are inconsistent: {names}")

    colours = [p.get("color") for p in parts]
    if colours.count("RED") == 5 and colours.count("BLUE") == 5:
        distractors = [c for c in colours if c not in ("RED", "BLUE")]
        if len(distractors) == 5 and len(set(distractors)) == 5:
            ok("Workpiece mix is 5 RED, 5 BLUE and 5 distinct distractor colours.")
        else:
            error(f"Unexpected distractor mix: {distractors}")
    else:
        error(f"Unexpected target-colour mix: RED={colours.count('RED')}, BLUE={colours.count('BLUE')}")

    expected_perimeter = 2.0 * (cfg["y_front"] - cfg["y_rear"]) + 2.0 * math.pi * cfg["R_loop"]
    if close(cfg["P_loop"], expected_perimeter, 1e-4):
        ok("Configured conveyor perimeter matches two straights plus two semicircular turns.")
    else:
        error(f"P_loop mismatch: configured={cfg['P_loop']}, calculated={expected_perimeter}")

    # 4. Camera and conveyor world geometry vs JSON.
    camera = world.find(".//model[@name='overhead_vision_sensor']")
    if camera is None:
        error("Overhead camera model not found in SDF.")
    else:
        pose = [float(v) for v in child_text(camera, "pose", "0 0 0 0 0 0").split()]
        sensor = camera.find(".//sensor[@type='camera']")
        if all(close(a, b) for a, b in zip(pose[:3], [cfg["cam_x"], cfg["cam_y"], cfg["cam_z"]])):
            ok("Camera model position matches kinematics_config.json.")
        else:
            error(f"Camera position mismatch: SDF={pose[:3]}, config={[cfg['cam_x'],cfg['cam_y'],cfg['cam_z']]}")
        if sensor is not None:
            fov = float(child_text(sensor, "camera/horizontal_fov", "0"))
            width = int(child_text(sensor, "camera/image/width", "0"))
            height = int(child_text(sensor, "camera/image/height", "0"))
            rate = float(child_text(sensor, "update_rate", "0"))
            if close(fov, cfg["fov_h"]):
                ok("Camera horizontal FOV matches configuration.")
            else:
                error(f"Camera FOV mismatch: SDF={fov}, config={cfg['fov_h']}")
            ok(f"Camera image dimensions and rate: {width}x{height} at {rate:g} Hz.")
        else:
            error("Camera sensor element is missing.")

    conveyor = world.find(".//model[@name='conveyor_belt']")
    if conveyor is None:
        error("conveyor_belt model not found in SDF.")
    else:
        pose = [float(v) for v in child_text(conveyor, "pose", "0 0 0 0 0 0").split()]
        if close(pose[0], cfg["belt_x"]) and close(pose[1], cfg["belt_y"]):
            ok("Conveyor world origin matches belt_x/belt_y.")
        else:
            error(f"Conveyor origin mismatch: SDF={pose[:2]}, config={[cfg['belt_x'],cfg['belt_y']]}")

    # 5. Physics settings and carrier / detachable-joint inventory.
    physics = world.find(".//physics")
    if physics is not None:
        step = float(child_text(physics, "max_step_size", "0"))
        rate = float(child_text(physics, "real_time_update_rate", "0"))
        if close(step, 0.002) and close(rate, 500.0) and close(step * rate, 1.0):
            ok("Gazebo physics is configured for 2 ms steps / 500 Hz target.")
        else:
            warn(f"Physics settings are step={step}s, update_rate={rate}Hz; verify documentation.")
    else:
        error("SDF physics block is missing.")

    puck_links = {e.attrib.get("name") for e in world.findall(".//link") if e.attrib.get("name")}
    missing_pucks = [f"puck_{i}" for i in range(1, 16) if f"puck_{i}" not in puck_links]
    if not missing_pucks:
        ok("SDF contains all 15 puck links.")
    else:
        error(f"Missing carrier puck links: {missing_pucks}")

    weld_models = re.findall(r"<child_model>part_(\d{2})</child_model>", xacro_text)
    if sorted(weld_models) == [f"{i:02d}" for i in range(1, 16)]:
        ok("URDF/Xacro declares detachable-joint plugins for all 15 workpieces.")
    else:
        error(f"Detachable-joint plugin inventory is incomplete: {weld_models}")

    # 6. ROS topics, controllers, launch bridges and install metadata.
    for jn in expected_rev:
        if f"<joint_name>{jn}</joint_name>" not in xacro_text:
            error(f"Gazebo position controller missing for {jn}.")
        if f"/jork_arm/{jn}/cmd_pos" not in script_text or f"/jork_arm/{jn}/cmd_pos" not in launch_text:
            error(f"Command topic for {jn} is not consistently present in node and launch bridge.")
    for key, jn in (("fl", "finger_left"), ("fr", "finger_right")):
        topic = f"/jork_arm/{jn}/cmd_pos"
        if topic not in script_text or topic not in launch_text:
            error(f"Gripper command topic missing from script or launch bridge: {topic}")
    if ("f'/conveyor/puck_{pid}/cmd_x'" in script_text and
        "f'/conveyor/puck_{pid}/cmd_y'" in script_text and
        "f'/conveyor/puck_{pid}/cmd_x" in launch_text and
        "f'/conveyor/puck_{pid}/cmd_y" in launch_text and
        "range(1, 16)" in script_text and "range(1, 16)" in launch_text):
        ok("All 30 carrier command topics are generated consistently for puck IDs 1..15.")
    else:
        error("Carrier X/Y topic templates or 1..15 loops are inconsistent between node and launch bridge.")

    cmake = CMAKE_PATH.read_text(encoding="utf-8")
    if "scripts/vision_sorting_cell.py" in cmake and "scripts/check_project_consistency.py" in cmake:
        ok("CMake installs the sorting node and consistency checker.")
    else:
        error("CMake must install both the sorting node and consistency checker.")

    exec_deps = {e.text for e in package.findall("exec_depend") if e.text}
    required_deps = {"rclpy", "std_msgs", "sensor_msgs", "ament_index_python", "python3-numpy", "rosgraph_msgs"}
    missing_deps = sorted(required_deps - exec_deps)
    if not missing_deps:
        ok("package.xml declares the ROS/Python runtime dependencies used by the sorting node.")
    else:
        error(f"package.xml missing runtime dependencies: {missing_deps}")

    # 7. Explicit engineering-review warnings (not static-test failures).
    friction_values = [float(v) for v in re.findall(r"<mu[12]>([0-9.]+)</mu[12]>", xacro_text)]
    if friction_values and max(friction_values) >= 10.0:
        warn(f"Finger contact friction is configured as high as {max(friction_values):g}; treat as simulation-tuned, not physical validation.")
    j2_limit = joints.get("joint_2").find("limit") if joints.get("joint_2") is not None else None
    if j2_limit is not None:
        effort = float(j2_limit.attrib.get("effort", "0"))
        if effort > 10.71:
            warn(f"joint_2 Gazebo/URDF effort limit is {effort:g} N m, above the 10.71 N m reference output estimate; actuator saturation is not represented.")
    if "expected_color" in script_text and "detected_color = cam_det['color'] if (cam_det is not None) else expected_color" in script_text:
        warn("No-detection path falls back to configured workpiece colour; camera-only classification is not tested.")
    if "held = welded" in script_text:
        warn("Finger-position grasp check is overwritten by detachable-joint confirmation; contact grasp is not verified.")
    if "held_model = None" in script_text:
        warn("Attached-object TCP pose synchronisation is disabled in the current sorting path.")
    if "cam_det['pos']" in script_text and "px = self.belt_x + xr" in script_text:
        warn("Pickup XY is model/carrier-derived; camera centroid is calculated but not used as the manipulator target.")
    if "cam_det['angle_rad']" in script_text and "solve_ik(px, py, z_now, roll=roll)" in script_text:
        warn("Image principal-axis angle is calculated but is not connected to wrist-roll control.")

    return finish()


def finish():
    print("\\n=== SORTING CELL CONSISTENCY CHECK ===")
    for item in passed:
        print("[PASS] " + item)
    for item in warnings:
        print("[WARN] " + item)
    for item in errors:
        print("[FAIL] " + item)
    print(f"\\nResult: {len(passed)} passed, {len(warnings)} warnings, {len(errors)} failures.")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
