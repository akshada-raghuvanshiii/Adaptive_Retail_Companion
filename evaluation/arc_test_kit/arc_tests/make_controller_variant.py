#!/usr/bin/env python3
"""Write a copy of the TEB-like controller params with a stock Nav2 controller (A3).

Your teb_controller.yaml is read, never modified. Every key is kept (frames,
costmap, goal checker, plugin id) and only the controller plugin block is
swapped, so planned_path_to_teb keeps calling the same FollowPath id ('TEB').
The new block takes the TEB block's max_vel_x / max_vel_theta, so the
comparison is between controllers at equal speed limits.

    python3 arc_tests/make_controller_variant.py \
        --in ~/slam_ws/src/arc_planner/config/teb_controller.yaml \
        --out ~/arc_test_kit/configs/rpp_controller.yaml --type rpp
"""
import argparse
import copy
import os
import sys

import yaml

RPP = {
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "desired_linear_vel": 0.15,
    "lookahead_dist": 0.4,
    "min_lookahead_dist": 0.25,
    "max_lookahead_dist": 0.7,
    "lookahead_time": 1.5,
    "rotate_to_heading_angular_vel": 1.0,
    "transform_tolerance": 0.2,
    "use_velocity_scaled_lookahead_dist": False,
    "min_approach_linear_velocity": 0.05,
    "approach_velocity_scaling_dist": 0.4,
    "use_collision_detection": True,
    "max_allowed_time_to_collision_up_to_carrot": 1.0,
    "use_regulated_linear_velocity_scaling": True,
    "use_cost_regulated_linear_velocity_scaling": False,
    "regulated_linear_scaling_min_radius": 0.9,
    "regulated_linear_scaling_min_speed": 0.1,
    "use_rotate_to_heading": True,
    "rotate_to_heading_min_angle": 0.785,
    "allow_reversing": False,
    "max_angular_accel": 3.2,
    "max_robot_pose_search_dist": 10.0,
}

DWB = {
    "plugin": "dwb_core::DWBLocalPlanner",
    "min_vel_x": 0.0, "max_vel_x": 0.18, "min_vel_y": 0.0, "max_vel_y": 0.0,
    "max_vel_theta": 1.0, "min_speed_xy": 0.0, "max_speed_xy": 0.18, "min_speed_theta": 0.0,
    "acc_lim_x": 2.5, "acc_lim_y": 0.0, "acc_lim_theta": 3.2,
    "decel_lim_x": -2.5, "decel_lim_y": 0.0, "decel_lim_theta": -3.2,
    "vx_samples": 20, "vy_samples": 0, "vtheta_samples": 20, "sim_time": 1.5,
    "linear_granularity": 0.05, "angular_granularity": 0.025, "transform_tolerance": 0.2,
    "xy_goal_tolerance": 0.25, "trans_stopped_velocity": 0.25, "short_circuit_trajectory_evaluation": True,
    "stateful": True,
    "critics": ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"],
    "BaseObstacle.scale": 0.02, "PathAlign.scale": 32.0, "PathAlign.forward_point_distance": 0.1,
    "GoalAlign.scale": 24.0, "GoalAlign.forward_point_distance": 0.1, "PathDist.scale": 32.0,
    "GoalDist.scale": 24.0, "RotateToGoal.scale": 32.0, "RotateToGoal.slowing_factor": 5.0,
    "RotateToGoal.lookahead_time": -1.0,
}


def find_controller_params(doc):
    """Return the ros__parameters dict of the controller_server, wherever it sits."""
    if isinstance(doc, dict):
        if "controller_plugins" in doc:
            return doc
        for v in doc.values():
            got = find_controller_params(v)
            if got is not None:
                return got
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--type", choices=["rpp", "dwb", "progress"], default="rpp")
    ap.add_argument("--radius", type=float, default=0.25, help="progress: required_movement_radius (m)")
    ap.add_argument("--allowance", type=float, default=20.0, help="progress: movement_time_allowance (s)")
    a = ap.parse_args()
    src = os.path.expanduser(a.src)
    with open(src) as f:
        doc = yaml.safe_load(f)
    out = copy.deepcopy(doc)
    params = find_controller_params(out)
    if params is None:
        sys.exit("No 'controller_plugins' found in that file - is it the controller_server params?")
    if a.type == "progress":
        # Phase 2: the same TEB-like controller, only the progress check is relaxed. ARC's file defines no
        # progress checker, so Nav2 uses its default (SimpleProgressChecker, 0.5 m within 10 s).
        for k in ("progress_checker_plugin", "progress_checker_plugins", "progress_checker"):
            if k in params:
                print(f"note: '{k}' already set in the source ({params[k]}); overriding")
        params["progress_checker_plugins"] = ["progress_checker"]
        params["progress_checker"] = {"plugin": "nav2_controller::SimpleProgressChecker",
                                      "required_movement_radius": float(a.radius),
                                      "movement_time_allowance": float(a.allowance)}
        print(f"progress check: {a.radius} m within {a.allowance} s (Nav2 default: 0.5 m within 10 s)")
        write(out, a.out, src)
        return
    plugins = params["controller_plugins"]
    if isinstance(plugins, str):
        plugins = [plugins]
    block = RPP if a.type == "rpp" else DWB
    for pid in plugins:
        old = params.get(pid, {})
        print(f"replacing plugin '{pid}': {old.get('plugin', '?')} -> {block['plugin']}")
        new = dict(block)
        # Same speed limits as the TEB-like controller, so A3 compares controllers, not speeds.
        vmax = old.get("max_vel_x")
        wmax = old.get("max_vel_theta")
        if vmax is not None:
            if a.type == "rpp":
                new["desired_linear_vel"] = float(vmax)
            else:
                new["max_vel_x"] = new["max_speed_xy"] = float(vmax)
        if wmax is not None:
            if a.type == "rpp":
                new["rotate_to_heading_angular_vel"] = float(wmax)
            else:
                new["max_vel_theta"] = float(wmax)
        alim = old.get("acc_lim_theta")
        if alim is not None:
            if a.type == "rpp":
                new["max_angular_accel"] = float(alim)
            else:
                new["acc_lim_theta"], new["decel_lim_theta"] = float(alim), -float(alim)
        print(f"  limits copied from {pid}: max_vel_x={vmax}, max_vel_theta={wmax}, acc_lim_theta={alim}")
        params[pid] = new
    write(out, a.out, src)


def write(doc, path, src):
    os.makedirs(os.path.dirname(os.path.abspath(os.path.expanduser(path))), exist_ok=True)
    with open(os.path.expanduser(path), "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"wrote {path} (original untouched: {src})")


if __name__ == "__main__":
    main()
