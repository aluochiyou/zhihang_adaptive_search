#!/usr/bin/env python3
"""Static and optional live competition-readiness checks for V6.7.19."""
from __future__ import annotations

import argparse
import ast
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml


B = Path(__file__).resolve().parent
P = B / "zhihang_adaptive_search_v6"


def nested(data: Dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        cur = cur[key]
    return cur


def run(args: List[str], timeout: float = 8.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(P / "config/adaptive_search_formal.yaml"),
    )
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--report", default=str(B / "COMPETITION_READINESS_RESULT.json"))
    args = parser.parse_args()

    cfg_path = Path(args.config).expanduser().resolve()
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    checks: List[Tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append((name, bool(ok), detail))

    static_targets = nested(cfg, "perception", "static_targets")
    dynamic_targets = nested(cfg, "perception", "dynamic_targets")

    check("three enabled aircraft",
          nested(cfg, "mission", "enabled_vehicle_ids") == [0, 1, 2],
          str(nested(cfg, "mission", "enabled_vehicle_ids")))
    check("six static targets", len(static_targets) == 6, str(static_targets))
    check("two dynamic targets", len(dynamic_targets) == 2, str(dynamic_targets))
    check("formal source is YOLO projected",
          nested(cfg, "perception", "selected_result_source") == "yolo_projected",
          str(nested(cfg, "perception", "selected_result_source")))
    check("formal truth relay disabled",
          not bool(nested(cfg, "validation", "truth_target_relay_enabled")),
          str(nested(cfg, "validation", "truth_target_relay_enabled")))
    check("validation evaluator disabled in formal mode",
          not bool(nested(cfg, "perception", "detection_event_validation", "enabled")),
          str(nested(cfg, "perception", "detection_event_validation", "enabled")))
    check("real perception minimum is 10 Hz",
          float(nested(cfg, "perception", "minimum_required_fps")) >= 10.0,
          str(nested(cfg, "perception", "minimum_required_fps")))
    check("32 minute search budget",
          float(nested(cfg, "mission", "mission_timeout_seconds")) == 1920.0,
          str(nested(cfg, "mission", "mission_timeout_seconds")))
    check("3 minute return reserve",
          float(nested(cfg, "return_strategy", "reserved_return_seconds")) == 180.0,
          str(nested(cfg, "return_strategy", "reserved_return_seconds")))
    check("35 minute total limit",
          float(nested(cfg, "return_strategy", "competition_total_limit_seconds")) == 2100.0,
          str(nested(cfg, "return_strategy", "competition_total_limit_seconds")))

    tracking = nested(cfg, "tracking")
    check("dynamic FW-to-MC gate is 30 m",
          float(tracking["transition_to_mc_radius_m"]) == 30.0,
          str(tracking["transition_to_mc_radius_m"]))
    check("dynamic reacquisition square spiral",
          tracking.get("reacquisition_pattern") == "square_spiral",
          str(tracking.get("reacquisition_pattern")))
    check("dynamic reacquisition spacing 90 m",
          float(tracking["reacquisition_lane_spacing_m"]) == 90.0,
          str(tracking["reacquisition_lane_spacing_m"]))
    check("dynamic reacquisition speed 6 m/s",
          float(tracking["reacquisition_speed_mps"]) == 6.0,
          str(tracking["reacquisition_speed_mps"]))
    check("cross-aircraft target injection enabled",
          bool(nested(cfg, "tracking", "external_injection", "enabled")),
          str(nested(cfg, "tracking", "external_injection", "enabled")))

    sv = nested(cfg, "static_verify")
    check("static hover altitude 30 m", float(sv["hover_altitude_m"]) == 30.0,
          str(sv["hover_altitude_m"]))
    check("static hover XY tolerance 5 m", float(sv["hover_xy_tolerance_m"]) == 5.0,
          str(sv["hover_xy_tolerance_m"]))
    check("static hover Z tolerance 1 m", float(sv["hover_z_tolerance_m"]) == 1.0,
          str(sv["hover_z_tolerance_m"]))
    check("static hover yaw 0 deg", float(sv["hover_yaw_deg"]) == 0.0,
          str(sv["hover_yaw_deg"]))
    check("static hover stable 3 s", float(sv["stable_seconds"]) == 3.0,
          str(sv["stable_seconds"]))
    check("person_white west offset 10 m",
          sv["target_hover_offset_xy_m"].get("person_white") == [-10.0, 0.0],
          str(sv["target_hover_offset_xy_m"].get("person_white")))
    check("static HOLD recovery enabled", bool(sv["hold_recovery_enabled"]),
          str(sv["hold_recovery_enabled"]))

    manager = (P / "scripts/mission_manager.py").read_text(encoding="utf-8")
    estimator = (P / "scripts/vision_target_state_estimator.py").read_text(encoding="utf-8")
    flight = (P / "scripts/vehicle_flight_agent.py").read_text(encoding="utf-8")
    check("manager has no Gazebo target-truth subscription",
          "/gazebo/model_states" not in manager and "ModelStates" not in manager)
    check("formal estimator has no Gazebo target truth",
          "/gazebo/model_states" not in estimator and "ModelStates" not in estimator)
    check("first-ARMED competition clock retained",
          "competition clock started at first ARMED" in manager)
    check("maximum 32-minute return trigger retained",
          "maximum 32-minute search budget reached from first ARMED" in manager)
    check("invalid fixed-wing zero-setpoint guard retained",
          "FW_ZERO_VELOCITY_HANDOVER_BLOCKED" in flight)
    check("forced HOLD recovery retained",
          "OFFBOARD_HOLD_RECOVERY_ATTEMPT" in flight)
    bridge = (P / "scripts/competition_result_publisher.py").read_text(encoding="utf-8")
    manager_launch = (P / "launch/manager.launch").read_text(encoding="utf-8")
    bag_script = (B / "terminal_commands/06_score1_bag.sh").read_text(encoding="utf-8")
    check("official static/dynamic topic bridge installed",
          "competition_result_publisher.py" in manager_launch and
          "/zhihang2026/static_targets/pose" in bridge and
          "/zhihang2026/dynamic_targets/pose" in bridge)
    check("official final payload is category/endpoints based",
          "build_competition_final_results" in manager and
          "category_static_target" in manager and
          "start_and_end" in manager)
    check("partial official result does not suppress scoreable component",
          "best_available_detection" in manager and
          "external_target_states.get(name" in manager and
          "if not static_rows and not dynamic_rows" in bridge)
    required_bag_topics = [
        "/standard_vtol_0/mavros/state",
        "/standard_vtol_1/mavros/state",
        "/standard_vtol_2/mavros/state",
        "/gazebo/model_states",
        "/xtdrone/standard_vtol_0/cmd",
        "/xtdrone/standard_vtol_1/cmd",
        "/xtdrone/standard_vtol_2/cmd",
        "/zhihang2026/static_targets/pose",
        "/zhihang2026/dynamic_targets/pose",
    ]
    check("score1 bag contains every official required topic",
          all(topic in bag_script for topic in required_bag_topics))

    protected = [
        Path(os.environ.get("ZHIHANG_COMMUNICATION_DIR",
                            str(Path.home() / "XTDrone/communication")))
        / "vtol_communication.py",
        Path(os.environ.get("ZHIHANG_COMMUNICATION_DIR",
                            str(Path.home() / "XTDrone/communication")))
        / "multi_vehicle_communication.sh",
        Path(os.environ.get("ZHIHANG_COMMUNICATION_DIR",
                            str(Path.home() / "XTDrone/communication")))
        / "multi_vehicle_commonication.sh",
    ]
    package_paths = {p.resolve() for p in B.rglob("*") if p.is_file()}
    check("protected XTDrone communication files are not packaged",
          all(p.resolve() not in package_paths for p in protected))

    if args.live:
        master = run(["bash", "-lc", "rostopic list >/dev/null 2>&1"])
        check("live ROS master reachable", master.returncode == 0,
              master.stderr.strip())
        if master.returncode == 0:
            topics = run(["bash", "-lc", "rostopic list"])
            available = set(topics.stdout.splitlines())
            required_topics = []
            for vid in (0, 1, 2):
                required_topics.extend([
                    f"/standard_vtol_{vid}/mavros/state",
                    f"/standard_vtol_{vid}/mavros/local_position/pose",
                    f"/standard_vtol_{vid}/camera/image_raw",
                ])
            missing = [t for t in required_topics if t not in available]
            check("three MAVROS/pose/camera pipelines visible",
                  not missing, f"missing={missing}")

    failed = [name for name, ok, _ in checks if not ok]
    report = {
        "version": "6.7.19",
        "config": str(cfg_path),
        "live": args.live,
        "checks": [
            {"name": name, "pass": ok, "detail": detail}
            for name, ok, detail in checks
        ],
        "passed": not failed,
        "failed": failed,
    }
    Path(args.report).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    for name, ok, detail in checks:
        print(f"{'PASS' if ok else 'FAIL'} {name}" + (f" :: {detail}" if detail else ""))
    if failed:
        print("[ERROR] competition-readiness checks failed:", ", ".join(failed))
        return 2
    print("[OK] all competition-readiness checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
