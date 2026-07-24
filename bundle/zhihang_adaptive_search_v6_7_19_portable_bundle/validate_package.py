#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import ast
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

B = Path(__file__).resolve().parent
P = B / "zhihang_adaptive_search_v6"
failed = []


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (f" :: {detail}" if detail else ""))
    if not ok:
        failed.append(name)


# Syntax and serialization checks.
syntax_ok = True
for path in B.rglob("*.py"):
    try:
        ast.parse(path.read_text(encoding="utf-8"))
    except Exception as exc:
        syntax_ok = False
        print(f"Python syntax failure {path}: {exc}")
check("all Python files parse", syntax_ok)

shell_ok = True
for path in B.rglob("*.sh"):
    result = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
    if result.returncode:
        shell_ok = False
        print(f"Shell syntax failure {path}: {result.stderr.strip()}")
check("all Shell files parse", shell_ok)

launch_ok = True
for path in P.glob("launch/*.launch"):
    try:
        ET.parse(path)
    except Exception as exc:
        launch_ok = False
        print(f"Launch XML failure {path}: {exc}")
check("all ROS launch XML parses", launch_ok)

configs = []
for name in ("adaptive_search.yaml", "adaptive_search_formal.yaml"):
    try:
        configs.append(yaml.safe_load((P / "config" / name).read_text(encoding="utf-8")))
    except Exception as exc:
        print(f"YAML failure {name}: {exc}")
check("both YAML configurations parse", len(configs) == 2)
check("bundle version is 6.7.19", (B / "VERSION").read_text().strip() == "6.7.19")
check("ROS package version is 6.7.19",
      "<version>6.7.19</version>" in (P / "package.xml").read_text(encoding="utf-8"))

# Core behavior retained from V6.7.18.
flight = (P / "scripts/vehicle_flight_agent.py").read_text(encoding="utf-8")
manager = (P / "scripts/mission_manager.py").read_text(encoding="utf-8")
perception = (P / "scripts/vehicle_perception_agent.py").read_text(encoding="utf-8")
estimator = (P / "scripts/vision_target_state_estimator.py").read_text(encoding="utf-8")
worker = (P / "scripts/yolo26_single_worker.py").read_text(encoding="utf-8")
manager_launch = (P / "launch/manager.launch").read_text(encoding="utf-8")

check("formal estimator remains visual-only and truth-free",
      "target_localization_report" in estimator and
      "/gazebo/model_states" not in estimator and "ModelStates" not in estimator)
check("formal/validation estimator separation retained",
      "vision_target_state_estimator.py" in manager_launch and
      "validation_target_state_relay.py" in manager_launch)
check("30 m dynamic FW-to-MC gate retained",
      "TRACK_FW_30M_TRANSITION_GATE_REACHED" in flight and
      "'transition_to_mc_radius_m', 30.0" in flight)
check("fixed-wing zero-velocity/HOLD guard retained",
      "FW_ZERO_VELOCITY_HANDOVER_BLOCKED" in flight and
      "OFFBOARD_HOLD_RECOVERY_ATTEMPT" in flight and
      "STATIC_VERIFY_HOLD_GUARD_TRIGGERED" in flight)
check("dynamic yaw/loss recovery retained",
      "protected_tracking_yaw_rate" in flight and
      "TRACK_LOSS_CONTINUE_TO_LAST_KNOWN" in flight and
      "TRACK_LAST_KNOWN_POSITION_REACHED_CONTINUE_MOTION" in flight)
check("candidate-aware static management retained",
      "STATIC_CANDIDATE_GROUP_CREATED" in manager and
      "STATIC_CANDIDATE_NEARBY_MERGED" in manager and
      "pending_static_candidate_rows" in manager)
check("raw-image rejection evidence retained",
      "STATIC_CANDIDATE_REJECTED" in flight and
      "static_rejection_images" in perception and
      "image_is_unannotated_original_camera_frame" in perception)
check("prius_hybrid_camo / suv_camo firewall retained",
      "PRIUS_CAMO_UPDATE_REJECTED_NEAR_SUV_CAMO" in manager and
      "_prius_camo_conflicts_with_suv" in manager and
      "SuvMotionGate" in flight)
check("manager remains free of target truth",
      "/gazebo/model_states" not in manager and "ModelStates" not in manager)
competition_bridge = (P / "scripts/competition_result_publisher.py").read_text(encoding="utf-8")
check("official competition result bridge installed",
      "competition_result_publisher.py" in manager_launch and
      "header_timestamp" in competition_bridge and
      "categories" in competition_bridge and
      "/zhihang2026/static_targets/pose" in competition_bridge and
      "/zhihang2026/dynamic_targets/pose" in competition_bridge)
check("manager publishes one finalized official result payload",
      "build_competition_final_results" in manager and
      "competition_final_results" in manager and
      "model_name': 'static_target'" in manager)
check("official output keeps partial scoreable results",
      "best_available_detection" in manager and
      "external_target_states.get(name" in manager and
      "if not static_rows and not dynamic_rows" in competition_bridge)

# Portability framework.
required_portable = [
    "machine_profile.example.env",
    "load_machine_profile.sh",
    "configure_portable_machine.py",
    "configure_portable_machine.sh",
    "yolo_runtime_common.sh",
    "setup_yolo_runtime.sh",
    "verify_yolo_runtime.py",
    "doctor_portable.sh",
    "competition_readiness_check.py",
    "launch_portable_formal_one_click.sh",
    "install_portable.sh",
    "export_yolo_runtime_manifest.sh",
    "benchmark_three_yolo_capacity.sh",
]
for name in required_portable:
    check(f"portable file exists: {name}", (B / name).is_file())

profile_loader = (B / "load_machine_profile.sh").read_text(encoding="utf-8")
runtime_common = (B / "yolo_runtime_common.sh").read_text(encoding="utf-8")
terminal_common = (B / "terminal_launcher_common.sh").read_text(encoding="utf-8")
runner = (B / "run_vehicle_terminal.sh").read_text(encoding="utf-8")
launcher = (B / "launch_mission_formal_one_click.sh").read_text(encoding="utf-8")

check("machine profile supports variable workspace/XTDrone/QGC",
      "ZHIHANG_WS" in profile_loader and
      "ZHIHANG_XTDRONE_ROOT" in profile_loader and
      "ZHIHANG_QGC_EXECUTABLE" in profile_loader)
check("YOLO supports conda/venv/system/direct",
      all(x in runtime_common for x in ("conda)", "venv)", "system)", "direct)")))
check("terminal supports desktop and tmux backends",
      all(x in terminal_common for x in ("gnome-terminal)", "xterm)", "konsole)", "tmux)")))
check("vehicle runner uses profile-selected device/settings",
      "zh_resolve_device_for_vehicle" in runner and
      "ZHIHANG_YOLO_ADAPTIVE_SIZES" in runner and
      "ZHIHANG_YOLO_START_LOCAL_WORKERS" in runner)
check("YOLO worker auto-device and quantize fallback installed",
      "requested_device == 'auto'" in worker and
      "runtime rejected quantize" in worker)
check("formal launcher preserves path normalization and real-pipeline barrier",
      "zh_resolve_model_path" in launcher and
      "ARG-NORMALIZED" in launcher and
      "three real YOLO camera pipelines" in launcher)

for cfg, label in zip(configs, ("validation", "formal")):
    t = cfg["tracking"]
    sv = cfg["static_verify"]
    cm = cfg["static_candidate_management"]
    check(f"{label} dynamic transition gate is 30 m",
          float(t["transition_to_mc_radius_m"]) == 30.0)
    check(f"{label} 90 m / 6 mps square-spiral recovery",
          float(t["reacquisition_lane_spacing_m"]) == 90.0 and
          float(t["reacquisition_speed_mps"]) == 6.0 and
          t.get("reacquisition_pattern") == "square_spiral")
    check(f"{label} static precise-hover requirements",
          float(sv["hover_altitude_m"]) == 30.0 and
          float(sv["hover_xy_tolerance_m"]) == 5.0 and
          float(sv["hover_z_tolerance_m"]) == 1.0 and
          float(sv["hover_yaw_deg"]) == 0.0 and
          float(sv["stable_seconds"]) == 3.0)
    check(f"{label} candidate grouping/confusion firewall",
          float(cm["nearby_merge_radius_m"]) == 25.0 and
          int(cm["maximum_candidates_per_target"]) == 5 and
          float(cm["prius_suv_confusion"]["exclusion_radius_m"]) == 60.0)

# Run pure tests only; no ROS master, GPU or model is required.
for test in sorted((B / "tests").glob("test_*.py")):
    result = subprocess.run([sys.executable, str(test)], capture_output=True, text=True)
    if result.stdout:
        print(result.stdout.rstrip())
    if result.stderr:
        print(result.stderr.rstrip())
    check(f"runtime test {test.name}", result.returncode == 0)

for test in sorted((B / "tests").glob("test_*.sh")):
    result = subprocess.run(["bash", str(test)], capture_output=True, text=True)
    if result.stdout:
        print(result.stdout.rstrip())
    if result.stderr:
        print(result.stderr.rstrip())
    check(f"shell regression {test.name}", result.returncode == 0)

# Competition requirements encoded in formal YAML and source.
readiness = subprocess.run(
    [sys.executable, str(B / "competition_readiness_check.py")],
    capture_output=True,
    text=True,
)
if readiness.stdout:
    print(readiness.stdout.rstrip())
if readiness.stderr:
    print(readiness.stderr.rstrip())
check("competition-readiness static checks", readiness.returncode == 0)

if failed:
    print("V6.7.19 CHECKS FAILED:", ", ".join(failed))
    sys.exit(1)
print("ALL V6.7.19 CHECKS PASSED")
