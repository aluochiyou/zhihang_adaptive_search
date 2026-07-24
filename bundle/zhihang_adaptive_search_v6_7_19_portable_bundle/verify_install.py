#!/usr/bin/env python3
from pathlib import Path
import ast
import sys
import xml.etree.ElementTree as ET
import yaml

ws = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.home() / 'xtdrone_competition_ws'
pkg = ws / 'src/zhihang_adaptive_search_v6'
failed = []


def check(name, ok):
    print(('PASS ' if ok else 'FAIL ') + name)
    if not ok:
        failed.append(name)


check('installed ROS package exists', pkg.is_dir())
if pkg.is_dir():
    check('installed version 6.7.19', '<version>6.7.19</version>' in
          (pkg / 'package.xml').read_text(encoding='utf-8'))
    files = [
        pkg / 'scripts/mission_manager.py',
        pkg / 'scripts/vehicle_flight_agent.py',
        pkg / 'scripts/vehicle_perception_agent.py',
        pkg / 'scripts/vision_target_state_estimator.py',
        pkg / 'scripts/yolo26_single_worker.py',
        pkg / 'src/zhihang_adaptive_search_v6/target_localization.py',
        pkg / 'src/zhihang_adaptive_search_v6/tracking_recovery.py',
    ]
    for path in files:
        try:
            ast.parse(path.read_text(encoding='utf-8')); ok = True
        except Exception:
            ok = False
        check('syntax ' + path.name, ok)
    for path in (pkg / 'launch').glob('*.launch'):
        try:
            ET.parse(path); ok = True
        except Exception:
            ok = False
        check('launch XML ' + path.name, ok)
    cfg = yaml.safe_load((pkg / 'config/adaptive_search.yaml').read_text(encoding='utf-8'))
    formal = yaml.safe_load((pkg / 'config/adaptive_search_formal.yaml').read_text(encoding='utf-8'))
    flight = (pkg / 'scripts/vehicle_flight_agent.py').read_text(encoding='utf-8')
    manager = (pkg / 'scripts/mission_manager.py').read_text(encoding='utf-8')
    perception = (pkg / 'scripts/vehicle_perception_agent.py').read_text(encoding='utf-8')
    estimator = (pkg / 'scripts/vision_target_state_estimator.py').read_text(encoding='utf-8')
    worker = (pkg / 'scripts/yolo26_single_worker.py').read_text(encoding='utf-8')
    check('truth-free formal visual estimator installed',
          'target_localization_report' in estimator and '/gazebo/model_states' not in estimator)
    check('30m dynamic FW-to-MC gate retained',
          cfg['tracking']['transition_to_mc_radius_m'] == 30.0 and
          formal['tracking']['transition_to_mc_radius_m'] == 30.0 and
          'TRACK_FW_30M_TRANSITION_GATE_REACHED' in flight)
    check('fixed-wing HOLD/invalid-setpoint recovery installed',
          'FW_ZERO_VELOCITY_HANDOVER_BLOCKED' in flight and
          'OFFBOARD_HOLD_RECOVERY_ATTEMPT' in flight and
          'STATIC_VERIFY_HOLD_GUARD_TRIGGERED' in flight)
    check('dynamic yaw/loss continuation protection installed',
          'protected_tracking_yaw_rate' in flight and
          'TRACK_LOSS_CONTINUE_TO_LAST_KNOWN' in flight and
          'TRACK_LAST_KNOWN_POSITION_REACHED_CONTINUE_MOTION' in flight and
          formal['tracking']['yaw_adjust_enable_radius_m'] == 20.0)
    check('candidate-aware static management installed',
          'STATIC_CANDIDATE_GROUP_CREATED' in manager and
          'STATIC_CANDIDATE_PRECISE_CONFIRMED' in flight and
          'STATIC_CANDIDATE_REJECTED' in flight and
          formal['static_candidate_management']['nearby_merge_radius_m'] == 25.0)
    check('prius/SUV confusion firewall installed',
          'PRIUS_CAMO_UPDATE_REJECTED_NEAR_SUV_CAMO' in manager and
          formal['static_candidate_management']['prius_suv_confusion']['exclusion_radius_m'] == 60.0)
    check('rejected candidate original image saving installed',
          'static_rejection_images' in perception and
          'image_is_unannotated_original_camera_frame' in perception)
    check('portable YOLO auto-device/quantize fallback installed',
          "requested_device == 'auto'" in worker and
          'runtime rejected quantize' in worker)
    check('existing spiral/motion/static requirements retained',
          formal['tracking']['reacquisition_lane_spacing_m'] == 90.0 and
          formal['tracking']['reacquisition_speed_mps'] == 6.0 and
          formal['tracking']['motion_validation']['observation_seconds'] == 5.0 and
          formal['static_verify']['hover_altitude_m'] == 30.0 and
          formal['static_verify']['stable_seconds'] == 3.0)
    check('workspace overlay exists', (ws / 'devel/setup.bash').is_file())

print('ALL V6.7.19 INSTALL CHECKS PASSED' if not failed else 'V6.7.19 INSTALL CHECKS FAILED')
sys.exit(1 if failed else 0)
