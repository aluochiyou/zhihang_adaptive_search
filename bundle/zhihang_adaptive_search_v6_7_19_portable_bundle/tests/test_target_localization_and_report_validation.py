#!/usr/bin/env python3
"""ROS-free runtime test for V6.7.11 localization and report validation."""
from pathlib import Path
import json
import sys
import tempfile
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'zhihang_adaptive_search_v6'

# Minimal std_msgs stub used by the pure localization module.
std_msgs = types.ModuleType('std_msgs')
std_msgs_msg = types.ModuleType('std_msgs.msg')
class String:
    def __init__(self, data=''):
        self.data = data
std_msgs_msg.String = String
std_msgs.msg = std_msgs_msg
sys.modules['std_msgs'] = std_msgs
sys.modules['std_msgs.msg'] = std_msgs_msg

sys.path.insert(0, str(PKG / 'src'))
from zhihang_adaptive_search_v6.target_localization import (  # noqa: E402
    LocalizationEventValidator,
    TargetLocalizationTracker,
    localize_pixel_to_ground,
)


class Publisher:
    def __init__(self):
        self.rows = []

    def publish(self, msg):
        self.rows.append(json.loads(msg.data))


camera = {
    'width': 1280, 'height': 720,
    'fx': 369.502083, 'fy': 369.502083,
    'cx': 640.0, 'cy': 360.0,
    'ground_z_m': 0.2,
    'distortion_model': 'plumb_bob',
    'distortion_coefficients': [],
}
loc_cfg = {
    'enabled': True,
    'minimum_confidence': 0.50,
    'minimum_consecutive_frames': 3,
    'maximum_frame_gap_seconds': 0.35,
    'sample_window_frames': 6,
    'spatial_consistency_radius_m': 30.0,
    'maximum_horizontal_std_m': 12.0,
    'maximum_ground_range_m': 350.0,
    'image_anchor_mode': 'center',
    'dynamic_report_hz': 5.0,
    'static_repeat_seconds': 9999.0,
    'optical_to_body_matrix': [
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    'camera_position_body_m': [0.0, 0.0, 0.0],
    'management_source_names': ['yolo_projected'],
}
flight = {
    'phase': 'SEARCH_DYNAMIC_LEFT',
    'detection_valid': True,
    'world_position': [100.0, 200.0, 40.0],
    'world_yaw': 0.0,
    'world_attitude_rpy_rad': [0.0, 0.0, 0.0],
    'world_orientation_xyzw': [0.0, 0.0, 0.0, 1.0],
}

# Image centre must intersect directly below the aircraft.
projection = localize_pixel_to_ground(
    flight['world_position'], flight, camera, loc_cfg, 640.0, 360.0)
assert projection['valid'], projection
assert np.linalg.norm(np.asarray(projection['position_world']) -
                      np.asarray([100.0, 200.0, 0.2])) < 1e-6, projection

tracker = TargetLocalizationTracker(
    0, 'unit_localization', loc_cfg,
    ['person_red'], {'person_red': 'dynamic'},
    selected_result_source='yolo_projected')
image = np.zeros((720, 1280, 3), dtype=np.uint8)
detection = [{
    'class_name': 'person_red', 'confidence': 0.85,
    'xyxy': [620.0, 340.0, 660.0, 380.0],
    'center': [640.0, 360.0],
}]
reports = []
for index in range(3):
    reports = tracker.update(image, detection, 1.0 + index * 0.1,
                             index, flight, camera)
assert len(reports) == 1, reports
report = reports[0]
assert report['consecutive_frames'] >= 3
assert report['confidence'] >= 0.50
assert report['selected_as_management_result'] is True
assert np.linalg.norm(np.asarray(report['position_world']) -
                      np.asarray([100.0, 200.0, 0.2])) < 1e-6

# Event-level validation: correct, miss and wrong report with raw+annotated pairs.
output = Path(tempfile.mkdtemp(prefix='v6711_localization_'))
event_pub, summary_pub = Publisher(), Publisher()
validator = LocalizationEventValidator(
    0, 'unit_localization', {
        'enabled': True,
        'minimum_expected_visibility_seconds': 0.4,
        'minimum_expected_frames': 4,
        'event_gap_seconds': 0.2,
        'platform_exclusion_radius_m': 10.0,
        'minimum_evaluation_altitude_m': 15.0,
        'expected_fov_margin_ratio': 0.88,
        'report_confidence_threshold': 0.50,
        'correct_localization_error_m': 20.0,
        'minimum_correct_reports': 1,
        'wrong_report_minimum_reports': 1,
        'wrong_report_event_gap_seconds': 0.2,
        'save_correct_event_image': True,
        'save_raw_and_annotated_images': True,
    }, ['person_red'], {'person_red': 'dynamic'},
    event_pub, summary_pub, output, localization_cfg=loc_cfg)
home = np.asarray([-1000.0, -1000.0, 0.2])
positions = {
    'standard_vtol_0': np.asarray([100.0, 200.0, 40.0]),
    'person_red': np.asarray([100.0, 200.0, 0.2]),
}
# Correct event. One stable localization report is enough because it already
# represents three consecutive high-confidence frames.
for index in range(6):
    frame_reports = [report] if index == 2 else []
    validator.update(image, detection, frame_reports, 2.0 + index * 0.1,
                     index, flight, positions, camera, home)
positions['person_red'] = np.asarray([500.0, 500.0, 0.2])
for index in range(3):
    validator.update(image, [], [], 2.6 + index * 0.1,
                     10 + index, flight, positions, camera, home)

# Miss event.
positions['person_red'] = np.asarray([100.0, 200.0, 0.2])
for index in range(6):
    validator.update(image, [], [], 4.0 + index * 0.1,
                     20 + index, flight, positions, camera, home)
positions['person_red'] = np.asarray([500.0, 500.0, 0.2])
for index in range(3):
    validator.update(image, [], [], 4.6 + index * 0.1,
                     30 + index, flight, positions, camera, home)

# Wrong report when no valid target should appear.
wrong = dict(report)
wrong['ros_time'] = 6.0
wrong['source_ros_time'] = 6.0
for index in range(2):
    validator.update(image, detection, [wrong], 6.0 + index * 0.1,
                     40 + index, flight, positions, camera, home)
for index in range(3):
    validator.update(image, [], [], 6.3 + index * 0.1,
                     50 + index, flight, positions, camera, home)
validator.finalize('unit_complete')

summary = validator.summary()
counts = summary['counts']
assert counts['correct'] == 1, counts
assert counts['miss'] == 1, counts
assert counts['wrong_report'] == 1, counts
assert summary['localization']['evaluated_reports'] >= 1, summary
raw = list((output / 'detection_validation_events').glob('RAW_*.jpg'))
annotated = list((output / 'detection_validation_events').glob('ANNOTATED_*.jpg'))
metadata = list((output / 'detection_validation_events').glob('*.json'))
assert len(raw) >= 3, raw
assert len(annotated) >= 3, annotated
assert len(metadata) >= 3, metadata
print('PASS V6.7.11 target localization and event-level report validation')
