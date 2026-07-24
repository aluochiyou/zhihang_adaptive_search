#!/usr/bin/env python3
"""ROS-free unit test for the V6.7.9 event-level YOLO validator."""
from pathlib import Path
import importlib.util
import sys
import tempfile
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'zhihang_adaptive_search_v6'

# Minimal import stubs; the class under test itself uses only standard Python,
# NumPy and OpenCV after construction.
rospy = types.ModuleType('rospy')
rospy.is_shutdown = lambda: False
rospy.logwarn = lambda *args, **kwargs: None
rospy.logwarn_throttle = lambda *args, **kwargs: None
rospy.loginfo = lambda *args, **kwargs: None
rospy.Time = type('Time', (), {'now': staticmethod(
    lambda: types.SimpleNamespace(to_sec=lambda: 0.0, to_nsec=lambda: 0))})
sys.modules['rospy'] = rospy
bridge = types.ModuleType('cv_bridge')
bridge.CvBridge = object
sys.modules['cv_bridge'] = bridge
for base in ('gazebo_msgs', 'sensor_msgs', 'std_msgs'):
    parent = types.ModuleType(base)
    child = types.ModuleType(base + '.msg')
    parent.msg = child
    sys.modules[base] = parent
    sys.modules[base + '.msg'] = child
for module_name, class_name in (
    ('gazebo_msgs.msg', 'ModelStates'),
    ('sensor_msgs.msg', 'CameraInfo'),
    ('sensor_msgs.msg', 'Image'),
    ('std_msgs.msg', 'String'),
):
    setattr(sys.modules[module_name], class_name,
            type(class_name, (), {'__init__': lambda self, **kw: self.__dict__.update(kw)}))

sys.path.insert(0, str(PKG / 'src'))
spec = importlib.util.spec_from_file_location(
    'v677_perception', PKG / 'scripts/vehicle_perception_agent.py')
module = importlib.util.module_from_spec(spec)
sys.modules['v677_perception'] = module
assert spec.loader is not None
spec.loader.exec_module(module)


class Publisher:
    def __init__(self):
        self.rows = []

    def publish(self, msg):
        self.rows.append(msg.data)


output = Path(tempfile.mkdtemp(prefix='v677_validator_'))
event_pub, summary_pub = Publisher(), Publisher()
validator = module.DetectionEventValidator(
    0,
    'unit_test',
    {
        'enabled': True,
        'high_confidence_threshold': 0.60,
        'minimum_expected_visibility_seconds': 0.50,
        'minimum_expected_frames': 5,
        'correct_minimum_consecutive_frames': 3,
        'correct_minimum_frame_ratio': 0.25,
        'false_positive_minimum_consecutive_frames': 3,
        'false_positive_minimum_seconds': 0.20,
        'event_gap_seconds': 0.20,
        'platform_exclusion_radius_m': 10.0,
        'minimum_evaluation_altitude_m': 15.0,
        'expected_fov_margin_ratio': 0.88,
    },
    ['person_red'],
    {'person_red': 'dynamic'},
    event_pub,
    summary_pub,
    output,
)
image = np.zeros((720, 1280, 3), dtype=np.uint8)
camera = {
    'width': 1280, 'height': 720,
    'fx': 369.5, 'fy': 369.5, 'cx': 640.0, 'cy': 360.0,
    'u_right_sign': 1.0, 'v_forward_sign': -1.0, 'ground_z_m': 0.2,
}
flight = {
    'phase': 'SEARCH_DYNAMIC_LEFT', 'detection_valid': True,
    'world_yaw': 0.0, 'world_position': [0.0, 0.0, 40.0],
}
home = np.asarray([-1000.0, 0.0, 0.2])
positions = {
    'standard_vtol_0': np.asarray([0.0, 0.0, 40.0]),
    'person_red': np.asarray([10.0, 0.0, 0.2]),
}

# One continuous expected-visibility event without reliable recognition => miss.
for index in range(10):
    validator.update(image, [], index * 0.1, index, flight, positions, camera, home)
positions['person_red'] = np.asarray([500.0, 0.0, 0.2])
for index in range(4):
    validator.update(image, [], 1.0 + index * 0.1, 10 + index,
                     flight, positions, camera, home)

# One expected-visibility event with reliable recognition => correct.
detection = [{'class_name': 'person_red', 'confidence': 0.90,
              'xyxy': [600, 300, 680, 400]}]
positions['person_red'] = np.asarray([10.0, 0.0, 0.2])
for index in range(10):
    validator.update(image, detection, 2.0 + index * 0.1, 20 + index,
                     flight, positions, camera, home)
positions['person_red'] = np.asarray([500.0, 0.0, 0.2])
for index in range(4):
    validator.update(image, [], 3.0 + index * 0.1, 30 + index,
                     flight, positions, camera, home)

# No expected target, repeated high-confidence task-class output => false positive.
for index in range(5):
    validator.update(image, detection, 4.0 + index * 0.1, 40 + index,
                     flight, positions, camera, home)
for index in range(4):
    validator.update(image, [], 4.5 + index * 0.1, 45 + index,
                     flight, positions, camera, home)
validator.finalize('unit_test_complete')

counts = validator.summary()['counts']
assert counts['correct'] == 1, counts
assert counts['miss'] == 1, counts
assert counts['false_positive'] == 1, counts
assert len(event_pub.rows) == 3, event_pub.rows
assert any(output.glob('detection_validation_events/MISS_*.jpg'))
assert any(output.glob('detection_validation_events/FALSE_POSITIVE_*.jpg'))
print('PASS event-level validator synthetic correct/miss/false-positive test')
