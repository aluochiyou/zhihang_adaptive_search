#!/usr/bin/env python3
from pathlib import Path
import math
import sys

import numpy as np
import yaml

B = Path(__file__).resolve().parents[1]
P = B / 'zhihang_adaptive_search_v6'
sys.path.insert(0, str(P / 'src'))

from zhihang_adaptive_search_v6.tracking_recovery import (  # noqa: E402
    DynamicTargetFilter,
    generate_mc_reacquisition_waypoints,
    possible_target_radius,
    static_hover_point,
    weighted_position_fusion,
    yaw_rate_command,
)


def require(condition, message):
    if not condition:
        raise AssertionError(message)
    print('PASS', message)


require(abs(yaw_rate_command(0.0, math.pi, 1.2, 0.35, 3.0)) <= 0.35 + 1e-9,
        'yaw command is bounded')
require(yaw_rate_command(0.0, math.radians(1.0), 1.2, 0.35, 3.0) == 0.0,
        'yaw deadband suppresses chatter')
require(abs(possible_target_radius(40.0, 1.5, 20.0, 250.0, 5.0) - 85.0) < 1e-9,
        'time-expanded target uncertainty radius')

bounds = {'safe_x_min': 100.0, 'safe_x_max': 1900.0,
          'safe_y_min': -900.0, 'safe_y_max': 900.0}
points = generate_mc_reacquisition_waypoints(
    [1800.0, 850.0, 0.2], 180.0, 40.0, 30.2, bounds,
    start=[1700.0, 800.0, 30.2])
require(len(points) >= 4, 'reacquisition pattern has multiple scan legs')
require(all(100.0 <= p[0] <= 1900.0 and -900.0 <= p[1] <= 900.0
            and abs(p[2] - 30.2) < 1e-9 for p in points),
        'reacquisition pattern remains inside safe area at tracking altitude')

hover = static_hover_point([1532.0, -519.0, 0.2], 'person_white', 30.0,
                           {'person_white': [-10.0, 0.0]})
require(np.allclose(hover, [1522.0, -519.0, 30.2]),
        'person_white hover point is 10 m west and 30 m above frozen target')

filt = DynamicTargetFilter(0.8, 1.2, 35.0)
filt.reset([100.0, 100.0, 0.2], [1.0, 0.0, 0.0], 10.0)
p, v, accepted = filt.update([101.0, 100.0, 0.2], [1.0, 0.0, 0.0], 11.0)
require(accepted and p[0] > 100.0, 'dynamic target filter accepts consistent measurement')
_, _, accepted = filt.update([400.0, 400.0, 0.2], [0.0, 0.0, 0.0], 12.0)
require(not accepted, 'dynamic target filter rejects large geolocation jump')

fusion = weighted_position_fusion([
    {'target_name': 'car_opel', 'position_world': [10.0, 20.0, 0.2], 'confidence': 0.8},
    {'target_name': 'car_opel', 'position_world': [11.0, 20.0, 0.2], 'confidence': 0.9},
    {'target_name': 'car_opel', 'position_world': [10.5, 20.5, 0.2], 'confidence': 0.85},
], 'car_opel', 2, 12.0)
require(fusion is not None and fusion['report_count'] == 3,
        'hover localization reports fuse into refined static position')

flight = (P / 'scripts/vehicle_flight_agent.py').read_text(encoding='utf-8')
manager = (P / 'scripts/mission_manager.py').read_text(encoding='utf-8')
localization = (P / 'src/zhihang_adaptive_search_v6/target_localization.py').read_text(encoding='utf-8')
require('TRACK_MC_LAST_KNOWN_APPROACH' in flight and 'TRACK_MC_REACQUIRE' in flight,
        'flight state machine contains last-known approach and MC reacquisition')
require('generate_square_spiral_reacquisition_waypoints' in flight and 'possible_target_radius' in flight,
        'flight controller uses time-expanded autonomous reacquisition geometry')
require('relative_velocity_damping_gain' in flight and 'kd_xy' not in flight[flight.index('def execute_track'):flight.index('def order_static_targets')],
        'dynamic controller removes noisy error derivative and adds velocity damping')
verify_segment = flight[flight.index('def verify_one_static_target'):flight.index('def execute_static_verify')]
require('self.set_velocity_world(np.zeros(3))\n        self.set_phase(\'STATIC_TRANSITION_MC\')' not in verify_segment,
        'static FW-to-MC transition no longer sends zero FW velocity')
require("target_hover_offset_xy_m" in flight and 'hover_yaw_deg' in flight,
        'static verification applies target offset and explicit yaw')
require('STATIC_POSITION_REFINED' in flight and 'STATIC_POSITION_REFINED' in manager,
        'hover refinement is reported to and consumed by manager')
require('candidate_frozen_until_hover_refinement' in manager and
        "row['frozen_target_world']" in manager,
        'candidate-aware static position freeze policy is encoded in assignment data')
require("phase.startswith('TRACK_')" in localization,
        'localizer remains active during transition, last-known approach and reacquisition')

for name in ('adaptive_search.yaml', 'adaptive_search_formal.yaml'):
    cfg = yaml.safe_load((P / 'config' / name).read_text(encoding='utf-8'))
    require(cfg['tracking']['position_kp_xy'] == 0.45 and
            cfg['tracking']['maximum_xy_acceleration_mps2'] == 1.5,
            f'{name} installs smoother dynamic controller gains')
    require(cfg['static_verify']['hover_yaw_deg'] == 0.0 and
            cfg['static_verify']['target_hover_offset_xy_m']['person_white'] == [-10.0, 0.0],
            f'{name} installs yaw-zero and person_white west offset')
    require(cfg['perception']['target_localization']['static_hover_report_hz'] == 2.0,
            f'{name} enables repeated hover localization reports')

launch = (P / 'launch/manager.launch').read_text(encoding='utf-8')
require('vision_target_state_estimator.py' in launch and 'unless="$(arg validation_truth_relay)"' in launch,
        'formal mode launches truth-free visual target-state estimator')
print('V6.7.19 TRACKING RECOVERY AND STATIC VERIFY TEST PASSED')
