#!/usr/bin/env python3
from pathlib import Path
import ast
import math
import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'zhihang_adaptive_search_v6'
flight = (PKG / 'scripts/vehicle_flight_agent.py').read_text(encoding='utf-8')
manager = (PKG / 'scripts/mission_manager.py').read_text(encoding='utf-8')
perception = (PKG / 'scripts/vehicle_perception_agent.py').read_text(encoding='utf-8')
for source in (flight, manager, perception):
    ast.parse(source)

# Fixed-wing HOLD protection.
assert 'FW_ZERO_VELOCITY_HANDOVER_BLOCKED' in flight
assert 'OFFBOARD_HOLD_RECOVERY_ATTEMPT' in flight
assert 'STATIC_VERIFY_HOLD_GUARD_TRIGGERED' in flight
static_exception = flight[flight.index('def _recover_static_verify_exception'):
                          flight.index('def execute_static_verify')]
assert 'self.set_velocity_world(np.zeros(3))' not in static_exception

# Dynamic tracking: no stop-before-recovery and yaw only close to fresh target.
track = flight[flight.index('def execute_track'):
               flight.index('def order_static_targets')]
assert 'TRACK_LOSS_CONTINUE_TO_LAST_KNOWN' in track
assert 'TRACK_LAST_KNOWN_POSITION_REACHED_CONTINUE_MOTION' in flight
assert 'protected_tracking_yaw_rate' in flight
assert 'yaw_adjust_enable_radius_m' in flight
assert "self.set_velocity_world(\n                    np.zeros(3),\n                    self.tracking_yaw_rate(fallback_yaw))" not in track

# Static candidate lifecycle and false-candidate evidence.
assert 'STATIC_CANDIDATE_GROUP_CREATED' in manager
assert 'STATIC_CANDIDATE_NEARBY_MERGED' in manager
assert 'STATIC_CANDIDATE_PRECISE_CONFIRMED' in flight
assert 'STATIC_CANDIDATE_REJECTED' in flight
verify = flight[flight.index('def verify_one_static_target'):
                flight.index('def _recover_static_verify_exception')]
assert "self.static_confirmed.add(name)" not in verify
assert 'no_valid_target_localization_during_mc_hover' in verify
assert 'static_rejection_images' in perception
assert 'image_is_unannotated_original_camera_frame' in perception

# The observed camo/SUV positions in the supplied log are about 47.2 m apart;
# the 60 m exclusion gate must therefore block this confusion case.
d = math.hypot(1790.5 - 1758.958334473418,
               -304.4 - (-269.246311175153))
assert 47.0 < d < 48.0

for name in ('adaptive_search.yaml', 'adaptive_search_formal.yaml'):
    cfg = yaml.safe_load((PKG / 'config' / name).read_text(encoding='utf-8'))
    assert float(cfg['tracking']['transition_to_mc_radius_m']) == 30.0
    assert float(cfg['tracking']['yaw_adjust_enable_radius_m']) == 20.0
    assert float(cfg['tracking']['yaw_adjust_max_target_age_seconds']) == 0.8
    assert float(cfg['tracking']['loss_continue_speed_mps']) == 2.5
    assert cfg['static_verify']['hold_recovery_enabled'] is True
    assert cfg['static_verify']['reject_candidate_when_no_refinement'] is True
    candidate = cfg['static_candidate_management']
    assert float(candidate['nearby_merge_radius_m']) == 25.0
    assert int(candidate['maximum_candidates_per_target']) == 5
    assert float(candidate['prius_suv_confusion']['exclusion_radius_m']) == 60.0

print('V6.7.19 HOLD/YAW/CANDIDATE/PRIUS-SUV SAFETY TEST PASSED')
