#!/usr/bin/env python3
"""Synthetic regression test for V6.7.9 conditional route preservation."""
from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / 'zhihang_adaptive_search_v6' / 'src'
SCRIPT = ROOT / 'zhihang_adaptive_search_v6' / 'scripts' / 'mission_manager.py'
sys.path.insert(0, str(PKG_SRC))


class _Now:
    def to_sec(self):
        return 1234.5


class _Time:
    @staticmethod
    def now():
        return _Now()


rospy = types.ModuleType('rospy')
rospy.Time = _Time
rospy.logwarn = lambda *args, **kwargs: None
rospy.loginfo = lambda *args, **kwargs: None
rospy.logwarn_throttle = lambda *args, **kwargs: None
rospy.loginfo_throttle = lambda *args, **kwargs: None
sys.modules['rospy'] = rospy
std_msgs = types.ModuleType('std_msgs')
std_msgs_msg = types.ModuleType('std_msgs.msg')
std_msgs_msg.Bool = type('Bool', (), {})
std_msgs_msg.String = type('String', (), {})
std_msgs.msg = std_msgs_msg
sys.modules['std_msgs'] = std_msgs
sys.modules['std_msgs.msg'] = std_msgs_msg

spec = importlib.util.spec_from_file_location('v679_manager', SCRIPT)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)
MissionManager = module.MissionManager


def make_manager(temp_dir: str):
    mgr = MissionManager.__new__(MissionManager)
    mgr.cfg = {
        'static_search': {
            'preserve_current_route_after_two_dynamic_targets': True,
            'allow_pair_guided_interrupt_during_committed_route': True,
            'allow_verify_interrupt_during_committed_route': True,
            'allow_empty_remainder_interrupt_during_committed_route': True,
            'conditional_commitment_uncovered_cell_threshold': 3,
            'conditional_commitment_relevance_check_seconds': 5.0,
            'conditional_commitment_max_boundary_wait_seconds': 75.0,
            'static_target_pairs': [['s1', 's2'], ['s3', 's4']],
        },
        'straight_detection_gate': {
            'valid_segment_types': ['SEARCH_STRAIGHT', 'SEARCH_CONNECTOR_STRAIGHT'],
            'segment_edge_exclusion_m': 35.0,
        },
        'mission': {'search_altitude_m': 40.0},
        'perception': {
            'static_targets': ['s1', 's2', 's3', 's4'],
            'dynamic_targets': ['d1', 'd2'],
        },
    }
    mgr.dynamic_targets = ['d1', 'd2']
    mgr.dynamic_assignments = {'d1': 2, 'd2': 0}
    mgr.vehicle_tracking = {2: 'd1', 0: 'd2'}
    mgr.vehicle_ids = [0, 1, 2]
    mgr.static_targets = ['s1', 's2', 's3', 's4']
    mgr.static_detected = {}
    mgr.static_confirmed = {}
    mgr.static_target_pairs = [('s1', 's2'), ('s3', 's4')]
    mgr.static_pair_map = {'s1': 's2', 's2': 's1', 's3': 's4', 's4': 's3'}
    own_route = {
        'route_id': 'dynamic_right_initial_inner_to_outer',
        'waypoints': [{'point': [0, 0, 40]} for _ in range(42)],
        'altitude_m': 40.0,
    }
    mgr.assignments = {1: {'mode': 'SEARCH_DYNAMIC_RIGHT', 'route': own_route}}
    mgr.primary_routes = {
        0: {'route_id': 'dynamic_left_initial_inner_to_outer', 'waypoints': [], 'altitude_m': 39.5},
        1: own_route,
        2: {'route_id': 'static_initial_center_out_then_square_outward', 'waypoints': [], 'altitude_m': 40.5},
    }
    mgr.route_progress_history = {0: {}, 1: {own_route['route_id']: 14}, 2: {}}
    mgr.route_completed_history = set()
    mgr.static_route_commitment = {}
    mgr.static_route_handoff_started = set()
    mgr.static_strategy_stage = {}
    mgr.static_strategy_history = []
    mgr.flight_status = {1: {
        'world_position': [1000, 0, 40],
        'route_id': own_route['route_id'],
        'route_waypoint_index': 14,
        'assignment_mode': 'SEARCH_DYNAMIC_RIGHT',
        'detection_valid': True,
        'detection_segment_type': 'SEARCH_STRAIGHT',
        'segment_along_m': 300.0,
        'segment_length_m': 900.0,
    }}
    mgr.coverage_samples = []
    mgr.static_residual_routes = []
    mgr.static_residual_pass_index = 0
    mgr.static_pair_guided_pass_index = 0
    mgr.static_pair_guided_signature = None
    mgr.plan = {'model_state_analysis': {}, 'static_residual_route': {'waypoints': []}}
    mgr.run_dir = Path(temp_dir)
    mgr.remaining_static_vehicle = lambda: 1
    mgr.record_static_strategy = lambda *args, **kwargs: None
    return mgr, own_route


def main() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        mgr, own_route = make_manager(temp_dir)
        published = []

        def publish_assignment(vid, mode, route=None, **kwargs):
            published.append((vid, mode, None if route is None else route.get('route_id')))
            mgr.assignments[vid] = {'mode': mode, 'route': route, **kwargs}

        mgr.publish_assignment = publish_assignment
        pair_route = {
            'route_id': 'static_pair_guided_test',
            'waypoints': [{'point': [1, 2, 40]}, {'point': [3, 4, 40]}],
        }
        mgr.build_static_pair_guided_route = lambda vid: pair_route

        # Routine role change preserves the current route.
        mgr.maybe_assign_static_role(force_replan=True)
        assert published == [], published
        lock = mgr.static_route_commitment[1]
        assert lock['stage'] == 'PRESERVE_CURRENT_OWN_ROUTE'
        assert lock['committed_route_progress'] == 14

        # Finding one member of every still-missing pair upgrades the strategy.
        # The request is queued, but a detection-valid straight leg is not cut.
        mgr.static_detected = {'s1': {}, 's3': {}}
        mgr.maybe_assign_static_role(force_replan=True)
        assert published == [], published
        pending = mgr.static_route_commitment[1]['pending_transition']
        assert pending['action'] == 'PAIR_GUIDED'

        # At the next waypoint boundary the preserved route is released and the
        # targeted pair-guided route is published.
        mgr.flight_status[1]['route_waypoint_index'] = 15
        mgr.update_static_route_commitment_from_status(1, mgr.flight_status[1])
        assert published == [(1, 'SEARCH_STATIC_PAIR_GUIDED', pair_route['route_id'])], published
        assert 1 not in mgr.static_route_commitment

        # Separate all-static case: VERIFY_STATIC must also be allowed to
        # interrupt a preserved broad route at the next boundary.
        mgr2, _ = make_manager(temp_dir)
        published2 = []

        def publish2(vid, mode, route=None, **kwargs):
            published2.append((vid, mode, None if route is None else route.get('route_id')))
            mgr2.assignments[vid] = {'mode': mode, 'route': route, **kwargs}

        mgr2.publish_assignment = publish2
        mgr2.maybe_assign_static_role(force_replan=True)
        mgr2.static_detected = {name: {'target_name': name} for name in mgr2.static_targets}
        mgr2.maybe_assign_static_role(force_replan=True)
        assert mgr2.static_route_commitment[1]['pending_transition']['action'] == 'VERIFY_STATIC'
        mgr2.flight_status[1]['route_waypoint_index'] = 15
        mgr2.update_static_route_commitment_from_status(1, mgr2.flight_status[1])
        assert published2 and published2[-1][1] == 'VERIFY_STATIC', published2

        # An exhausted static-prior remainder may skip the rest of the own route
        # and proceed to the opposite side instead of flying for its own sake.
        mgr3, _ = make_manager(temp_dir)
        published3 = []
        mgr3.publish_assignment = lambda vid, mode, route=None, **kwargs: published3.append(
            (vid, mode, None if route is None else route.get('route_id')))
        mgr3.maybe_assign_static_role(force_replan=True)
        mgr3.committed_route_relevance = lambda vid, commitment: {
            'uncovered_cells': 0, 'candidate_cells': 0, 'owner_vehicle': 1,
            'route_id': own_route['route_id'], 'progress': 14,
            'route_waypoints': 42, 'source_unflown_segments': 0,
        }
        opposite = {
            'route_id': 'conditional_opposite',
            'waypoints': [{'point': [10, 20, 40]}],
        }
        mgr3.build_precise_unfinished_static_route = lambda owner, assigned, stage: opposite
        transition = mgr3.desired_static_commitment_transition(1, check_relevance=True)
        assert transition and transition['action'] == 'OPPOSITE_REMAINDER', transition
        mgr3.request_static_commitment_transition(1, transition)
        mgr3.flight_status[1]['route_waypoint_index'] = 15
        mgr3.update_static_route_commitment_from_status(1, mgr3.flight_status[1])
        assert published3[-1][1] == 'SEARCH_STATIC_OPPOSITE_REMAINDER', published3
        assert mgr3.static_route_commitment[1]['stage'] == 'PRESERVE_OPPOSITE_REMAINDER'

        summary = {
            'routine_replan_preserves_current_route': True,
            'pair_guided_interrupts_at_waypoint_boundary': True,
            'all_static_interrupts_to_verify': True,
            'empty_remainder_skips_to_opposite': True,
            'opposite_remainder_is_also_conditional': True,
        }
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        print('CONDITIONAL STATIC ROUTE PRESERVATION TEST PASSED')


if __name__ == '__main__':
    main()
