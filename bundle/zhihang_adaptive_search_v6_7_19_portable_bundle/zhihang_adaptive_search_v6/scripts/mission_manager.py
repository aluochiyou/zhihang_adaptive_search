#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import math
import os
import threading
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rospy
from std_msgs.msg import Bool, String

from zhihang_adaptive_search_v6.common import (
    build_plan,
    dump_json,
    filter_static_prior_points_by_regions,
    filter_static_prior_points_by_route_remainder,
    filter_uncovered_static_prior_points,
    generate_static_gap_route,
    generate_dynamic_distribution_inward_route,
    generate_static_prior_grid_points,
    payload_checksum,
    wrap_pi,
)

NS = '/zhihang/search_v6'
PARAM_ROOT = '/zhihang_search_v6'


class MissionManager:
    def __init__(self) -> None:
        rospy.init_node('mission_manager_v6')
        self.cfg = rospy.get_param(PARAM_ROOT)
        self.plan_only = bool(rospy.get_param('~plan_only', self.cfg['mission'].get('plan_only', False)))
        self.lock = threading.RLock()
        self.vehicle_ids = [int(v) for v in self.cfg['mission']['enabled_vehicle_ids']]
        self.plan, self.primary_routes = build_plan(self.cfg)
        role_cfg = self.cfg.get('role_policy', {})
        self.dynamic_inspection_vehicle_ids = [
            int(v) for v in role_cfg.get('dynamic_inspection_vehicle_ids', [0, 1])
        ]
        self.maneuver_inspection_vehicle_id = int(
            role_cfg.get('maneuver_inspection_vehicle_id', 2)
        )
        self.vehicle_roles: Dict[int, str] = {
            v: ('MANEUVER_INSPECTION' if v == self.maneuver_inspection_vehicle_id
                else 'DYNAMIC_INSPECTION')
            for v in self.vehicle_ids
        }
        self.run_dir = Path(self.plan['run_dir'])
        self.mission_id = self.run_dir.name
        self.plan['mission_id'] = self.mission_id
        dump_json(self.run_dir / 'search_plan.json', self.plan)
        (self.run_dir / 'task_packets').mkdir(parents=True, exist_ok=True)

        self.task_packets: Dict[int, dict] = {}
        self.task_pubs = {}
        self.assignment_pubs = {}
        self.target_pubs = {}
        self.flight_ack = {v: False for v in self.vehicle_ids}
        self.perception_ack = {v: False for v in self.vehicle_ids}
        self.flight_ready = {v: False for v in self.vehicle_ids}
        self.perception_ready = {v: False for v in self.vehicle_ids}
        self.flight_status: Dict[int, dict] = {}
        self.perception_status: Dict[int, dict] = {}
        self.flight_results: Dict[int, dict] = {}
        self.assignments: Dict[int, dict] = {}
        self.assignment_seq = {v: 0 for v in self.vehicle_ids}
        self.last_assignment_wall = {v: 0.0 for v in self.vehicle_ids}
        self.detections: List[dict] = []
        self.yolo_reports: List[dict] = []
        self.target_localization_reports: List[dict] = []
        self.latest_target_localization: Dict[str, dict] = {}
        # Validation-only event-level YOLO quality records.  These messages are
        # produced by the perception agents and are never used by assignment,
        # route planning, tracking or flight control.
        self.detection_validation_events: List[dict] = []
        self.detection_validation_summaries: Dict[int, dict] = {}
        self.static_targets = list(self.cfg['perception']['static_targets'])
        self.dynamic_targets = list(self.cfg['perception']['dynamic_targets'])
        self.static_detected: Dict[str, dict] = {}
        self.static_confirmed: Dict[str, dict] = {}
        self.static_candidate_cfg = dict(
            self.cfg.get('static_candidate_management', {}))
        self.static_candidates: Dict[str, List[dict]] = {
            name: [] for name in self.static_targets}
        self.static_rejected_regions: List[dict] = []
        self.static_candidate_sequence = 0
        self.dynamic_tracks: Dict[str, List[dict]] = {name: [] for name in self.dynamic_targets}
        self.dynamic_assignments: Dict[str, int] = {}
        self.vehicle_tracking: Dict[int, str] = {}
        # Dynamic assignments may be provisional. person_red is lockable
        # immediately; suv_camo must prove motion for five seconds before it
        # consumes a permanent tracker slot.
        self.dynamic_assignment_confirmed: Dict[str, bool] = {}
        self.dynamic_provisional: Dict[str, dict] = {}
        self.pre_tracking_assignment: Dict[int, dict] = {}
        self.pre_tracking_role: Dict[int, str] = {}
        self.suv_false_positive_regions: List[dict] = []
        # Truth firewall: the manager has no Gazebo model-state subscriber.
        # An optional validation-only relay may publish generic target states on
        # /zhihang/search_v6/tracking/target_state; formal mode replaces it with
        # a visual multi-object tracker using the same message schema.
        self.external_target_states: Dict[str, dict] = {}
        self.static_high_risk_history: List[dict] = []
        self.events: List[dict] = []
        self.start_payload: Optional[dict] = None
        self.abort_sent = False
        self.return_sent = False
        self.heartbeat_seq = 0
        self.mission_start_ros: Optional[float] = None
        self.first_arm_ros: Optional[float] = None
        self.first_arm_wall: Optional[float] = None
        self.return_sent_ros: Optional[float] = None
        self.return_sent_wall: Optional[float] = None
        self.return_budget_exceeded_logged = False
        self.competition_limit_exceeded_logged = False
        self.start_authorization_required = bool(self.cfg['mission'].get('start_authorization_required', False))
        self.application_ready = False
        self.start_authorized = not self.start_authorization_required
        self.start_authorization_payload: Dict[str, object] = {}
        # V6.4: actual search coverage collected from all three aircraft while
        # their straight-segment detection gates are valid. These samples are
        # used to generate a static-only residual route after both dynamic
        # targets have dedicated trackers.
        self.coverage_samples: List[dict] = []
        self.last_coverage_sample: Dict[int, dict] = {}
        self.static_residual_pass_index = 0
        self.static_residual_routes: List[dict] = []
        self.static_pair_guided_pass_index = 0
        self.static_pair_guided_signature = None
        self.dynamic_distribution_pass_index = {0: 0, 1: 0}
        # Preserve each aircraft's maximum progress on every route even after
        # it changes assignment to dynamic tracking. V6.7.3 only inspected the
        # current route_id, which could reset the remainder to the full original
        # dynamic-search route after a tracker handover.
        self.route_progress_history: Dict[int, Dict[str, int]] = {
            v: {} for v in self.vehicle_ids
        }
        self.route_completed_history = set()
        self.static_strategy_stage: Dict[int, str] = {}
        self.static_strategy_history: List[dict] = []
        # V6.7.19 conditional route-preservation guard.  When the second dynamic
        # target is assigned, the remaining aircraft keeps its current coverage
        # route by default so a routine role change cannot cut a useful search
        # leg.  The route is NOT an objective by itself: all-static-detected,
        # pair-guided-ready, exhausted static-prior remainder, stale assignment,
        # or safety/deadline conditions may release it at the next safe waypoint
        # boundary.  The same conditional rule applies to the opposite-side
        # remainder.
        self.static_route_commitment: Dict[int, dict] = {}
        self.static_route_handoff_started = set()
        self.static_verify_complete_vehicle: Optional[int] = None
        configured_pairs = self.cfg.get('static_search', {}).get('static_target_pairs', [])
        self.static_target_pairs = [tuple(map(str, pair[:2])) for pair in configured_pairs if len(pair) >= 2]
        self.static_pair_map: Dict[str, str] = {}
        for left, right in self.static_target_pairs:
            self.static_pair_map[left] = right
            self.static_pair_map[right] = left
        self.return_wait_warnings = 0
        self.mission_end_reason = ''
        self.failure_safe_return_triggered = False

        self.heartbeat_pub = rospy.Publisher(f'{NS}/manager/heartbeat', String, queue_size=10, latch=True)
        self.start_pub = rospy.Publisher(f'{NS}/manager/start', String, queue_size=1, latch=True)
        self.abort_pub = rospy.Publisher(f'{NS}/manager/abort', Bool, queue_size=1, latch=True)
        self.complete_pub = rospy.Publisher(f'{NS}/manager/mission_complete', String, queue_size=1, latch=True)
        self.competition_results_pub = rospy.Publisher(
            f'{NS}/manager/competition_final_results', String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(f'{NS}/manager/status', String, queue_size=10, latch=True)
        self.reclassification_pub = rospy.Publisher(
            f'{NS}/tracking/reclassification', String, queue_size=10, latch=True)
        self.application_ready_pub = rospy.Publisher(f'{NS}/manager/application_ready', Bool, queue_size=1, latch=True)
        rospy.Subscriber(f'{NS}/manager/start_authorization', String, self.start_authorization_cb, queue_size=5)

        for vid in self.vehicle_ids:
            prefix = f'{NS}/vehicle_{vid}'
            self.task_pubs[vid] = rospy.Publisher(f'{NS}/manager/task/vehicle_{vid}', String, queue_size=1, latch=True)
            self.assignment_pubs[vid] = rospy.Publisher(f'{NS}/manager/assignment/vehicle_{vid}', String, queue_size=2, latch=True)
            self.target_pubs[vid] = rospy.Publisher(f'{NS}/manager/target/vehicle_{vid}', String, queue_size=10)
            rospy.Subscriber(f'{prefix}/task_ack/flight', String, lambda m, v=vid: self.ack_cb(m, v, 'flight'), queue_size=5)
            rospy.Subscriber(f'{prefix}/task_ack/perception', String, lambda m, v=vid: self.ack_cb(m, v, 'perception'), queue_size=5)
            rospy.Subscriber(f'{prefix}/flight_status', String, lambda m, v=vid: self.flight_status_cb(m, v), queue_size=50)
            rospy.Subscriber(f'{prefix}/perception_status', String, lambda m, v=vid: self.perception_status_cb(m, v), queue_size=50)
            rospy.Subscriber(f'{prefix}/detection_report', String, lambda m, v=vid: self.detection_cb(m, v), queue_size=500)
            rospy.Subscriber(f'{prefix}/yolo_report', String, lambda m, v=vid: self.yolo_cb(m, v), queue_size=500)
            rospy.Subscriber(f'{prefix}/target_localization_report', String,
                             lambda m, v=vid: self.target_localization_cb(m, v), queue_size=500)
            rospy.Subscriber(f'{prefix}/detection_validation_event', String,
                             lambda m, v=vid: self.detection_validation_event_cb(m, v), queue_size=200)
            rospy.Subscriber(f'{prefix}/detection_validation_summary', String,
                             lambda m, v=vid: self.detection_validation_summary_cb(m, v), queue_size=10)
            rospy.Subscriber(f'{prefix}/event', String, lambda m, v=vid: self.event_cb(m, v), queue_size=200)
            rospy.Subscriber(f'{prefix}/result', String, lambda m, v=vid: self.result_cb(m, v), queue_size=5)
        rospy.Subscriber(f'{NS}/tracking/target_state', String, self.target_state_cb, queue_size=100)

        self.make_task_packets()
        self.timer_heartbeat = rospy.Timer(rospy.Duration(1.0 / max(float(self.cfg['mission']['manager_heartbeat_hz']), 0.1)), self.heartbeat_tick)
        self.timer_tasks = rospy.Timer(rospy.Duration(2.0), self.republish_tasks)
        self.timer_assign = rospy.Timer(rospy.Duration(1.0 / max(float(self.cfg['mission']['assignment_publish_hz']), 0.1)), self.assignment_tick)
        self.timer_status = rospy.Timer(rospy.Duration(0.5), self.publish_status)

    def start_authorization_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if not bool(row.get('authorized', False)):
                return
            mission = str(row.get('mission_id', ''))
            if mission and mission != self.mission_id:
                return
            with self.lock:
                self.start_authorized = True
                self.start_authorization_payload = dict(row)
            dump_json(self.run_dir / 'start_authorization.json', row)
            rospy.logwarn('V6.7.19 start authorized after target-motion launch: %s',
                          json.dumps(row, ensure_ascii=False))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'invalid V6.7.19 start authorization: %s', exc)

    def make_task_packets(self) -> None:
        for vid in self.vehicle_ids:
            endpoint = self.cfg['perception']['yolo_endpoints'][str(vid)]
            cruise_altitude = float(self.primary_routes[vid].get(
                'altitude_m', self.cfg['mission'].get('search_altitude_m', 40.0)))
            tracking_cfg = dict(self.cfg['tracking'])
            tracking_cfg['intercept_altitude_m'] = cruise_altitude
            return_cfg = {
                **{k: v for k, v in self.cfg['return_strategy'].items()
                   if k not in ('landing_offset_xy_m', 'return_gate_offset_xy_m')},
                'landing_offset_xy_m': self.cfg['return_strategy']['landing_offset_xy_m'][str(vid)],
                'return_gate_offset_xy_m': self.cfg['return_strategy']['return_gate_offset_xy_m'][str(vid)],
                'fixed_wing_return_altitude_m': cruise_altitude,
            }
            packet = {
                'schema_version': 1,
                'mission_id': self.mission_id,
                'scene_id': self.cfg['mission']['scene_id'],
                'vehicle_id': vid,
                'initial_functional_role': self.vehicle_roles.get(vid),
                'role_policy': self.cfg.get('role_policy', {}),
                'primary_route': self.primary_routes[vid],
                'static_residual_route': self.plan['static_residual_route'],
                'flight_control': self.cfg['flight_control'],
                'coordinate_transform': self.cfg['coordinate_transform'],
                'return_strategy': return_cfg,
                'mission_parameters': {
                    'search_altitude_m': cruise_altitude,
                    'cruise_altitude_m': cruise_altitude,
                    'track_altitude_m': self.cfg['mission']['track_altitude_m'],
                    'static_verify_altitude_m': self.cfg['mission']['static_verify_altitude_m'],
                    'takeoff_transition_height_m': self.cfg['mission']['takeoff_transition_height_m'],
                    'start_delay_seconds': self.cfg['mission']['takeoff_stagger_seconds'][vid],
                    'vehicle_output_root': self.cfg['mission']['vehicle_output_root'],
                },
                'tracking': tracking_cfg,
                'static_verify': self.cfg['static_verify'],
                'straight_detection_gate': self.cfg['straight_detection_gate'],
                'camera': {
                    'image_topic': self.cfg['camera']['topics'][str(vid)],
                    'camera_info_topic': self.cfg['camera']['camera_info_topics'][str(vid)],
                    'fallback': self.cfg['camera'],
                },
                'perception': self.cfg['perception'],
                'yolo_endpoint': endpoint,
                'manager_policy': {'heartbeat_timeout_seconds': self.cfg['mission']['manager_heartbeat_timeout_seconds']},
            }
            packet['checksum'] = payload_checksum(packet)
            self.task_packets[vid] = packet
            dump_json(self.run_dir / 'task_packets' / f'vehicle_{vid}.json', packet)
            self.task_pubs[vid].publish(String(data=json.dumps(packet, ensure_ascii=False)))
        rospy.loginfo('V6 task packets created mission=%s run_dir=%s', self.mission_id, self.run_dir)

    def republish_tasks(self, _event=None) -> None:
        if self.plan_only:
            return
        for vid in self.vehicle_ids:
            if not (self.flight_ack[vid] and self.perception_ack[vid]):
                self.task_pubs[vid].publish(String(data=json.dumps(self.task_packets[vid], ensure_ascii=False)))

    def heartbeat_tick(self, _event=None) -> None:
        self.heartbeat_seq += 1
        row = {'mission_id': self.mission_id, 'sequence': self.heartbeat_seq,
               'ros_time': rospy.Time.now().to_sec(), 'wall_time': time.time(), 'abort_sent': self.abort_sent}
        self.heartbeat_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))

    def ack_cb(self, msg: String, vid: int, component: str) -> None:
        try:
            row = json.loads(msg.data)
            valid = (row.get('mission_id') == self.mission_id and int(row.get('vehicle_id', -1)) == vid
                     and row.get('checksum') == self.task_packets[vid]['checksum'] and bool(row.get('accepted', False)))
        except Exception:
            valid = False
        if component == 'flight': self.flight_ack[vid] = valid
        else: self.perception_ack[vid] = valid

    def flight_status_cb(self, msg: String, vid: int) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') != self.mission_id: return
        except Exception:
            return
        with self.lock:
            self.flight_status[vid] = row
            self.flight_ready[vid] = bool(row.get('ready', False))
            route_id = str(row.get('route_id', ''))
            if route_id:
                route_index = max(0, int(row.get('route_waypoint_index', 0)))
                previous_index = self.route_progress_history[int(vid)].get(route_id, 0)
                self.route_progress_history[int(vid)][route_id] = max(previous_index, route_index)
            if row.get('armed') and self.first_arm_ros is None:
                self.first_arm_ros = float(row.get('ros_time', rospy.Time.now().to_sec()))
                self.first_arm_wall = time.monotonic()
                rospy.logwarn('V6.7.2 competition clock started at first ARMED ros_time=%.3f',
                              self.first_arm_ros)
            for name in row.get('static_confirmed', []):
                if name not in self.static_confirmed:
                    self.static_confirmed[name] = {'target_name': name, 'vehicle_id': vid,
                                                   'ros_time': row.get('ros_time'), 'source': 'flight_status'}
            self.record_coverage_sample(vid, row)
        # Conditional route transitions are evaluated outside the state lock.
        # Route generation and assignment publication may be comparatively
        # expensive and must not block the high-rate status update critical
        # section.
        self.update_static_route_commitment_from_status(int(vid), row)

    def record_coverage_sample(self, vid: int, row: dict) -> None:
        """Record thinned FOV samples from valid straight search segments.

        Only the region where perception was actually allowed is credited. A
        sample represents the oriented down-looking camera footprint at the
        reported aircraft pose; it is shared by the static gap planner across
        all three aircraft.
        """
        if not bool(row.get('detection_valid', False)):
            return
        if not str(row.get('phase', '')).startswith('SEARCH_'):
            return
        position = row.get('world_position')
        if not isinstance(position, list) or len(position) < 3:
            return
        now_ros = float(row.get('ros_time', rospy.Time.now().to_sec()))
        yaw = float(row.get('world_yaw', 0.0))
        spacing = float(self.cfg['static_search'].get('coverage_sample_spacing_m', 18.0))
        max_interval = float(self.cfg['static_search'].get('coverage_sample_max_interval_seconds', 2.0))
        min_yaw_change = math.radians(float(self.cfg['static_search'].get('coverage_sample_yaw_change_deg', 8.0)))
        previous = self.last_coverage_sample.get(vid)
        if previous is not None:
            p0 = np.asarray(previous['position'], dtype=float)
            p1 = np.asarray(position, dtype=float)
            moved = float(np.linalg.norm(p1[:2] - p0[:2]))
            elapsed = now_ros - float(previous['ros_time'])
            yaw_change = abs(wrap_pi(yaw - float(previous['yaw'])))
            if moved < spacing and elapsed < max_interval and yaw_change < min_yaw_change:
                return
        sample = {
            'mission_id': self.mission_id,
            'vehicle_id': int(vid),
            'ros_time': now_ros,
            'position': [float(x) for x in position[:3]],
            'yaw': yaw,
            'phase': str(row.get('phase', '')),
            'route_id': str(row.get('route_id', '')),
            'leg_id': str(row.get('detection_leg_id', '')),
            'segment_type': str(row.get('detection_segment_type', '')),
        }
        self.coverage_samples.append(sample)
        self.last_coverage_sample[vid] = sample
        max_samples = int(self.cfg['static_search'].get('coverage_max_samples', 12000))
        if len(self.coverage_samples) > max_samples:
            del self.coverage_samples[:len(self.coverage_samples)-max_samples]
        with open(self.run_dir / 'coverage_samples.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(sample, ensure_ascii=False) + '\n')

    def perception_status_cb(self, msg: String, vid: int) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') != self.mission_id: return
        except Exception:
            return
        with self.lock:
            self.perception_status[vid] = row
            self.perception_ready[vid] = bool(row.get('ready', False))

    def yolo_cb(self, msg: String, vid: int) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') != self.mission_id:
                return
            with self.lock:
                self.yolo_reports.append(row)
            with open(self.run_dir / 'yolo_reports.jsonl', 'a', encoding='utf-8') as fp:
                fp.write(json.dumps(row, ensure_ascii=False) + '\n')
        except Exception:
            return

    def target_localization_cb(self, msg: String, vid: int) -> None:
        """Receive stable multi-frame target positions for logging and review.

        The normal detection_report path still decides whether a report enters
        mission management. This callback never bypasses selected_result_source;
        it gives the management terminal and evaluation files full visibility
        into every localization result, including validation-mode YOLO reports.
        """
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') not in (None, '', self.mission_id):
                return
            row['vehicle_id'] = int(vid)
            position = row.get('position_world')
            if not isinstance(position, list) or len(position) < 3:
                return
            if not all(math.isfinite(float(x)) for x in position[:3]):
                return
            with self.lock:
                self.target_localization_reports.append(row)
                if len(self.target_localization_reports) > 20000:
                    del self.target_localization_reports[:-20000]
                name = str(row.get('target_name', ''))
                if name:
                    self.latest_target_localization[name] = dict(row)
            with open(self.run_dir / 'target_localization_reports.jsonl',
                      'a', encoding='utf-8') as fp:
                fp.write(json.dumps(row, ensure_ascii=False) + '\n')
            rospy.loginfo(
                'V6.7.19 LOCALIZATION v%d target=%s conf=%.2f '
                'world=(%.1f,%.1f,%.1f) std_xy=%.1fm frames=%d selected=%s',
                vid, row.get('target_name'), float(row.get('confidence', 0.0)),
                float(position[0]), float(position[1]), float(position[2]),
                float(row.get('horizontal_std_m', 0.0)),
                int(row.get('consecutive_frames', 0)),
                bool(row.get('selected_as_management_result', False)))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'target localization parse failed v%d: %s', vid, exc)

    def detection_validation_event_cb(self, msg: String, vid: int) -> None:
        """Store event-level validation results without feeding them back to planning."""
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') not in (None, '', self.mission_id):
                return
            row['vehicle_id'] = int(vid)
            self.detection_validation_events.append(row)
            with open(self.run_dir / 'detection_validation_events.jsonl', 'a', encoding='utf-8') as fp:
                fp.write(json.dumps(row, ensure_ascii=False) + '\n')
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'validation event parse failed v%d: %s', vid, exc)

    def detection_validation_summary_cb(self, msg: String, vid: int) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') not in (None, '', self.mission_id):
                return
            row['vehicle_id'] = int(vid)
            self.detection_validation_summaries[int(vid)] = row
            dump_json(self.run_dir / f'detection_validation_summary_v{int(vid)}.json', row)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'validation summary parse failed v%d: %s', vid, exc)

    def event_cb(self, msg: String, vid: int) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') != self.mission_id:
                return
        except Exception:
            return
        self.events.append(row)
        event = str(row.get('event', ''))

        if event == 'DYNAMIC_TARGET_MOTION_CONFIRMED':
            self.confirm_dynamic_assignment(
                str(row.get('target_name', '')), int(vid), row)
        elif event == 'DYNAMIC_TARGET_STATIC_FALSE_POSITIVE':
            self.reject_false_suv_assignment(int(vid), row)
        elif event == 'STATIC_POSITION_REFINED':
            # Candidate status is finalized by the explicit precise-confirmed
            # event below; keep this event for detailed refinement logging.
            name = str(row.get('target_name', ''))
            if name:
                dump_json(
                    self.run_dir / (
                        f"static_target_refined_{name}_"
                        f"{str(row.get('candidate_id', 'candidate'))}.json"),
                    row)
        elif event == 'STATIC_CANDIDATE_PRECISE_CONFIRMED':
            self.resolve_static_candidate(row, confirmed=True)
        elif event == 'STATIC_CANDIDATE_REJECTED':
            self.resolve_static_candidate(row, confirmed=False)
        elif event in ('STATIC_CONFIRMED_FINAL', 'STATIC_CONFIRMED'):
            name = str(row.get('target_name', ''))
            if name:
                # STATIC_CONFIRMED is accepted for backward compatibility;
                # V6.7.19 normally uses STATIC_CONFIRMED_FINAL.
                self.static_confirmed[name] = copy.deepcopy(row)
        elif event == 'STATIC_VERIFY_COMPLETE':
            for final in row.get('final_static_targets', []):
                name = str(final.get('target_name', ''))
                if name in self.static_targets:
                    self.static_confirmed[name] = copy.deepcopy(final)
                    self.static_detected[name] = copy.deepcopy(final)
            for name in row.get('confirmed_targets', []):
                if name in self.static_targets and name not in self.static_confirmed:
                    self.static_confirmed[name] = {
                        **row, 'target_name': name}
            self.static_verify_complete_vehicle = int(vid)
            rospy.logwarn(
                'V6.7.19 v%d completed candidate-aware static verification; '
                'confirmed=%s', vid, sorted(self.static_confirmed))
            if self.trackers_complete() and self.static_complete() and not self.return_sent:
                self.mission_end_reason = (
                    'two dynamic tracks sustained and every static class has '
                    'one highest-confidence precise position')
                self.publish_return_all(self.mission_end_reason)
        elif event == 'STATIC_VERIFY_INCOMPLETE':
            rospy.logwarn(
                'V6.7.19 v%d static verification incomplete; '
                'reissuing remaining targets=%s',
                vid, row.get('failed_targets', []))
            rospy.Timer(
                rospy.Duration(float(
                    self.cfg['static_verify'].get(
                        'incomplete_reissue_delay_seconds', 1.0))),
                lambda _event: self.maybe_assign_static_role(
                    force_replan=True),
                oneshot=True)

        if event == 'ROUTE_COMPLETE':
            route_id = str(row.get('route_id', ''))
            if route_id:
                self.route_completed_history.add((int(vid), route_id))
                total = max(0, int(row.get('route_waypoints', 0)))
                self.route_progress_history[int(vid)][route_id] = max(
                    self.route_progress_history[int(vid)].get(
                        route_id, 0), total)
            self.handle_route_complete(vid, row)

    @staticmethod
    def _competition_position(row: dict) -> Optional[List[float]]:
        """Extract a finite world/ground position from a finalized result row."""
        if not isinstance(row, dict):
            return None
        for key in (
                'refined_position_world', 'confirmed_position_world',
                'position_world', 'target_position_world', 'position'):
            value = row.get(key)
            if isinstance(value, (list, tuple)) and len(value) >= 2:
                try:
                    out = [float(value[0]), float(value[1]),
                           float(value[2]) if len(value) >= 3 else 0.0]
                except (TypeError, ValueError):
                    continue
                if all(math.isfinite(x) for x in out):
                    return out
        # Some precise-verification events store the final position in a nested
        # payload. Keep this generic so it remains compatible with older rows.
        for key in ('final', 'target', 'localization', 'candidate'):
            nested = row.get(key)
            if isinstance(nested, dict):
                position = MissionManager._competition_position(nested)
                if position is not None:
                    return position
        return None

    @staticmethod
    def _competition_timestamp_ns(row: dict) -> int:
        """Convert a ROS-time field to the nanosecond integer required by the rules."""
        now_ns = int(rospy.Time.now().to_nsec())
        if not isinstance(row, dict):
            return now_ns
        for key in (
                'timestamp_ns', 'detection_timestamp_ns', 'ros_timestamp_ns',
                'timestamp', 'ros_time', 'last_ros_time', 'first_ros_time'):
            value = row.get(key)
            if value is None:
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(numeric) or numeric < 0.0:
                continue
            # Values above 1e12 are already nanoseconds; ordinary ROS simulation
            # timestamps are represented as seconds and must be scaled.
            return int(round(numeric if numeric >= 1.0e12 else numeric * 1.0e9))
        return now_ns

    def build_competition_final_results(self) -> dict:
        """Build the single final payload consumed by the official-topic bridge.

        The official static topic is category-based, therefore every confirmed
        static model is published with model_name='static_target'.  The original
        six model identities are retained in source_model_name for the attitude
        map and review artifacts. Dynamic target start/end samples retain their
        model names as required for trajectory comparison.
        """
        static_entries = []
        for name in self.static_targets:
            # Prefer the completed hover-refined result.  At a timeout or a
            # partial mission, retain the best YOLO-only localization instead
            # of discarding an otherwise scoreable target.
            confirmed = copy.deepcopy(self.static_confirmed.get(name, {}))
            detected = copy.deepcopy(self.static_detected.get(name, {}))
            row = confirmed or detected
            position = self._competition_position(row)
            if position is None:
                continue
            static_entries.append({
                'timestamp_ns': self._competition_timestamp_ns(row),
                'model_name': 'static_target',
                'source_model_name': name,
                'quality': ('precise_confirmed' if confirmed
                            else 'best_available_detection'),
                'x': float(position[0]),
                'y': float(position[1]),
                'confidence': float(row.get('confidence', row.get('best_confidence', 0.0)) or 0.0),
            })

        dynamic_entries = []
        for name in self.dynamic_targets:
            valid = []
            for row in self.dynamic_tracks.get(name, []):
                position = self._competition_position(row)
                if position is None:
                    continue
                valid.append((self._competition_timestamp_ns(row), position, row))
            # A reliable cross-aircraft or latest visual state can still
            # provide a scoreable endpoint when no local track samples were
            # retained (for example, a late assignment immediately before the
            # 32-minute return barrier).
            if not valid:
                fallback = copy.deepcopy(self.external_target_states.get(name, {}))
                position = self._competition_position(fallback)
                if position is not None:
                    valid.append((self._competition_timestamp_ns(fallback),
                                  position, fallback))
            valid.sort(key=lambda item: item[0])
            if not valid:
                continue
            selected = [valid[0]]
            if len(valid) > 1:
                selected.append(valid[-1])
            for endpoint, (timestamp_ns, position, row) in zip(
                    ('start', 'end'), selected):
                dynamic_entries.append({
                    'timestamp_ns': int(timestamp_ns),
                    'model_name': name,
                    'endpoint': endpoint,
                    'x': float(position[0]),
                    'y': float(position[1]),
                    'confidence': float(row.get('confidence', 0.0) or 0.0),
                })

        return {
            'schema_version': 1,
            'mission_id': self.mission_id,
            'header_timestamp_ns': int(rospy.Time.now().to_nsec()),
            'static_topic': '/zhihang2026/static_targets/pose',
            'dynamic_topic': '/zhihang2026/dynamic_targets/pose',
            'static_entries': static_entries[:20],
            'dynamic_entries': dynamic_entries[:20],
            'static_model_name_rule': 'category_static_target',
            'dynamic_model_name_rule': 'model_name_start_and_end',
            'source': 'formal_yolo_finalized_results',
            'truth_used_for_planning_or_output': False,
            'run_dir': str(self.run_dir),
        }

    def result_cb(self, msg: String, vid: int) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') == self.mission_id:
                self.flight_results[vid] = row
        except Exception:
            return



    def all_dynamic_tracks_confirmed(self) -> bool:
        assignments = getattr(self, 'dynamic_assignments', {})
        targets = getattr(self, 'dynamic_targets', [])
        if len(assignments) != len(targets):
            return False
        confirmed = getattr(self, 'dynamic_assignment_confirmed', None)
        # Backward-compatible for isolated legacy unit-test fixtures that
        # construct MissionManager via __new__ without running __init__.
        if confirmed is None:
            return True
        return all(bool(confirmed.get(name, False)) for name in targets)

    def is_suv_false_region(self, position) -> bool:
        if not isinstance(position, (list, tuple, np.ndarray)) or len(position) < 2:
            return False
        now = rospy.Time.now().to_sec()
        point = np.asarray(position, dtype=float)
        active = []
        hit = False
        for region in self.suv_false_positive_regions:
            if now > float(region.get('expires_ros_time', now)):
                continue
            active.append(region)
            centre = np.asarray(region.get('position', [0.0, 0.0, 0.2]), dtype=float)
            if float(np.linalg.norm(point[:2] - centre[:2])) <= float(
                    region.get('radius_m', 20.0)):
                hit = True
        self.suv_false_positive_regions = active
        return hit

    def reliable_external_state(self, row: dict) -> bool:
        cfg = self.cfg.get('tracking', {}).get('external_injection', {})
        age = max(
            0.0,
            rospy.Time.now().to_sec()
            - float(row.get('source_ros_time',
                            row.get('ros_time', rospy.Time.now().to_sec()))))
        return bool(
            row.get('external_injection_reliable', False)
            or (
                float(row.get('confidence', 0.0))
                >= float(cfg.get('minimum_confidence', 0.65))
                and float(row.get('horizontal_std_m', float('inf')))
                <= float(cfg.get('maximum_horizontal_std_m', 5.0))
                and age <= float(cfg.get('maximum_age_seconds', 1.0))
            )
        )

    def confirm_dynamic_assignment(self, name: str, vid: int, event_row: dict) -> None:
        if self.dynamic_assignments.get(name) != int(vid):
            return
        self.dynamic_assignment_confirmed[name] = True
        provisional = self.dynamic_provisional.get(name)
        if provisional is not None:
            provisional['confirmed'] = True
            provisional['confirmation_event'] = copy.deepcopy(event_row)
        assignment = self.assignments.get(int(vid))
        if assignment is not None:
            assignment['provisional_dynamic_target'] = False
            assignment['dynamic_lock_confirmed'] = True
        rospy.logwarn(
            'V6.7.19 dynamic target motion confirmed target=%s tracker=v%d '
            'displacement=%.2fm speed=%.2fm/s',
            name, vid, float(event_row.get('displacement_m', 0.0)),
            float(event_row.get('speed_mps', 0.0)))
        if self.all_dynamic_tracks_confirmed():
            remaining = self.remaining_static_vehicle()
            if remaining is not None:
                self.vehicle_roles[int(remaining)] = 'STATIC_SEARCH'
                if int(remaining) in self.assignments:
                    self.assignments[int(remaining)]['functional_role'] = 'STATIC_SEARCH'
                rospy.logwarn(
                    'V6.7.19 both dynamic targets confirmed; '
                    'v%d transitions to STATIC_SEARCH', int(remaining))
            self.maybe_assign_static_role()

    def resume_route_for_owner(self, owner_vid: int, assigned_vid: int) -> tuple:
        """Return the nearest safe resumption of one original dynamic route."""
        owner_vid = int(owner_vid)
        assigned_vid = int(assigned_vid)
        route = copy.deepcopy(self.primary_routes[owner_vid])
        source_route_id = str(route.get('route_id', f'primary_v{owner_vid}'))
        waypoints = list(route.get('waypoints', []))
        progress = int(self.route_progress_history.get(owner_vid, {}).get(
            source_route_id, 0))
        completed = (owner_vid, source_route_id) in self.route_completed_history
        if completed or progress >= len(waypoints):
            route = self.build_dynamic_distribution_continuation(owner_vid)
            source_route_id = str(route.get('route_id', source_route_id))
            waypoints = list(route.get('waypoints', []))
            progress = 0
        else:
            # route_waypoint_index is 1-based and may denote an in-progress
            # segment. Re-fly that segment's entry instead of leaving a gap.
            start_index = max(0, progress - 1)
            waypoints = waypoints[start_index:]
            route['resume_from_waypoint_index'] = int(start_index)
        assigned_altitude = float(self.primary_routes.get(
            assigned_vid, route).get(
                'altitude_m',
                self.cfg['mission'].get('search_altitude_m', 40.0)))
        for waypoint in waypoints:
            point = list(waypoint.get('point', []))
            if len(point) >= 3:
                point[2] = assigned_altitude
                waypoint['point'] = point
        route['waypoints'] = waypoints
        route['altitude_m'] = assigned_altitude
        route['source_route_owner'] = owner_vid
        route['assigned_vehicle'] = assigned_vid
        route['source_route_id'] = source_route_id
        route['route_id'] = (
            f'{source_route_id}_resume_owner{owner_vid}_by_v{assigned_vid}_'
            f'{int(rospy.Time.now().to_sec() * 1000)}')
        mode = (
            'SEARCH_DYNAMIC_LEFT' if owner_vid == 0
            else 'SEARCH_DYNAMIC_RIGHT')
        if 'distribution' in source_route_id.lower():
            mode = (
                'SEARCH_DYNAMIC_DISTRIBUTION_LEFT' if owner_vid == 0
                else 'SEARCH_DYNAMIC_DISTRIBUTION_RIGHT')
        return mode, route

    def route_entry_distance(self, vid: int, route: dict) -> float:
        waypoints = list(route.get('waypoints', []))
        if not waypoints:
            return float('inf')
        current = self.flight_status.get(int(vid), {}).get('world_position')
        if not isinstance(current, list) or len(current) < 2:
            return 0.0
        return float(np.linalg.norm(
            np.asarray(current, dtype=float)[:2]
            - np.asarray(waypoints[0]['point'], dtype=float)[:2]))

    def restore_dynamic_search_after_false_suv(
            self, false_tracker: int,
            other_tracker_exists: bool) -> None:
        false_tracker = int(false_tracker)
        self.static_route_commitment.pop(false_tracker, None)
        self.static_route_handoff_started.discard(false_tracker)

        if not other_tracker_exists:
            saved = copy.deepcopy(
                self.pre_tracking_assignment.get(false_tracker, {}))
            role = self.pre_tracking_role.get(
                false_tracker, 'DYNAMIC_INSPECTION')
            self.vehicle_roles[false_tracker] = role
            if saved and str(saved.get('mode', '')).startswith('SEARCH_'):
                route = copy.deepcopy(saved.get('route') or {})
                source_id = str(route.get('route_id', ''))
                progress = int(self.route_progress_history.get(
                    false_tracker, {}).get(source_id, 0))
                points = list(route.get('waypoints', []))
                route['waypoints'] = points[max(0, progress - 1):]
                route['source_route_id'] = source_id
                route['route_id'] = (
                    f'{source_id}_rollback_v{false_tracker}_'
                    f'{int(rospy.Time.now().to_sec() * 1000)}')
                self.publish_assignment(
                    false_tracker, str(saved.get('mode')),
                    route=route,
                    reason=(
                        'suv_camo failed five-second motion validation; '
                        'restore the aircraft previous unfinished route'))
            else:
                owner = (
                    false_tracker
                    if false_tracker in (0, 1) else 0)
                mode, route = self.resume_route_for_owner(
                    owner, false_tracker)
                self.publish_assignment(
                    false_tracker, mode, route=route,
                    reason=(
                        'suv_camo failed motion validation; restore '
                        'the previous inspection role using the nearest '
                        'unfinished dynamic route'))
            return

        free = [false_tracker]
        free.extend(
            int(v) for v, role in self.vehicle_roles.items()
            if role == 'STATIC_SEARCH'
            and int(v) != false_tracker)
        free.extend(
            int(v) for v in self.vehicle_ids
            if int(v) not in self.vehicle_tracking
            and int(v) not in free
            and int(v) != self.maneuver_inspection_vehicle_id)
        free = list(dict.fromkeys(free))[:2]
        if len(free) < 2:
            rospy.logwarn(
                'V6.7.19 false SUV rollback found only %d '
                'free aircraft', len(free))

        # Generate each unfinished left/right route only once. Distribution
        # continuation generation increments a pass counter, so generating it
        # separately for every aircraft candidate would silently skip passes.
        templates = {}
        template_vehicle = free[0] if free else false_tracker
        for owner in (0, 1):
            templates[owner] = self.resume_route_for_owner(
                owner, template_vehicle)

        candidate = {}
        for aircraft in free:
            candidate[aircraft] = {}
            for owner in (0, 1):
                mode, route_template = templates[owner]
                route = copy.deepcopy(route_template)
                altitude = float(self.primary_routes.get(
                    aircraft, route).get(
                        'altitude_m',
                        self.cfg['mission'].get(
                            'search_altitude_m', 40.0)))
                for waypoint in route.get('waypoints', []):
                    point = list(waypoint.get('point', []))
                    if len(point) >= 3:
                        point[2] = altitude
                        waypoint['point'] = point
                route['altitude_m'] = altitude
                route['assigned_vehicle'] = int(aircraft)
                source_route_id = str(route.get(
                    'source_route_id',
                    route.get('route_id', 'dynamic')))
                route['route_id'] = (
                    f'{source_route_id}_rollback_owner{owner}_'
                    f'by_v{aircraft}_'
                    f'{int(rospy.Time.now().to_sec() * 1000)}')
                candidate[aircraft][owner] = (
                    self.route_entry_distance(aircraft, route),
                    mode, route)

        assignment_pairs = []
        if len(free) >= 2:
            a, b = free[:2]
            direct = (
                candidate[a][0][0] + candidate[b][1][0])
            crossed = (
                candidate[a][1][0] + candidate[b][0][0])
            assignment_pairs = (
                [(a, 0), (b, 1)]
                if direct <= crossed
                else [(a, 1), (b, 0)])
        elif free:
            aircraft = free[0]
            owner = min(
                (0, 1),
                key=lambda o: candidate[aircraft][o][0])
            assignment_pairs = [(aircraft, owner)]

        for aircraft, owner in assignment_pairs:
            self.vehicle_roles[int(aircraft)] = (
                'DYNAMIC_INSPECTION')
            self.static_route_commitment.pop(
                int(aircraft), None)
            self.static_route_handoff_started.discard(
                int(aircraft))
            _, mode, route = candidate[aircraft][owner]
            self.publish_assignment(
                int(aircraft), mode, route=route,
                reason=(
                    'suv_camo was stationary and reclassified '
                    'as prius_hybrid_camo; two free aircraft '
                    'resume the nearest unfinished left/right '
                    'dynamic-search routes'))
            rospy.logwarn(
                'V6.7.19 rollback v%d -> route owner v%d '
                'route=%s',
                aircraft, owner, route.get('route_id'))

    def reject_false_suv_assignment(self, vid: int, event_row: dict) -> None:
        name = str(event_row.get('target_name', ''))
        if name != 'suv_camo' or self.dynamic_assignments.get(name) != int(vid):
            return
        estimate = self.estimate_target(name) or {}
        position = event_row.get(
            'reclassified_position_world',
            event_row.get('position_world', estimate.get('position')))
        if not isinstance(position, list) or len(position) < 3:
            rospy.logerr(
                'V6.7.19 cannot rollback false SUV without a finite position')
            return
        position = [float(x) for x in position[:3]]
        cfg = self.cfg.get('tracking', {}).get('suv_false_positive', {})
        radius = float(cfg.get('suppression_radius_m', 20.0))
        seconds = float(cfg.get('suppression_seconds', 1800.0))
        now = rospy.Time.now().to_sec()
        region = {
            'position': position,
            'radius_m': radius,
            'expires_ros_time': now + seconds,
            'source_vehicle_id': int(vid),
        }
        self.suv_false_positive_regions.append(region)

        static_row = {
            'mission_id': self.mission_id,
            'target_name': 'prius_hybrid_camo',
            'kind': 'static',
            'target_world': position,
            'ground_xy': position[:2],
            'confidence': float(event_row.get('mean_confidence', 0.0)),
            'horizontal_std_m': float(event_row.get(
                'horizontal_std_m', 0.0)),
            'vehicle_id': int(vid),
            'ros_time': now,
            'source': 'suv_motion_gate_stationary_reclassification',
            'reclassified_from': 'suv_camo',
            'skip_static_verify': True,
            'precise_without_hover': True,
            'motion_validation': copy.deepcopy(event_row),
        }
        static_row['candidate_id'] = 'SC_prius_hybrid_camo_motion_gate'
        static_row['status'] = 'confirmed'
        static_row['precise_confidence'] = float(static_row.get('confidence', 0.0))
        self.static_candidates['prius_hybrid_camo'] = [copy.deepcopy(static_row)]
        self.static_detected['prius_hybrid_camo'] = copy.deepcopy(static_row)
        self.static_confirmed['prius_hybrid_camo'] = copy.deepcopy(static_row)
        dump_json(
            self.run_dir / 'static_target_reclassified_prius_hybrid_camo.json',
            static_row)
        message = {
            'mission_id': self.mission_id,
            'source_target': 'suv_camo',
            'reclassified_target': 'prius_hybrid_camo',
            'position': position,
            'suppression_radius_m': radius,
            'expires_ros_time': now + seconds,
            'ros_time': now,
        }
        self.reclassification_pub.publish(
            String(data=json.dumps(message, ensure_ascii=False)))

        self.dynamic_assignments.pop('suv_camo', None)
        self.dynamic_assignment_confirmed.pop('suv_camo', None)
        self.dynamic_provisional.pop('suv_camo', None)
        self.external_target_states.pop('suv_camo', None)
        self.dynamic_tracks['suv_camo'] = []
        self.vehicle_tracking.pop(int(vid), None)
        other_tracker_exists = any(
            bool(self.dynamic_assignment_confirmed.get(target, False))
            for target in self.dynamic_assignments)
        rospy.logwarn(
            'V6.7.19 suv_camo stationary for five seconds; '
            'reclassified to prius_hybrid_camo at %s, '
            'other_confirmed_tracker=%s',
            position, other_tracker_exists)
        self.restore_dynamic_search_after_false_suv(
            int(vid), other_tracker_exists)

    def target_state_cb(self, msg: String) -> None:
        """Accept the truth-free multi-UAV visual target-state stream.

        Reliable states from an aircraft other than the assigned tracker are
        immediately forwarded as external information injection.  A state
        inside a five-second SUV false-positive suppression region is ignored.
        """
        try:
            row = json.loads(msg.data)
            mission = str(row.get('mission_id', ''))
            if mission and mission != self.mission_id:
                return
            name = str(row.get('target_name', ''))
            if name not in self.dynamic_targets:
                return
            p = row.get('position')
            v = row.get('velocity', [0.0, 0.0, 0.0])
            if not isinstance(p, list) or len(p) < 3:
                return
            if not all(math.isfinite(float(x)) for x in p[:3] + list(v[:3])):
                return
            if name == 'suv_camo' and self.is_suv_false_region(p):
                return
            normalized = {
                **row,
                'position': [float(x) for x in p[:3]],
                'velocity': [float(x) for x in v[:3]],
                'source_ros_time': float(row.get(
                    'source_ros_time',
                    row.get('ros_time', rospy.Time.now().to_sec()))),
            }
            with self.lock:
                self.external_target_states[name] = normalized
            tracker = self.dynamic_assignments.get(name)
            if tracker is not None and self.reliable_external_state(normalized):
                normalized['external_information_injection'] = (
                    int(normalized.get('source_vehicle_id', tracker))
                    != int(tracker))
                self.target_pubs[int(tracker)].publish(
                    String(data=json.dumps(normalized, ensure_ascii=False)))
        except Exception:
            return

    def _static_row_position(self, row: dict) -> Optional[np.ndarray]:
        raw = row.get('target_world')
        if not isinstance(raw, list) or len(raw) < 3:
            xy = row.get('ground_xy')
            if not isinstance(xy, list) or len(xy) < 2:
                return None
            raw = [xy[0], xy[1], float(
                self.cfg['camera'].get('ground_z_m', 0.2))]
        try:
            point = np.asarray(raw[:3], dtype=float)
        except Exception:
            return None
        return point if np.all(np.isfinite(point)) else None

    def _write_static_candidate_event(self, event: str, **fields) -> None:
        row = {
            'mission_id': self.mission_id,
            'event': event,
            'ros_time': rospy.Time.now().to_sec(),
            **fields,
        }
        with open(self.run_dir / 'static_candidate_events.jsonl',
                  'a', encoding='utf-8') as fp:
            fp.write(json.dumps(row, ensure_ascii=False) + '\n')

    def _fresh_suv_positions(self) -> List[np.ndarray]:
        now = rospy.Time.now().to_sec()
        maximum_age = float(self.static_candidate_cfg.get(
            'prius_suv_confusion', {}).get(
                'maximum_suv_state_age_seconds', 30.0))
        rows = []
        external = self.external_target_states.get('suv_camo')
        if external:
            rows.append(external)
        rows.extend(self.dynamic_tracks.get('suv_camo', [])[-20:])
        out = []
        for row in rows:
            stamp = float(row.get('source_ros_time', row.get(
                'ros_time', now)))
            if now - stamp > maximum_age:
                continue
            point = self._static_row_position({
                'target_world': row.get('position', row.get('target_world')),
                'ground_xy': row.get('ground_xy')})
            if point is not None:
                out.append(point)
        return out

    def _prius_camo_conflicts_with_suv(self, point: np.ndarray) -> bool:
        cfg = dict(self.static_candidate_cfg.get(
            'prius_suv_confusion', {}))
        if not bool(cfg.get('enabled', True)):
            return False
        radius = float(cfg.get('exclusion_radius_m', 45.0))
        return any(float(np.linalg.norm(point[:2] - p[:2])) <= radius
                   for p in self._fresh_suv_positions())

    def _inside_rejected_static_region(
            self, name: str, point: np.ndarray) -> bool:
        now = rospy.Time.now().to_sec()
        active = []
        found = False
        for region in self.static_rejected_regions:
            if float(region.get('expires_ros_time', 0.0)) <= now:
                continue
            active.append(region)
            if (str(region.get('target_name', '')) == name
                    and float(np.linalg.norm(
                        point[:2] - np.asarray(
                            region['position'], dtype=float)[:2]))
                    <= float(region.get('radius_m', 20.0))):
                found = True
        self.static_rejected_regions = active
        return found

    def _refresh_static_detected(self, name: str) -> None:
        candidates = [c for c in self.static_candidates.get(name, [])
                      if str(c.get('status', 'pending'))
                      in ('pending', 'confirmed')]
        if not candidates:
            self.static_detected.pop(name, None)
            self.static_confirmed.pop(name, None)
            return
        confirmed = [c for c in candidates
                     if c.get('status') == 'confirmed']
        pool = confirmed or candidates
        best = max(pool, key=lambda c: float(c.get(
            'precise_confidence', c.get('confidence', 0.0))))
        self.static_detected[name] = copy.deepcopy(best)
        if confirmed:
            self.static_confirmed[name] = copy.deepcopy(best)

    def update_static_candidate(self, row: dict, vid: int) -> str:
        name = str(row.get('target_name', ''))
        point = self._static_row_position(row)
        if name not in self.static_targets or point is None:
            return 'ignored'
        confidence = float(row.get('confidence', 0.0))
        cfg = self.static_candidate_cfg
        if self._inside_rejected_static_region(name, point):
            self._write_static_candidate_event(
                'STATIC_CANDIDATE_SUPPRESSED_REJECTED_REGION',
                target_name=name, vehicle_id=int(vid),
                target_world=point.tolist(), confidence=confidence)
            return 'suppressed'
        if (name == 'prius_hybrid_camo'
                and self._prius_camo_conflicts_with_suv(point)):
            self._write_static_candidate_event(
                'PRIUS_CAMO_UPDATE_REJECTED_NEAR_SUV_CAMO',
                target_name=name, vehicle_id=int(vid),
                target_world=point.tolist(), confidence=confidence,
                exclusion_radius_m=float(cfg.get(
                    'prius_suv_confusion', {}).get(
                        'exclusion_radius_m', 45.0)))
            rospy.logwarn_throttle(
                1.0, 'V6.7.19 reject prius_hybrid_camo update near '
                'fresh suv_camo state')
            return 'suppressed'

        merge_radius = float(cfg.get('nearby_merge_radius_m', 25.0))
        groups = self.static_candidates.setdefault(name, [])
        nearby = None
        nearby_distance = float('inf')
        for candidate in groups:
            if candidate.get('status') == 'rejected':
                continue
            cp = self._static_row_position(candidate)
            if cp is None:
                continue
            distance = float(np.linalg.norm(point[:2] - cp[:2]))
            if distance < nearby_distance:
                nearby_distance = distance
                nearby = candidate
        if nearby is not None and nearby_distance <= merge_radius:
            nearby['report_count'] = int(nearby.get('report_count', 1)) + 1
            nearby['last_report_ros_time'] = rospy.Time.now().to_sec()
            nearby.setdefault('report_event_ids', []).append(
                row.get('report_event_id'))
            if confidence > float(nearby.get('confidence', 0.0)):
                preserved = {
                    'candidate_id': nearby['candidate_id'],
                    'status': nearby.get('status', 'pending'),
                    'report_count': nearby['report_count'],
                    'report_event_ids': nearby['report_event_ids'],
                    'first_report_ros_time': nearby.get(
                        'first_report_ros_time'),
                }
                nearby.clear()
                nearby.update(copy.deepcopy(row))
                nearby.update(preserved)
                nearby['target_world'] = point.tolist()
                nearby['ground_xy'] = point[:2].tolist()
                nearby['confidence'] = confidence
                action = 'nearby_replaced_by_higher_confidence'
            else:
                action = 'nearby_kept_existing_higher_confidence'
            self._refresh_static_detected(name)
            self._write_static_candidate_event(
                'STATIC_CANDIDATE_NEARBY_MERGED', target_name=name,
                candidate_id=nearby['candidate_id'], action=action,
                distance_m=nearby_distance,
                retained_confidence=float(nearby.get('confidence', 0.0)),
                incoming_confidence=confidence)
            return 'updated'

        # prius_hybrid_camo uses one non-conflicting best hypothesis; ordinary
        # static classes retain spatially distinct hypotheses for verification.
        if name == 'prius_hybrid_camo' and groups:
            current = max(groups, key=lambda c: float(c.get(
                'confidence', 0.0)))
            if confidence <= float(current.get('confidence', 0.0)):
                self._write_static_candidate_event(
                    'PRIUS_CAMO_DISTANT_LOWER_CONFIDENCE_IGNORED',
                    target_name=name, target_world=point.tolist(),
                    incoming_confidence=confidence,
                    retained_candidate_id=current.get('candidate_id'),
                    retained_confidence=float(current.get('confidence', 0.0)))
                return 'suppressed'
            for candidate in groups:
                candidate['status'] = 'superseded'

        maximum = max(1, int(cfg.get('maximum_candidates_per_target', 5)))
        active_count = sum(1 for c in groups if c.get('status') == 'pending')
        if active_count >= maximum:
            worst = min((c for c in groups if c.get('status') == 'pending'),
                        key=lambda c: float(c.get('confidence', 0.0)))
            if confidence <= float(worst.get('confidence', 0.0)):
                return 'suppressed'
            worst['status'] = 'superseded'

        self.static_candidate_sequence += 1
        candidate = copy.deepcopy(row)
        candidate_id = (
            f"SC_{name}_{self.static_candidate_sequence:04d}_"
            f"{point[0]:.1f}_{point[1]:.1f}")
        candidate.update({
            'candidate_id': candidate_id,
            'status': 'pending',
            'target_world': point.tolist(),
            'ground_xy': point[:2].tolist(),
            'confidence': confidence,
            'report_count': 1,
            'report_event_ids': [row.get('report_event_id')],
            'first_report_ros_time': rospy.Time.now().to_sec(),
            'last_report_ros_time': rospy.Time.now().to_sec(),
        })
        groups.append(candidate)
        self._refresh_static_detected(name)
        self._write_static_candidate_event(
            'STATIC_CANDIDATE_GROUP_CREATED', target_name=name,
            candidate_id=candidate_id, vehicle_id=int(vid),
            target_world=point.tolist(), confidence=confidence,
            active_candidate_count=active_count + 1)
        return 'created'

    def resolve_static_candidate(self, event_row: dict,
                                 confirmed: bool) -> None:
        name = str(event_row.get('target_name', ''))
        candidate_id = str(event_row.get('candidate_id', ''))
        with self.lock:
            groups = self.static_candidates.setdefault(name, [])
            candidate = next((c for c in groups
                              if str(c.get('candidate_id', '')) == candidate_id),
                             None)
            if candidate is None:
                raw = event_row.get(
                    'frozen_target_world', event_row.get(
                        'target_world', [0.0, 0.0, 0.2]))
                candidate = {
                    'target_name': name,
                    'candidate_id': candidate_id,
                    'target_world': list(raw[:3]),
                    'ground_xy': list(raw[:2]),
                    'confidence': float(event_row.get(
                        'candidate_confidence', 0.0)),
                }
                groups.append(candidate)
            if confirmed:
                refined = event_row.get(
                    'refined_target_world', event_row.get('target_world'))
                candidate['status'] = 'confirmed'
                candidate['target_world'] = [float(x) for x in refined[:3]]
                candidate['ground_xy'] = [
                    float(refined[0]), float(refined[1])]
                candidate['precise_confidence'] = float(event_row.get(
                    'precise_confidence',
                    candidate.get('confidence', 0.0)))
                candidate['precise_event'] = copy.deepcopy(event_row)
            else:
                candidate['status'] = 'rejected'
                candidate['rejection_event'] = copy.deepcopy(event_row)
                point = self._static_row_position(candidate)
                if point is not None:
                    seconds = float(self.static_candidate_cfg.get(
                        'rejected_region_suppression_seconds', 900.0))
                    radius = float(self.static_candidate_cfg.get(
                        'rejected_region_radius_m', 25.0))
                    self.static_rejected_regions.append({
                        'target_name': name,
                        'position': point.tolist(),
                        'radius_m': radius,
                        'expires_ros_time': (
                            rospy.Time.now().to_sec() + seconds),
                    })
            self._refresh_static_detected(name)
            snapshot = {
                'mission_id': self.mission_id,
                'candidates': copy.deepcopy(self.static_candidates),
                'confirmed': copy.deepcopy(self.static_confirmed),
                'rejected_regions': copy.deepcopy(
                    self.static_rejected_regions),
            }
        dump_json(self.run_dir / 'static_candidate_state.json', snapshot)

    def pending_static_candidate_rows(self) -> List[dict]:
        rows = []
        candidate_lock = getattr(self, 'lock', threading.RLock())
        with candidate_lock:
            candidate_map = getattr(self, 'static_candidates', {})
            # Backward-compatible in-memory upgrade for restored pre-V6.7.19
            # state and the package regression harness.
            if not candidate_map:
                candidate_map = {name: [] for name in self.static_targets}
            for name, detected in getattr(
                    self, 'static_detected', {}).items():
                if candidate_map.get(name):
                    continue
                row = copy.deepcopy(detected)
                point = self._static_row_position(row)
                if point is None:
                    point = np.asarray([
                        0.0, 0.0,
                        float(self.cfg.get('camera', {}).get(
                            'ground_z_m', 0.2))])
                    row['target_world'] = point.tolist()
                    row['ground_xy'] = point[:2].tolist()
                row.setdefault('target_name', name)
                row.setdefault(
                    'candidate_id',
                    f'SC_LEGACY_{name}_{point[0]:.1f}_{point[1]:.1f}')
                row.setdefault(
                    'status',
                    'confirmed' if name in getattr(
                        self, 'static_confirmed', {}) else 'pending')
                candidate_map.setdefault(name, []).append(row)
            self.static_candidates = candidate_map
            for name in self.static_targets:
                for candidate in candidate_map.get(name, []):
                    if candidate.get('status', 'pending') != 'pending':
                        continue
                    row = copy.deepcopy(candidate)
                    frozen = self._static_row_position(row)
                    if frozen is None:
                        continue
                    row['frozen_target_world'] = frozen.tolist()
                    row['position_lock'] = (
                        'candidate_frozen_until_hover_refinement')
                    rows.append(row)
        return rows

    def detection_cb(self, msg: String, vid: int) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') != self.mission_id:
                return
            if not bool(row.get('selected_as_management_result', False)):
                return
        except Exception:
            return
        with open(self.run_dir / 'all_detections.jsonl', 'a',
                  encoding='utf-8') as fp:
            fp.write(json.dumps(row, ensure_ascii=False) + '\n')
        new_static = False
        with self.lock:
            self.detections.append(row)
            name = str(row.get('target_name', ''))
            kind = str(row.get('kind', ''))
            if kind == 'static' and name in self.static_targets:
                candidate_action = self.update_static_candidate(row, vid)
                # A nearby higher-confidence update changes the retained report
                # but must not interrupt an active precise-verification flight.
                new_static = candidate_action == 'created'
                if candidate_action in ('created', 'updated'):
                    rospy.loginfo(
                        'V6.7.19 static candidate %s target=%s by v%d',
                        candidate_action, name, vid)
            elif kind == 'dynamic' and name in self.dynamic_targets:
                position = row.get(
                    'target_world',
                    [*row.get('ground_xy', [0.0, 0.0]), 0.2])
                if name == 'suv_camo' and self.is_suv_false_region(position):
                    rospy.logwarn_throttle(
                        2.0,
                        'V6.7.19 suppress repeated suv_camo detection '
                        'inside reclassified prius_hybrid_camo region')
                    return
                track = self.dynamic_tracks.setdefault(name, [])
                track.append(row)
                if len(track) > 2000:
                    del track[:-2000]
                if name not in self.dynamic_assignments:
                    self.assign_dynamic_target(
                        name, detector_vehicle=vid)
        if new_static:
            state = self.static_pair_state()
            static_vid = self.remaining_static_vehicle()
            current_mode = (
                '' if static_vid is None
                else str(self.assignments.get(
                    static_vid, {}).get('mode', '')))
            force = (
                (not state['both_missing_pairs'])
                or current_mode == 'SEARCH_STATIC_PAIR_GUIDED')
            self.maybe_assign_static_role(force_replan=force)

    def estimate_target(self, name: str) -> Optional[dict]:
        source_mode = str(self.cfg.get(
            'tracking', {}).get(
                'target_stream_source', 'external_track_topic'))
        if source_mode == 'external_track_topic' and name in self.dynamic_assignments:
            with self.lock:
                row = dict(self.external_target_states.get(name, {}))
            if row:
                result = {
                    'mission_id': self.mission_id,
                    'target_name': name,
                    'position': row['position'],
                    'velocity': row.get('velocity', [0.0, 0.0, 0.0]),
                    'source_ros_time': float(
                        row.get('source_ros_time', 0.0)),
                    'manager_ros_time': rospy.Time.now().to_sec(),
                    'source': str(
                        row.get('source', 'external_track_topic')),
                    'source_vehicle_id': row.get('source_vehicle_id'),
                    'confidence': float(row.get('confidence', 0.0)),
                    'horizontal_std_m': float(
                        row.get('horizontal_std_m', 0.0)),
                    'measurement_id': row.get('measurement_id', ''),
                    'measurement_age_seconds': float(
                        row.get('measurement_age_seconds', 0.0)),
                    'external_injection_reliable': bool(
                        row.get('external_injection_reliable', False)),
                    'track_reinitialized': bool(
                        row.get('track_reinitialized', False)),
                }
                tracker = self.dynamic_assignments.get(name)
                result['external_information_injection'] = bool(
                    tracker is not None
                    and result.get('source_vehicle_id') is not None
                    and int(result['source_vehicle_id']) != int(tracker)
                    and self.reliable_external_state(result))
                return result
        track = self.dynamic_tracks.get(name, [])
        if not track:
            return None
        last = track[-1]
        p = np.asarray(
            last.get(
                'target_world',
                [*last.get('ground_xy', [0, 0]), 0.2]),
            dtype=float)
        vel = np.zeros(3)
        if len(track) >= 2:
            a, b = track[-2], track[-1]
            pa = np.asarray(
                a.get(
                    'target_world',
                    [*a.get('ground_xy', [0, 0]), 0.2]),
                dtype=float)
            ta = float(a.get('ros_time', 0.0))
            tb = float(b.get('ros_time', 0.0))
            if tb - ta > 1e-3:
                vel = (p - pa) / (tb - ta)
                speed = float(np.linalg.norm(vel[:2]))
                if speed > 3.0:
                    vel *= 3.0 / speed
        source_stamp = float(
            last.get('ros_time', rospy.Time.now().to_sec()))
        return {
            'mission_id': self.mission_id,
            'target_name': name,
            'position': p.tolist(),
            'velocity': vel.tolist(),
            'source_ros_time': source_stamp,
            'manager_ros_time': rospy.Time.now().to_sec(),
            'source': 'fov_detection_extrapolation',
            'source_vehicle_id': int(last.get('vehicle_id', -1)),
            'confidence': float(last.get('confidence', 0.0)),
            'horizontal_std_m': float(
                last.get('horizontal_std_m', 0.0)),
            'external_injection_reliable': False,
        }

    def assign_dynamic_target(
            self, name: str, detector_vehicle: int) -> None:
        """Assign a provisional or confirmed dynamic tracker.

        ``person_red`` is unambiguous and locks immediately. ``suv_camo`` is
        provisional until the tracker reports five seconds of actual motion.
        """
        if name in self.dynamic_assignments:
            return
        if len(self.dynamic_assignments) == 0:
            tracker = self.maneuver_inspection_vehicle_id
            reason = (
                'first dynamic target: manoeuvre-inspection aircraft becomes '
                'a dynamic tracker')
        else:
            candidates = [
                v for v in self.dynamic_inspection_vehicle_ids
                if v not in self.vehicle_tracking
            ]
            tracker = (
                detector_vehicle if detector_vehicle in candidates
                else (candidates[0] if candidates else None))
            if tracker is None:
                return
            reason = (
                'second dynamic target: detecting free dynamic-inspection '
                'aircraft becomes the second dynamic tracker')
        tracker = int(tracker)
        self.pre_tracking_assignment[tracker] = copy.deepcopy(
            self.assignments.get(tracker, {}))
        self.pre_tracking_role[tracker] = str(
            self.vehicle_roles.get(tracker, 'DYNAMIC_INSPECTION'))
        self.dynamic_assignments[name] = tracker
        self.vehicle_tracking[tracker] = name
        self.vehicle_roles[tracker] = 'DYNAMIC_TRACKING'
        confirmed = name != 'suv_camo'
        self.dynamic_assignment_confirmed[name] = confirmed
        self.dynamic_provisional[name] = {
            'target_name': name,
            'tracker': tracker,
            'detector_vehicle': int(detector_vehicle),
            'confirmed': bool(confirmed),
            'prior_role': self.pre_tracking_role[tracker],
            'prior_assignment': copy.deepcopy(
                self.pre_tracking_assignment[tracker]),
            'assigned_ros_time': rospy.Time.now().to_sec(),
        }
        estimate = self.estimate_target(name)
        extra = {
            'provisional_dynamic_target': not confirmed,
            'motion_lock_required': name == 'suv_camo',
            'dynamic_lock_confirmed': confirmed,
            'previous_functional_role': self.pre_tracking_role[tracker],
            'previous_assignment_mode': self.pre_tracking_assignment[
                tracker].get('mode'),
        }
        self.publish_assignment(
            tracker, 'TRACK_DYNAMIC',
            target_name=name, target_estimate=estimate,
            reason=(
                reason if confirmed else
                reason + '; suv_camo remains provisional until five-second '
                'motion validation succeeds'),
            extra_fields=extra)
        rospy.logwarn(
            'V6.7.19 assignment dynamic=%s tracker=v%d detector=v%d '
            'provisional=%s', name, tracker, detector_vehicle, not confirmed)

        # Do not lock the remaining aircraft into static search while suv_camo
        # is still provisional. It keeps its current useful dynamic route.
        if self.all_dynamic_tracks_confirmed():
            remaining = self.remaining_static_vehicle()
            if remaining is not None:
                self.vehicle_roles[int(remaining)] = 'STATIC_SEARCH'
                if int(remaining) in self.assignments:
                    self.assignments[int(remaining)][
                        'functional_role'] = 'STATIC_SEARCH'
            self.maybe_assign_static_role()

    def remaining_static_vehicle(self) -> Optional[int]:
        available = [
            v for v in self.dynamic_inspection_vehicle_ids
            if v not in self.vehicle_tracking
        ]
        if not available:
            return None
        return min(available)


    def current_assignment_route_id(self, vid: int) -> str:
        row = self.assignments.get(int(vid), {})
        route = row.get('route') or {}
        return str(route.get('route_id', ''))

    def route_commitment_enabled(self) -> bool:
        return bool(self.cfg.get('static_search', {}).get(
            'preserve_current_route_after_two_dynamic_targets', True))

    def committed_static_route_active(self, vid: int) -> bool:
        row = self.static_route_commitment.get(int(vid)) or {}
        return bool(row.get('active', False))

    def begin_static_route_commitment(self, vid: int) -> bool:
        """Conditionally preserve the remaining aircraft's current route.

        The route is preserved only while it remains the best static-search
        action.  It may be released when all static targets are detected, every
        missing target becomes pair-guidable, its remaining public-static-prior
        cells are exhausted, the assignment becomes stale, or a safety/deadline
        transition supersedes search.  Release normally occurs at the next
        waypoint boundary instead of cutting a detection-valid straight leg.
        """
        vid = int(vid)
        if not self.route_commitment_enabled() or vid in self.static_route_handoff_started:
            return False
        existing = self.static_route_commitment.get(vid) or {}
        if bool(existing.get('active', False)):
            return True
        assignment = self.assignments.get(vid, {})
        mode = str(assignment.get('mode', ''))
        route = assignment.get('route') or {}
        route_id = str(route.get('route_id', ''))
        coverage_modes = {
            'SEARCH_DYNAMIC_LEFT', 'SEARCH_DYNAMIC_RIGHT',
            'SEARCH_DYNAMIC_DISTRIBUTION_LEFT',
            'SEARCH_DYNAMIC_DISTRIBUTION_RIGHT',
            'SEARCH_STATIC_CENTER_OUT',
        }
        if mode not in coverage_modes or not route_id:
            return False
        if (vid, route_id) in self.route_completed_history:
            return False
        progress = int(self.route_progress_history.get(vid, {}).get(route_id, 0))
        total = len(route.get('waypoints', []))
        commitment = {
            'active': True,
            'conditional': True,
            'stage': 'PRESERVE_CURRENT_OWN_ROUTE',
            'vehicle_id': vid,
            'committed_mode': mode,
            'committed_route_id': route_id,
            'committed_route_progress': progress,
            'committed_route_waypoints': total,
            'opposite_owner_vehicle': self.opposite_side_vehicle(vid),
            'started_ros_time': rospy.Time.now().to_sec(),
            'started_wall_time': time.monotonic(),
            'last_relevance_check_wall': 0.0,
            'pending_transition': None,
            'reason': (
                'two dynamic targets are tracking; preserve the current useful '
                'coverage route unless a higher-value static strategy becomes available'),
        }
        self.static_route_handoff_started.add(vid)
        self.static_route_commitment[vid] = commitment
        self.static_strategy_stage[vid] = 'PRESERVE_CURRENT_OWN_ROUTE'
        with open(self.run_dir / 'static_route_commitment.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(commitment, ensure_ascii=False) + '\n')
        rospy.logwarn(
            'V6.7.19 conditional static-route preserve v%d stage=PRESERVE_CURRENT_OWN_ROUTE '
            'mode=%s route=%s progress=%d/%d; routine replans are blocked, but '
            'VERIFY_STATIC, PAIR_GUIDED and exhausted-remainder transitions remain allowed',
            vid, mode, route_id, progress, total)
        return True

    def committed_route_relevance(self, vid: int, commitment: dict) -> dict:
        """Return uncovered public-static-prior cells relevant to a commitment.

        This is a side-effect-free relevance check.  It does not create a route,
        increment a planner pass, or write a planner result file.
        """
        vid = int(vid)
        stage = str(commitment.get('stage', ''))
        owner = vid if stage == 'PRESERVE_CURRENT_OWN_ROUTE' else int(
            commitment.get('opposite_owner_vehicle', self.opposite_side_vehicle(vid)))
        route = self.primary_routes[int(owner)]
        progress = self.primary_route_progress(int(owner))
        sc = self.cfg['static_search']
        minimum_priority = float(sc.get('precise_remainder_priority_floor', 0.72))
        corridor_half_width = float(sc.get('precise_remainder_corridor_half_width_m', 75.0))
        grid_step = float(sc.get('precise_remainder_grid_resolution_m',
                                 sc.get('risk_grid_resolution_m', 35.0)))
        public_prior = generate_static_prior_grid_points(
            self.cfg, self.plan['model_state_analysis'],
            minimum_priority=minimum_priority, grid_step_m=grid_step)
        candidates, source_segments = filter_static_prior_points_by_route_remainder(
            public_prior, route, progress, corridor_half_width)
        altitude = float(self.primary_routes[vid].get(
            'altitude_m', self.cfg['mission'].get('search_altitude_m', 40.0)))
        uncovered = filter_uncovered_static_prior_points(
            self.cfg, list(self.coverage_samples), candidates,
            assigned_altitude_m=altitude)
        return {
            'owner_vehicle': int(owner),
            'route_id': str(route.get('route_id', '')),
            'progress': int(progress),
            'route_waypoints': len(route.get('waypoints', [])),
            'source_unflown_segments': len(source_segments),
            'candidate_cells': len(candidates),
            'uncovered_cells': len(uncovered),
        }

    def desired_static_commitment_transition(
            self, vid: int, check_relevance: bool = False) -> Optional[dict]:
        """Return a higher-value strategy that may interrupt a preserved route."""
        vid = int(vid)
        commitment = self.static_route_commitment.get(vid) or {}
        if not bool(commitment.get('active', False)):
            return None
        all_static = all(name in self.static_detected for name in self.static_targets)
        if all_static and bool(self.cfg['static_search'].get(
                'allow_verify_interrupt_during_committed_route', True)):
            return {
                'action': 'VERIFY_STATIC',
                'reason': 'all six static targets are detected; route completion no longer adds discovery value',
            }
        pair_state = self.static_pair_state()
        if pair_state['all_missing_pair_guidable'] and bool(self.cfg['static_search'].get(
                'allow_pair_guided_interrupt_during_committed_route', True)):
            return {
                'action': 'PAIR_GUIDED',
                'reason': 'every missing static target now has a detected partner; switch to targeted pair-guided search',
            }
        status = self.flight_status.get(vid, {})
        active_route = str(status.get('route_id', ''))
        committed_route = str(commitment.get('committed_route_id', ''))
        active_mode = str(status.get('assignment_mode', ''))
        if active_route and committed_route and active_route != committed_route:
            return {
                'action': 'NORMAL_STATIC',
                'reason': f'preserved route became stale: active={active_route}, committed={committed_route}',
            }
        if active_mode.startswith(('RETURN', 'FAILED', 'DONE', 'EMERGENCY')):
            return None
        if check_relevance and bool(self.cfg['static_search'].get(
                'allow_empty_remainder_interrupt_during_committed_route', True)):
            relevance = self.committed_route_relevance(vid, commitment)
            threshold = int(self.cfg['static_search'].get(
                'conditional_commitment_uncovered_cell_threshold', 3))
            if int(relevance['uncovered_cells']) <= threshold:
                action = ('OPPOSITE_REMAINDER'
                          if str(commitment.get('stage')) == 'PRESERVE_CURRENT_OWN_ROUTE'
                          else 'NORMAL_STATIC')
                return {
                    'action': action,
                    'reason': (
                        'remaining committed route has no meaningful uncovered '
                        f'public-static-prior cells ({relevance["uncovered_cells"]} <= {threshold})'),
                    'relevance': relevance,
                }
        return None

    def request_static_commitment_transition(
            self, vid: int, transition: dict) -> bool:
        """Queue a strategy upgrade and release at a safe route boundary."""
        vid = int(vid)
        commitment = self.static_route_commitment.get(vid) or {}
        if not bool(commitment.get('active', False)):
            return False
        pending = commitment.get('pending_transition') or {}
        if str(pending.get('action', '')) == str(transition.get('action', '')):
            return self.try_execute_static_commitment_transition(vid)
        status = self.flight_status.get(vid, {})
        pending = {
            **transition,
            'requested_ros_time': rospy.Time.now().to_sec(),
            'requested_wall_time': time.monotonic(),
            'requested_route_id': str(status.get('route_id', commitment.get('committed_route_id', ''))),
            'requested_waypoint_index': int(status.get('route_waypoint_index', 0)),
        }
        commitment['pending_transition'] = pending
        self.static_route_commitment[vid] = commitment
        row = {**commitment, 'event': 'TRANSITION_REQUESTED'}
        with open(self.run_dir / 'static_route_commitment.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(row, ensure_ascii=False) + '\n')
        rospy.logwarn(
            'V6.7.19 conditional static-route transition requested v%d action=%s '
            'route=%s waypoint=%d reason=%s',
            vid, pending.get('action'), pending.get('requested_route_id'),
            pending.get('requested_waypoint_index'), pending.get('reason'))
        return self.try_execute_static_commitment_transition(vid)

    def static_commitment_safe_to_switch(self, vid: int, commitment: dict) -> bool:
        pending = commitment.get('pending_transition') or {}
        if not pending:
            return False
        status = self.flight_status.get(int(vid), {})
        committed_route = str(commitment.get('committed_route_id', ''))
        active_route = str(status.get('route_id', ''))
        if active_route and committed_route and active_route != committed_route:
            return True
        current_index = int(status.get('route_waypoint_index', 0))
        requested_index = int(pending.get('requested_waypoint_index', current_index))
        if current_index > requested_index:
            return True
        valid_types = set(self.cfg.get('straight_detection_gate', {}).get(
            'valid_segment_types', ['SEARCH_STRAIGHT', 'SEARCH_CONNECTOR_STRAIGHT']))
        segment_type = str(status.get('detection_segment_type', ''))
        detection_valid = bool(status.get('detection_valid', False))
        along = status.get('segment_along_m')
        length = status.get('segment_length_m')
        edge = float(self.cfg.get('straight_detection_gate', {}).get(
            'segment_edge_exclusion_m', 35.0))
        if not detection_valid:
            if segment_type and segment_type not in valid_types:
                return True
            if along is not None and length is not None:
                try:
                    if float(along) <= edge + 2.0 or float(length) - float(along) <= edge + 2.0:
                        return True
                except Exception:
                    pass
        maximum_wait = float(self.cfg['static_search'].get(
            'conditional_commitment_max_boundary_wait_seconds', 75.0))
        if time.monotonic() - float(pending.get('requested_wall_time', time.monotonic())) >= maximum_wait:
            rospy.logwarn(
                'V6.7.19 conditional route boundary wait exceeded %.1fs for v%d; '
                'executing higher-value strategy without waiting for another waypoint',
                maximum_wait, vid)
            return True
        return False

    def try_execute_static_commitment_transition(self, vid: int) -> bool:
        vid = int(vid)
        commitment = self.static_route_commitment.get(vid) or {}
        pending = commitment.get('pending_transition') or {}
        if not bool(commitment.get('active', False)) or not pending:
            return False
        if not self.static_commitment_safe_to_switch(vid, commitment):
            return False
        action = str(pending.get('action', 'NORMAL_STATIC'))
        release = {
            **commitment,
            'active': False,
            'event': 'CONDITIONAL_RELEASE',
            'released_ros_time': rospy.Time.now().to_sec(),
            'release_action': action,
            'release_reason': str(pending.get('reason', '')),
        }
        with open(self.run_dir / 'static_route_commitment.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(release, ensure_ascii=False) + '\n')
        self.static_route_commitment.pop(vid, None)
        rospy.logwarn(
            'V6.7.19 released preserved static route v%d action=%s reason=%s',
            vid, action, pending.get('reason'))
        if action in ('VERIFY_STATIC', 'PAIR_GUIDED'):
            self.maybe_assign_static_role(force_replan=True)
            return True
        if action == 'OPPOSITE_REMAINDER':
            if self.assign_committed_opposite_remainder(vid):
                return True
            self.continue_static_after_committed_routes(vid)
            return True
        self.continue_static_after_committed_routes(vid)
        return True

    def update_static_route_commitment_from_status(self, vid: int, _row: dict) -> None:
        """Evaluate pending/priority transitions from the high-rate flight status."""
        vid = int(vid)
        commitment = self.static_route_commitment.get(vid) or {}
        if not bool(commitment.get('active', False)):
            return
        if commitment.get('pending_transition'):
            self.try_execute_static_commitment_transition(vid)
            return
        now = time.monotonic()
        interval = float(self.cfg['static_search'].get(
            'conditional_commitment_relevance_check_seconds', 5.0))
        last = float(commitment.get('last_relevance_check_wall', 0.0))
        check_relevance = now - last >= interval
        if check_relevance:
            commitment['last_relevance_check_wall'] = now
            self.static_route_commitment[vid] = commitment
        transition = self.desired_static_commitment_transition(
            vid, check_relevance=check_relevance)
        if transition:
            self.request_static_commitment_transition(vid, transition)

    def assign_committed_opposite_remainder(self, vid: int) -> bool:
        """Assign the opposite remainder with the same conditional preservation."""
        vid = int(vid)
        opposite = self.opposite_side_vehicle(vid)
        if opposite is None:
            return False
        route = self.build_precise_unfinished_static_route(
            opposite, vid, 'OPPOSITE_REMAINDER')
        if not route or not route.get('waypoints'):
            return False
        reason = (
            'current own coverage is complete or no longer useful; cover only '
            'the opposite side unfinished public-static-prior cells, while '
            'allowing pair-guided/all-static strategy upgrades')
        self.record_static_strategy(vid, 'OPPOSITE_REMAINDER_CONDITIONAL', route, reason)
        self.publish_assignment(
            vid, 'SEARCH_STATIC_OPPOSITE_REMAINDER', route=route, reason=reason)
        self.static_route_commitment[vid] = {
            'active': True,
            'conditional': True,
            'stage': 'PRESERVE_OPPOSITE_REMAINDER',
            'vehicle_id': vid,
            'committed_mode': 'SEARCH_STATIC_OPPOSITE_REMAINDER',
            'committed_route_id': str(route.get('route_id', '')),
            'committed_route_progress': 0,
            'committed_route_waypoints': len(route.get('waypoints', [])),
            'opposite_owner_vehicle': int(opposite),
            'started_ros_time': rospy.Time.now().to_sec(),
            'started_wall_time': time.monotonic(),
            'last_relevance_check_wall': 0.0,
            'pending_transition': None,
            'reason': reason,
        }
        with open(self.run_dir / 'static_route_commitment.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(self.static_route_commitment[vid], ensure_ascii=False) + '\n')
        rospy.logwarn(
            'V6.7.19 conditional static-route preserve v%d '
            'stage=PRESERVE_OPPOSITE_REMAINDER owner=v%d route=%s waypoints=%d',
            vid, opposite, route.get('route_id'), len(route.get('waypoints', [])))
        return True

    def finish_static_route_commitment(self, vid: int, row: dict) -> bool:
        """Advance a conditionally preserved route after ROUTE_COMPLETE."""
        vid = int(vid)
        commitment = self.static_route_commitment.get(vid) or {}
        if not bool(commitment.get('active', False)):
            return False
        completed_route = str(row.get('route_id', ''))
        committed_route = str(commitment.get('committed_route_id', ''))
        if not completed_route or completed_route != committed_route:
            rospy.logwarn(
                'V6.7.19 ignoring unrelated ROUTE_COMPLETE while conditional '
                'route preservation is active v%d completed=%s committed=%s',
                vid, completed_route, committed_route)
            return False
        stage = str(commitment.get('stage', ''))
        completion_row = {
            **commitment,
            'active': False,
            'event': 'ROUTE_COMPLETE_RELEASE',
            'completed_ros_time': rospy.Time.now().to_sec(),
            'completed_route_id': completed_route,
        }
        with open(self.run_dir / 'static_route_commitment.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(completion_row, ensure_ascii=False) + '\n')
        self.static_route_commitment.pop(vid, None)
        if stage == 'PRESERVE_CURRENT_OWN_ROUTE':
            rospy.logwarn(
                'V6.7.19 v%d completed its preserved current route %s; evaluate '
                'whether opposite remainder, pair-guided search or verification is now best',
                vid, completed_route)
            if all(name in self.static_detected for name in self.static_targets):
                self.maybe_assign_static_role(force_replan=True)
                return True
            if self.static_pair_state()['all_missing_pair_guidable']:
                self.maybe_assign_static_role(force_replan=True)
                return True
            if self.assign_committed_opposite_remainder(vid):
                return True
            self.continue_static_after_committed_routes(vid)
            return True
        if stage == 'PRESERVE_OPPOSITE_REMAINDER':
            rospy.logwarn(
                'V6.7.19 v%d completed preserved opposite-side remainder %s; '
                'resume normal static strategy selection', vid, completed_route)
            self.continue_static_after_committed_routes(vid)
            return True
        return False

    def continue_static_after_committed_routes(self, vid: int) -> None:
        """Resume the existing static logic after conditional preservation."""
        if all(name in self.static_detected for name in self.static_targets):
            self.maybe_assign_static_role(force_replan=True)
            return
        state = self.static_pair_state()
        if state['all_missing_pair_guidable']:
            self.maybe_assign_static_role(force_replan=True)
            return
        route = self.build_static_residual_route(int(vid))
        route['strategy_stage'] = 'GLOBAL_GAP_FILL'
        reason = (
            'conditional own/opposite coverage no longer requires preservation; '
            'continue unchanged three-UAV-FOV global gap-fill logic')
        self.record_static_strategy(int(vid), 'GLOBAL_GAP_FILL', route, reason)
        self.publish_assignment(
            int(vid), 'SEARCH_STATIC_GLOBAL_GAP_FILL', route=route, reason=reason)

    def static_pair_state(self) -> dict:
        missing = {name for name in self.static_targets if name not in self.static_detected}
        both_missing = []
        guided = []
        complete = []
        for left, right in self.static_target_pairs:
            left_missing = left in missing
            right_missing = right in missing
            if left_missing and right_missing:
                both_missing.append([left, right])
            elif left_missing:
                guided.append({'missing_target': left, 'detected_partner': right})
            elif right_missing:
                guided.append({'missing_target': right, 'detected_partner': left})
            else:
                complete.append([left, right])
        return {
            'missing_targets': sorted(missing),
            'both_missing_pairs': both_missing,
            'pair_guided_missing': guided,
            'complete_pairs': complete,
            'all_missing_pair_guidable': bool(missing) and not both_missing,
        }

    def detected_static_point(self, name: str) -> Optional[np.ndarray]:
        row = self.static_detected.get(name)
        if not row:
            return None
        raw = row.get('target_world')
        if not isinstance(raw, list) or len(raw) < 3:
            ground = row.get('ground_xy')
            if not isinstance(ground, list) or len(ground) < 2:
                return None
            raw = [ground[0], ground[1], float(self.cfg['camera'].get('ground_z_m', 0.2))]
        point = np.asarray(raw[:3], dtype=float)
        return point if np.all(np.isfinite(point)) else None

    def opposite_side_vehicle(self, assigned_vehicle: int) -> Optional[int]:
        if assigned_vehicle == 0:
            return 1
        if assigned_vehicle == 1:
            return 0
        # This branch is not normally reached because vehicle 2 takes the first
        # dynamic target. Keep a deterministic fallback for robustness.
        candidates = [v for v in (0, 1) if v != assigned_vehicle]
        return candidates[0] if candidates else None

    def primary_route_progress(self, owner: int) -> int:
        """Return preserved progress on an aircraft's original search route."""
        route = self.primary_routes[int(owner)]
        route_id = str(route.get('route_id', ''))
        if (int(owner), route_id) in self.route_completed_history:
            return len(route.get('waypoints', []))
        progress = int(self.route_progress_history.get(int(owner), {}).get(route_id, 0))
        status = self.flight_status.get(int(owner), {})
        if str(status.get('route_id', '')) == route_id:
            progress = max(progress, int(status.get('route_waypoint_index', 0)))
        return max(0, min(progress, len(route.get('waypoints', []))))

    def build_precise_unfinished_static_route(
            self, owner: int, assigned_vehicle: int, stage: str) -> dict:
        """Search only static-prior cells in one owner's unflown corridors.

        The old implementation copied the remaining dynamic-search waypoints.
        Those routes were intentionally sized for moving targets and could span
        the entire safe world. V6.7.19 instead performs four intersections:

        public static high-risk prior
        ∩ owner's still-unflown valid search corridors
        − all three aircraft's actual FOV coverage
        = precise static remainder route.

        No target truth, dynamic-target position, or target start point enters
        this calculation.
        """
        owner = int(owner)
        assigned_vehicle = int(assigned_vehicle)
        route = self.primary_routes[owner]
        progress = self.primary_route_progress(owner)
        sc = self.cfg['static_search']
        minimum_priority = float(sc.get('precise_remainder_priority_floor', 0.72))
        corridor_half_width = float(sc.get('precise_remainder_corridor_half_width_m', 75.0))
        grid_step = float(sc.get('precise_remainder_grid_resolution_m',
                                 sc.get('risk_grid_resolution_m', 35.0)))
        max_length = float(sc.get('precise_remainder_max_route_length_m', 4200.0))
        public_prior = generate_static_prior_grid_points(
            self.cfg, self.plan['model_state_analysis'],
            minimum_priority=minimum_priority, grid_step_m=grid_step)
        candidate_points, source_segments = filter_static_prior_points_by_route_remainder(
            public_prior, route, progress, corridor_half_width)
        cruise_altitude = float(self.primary_routes[assigned_vehicle].get(
            'altitude_m', self.cfg['mission'].get('search_altitude_m', 40.0)))
        current = self.flight_status.get(assigned_vehicle, {}).get('world_position') or [
            0.0, 0.0, cruise_altitude]
        focus_metadata = [{
            'shape': 'rectangle',
            'name': f'{stage.lower()}_public_static_prior_corridor',
            'x_min': float(self.cfg['search_area']['safe_x_min']),
            'x_max': float(self.cfg['search_area']['safe_x_max']),
            'y_min': float(self.cfg['search_area']['safe_y_min']),
            'y_max': float(self.cfg['search_area']['safe_y_max']),
            'priority': minimum_priority,
            'source': f'public_static_prior_intersect_unflown_route_v{owner}',
            'target_names': [],
        }]
        planner_pass = self.static_residual_pass_index
        self.static_residual_pass_index += 1
        precise = generate_static_gap_route(
            self.cfg,
            self.plan['model_state_analysis'],
            list(self.coverage_samples),
            current,
            pass_index=planner_pass,
            focus_regions=focus_metadata,
            assigned_altitude_m=cruise_altitude,
            focus_grid_points=candidate_points,
            priority_floor_override=minimum_priority,
            max_route_length_override_m=max_length,
        )
        precise['route_id'] = (
            f'static_precise_{stage.lower()}_owner{owner}_v{assigned_vehicle}_'
            f'{int(rospy.Time.now().to_sec())}'
        )
        precise['strategy_stage'] = str(stage)
        precise['source_owner_vehicle'] = owner
        precise['source_route_id'] = route.get('route_id')
        precise['source_progress_index'] = progress
        precise['source_route_waypoint_count'] = len(route.get('waypoints', []))
        precise['source_unflown_segment_count'] = len(source_segments)
        precise['source_unflown_segments'] = source_segments
        precise['public_prior_candidate_count_before_corridor'] = len(public_prior)
        precise['public_prior_candidate_count_after_corridor'] = len(candidate_points)
        precise['precise_remainder_priority_floor'] = minimum_priority
        precise['precise_remainder_corridor_half_width_m'] = corridor_half_width
        precise['planner'] = (
            'public_static_prior_intersection_unflown_route_corridor_'
            'minus_three_uav_fov'
        )
        precise['description'] = (
            'static-only precise remainder; never continues the original '
            'moving-target route verbatim'
        )
        self.static_residual_routes.append(precise)
        dump_json(
            self.run_dir /
            f'static_precise_{stage.lower()}_owner{owner}_pass_{planner_pass:02d}.json',
            precise,
        )
        rospy.logwarn(
            'V6.7.19 precise static %s v%d owner=v%d progress=%d/%d '
            'segments=%d prior=%d corridor=%d uncovered=%d runs=%d length=%.1fm',
            stage, assigned_vehicle, owner, progress, len(route.get('waypoints', [])),
            len(source_segments), len(public_prior), len(candidate_points),
            int(precise.get('uncovered_grid_points', 0)),
            len(precise.get('scan_runs', [])),
            float(precise.get('planned_route_length_m', 0.0)),
        )
        return precise

    def record_static_strategy(self, vid: int, stage: str, route: Optional[dict], reason: str) -> None:
        self.static_strategy_stage[int(vid)] = str(stage)
        row = {
            'ros_time': rospy.Time.now().to_sec(),
            'vehicle_id': int(vid),
            'stage': str(stage),
            'route_id': None if route is None else route.get('route_id'),
            'reason': reason,
            'pair_state': self.static_pair_state(),
        }
        self.static_strategy_history.append(row)
        with open(self.run_dir / 'static_strategy_history.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(row, ensure_ascii=False) + '\n')

    def build_static_pair_guided_regions(self) -> List[dict]:
        state = self.static_pair_state()
        base_radius = float(self.cfg['static_search'].get('pair_search_radius_m', 500.0))
        margin = float(self.cfg['static_search'].get('pair_search_localization_margin_m', 30.0))
        radius = base_radius + max(0.0, margin)
        priority = float(self.cfg['static_search'].get('pair_guided_priority', 2.0))
        regions = []
        for item in state['pair_guided_missing']:
            partner = item['detected_partner']
            point = self.detected_static_point(partner)
            if point is None:
                continue
            regions.append({
                'shape': 'circle',
                'name': f"pair_{item['missing_target']}_around_{partner}",
                'center_x': float(point[0]),
                'center_y': float(point[1]),
                'radius_m': radius,
                'priority': priority,
                'source': f"detected_static_pair:{partner}",
                'target_names': [item['missing_target']],
                'detected_partner': partner,
                'pair_relation_max_distance_m': base_radius,
                'localization_margin_m': margin,
            })
        return regions

    def build_static_pair_guided_route(self, assigned_vehicle: int) -> dict:
        current = self.flight_status.get(assigned_vehicle, {}).get('world_position') or [
            0.0, 0.0, float(self.cfg['mission']['search_altitude_m'])]
        regions = self.build_static_pair_guided_regions()
        cruise_altitude = float(self.primary_routes[assigned_vehicle].get(
            'altitude_m', self.cfg['mission'].get('search_altitude_m', 40.0)))
        state = self.static_pair_state()
        signature = tuple(sorted(
            (item['missing_target'], item['detected_partner'])
            for item in state['pair_guided_missing']))
        if signature != self.static_pair_guided_signature:
            self.static_pair_guided_signature = signature
            self.static_pair_guided_pass_index = 0
        if not regions:
            return {
                'route_id': f'static_pair_guided_empty_{int(rospy.Time.now().to_sec())}',
                'role': 'STATIC_RESIDUAL', 'altitude_m': cruise_altitude, 'waypoints': [],
                'strategy_stage': 'PAIR_GUIDED', 'pair_state': state,
                'scan_runs': [], 'candidate_grid_points': 0, 'uncovered_grid_points': 0,
                'description': 'no valid detected-partner coordinates available for pair-guided search',
            }

        sc = self.cfg['static_search']
        initial_floor = float(sc.get('pair_guided_initial_priority_floor', 0.72))
        fallback_floor = float(sc.get('pair_guided_fallback_priority_floor', 0.40))
        floor = initial_floor if self.static_pair_guided_pass_index == 0 else fallback_floor
        grid_step = float(sc.get('pair_guided_grid_resolution_m',
                                 sc.get('risk_grid_resolution_m', 35.0)))
        max_length = float(sc.get('pair_guided_max_route_length_m', 3600.0))

        def plan_with_floor(minimum_priority: float) -> dict:
            public_prior = generate_static_prior_grid_points(
                self.cfg, self.plan['model_state_analysis'],
                minimum_priority=minimum_priority, grid_step_m=grid_step)
            focused_points = filter_static_prior_points_by_regions(public_prior, regions)
            planner_pass = self.static_residual_pass_index
            self.static_residual_pass_index += 1
            result = generate_static_gap_route(
                self.cfg,
                self.plan['model_state_analysis'],
                list(self.coverage_samples),
                current,
                pass_index=planner_pass,
                focus_regions=regions,
                assigned_altitude_m=cruise_altitude,
                focus_grid_points=focused_points,
                priority_floor_override=minimum_priority,
                max_route_length_override_m=max_length,
            )
            result['public_prior_candidate_count'] = len(public_prior)
            result['pair_intersection_candidate_count'] = len(focused_points)
            result['pair_guided_priority_floor'] = minimum_priority
            return result

        route = plan_with_floor(floor)
        relaxed = False
        if not route.get('waypoints') and floor > fallback_floor + 1e-9:
            # Do not stall when the high-priority core/buffer intersection was
            # already covered. Relax only inside the same partner-centred local
            # regions; never fall back to a whole-map sweep here.
            route = plan_with_floor(fallback_floor)
            floor = fallback_floor
            relaxed = True

        index = self.static_pair_guided_pass_index
        self.static_pair_guided_pass_index += 1
        route['route_id'] = (
            f'static_pair_guided_precise_pass{index:02d}_'
            f'{int(rospy.Time.now().to_sec())}'
        )
        route['assigned_vehicle'] = int(assigned_vehicle)
        route['strategy_stage'] = 'PAIR_GUIDED'
        route['pair_state'] = state
        route['missing_static_targets'] = [
            name for name in self.static_targets if name not in self.static_detected]
        route['generated_ros_time'] = rospy.Time.now().to_sec()
        route['pair_guided_relaxed_to_outer_prior'] = relaxed
        route['planner'] = (
            'detected_pair_neighbourhood_intersection_public_static_prior_'
            'minus_three_uav_fov'
        )
        self.static_residual_routes.append(route)
        dump_json(self.run_dir / f'static_pair_guided_route_pass_{index:02d}.json', route)
        rospy.logwarn(
            'V6.7.19 precise pair-guided route v%d pass=%d floor=%.2f '
            'regions=%d prior_intersection=%d uncovered=%d runs=%d length=%.1fm '
            'missing=%s',
            assigned_vehicle, index, floor, len(regions),
            int(route.get('pair_intersection_candidate_count', 0)),
            int(route.get('uncovered_grid_points', 0)),
            len(route.get('scan_runs', [])),
            float(route.get('planned_route_length_m', 0.0)),
            route['missing_static_targets'],
        )
        return route

    def assign_static_broad_stage(self, vid: int, stage: str) -> None:
        stage = str(stage)
        route = None
        if stage == 'OWN_REMAINDER':
            route = self.build_precise_unfinished_static_route(vid, vid, stage)
        elif stage == 'OPPOSITE_REMAINDER':
            opposite = self.opposite_side_vehicle(vid)
            if opposite is not None:
                route = self.build_precise_unfinished_static_route(opposite, vid, stage)
        elif stage == 'GLOBAL_GAP_FILL':
            route = self.build_static_residual_route(vid)
            route['strategy_stage'] = stage
        else:
            raise ValueError(f'unknown static broad-search stage {stage}')
        if route is None or not route.get('waypoints'):
            order = list(self.cfg['static_search'].get(
                'broad_search_stage_order', ['OWN_REMAINDER', 'OPPOSITE_REMAINDER', 'GLOBAL_GAP_FILL']))
            try:
                next_stage = order[order.index(stage) + 1]
            except (ValueError, IndexError):
                next_stage = 'GLOBAL_GAP_FILL'
            if next_stage == stage:
                # Last-resort c-stage: the coverage grid says no gaps remain,
                # but an entire static pair is still missing. Execute the
                # configured sparse full residual route once rather than hover.
                fallback = {**self.plan['static_residual_route']}
                fallback['route_id'] = f'static_global_sparse_revisit_v{vid}_{int(rospy.Time.now().to_sec())}'
                fallback['strategy_stage'] = 'GLOBAL_GAP_FILL_REVISIT'
                altitude = float(self.primary_routes[vid].get('altitude_m',
                                  self.cfg['mission'].get('search_altitude_m', 40.0)))
                fallback['altitude_m'] = altitude
                fallback['waypoints'] = [dict(w) for w in fallback.get('waypoints', [])]
                for waypoint in fallback['waypoints']:
                    point = list(waypoint.get('point', []))
                    if len(point) >= 3:
                        point[2] = altitude
                        waypoint['point'] = point
                route = fallback
                stage = 'GLOBAL_GAP_FILL_REVISIT'
            else:
                self.assign_static_broad_stage(vid, next_stage)
                return
        reason = {
            'OWN_REMAINDER': 'paired targets are both missing; first search only public static high-risk cells inside this aircraft unflown corridors',
            'OPPOSITE_REMAINDER': 'paired targets are both missing; next search only public static high-risk cells inside the opposite side aircraft unflown corridors',
            'GLOBAL_GAP_FILL': 'paired targets are both missing; finally cover other actual gaps from the three-UAV FOV map',
            'GLOBAL_GAP_FILL_REVISIT': 'paired targets remain both missing after nominal coverage; execute one sparse full-area revisit instead of hovering',
        }[stage]
        self.record_static_strategy(vid, stage, route, reason)
        self.publish_assignment(vid, f'SEARCH_STATIC_{stage}', route=route, reason=reason)

    def build_dynamic_distribution_continuation(self, vid: int) -> dict:
        """Build the next disjoint own-side outer-to-centre dynamic sweep."""
        vid = int(vid)
        if vid not in (0, 1):
            raise ValueError(f'dynamic distribution continuation is side-aircraft only: v{vid}')
        side = 'left' if vid == 0 else 'right'
        index = int(self.dynamic_distribution_pass_index.get(vid, 0))
        route = generate_dynamic_distribution_inward_route(
            self.cfg, side, self.plan['model_state_analysis'],
            pass_index=index, vehicle_id=vid)
        self.dynamic_distribution_pass_index[vid] = index + 1
        dump_json(self.run_dir / f'dynamic_{side}_distribution_pass_{index:02d}.json', route)
        rospy.logwarn(
            'V6.7.19 dynamic continuation v%d side=%s pass=%d lanes=%d route=%s',
            vid, side, index, len(route.get('lane_x', [])), route.get('route_id'))
        return route

    def build_unfinished_initial_route(self, assigned_vehicle: int) -> dict:
        """Merge only unfinished pieces of the three initial routes.

        This is the residual-search implementation requested by the user: the
        remaining aircraft does not restart a full rectangular sweep. It takes
        the unfinished portions of the left-inner-out, right-inner-out and core-outer-in routes after
        accounting for each aircraft's reported route progress.
        """
        chunks = []
        cruise_altitude = float(self.primary_routes.get(assigned_vehicle, {}).get(
            'altitude_m', self.cfg['mission'].get('search_altitude_m', 40.0)))
        for owner, route in self.primary_routes.items():
            status = self.flight_status.get(owner, {})
            progress = 0
            if str(status.get('route_id', '')) == str(route.get('route_id', '')):
                progress = max(0, int(status.get('route_waypoint_index', 0)) - 1)
            remainder = [dict(w) for w in route.get('waypoints', [])[progress:]]
            for waypoint in remainder:
                point = list(waypoint.get('point', []))
                if len(point) >= 3:
                    point[2] = cruise_altitude
                    waypoint['point'] = point
            if remainder:
                chunks.append({'owner': owner, 'route_id': route.get('route_id'),
                               'progress': progress, 'waypoints': remainder})
        current = np.asarray(self.flight_status.get(assigned_vehicle, {}).get('world_position') or [0, 0, 40], dtype=float)
        ordered = []
        while chunks:
            best = min(chunks, key=lambda c: float(np.linalg.norm(np.asarray(c['waypoints'][0]['point'])[:2]-current[:2])))
            chunks.remove(best)
            ordered.append(best)
            current = np.asarray(best['waypoints'][-1]['point'], dtype=float)
        merged = []
        for chunk in ordered:
            if merged:
                first = dict(chunk['waypoints'][0])
                first['segment_type'] = 'TRANSIT_ENTRY'
                first['detection_valid'] = False
                first['leg_id'] = f"residual_transit_to_{chunk['route_id']}"
                merged.append(first)
                merged.extend(chunk['waypoints'][1:])
            else:
                merged.extend(chunk['waypoints'])
        if not merged:
            return self.plan['static_residual_route']
        return {
            'route_id': f'adaptive_unsearched_initial_v{assigned_vehicle}_{int(rospy.Time.now().to_sec())}',
            'role': 'STATIC_RESIDUAL', 'altitude_m': cruise_altitude,
            'waypoints': merged,
            'source_chunks': [{'owner': c['owner'], 'route_id': c['route_id'], 'progress': c['progress']} for c in ordered],
            'description': 'unfinished initial-route portions retained only as dynamic-search fallback',
        }


    def build_global_static_risk_regions(self) -> List[dict]:
        """Return truth-free, mission-prior static search regions.

        These rectangles are configured before the mission and never depend on
        target detections, dynamic-target positions, initial positions, parent
        models, or Gazebo truth. Nested regions become one grid via max priority.
        """
        regions=[]
        safe=self.cfg['search_area']
        for raw in self.cfg['static_search'].get('high_risk_regions',[]):
            row={
                'shape':'rectangle','name':str(raw.get('name','risk')),
                'x_min':max(float(safe['safe_x_min']),float(raw['x_min'])),
                'x_max':min(float(safe['safe_x_max']),float(raw['x_max'])),
                'y_min':max(float(safe['safe_y_min']),float(raw['y_min'])),
                'y_max':min(float(safe['safe_y_max']),float(raw['y_max'])),
                'priority':float(raw.get('priority',1.0)),
                'source':'pre_mission_global_static_prior',
                'target_names':[],
            }
            if row['x_min']<=row['x_max'] and row['y_min']<=row['y_max']:
                regions.append(row)
        if not regions:
            regions=[{'shape':'rectangle','name':'safe_fallback',
                'x_min':float(safe['safe_x_min']),'x_max':float(safe['safe_x_max']),
                'y_min':float(safe['safe_y_min']),'y_max':float(safe['safe_y_max']),
                'priority':1.0,'source':'configured_safe_region','target_names':[]}]
        snapshot={'ros_time':rospy.Time.now().to_sec(),
            'planner':'global_prior_grid_minus_three_uav_fov',
            'truth_inputs_used':False,'missing_static_targets':[n for n in self.static_targets if n not in self.static_detected],
            'regions':regions}
        self.static_high_risk_history.append(snapshot)
        dump_json(self.run_dir/f"static_global_prior_pass_{self.static_residual_pass_index:02d}.json",snapshot)
        return regions

    def build_static_residual_route(self, assigned_vehicle: int) -> dict:
        """Plan uncovered global-prior cells using only actual three-UAV visual coverage."""
        current=self.flight_status.get(assigned_vehicle,{}).get('world_position') or [0.0,0.0,float(self.cfg['mission']['search_altitude_m'])]
        risk_zones=self.build_global_static_risk_regions()
        cruise_altitude = float(self.primary_routes.get(assigned_vehicle, {}).get(
            'altitude_m', self.cfg['mission'].get('search_altitude_m', 40.0)))
        route=generate_static_gap_route(self.cfg,self.plan['model_state_analysis'],
                                        list(self.coverage_samples),current,
                                        pass_index=self.static_residual_pass_index,
                                        focus_regions=risk_zones,
                                        assigned_altitude_m=cruise_altitude)
        route['assigned_vehicle']=int(assigned_vehicle)
        route['missing_static_targets']=[n for n in self.static_targets if n not in self.static_detected]
        route['generated_ros_time']=rospy.Time.now().to_sec()
        self.static_residual_pass_index += 1
        self.static_residual_routes.append(route)
        p=int(route['pass_index'])
        dump_json(self.run_dir/f'static_global_risk_route_pass_{p:02d}.json',route)
        dump_json(self.run_dir/f'static_high_risk_grid_pass_{p:02d}.json',{
            'route_id':route['route_id'],'focus_regions':route.get('focus_regions',[]),
            'candidate_grid_points':route.get('candidate_grid_points',0),
            'uncovered_grid_points':route.get('uncovered_grid_points',0),
            'component_count':route.get('high_risk_component_count',0),
            'coverage_samples':len(self.coverage_samples),
        })
        rospy.logwarn('V6.7 fast static global-risk route v%d pass=%d coverage=%d uncovered=%d/%d components=%d runs=%d missing=%s',
                      assigned_vehicle,p,len(self.coverage_samples),int(route.get('uncovered_grid_points',0)),
                      int(route.get('candidate_grid_points',0)),int(route.get('high_risk_component_count',0)),
                      len(route.get('scan_runs',[])),route['missing_static_targets'])
        return route

    def maybe_assign_static_role(self, force_replan: bool = False) -> None:
        if not self.all_dynamic_tracks_confirmed():
            return
        vid = self.remaining_static_vehicle()
        if vid is None:
            return
        all_static = all(name in self.static_detected for name in self.static_targets)
        pending_candidates = self.pending_static_candidate_rows()
        pair_state = self.static_pair_state()
        current = str(self.assignments.get(vid, {}).get('mode', ''))

        # V6.7.19: preserving a useful route is the default, not the mission
        # objective.  Higher-value states are allowed to request a safe-boundary
        # interruption.  Routine force_replan calls still cannot cut the route.
        if self.committed_static_route_active(vid):
            transition = self.desired_static_commitment_transition(
                vid, check_relevance=bool(force_replan))
            if transition:
                self.request_static_commitment_transition(vid, transition)
            return

        # If all six targets are already detected, verification is immediately
        # more valuable than completing any broad coverage route.
        if all_static:
            # Every spatially distinct candidate is verified. This prevents a
            # high-confidence first report from hiding a distant alternative.
            if not pending_candidates:
                return
            if current != 'VERIFY_STATIC' or force_replan:
                reason = (
                    'two dynamic targets are tracking and every static class has '
                    'at least one candidate; verify all pending spatial candidate '
                    'groups and retain one highest-confidence precise result per class')
                self.record_static_strategy(vid, 'VERIFY_STATIC', None, reason)
                self.publish_assignment(
                    vid, 'VERIFY_STATIC', static_targets=pending_candidates,
                    reason=reason)
            return

        # If every missing target has a detected partner, pair-guided search is
        # immediately more selective than finishing a broad own/opposite route.
        if pair_state['all_missing_pair_guidable']:
            if current != 'SEARCH_STATIC_PAIR_GUIDED' or force_replan:
                route = self.build_static_pair_guided_route(vid)
                if route.get('waypoints'):
                    reason = (
                        'every missing static target now has its paired static target detected; '
                        'search only uncovered cells inside the observed-partner 500m neighbourhoods')
                    self.record_static_strategy(vid, 'PAIR_GUIDED', route, reason)
                    self.publish_assignment(vid, 'SEARCH_STATIC_PAIR_GUIDED', route=route, reason=reason)
                else:
                    route = self.build_static_pair_guided_route(vid)
                    if route.get('waypoints'):
                        self.record_static_strategy(
                            vid, 'PAIR_GUIDED_REVISIT', route,
                            'targeted pair region revisit')
                        self.publish_assignment(
                            vid, 'SEARCH_STATIC_PAIR_GUIDED', route=route,
                            reason='targeted pair region revisit after nominal coverage was exhausted')
            return

        # Start conditional preservation only when broad coverage is still the
        # correct strategy.  Once handoff has started, this guard is not recreated.
        if self.begin_static_route_commitment(vid):
            return

        if current.startswith('SEARCH_STATIC_') and not force_replan:
            return
        self.assign_static_broad_stage(vid, 'OWN_REMAINDER')

    def handle_route_complete(self, vid: int, _row: dict) -> None:
        # The atomic static handoff owns the matching ROUTE_COMPLETE event.
        # Handle it before any normal dynamic/static transition so the manager
        # cannot replan or publish a replacement route mid-coverage.
        if self.finish_static_route_commitment(vid, _row):
            return
        current = str(self.assignments.get(vid, {}).get('mode', ''))
        if current in ('SEARCH_DYNAMIC_LEFT', 'SEARCH_DYNAMIC_RIGHT',
                       'SEARCH_DYNAMIC_DISTRIBUTION_LEFT',
                       'SEARCH_DYNAMIC_DISTRIBUTION_RIGHT') \
                and len(self.dynamic_assignments) < len(self.dynamic_targets):
            if int(vid) in (0, 1):
                route = self.build_dynamic_distribution_continuation(vid)
                side = 'LEFT' if int(vid) == 0 else 'RIGHT'
                self.publish_assignment(
                    vid, f'SEARCH_DYNAMIC_DISTRIBUTION_{side}', route=route,
                    reason='initial dynamic route complete and target remains missing; cover only this aircraft own distribution half from outer edge toward centre')
            return
        if not current.startswith('SEARCH_STATIC_'):
            return
        if all(name in self.static_detected for name in self.static_targets):
            self.maybe_assign_static_role(force_replan=True)
            return
        state = self.static_pair_state()
        if state['all_missing_pair_guidable']:
            self.maybe_assign_static_role(force_replan=True)
            return
        if current == 'SEARCH_STATIC_OWN_REMAINDER':
            self.assign_static_broad_stage(vid, 'OPPOSITE_REMAINDER')
        elif current == 'SEARCH_STATIC_OPPOSITE_REMAINDER':
            self.assign_static_broad_stage(vid, 'GLOBAL_GAP_FILL')
        else:
            # Pair-guided or global gap-fill passes are regenerated from the
            # latest three-UAV coverage until the target set becomes complete.
            if current == 'SEARCH_STATIC_PAIR_GUIDED':
                self.maybe_assign_static_role(force_replan=True)
            else:
                route = self.build_static_residual_route(vid)
                route['strategy_stage'] = 'GLOBAL_GAP_FILL'
                self.record_static_strategy(vid, 'GLOBAL_GAP_FILL', route,
                                            'global gap-fill pass complete; regenerate from updated actual coverage')
                self.publish_assignment(
                    vid, 'SEARCH_STATIC_GLOBAL_GAP_FILL', route=route,
                    reason='global gap-fill pass complete but paired targets are still both missing')

    def publish_assignment(self, vid: int, mode: str, route: Optional[dict] = None,
                           target_name: Optional[str] = None, target_estimate: Optional[dict] = None,
                           static_targets: Optional[List[dict]] = None, reason: str = '',
                           extra_fields: Optional[dict] = None) -> None:
        self.assignment_seq[vid] += 1
        row = {
            'mission_id': self.mission_id, 'vehicle_id': vid, 'sequence': self.assignment_seq[vid],
            'mode': mode, 'functional_role': self.vehicle_roles.get(int(vid)),
            'route': route, 'target_name': target_name,
            'target_estimate': target_estimate, 'static_targets': static_targets or [],
            'reason': reason, 'ros_time': rospy.Time.now().to_sec(), 'wall_time': time.time(),
        }
        if extra_fields:
            row.update(copy.deepcopy(extra_fields))
        self.assignments[vid] = row
        self.last_assignment_wall[vid] = time.monotonic()
        self.assignment_pubs[vid].publish(String(data=json.dumps(row, ensure_ascii=False)))
        with open(self.run_dir / 'assignments.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(row, ensure_ascii=False) + '\n')

    def publish_initial_assignments(self) -> None:
        self.publish_assignment(0, 'SEARCH_DYNAMIC_LEFT', route=self.primary_routes[0],
                                reason='V6.7.19 dynamic-inspection v0: centre-to-left north-south coverage at 39.5m')
        self.publish_assignment(1, 'SEARCH_DYNAMIC_RIGHT', route=self.primary_routes[1],
                                reason='V6.7.19 dynamic-inspection v1: centre-to-right north-south coverage at 40.0m')
        self.publish_assignment(2, 'SEARCH_STATIC_CENTER_OUT', route=self.primary_routes[2],
                                reason='V6.7.19 manoeuvre-inspection v2: east-west rows from north to south over the initial region, then unchanged outward-square continuation at 40.5m')

    def assignment_tick(self, _event=None) -> None:
        if self.plan_only or self.start_payload is None:
            return
        for vid, row in list(self.assignments.items()):
            self.assignment_pubs[vid].publish(String(data=json.dumps(row, ensure_ascii=False)))
        for name, vid in list(self.dynamic_assignments.items()):
            estimate = self.estimate_target(name)
            if estimate:
                self.target_pubs[vid].publish(String(data=json.dumps(estimate, ensure_ascii=False)))

    def publish_status(self, _event=None) -> None:
        now_wall = time.monotonic()
        search_elapsed = (
            None if self.first_arm_wall is None
            else max(0.0, now_wall - self.first_arm_wall)
        )
        return_elapsed = (
            None if self.return_sent_wall is None
            else max(0.0, now_wall - self.return_sent_wall)
        )
        row = {
            'mission_id': self.mission_id, 'ros_time': rospy.Time.now().to_sec(),
            'flight_ack': self.flight_ack, 'perception_ack': self.perception_ack,
            'flight_ready': self.flight_ready, 'perception_ready': self.perception_ready,
            'assignments': {str(k): v.get('mode') for k, v in self.assignments.items()},
            'vehicle_functional_roles': {str(k): v for k, v in self.vehicle_roles.items()},
            'role_policy': self.cfg.get('role_policy', {}),
            'dynamic_assignments': self.dynamic_assignments,
            'dynamic_assignment_confirmed': self.dynamic_assignment_confirmed,
            'dynamic_provisional': {
                name: {
                    'tracker': row.get('tracker'),
                    'confirmed': row.get('confirmed', False),
                    'assigned_ros_time': row.get('assigned_ros_time'),
                }
                for name, row in self.dynamic_provisional.items()
            },
            'suv_false_positive_regions': self.suv_false_positive_regions,
            'tracking_target_stream_source': self.cfg.get('tracking', {}).get('target_stream_source'),
            'static_planner_truth_free': True,
            'static_detected': sorted(self.static_detected),
            'static_confirmed': sorted(self.static_confirmed),
            'flight_results_received': sorted(self.flight_results),
            'coverage_samples': len(self.coverage_samples),
            'static_residual_pass_index': self.static_residual_pass_index,
            'static_pair_guided_pass_index': self.static_pair_guided_pass_index,
            'static_pair_state': self.static_pair_state(),
            'static_strategy_stage': {str(k): v for k, v in self.static_strategy_stage.items()},
            'static_route_commitment': {
                str(k): v for k, v in self.static_route_commitment.items()
                if bool(v.get('active', False))
            },
            'static_verify_complete_vehicle': self.static_verify_complete_vehicle,
            'target_localization_report_count': len(self.target_localization_reports),
            'latest_target_localization': self.latest_target_localization,
            'detection_validation_event_count': len(self.detection_validation_events),
            'detection_validation_summary_vehicles': sorted(self.detection_validation_summaries),
            'missing_static_targets': [name for name in self.static_targets if name not in self.static_detected],
            'return_sent': self.return_sent,
            'abort_sent': self.abort_sent,
            'mission_end_reason': self.mission_end_reason,
            'failure_safe_return_triggered': self.failure_safe_return_triggered,
            'application_ready': self.application_ready,
            'start_authorization_required': self.start_authorization_required,
            'start_authorized': self.start_authorized,
            'model_state_launch_policy': self.cfg['mission'].get('model_state_launch_policy'),
            'competition_clock_started': self.first_arm_wall is not None,
            'search_elapsed_seconds_from_first_arm': search_elapsed,
            'maximum_search_seconds': float(self.cfg['mission'].get('mission_timeout_seconds', 1920.0)),
            'return_elapsed_seconds': return_elapsed,
            'reserved_return_seconds': float(self.cfg['return_strategy'].get('reserved_return_seconds', 180.0)),
            'competition_total_limit_seconds': float(self.cfg['return_strategy'].get('competition_total_limit_seconds', 2100.0)),
        }
        self.status_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))

    def wait_condition(self, label: str, predicate, timeout: float) -> None:
        deadline = None if timeout <= 0 else time.monotonic() + timeout
        while not rospy.is_shutdown():
            if predicate():
                rospy.loginfo('%s: PASS', label); return
            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError(f'timeout waiting {label}')
            rospy.loginfo_throttle(2.0, 'waiting %s', label)
            rospy.sleep(0.2)
        raise rospy.ROSInterruptException()

    def issue_abort(self, reason: str) -> None:
        if self.abort_sent: return
        self.abort_sent = True
        self.abort_pub.publish(Bool(data=True))
        dump_json(self.run_dir / 'abort_reason.json', {'mission_id': self.mission_id, 'reason': reason})

    def trackers_complete(self) -> bool:
        minimum_seconds = float(self.cfg['tracking']['minimum_track_seconds'])
        minimum_points = int(self.cfg['tracking']['minimum_track_points'])
        if not self.all_dynamic_tracks_confirmed(): return False
        for name, vid in self.dynamic_assignments.items():
            row = self.flight_status.get(vid, {})
            if row.get('tracking_target') != name: return False
            if float(row.get('tracking_seconds', 0.0)) < minimum_seconds: return False
            if int(row.get('tracking_points', 0)) < minimum_points: return False
        return True

    def static_complete(self) -> bool:
        return all(name in self.static_confirmed for name in self.static_targets)

    def should_finish(self) -> bool:
        return self.trackers_complete() and self.static_complete()

    def publish_return_all(self, reason: str) -> None:
        if self.return_sent:
            return
        self.return_sent = True
        self.return_sent_wall = time.monotonic()
        self.return_sent_ros = rospy.Time.now().to_sec()
        budget = float(self.cfg['return_strategy'].get('reserved_return_seconds', 180.0))
        dump_json(self.run_dir / 'return_command.json', {
            'mission_id': self.mission_id,
            'reason': reason,
            'return_sent_ros_time': self.return_sent_ros,
            'search_elapsed_seconds_from_first_arm': (
                None if self.first_arm_wall is None
                else max(0.0, self.return_sent_wall - self.first_arm_wall)
            ),
            'reserved_return_seconds': budget,
        })
        rospy.logwarn('V6.7.19 RETURN sent; reserved return/landing budget=%.1fs reason=%s',
                      budget, reason)
        for vid in self.vehicle_ids:
            self.publish_assignment(vid, 'RETURN', reason=reason)

    def compute_evaluation(self) -> dict:
        armed = [float(r.get('events', {}).get('ARMED')) for r in self.flight_results.values() if r.get('events', {}).get('ARMED') is not None]
        finished = []
        for r in self.flight_results.values():
            events = r.get('events', {})
            value = events.get('DISARMED', events.get('LANDED'))
            if value is not None: finished.append(float(value))
        duration = max(finished) - min(armed) if armed and finished else None
        yolo = {}
        minimum_fps = float(self.cfg['perception']['minimum_required_fps'])
        for vid in self.vehicle_ids:
            rows = sorted([r for r in self.yolo_reports if int(r.get('vehicle_id', -1)) == vid],
                          key=lambda r: float(r.get('ros_time', 0.0)))
            fps = [float(r.get('worker_fps', 0.0)) for r in rows]
            inference = [float(r['inference_ms']) for r in rows if r.get('inference_ms') is not None]
            roundtrip = [float(r['roundtrip_ms']) for r in rows if r.get('roundtrip_ms') is not None]
            times = [float(r.get('ros_time', 0.0)) for r in rows]
            delivered = (len(times)-1)/max(times[-1]-times[0], 1e-6) if len(times) >= 2 else 0.0
            median_worker = float(np.median(fps)) if fps else 0.0
            yolo[str(vid)] = {
                'reports': len(rows), 'delivered_fps': delivered,
                'worker_fps_median': median_worker,
                'inference_ms_median': float(np.median(inference)) if inference else None,
                'inference_ms_p95': float(np.percentile(inference, 95)) if inference else None,
                'roundtrip_ms_median': float(np.median(roundtrip)) if roundtrip else None,
                'rate_pass': bool(delivered >= minimum_fps and median_worker >= minimum_fps),
            }
        first_dynamic = {}
        for name, rows in self.dynamic_tracks.items():
            if rows: first_dynamic[name] = rows[0].get('ros_time')

        validation_counts = {
            'correct': 0, 'miss': 0, 'wrong_report': 0,
            'false_positive': 0,
        }
        validation_by_vehicle = {}
        for vid in self.vehicle_ids:
            summary = self.detection_validation_summaries.get(vid, {})
            counts = dict(summary.get('counts', {}))
            for key in validation_counts:
                validation_counts[key] += int(counts.get(key, 0))
            validation_by_vehicle[str(vid)] = summary
        # Events are also retained here because a forced terminal close can occur
        # before the final latched summary reaches the manager.
        if self.detection_validation_events:
            fallback = {
                'correct': 0, 'miss': 0, 'wrong_report': 0,
                'false_positive': 0,
            }
            for row in self.detection_validation_events:
                event_type = str(row.get('event_type', ''))
                if event_type in fallback:
                    fallback[event_type] += 1
            for key, value in fallback.items():
                validation_counts[key] = max(validation_counts[key], value)

        localization_by_vehicle = {}
        localization_by_target = {}
        for vid in self.vehicle_ids:
            rows = [r for r in self.target_localization_reports
                    if int(r.get('vehicle_id', -1)) == vid]
            localization_by_vehicle[str(vid)] = {
                'report_count': len(rows),
                'selected_for_management_count': sum(
                    1 for r in rows if bool(r.get('selected_as_management_result', False))),
                'targets': sorted({str(r.get('target_name', '')) for r in rows
                                   if r.get('target_name')}),
                'median_horizontal_std_m': (
                    float(np.median([float(r.get('horizontal_std_m', 0.0)) for r in rows]))
                    if rows else None),
            }
        for name in self.static_targets + self.dynamic_targets:
            rows = [r for r in self.target_localization_reports
                    if str(r.get('target_name', '')) == name]
            localization_by_target[name] = {
                'report_count': len(rows),
                'vehicles': sorted({int(r.get('vehicle_id', -1)) for r in rows}),
                'latest_position_world': (None if not rows else rows[-1].get('position_world')),
                'median_confidence': (
                    float(np.median([float(r.get('confidence', 0.0)) for r in rows]))
                    if rows else None),
            }

        return {
            'schema_version': 8,
            'mission_id': self.mission_id,
            'mission_duration_seconds_first_arm_to_last_land': duration,
            'dynamic_targets': {
                'assignments': self.dynamic_assignments,
                'first_detection_ros_time': first_dynamic,
                'track_counts': {k: len(v) for k, v in self.dynamic_tracks.items()},
                'tracking_complete': self.trackers_complete(),
            },
            'static_targets': {
                'detected': sorted(self.static_detected),
                'confirmed': sorted(self.static_confirmed),
                'missing_detected': [name for name in self.static_targets if name not in self.static_detected],
                'missing_confirmed': [name for name in self.static_targets if name not in self.static_confirmed],
                'all_confirmed': self.static_complete(),
                'coverage_sample_count': len(self.coverage_samples),
                'residual_passes_generated': len(self.static_residual_routes),
                'pair_guided_passes_generated': self.static_pair_guided_pass_index,
                'pair_state': self.static_pair_state(),
                'strategy_history': self.static_strategy_history,
                'static_verify_complete_vehicle': self.static_verify_complete_vehicle,
                'high_risk_history_count': len(self.static_high_risk_history),
                'residual_route_summaries': [
                    {
                        'route_id': r.get('route_id'), 'pass_index': r.get('pass_index'),
                        'estimated_covered_ratio': r.get('estimated_covered_ratio'),
                        'uncovered_grid_points': r.get('uncovered_grid_points'),
                        'candidate_grid_points': r.get('candidate_grid_points'),
                        'scan_run_count': len(r.get('scan_runs', [])),
                        'fallback_full_static_pass': r.get('fallback_full_static_pass'),
                        'planner': r.get('planner'),
                        'high_risk_component_count': r.get('high_risk_component_count'),
                        'nominal_cross_track_coverage_ratio': r.get('nominal_cross_track_coverage_ratio'),
                        'planning_profile': r.get('planning_profile'),
                        'planned_route_length_m': r.get('planned_route_length_m'),
                    } for r in self.static_residual_routes
                ],
            },
            'flight_results': {str(k): v for k, v in self.flight_results.items()},
            'yolo26': {'per_vehicle': yolo, 'minimum_required_fps': minimum_fps,
                       'all_streams_pass': all(row['rate_pass'] for row in yolo.values()),
                       'in_flight_control_loop': False},
            'target_localization': {
                'enabled': bool(self.cfg.get('perception', {}).get(
                    'target_localization', {}).get('enabled', False)),
                'report_count': len(self.target_localization_reports),
                'per_vehicle': localization_by_vehicle,
                'per_target': localization_by_target,
                'reports_file': str(self.run_dir / 'target_localization_reports.jsonl'),
                'manager_feedback_topic': f'{NS}/vehicle_<id>/target_localization_report',
            },
            'detection_event_validation': {
                'enabled': bool(self.cfg.get('perception', {}).get(
                    'detection_event_validation', {}).get('enabled', False)),
                'counts': validation_counts,
                'event_count': len(self.detection_validation_events),
                'per_vehicle': validation_by_vehicle,
                'localization_per_vehicle': {
                    str(vid): self.detection_validation_summaries.get(vid, {}).get(
                        'localization', {}) for vid in self.vehicle_ids
                },
                'events_file': str(self.run_dir / 'detection_validation_events.jsonl'),
                'truth_firewall': (
                    'truth is consumed only inside validation perception agents; '
                    'manager stores finalized labels and never uses them for planning/control'
                ),
            },
            'selected_detection_source': self.cfg['perception']['selected_result_source'],
            'mission_end_reason': self.mission_end_reason,
            'failure_safe_return_triggered': self.failure_safe_return_triggered,
            'time_budget': {
                'clock_reference': 'first_aircraft_armed_to_last_aircraft_disarmed',
                'maximum_search_seconds': float(self.cfg['mission'].get('mission_timeout_seconds', 1920.0)),
                'reserved_return_seconds': float(self.cfg['return_strategy'].get('reserved_return_seconds', 180.0)),
                'competition_total_limit_seconds': float(self.cfg['return_strategy'].get('competition_total_limit_seconds', 2100.0)),
                'return_command_ros_time': self.return_sent_ros,
                'search_elapsed_at_return_seconds': (
                    None if self.return_sent_wall is None or self.first_arm_wall is None
                    else max(0.0, self.return_sent_wall - self.first_arm_wall)
                ),
                'return_elapsed_wall_seconds': (
                    None if self.return_sent_wall is None
                    else max(0.0, time.monotonic() - self.return_sent_wall)
                ),
            },
            'search_strategy': {
                'side_aircraft_direction': 'inner_to_outer',
                'vehicle_2_direction': 'initial_centre_to_both_sides_then_square_outward',
                'search_altitudes_m': self.plan.get('search_altitudes_m', {}),
                'static_residual_profile': self.cfg['static_search'].get('residual_planning_profile'),
                'static_residual_lane_spacing_m': self.cfg['static_search'].get('risk_lane_spacing_m'),
            },
            'startup_timing': {
                'authorization_required': self.start_authorization_required,
                'authorization_payload': self.start_authorization_payload,
                'mission_start_ros_time': self.mission_start_ros,
            },
            'proxy_rule': 'target projection inside configured camera FOV; FW search accepted only on valid straight segments',
        }

    def run(self) -> None:
        if self.plan_only:
            rospy.loginfo('V6 plan-only complete: %s', self.run_dir / 'search_plan.json')
            return
        self.wait_condition('flight/perception ACK', lambda: all(self.flight_ack[v] and self.perception_ack[v] for v in self.vehicle_ids),
                            float(self.cfg['mission']['task_ack_timeout_seconds']))
        self.wait_condition('three flight agents ready', lambda: all(self.flight_ready[v] for v in self.vehicle_ids),
                            float(self.cfg['mission']['ready_timeout_seconds']))
        self.wait_condition('three YOLO pipelines >=10Hz', lambda: all(self.perception_ready[v] for v in self.vehicle_ids),
                            float(self.cfg['mission']['ready_timeout_seconds']))
        self.application_ready = True
        self.application_ready_pub.publish(Bool(data=True))
        ready_row = {
            'mission_id': self.mission_id,
            'ros_time': rospy.Time.now().to_sec(),
            'wall_time': time.time(),
            'three_flight_agents_ready': True,
            'three_yolo_pipelines_ready': True,
            'awaiting_target_motion_start': self.start_authorization_required,
        }
        dump_json(self.run_dir / 'application_ready.json', ready_row)
        rospy.logwarn('V6.7.19 application ready; model_state.py may now be started immediately before authorization')
        if self.start_authorization_required:
            self.wait_condition(
                'target-motion start authorization',
                lambda: self.start_authorized,
                float(self.cfg['mission'].get('start_authorization_timeout_seconds', 180.0)))
        epoch = rospy.Time.now().to_sec() + float(self.cfg['mission']['start_barrier_lead_seconds'])
        self.mission_start_ros = epoch
        self.start_payload = {
            'mission_id': self.mission_id, 'ros_time': epoch, 'vehicle_ids': self.vehicle_ids,
            'start_authorization': self.start_authorization_payload,
            'search_altitudes_m': self.plan.get('search_altitudes_m', {}),
        }
        self.start_pub.publish(String(data=json.dumps(self.start_payload, ensure_ascii=False)))
        self.publish_initial_assignments()
        dump_json(self.run_dir / 'start_command.json', self.start_payload)
        rospy.loginfo('V6.7.19 adaptive mission started epoch=%.3f', epoch)

        manager_loop_start_wall = time.monotonic()
        hard = float(self.cfg['completion']['hard_deadline_seconds'])
        soft = float(self.cfg['completion']['soft_deadline_seconds'])
        static_extension = float(self.cfg['completion'].get('static_residual_extension_seconds', 120.0))
        legacy_absolute = hard + max(0.0, static_extension)
        maximum_search = float(self.cfg['mission'].get('mission_timeout_seconds', legacy_absolute))
        if maximum_search < hard:
            rospy.logwarn('mission_timeout_seconds %.1f < hard deadline %.1f; clamping to hard deadline',
                          maximum_search, hard)
            maximum_search = hard
        reserved_return = float(self.cfg['return_strategy'].get('reserved_return_seconds', 180.0))
        competition_total = float(self.cfg['return_strategy'].get('competition_total_limit_seconds',
                                                                    maximum_search + reserved_return))
        if maximum_search + reserved_return > competition_total + 1e-6:
            rospy.logwarn('configured search %.1fs + return %.1fs exceeds competition budget %.1fs',
                          maximum_search, reserved_return, competition_total)
        failure_action = str(self.cfg['mission'].get('vehicle_failure_action', 'coordinated_safe_return'))
        while not rospy.is_shutdown():
            now_wall = time.monotonic()
            elapsed = (
                0.0 if self.first_arm_wall is None
                else max(0.0, now_wall - self.first_arm_wall)
            )
            failed = [v for v, r in self.flight_status.items() if r.get('phase') == 'FAILED']
            # V6.4: a single-aircraft startup or in-flight failure must not send a
            # latched global abort to aircraft that are still able to land. The
            # failed agent already invokes its own emergency-land path when armed.
            # Other agents receive RETURN and either land independently or, when
            # still unarmed, acknowledge RETURN without taking off.
            if failed and not self.return_sent:
                reason = f'vehicle failure {failed}; coordinated independent safe return'
                if failure_action == 'global_abort':
                    self.mission_end_reason = reason + ' (configured global_abort)'
                    self.issue_abort(self.mission_end_reason)
                    break
                self.failure_safe_return_triggered = True
                self.mission_end_reason = reason
                rospy.logerr('V6.5 %s', reason)
                self.publish_return_all(reason)
            if self.should_finish():
                self.mission_end_reason = 'both dynamic tracks sustained and all static targets confirmed'
                self.publish_return_all(self.mission_end_reason)

            if self.first_arm_wall is not None and not self.return_sent:
                if elapsed >= maximum_search:
                    self.mission_end_reason = (
                        'maximum 32-minute search budget reached from first ARMED; '
                        'start fixed-wing independent return with best available result'
                    )
                    self.publish_return_all(self.mission_end_reason)
                elif elapsed >= hard:
                    dynamic_missing = [
                        name for name in self.dynamic_targets
                        if name not in self.dynamic_assignments
                    ]
                    static_missing = [
                        name for name in self.static_targets
                        if name not in self.static_detected
                    ]
                    static_unconfirmed = [
                        name for name in self.static_targets
                        if name not in self.static_confirmed
                    ]
                    trackers_incomplete = not self.trackers_complete()
                    if not dynamic_missing and static_missing:
                        self.maybe_assign_static_role()
                    rospy.logwarn_throttle(
                        5.0,
                        'V6.7.2 final search window active: elapsed=%.1fs remaining=%.1fs '
                        'dynamic_missing=%s static_missing=%s static_unconfirmed=%s '
                        'trackers_incomplete=%s',
                        elapsed, maximum_search - elapsed, dynamic_missing, static_missing,
                        static_unconfirmed, trackers_incomplete,
                    )
                elif elapsed >= soft and bool(self.cfg['completion']['allow_return_after_timeout']):
                    rospy.logwarn_throttle(
                        5.0,
                        'V6.7.2 soft deadline exceeded; continuing search until %.1fs maximum',
                        maximum_search,
                    )

            if self.return_sent:
                break
            self.publish_status()
            rospy.sleep(0.2)

        if not self.return_sent:
            if self.abort_sent:
                self.mission_end_reason = self.mission_end_reason or 'manager abort requested; agents use local emergency landing'
            else:
                self.mission_end_reason = self.mission_end_reason or 'manager loop ended; independent safe return'
                self.publish_return_all(self.mission_end_reason)

        # Keep the manager heartbeat and RETURN assignments alive until every
        # still-armed aircraft is down. A missing result produces diagnostics,
        # never a global abort that can interrupt another aircraft's landing.
        warning_period = float(self.cfg['return_strategy'].get('result_wait_warning_seconds', 30.0))
        reserved_return = float(self.cfg['return_strategy'].get('reserved_return_seconds', 180.0))
        competition_total = float(self.cfg['return_strategy'].get('competition_total_limit_seconds', 2100.0))
        next_warning = time.monotonic() + warning_period
        while not rospy.is_shutdown() and len(self.flight_results) < len(self.vehicle_ids):
            now_wall = time.monotonic()
            armed = [v for v in self.vehicle_ids if bool(self.flight_status.get(v, {}).get('armed', False))]
            if not armed:
                rospy.logwarn('V6.7.2 return supervision ending with missing result messages=%s but all reported aircraft are disarmed',
                              sorted(set(self.vehicle_ids) - set(self.flight_results)))
                break
            return_elapsed = (
                0.0 if self.return_sent_wall is None
                else max(0.0, now_wall - self.return_sent_wall)
            )
            competition_elapsed = (
                0.0 if self.first_arm_wall is None
                else max(0.0, now_wall - self.first_arm_wall)
            )
            if return_elapsed >= reserved_return and not self.return_budget_exceeded_logged:
                self.return_budget_exceeded_logged = True
                rospy.logerr(
                    'V6.7.2 180s return budget exceeded; continuing safety supervision, '
                    'never aborting an aircraft during landing. armed=%s', armed)
            if competition_elapsed >= competition_total and not self.competition_limit_exceeded_logged:
                self.competition_limit_exceeded_logged = True
                rospy.logerr(
                    'V6.7.2 competition total time %.1fs exceeded; time-score may be zero, '
                    'but safe landing supervision continues. armed=%s', competition_total, armed)
            if now_wall >= next_warning:
                self.return_wait_warnings += 1
                rospy.logwarn(
                    'V6.7.2 supervising returns: return_elapsed=%.1fs/%.1fs '
                    'competition_elapsed=%.1fs/%.1fs armed=%s missing_results=%s',
                    return_elapsed, reserved_return, competition_elapsed, competition_total,
                    armed, sorted(set(self.vehicle_ids) - set(self.flight_results)))
                next_warning = now_wall + warning_period
            self.publish_status()
            rospy.sleep(0.2)
        rospy.sleep(float(self.cfg['mission']['result_flush_seconds']))
        evaluation = self.compute_evaluation()
        dump_json(self.run_dir / 'evaluation.json', evaluation)
        dump_json(self.run_dir / 'flight_results.json', {str(k): v for k, v in self.flight_results.items()})
        dump_json(self.run_dir / 'final_results.json', {
            'static_detected': self.static_detected,
            'static_confirmed': self.static_confirmed,
            'dynamic_tracks': self.dynamic_tracks,
            'dynamic_assignments': self.dynamic_assignments,
            'dynamic_assignment_confirmed': self.dynamic_assignment_confirmed,
            'dynamic_provisional': {
                name: {
                    'tracker': row.get('tracker'),
                    'confirmed': row.get('confirmed', False),
                    'assigned_ros_time': row.get('assigned_ros_time'),
                }
                for name, row in self.dynamic_provisional.items()
            },
            'suv_false_positive_regions': self.suv_false_positive_regions,
            'tracking_target_stream_source': self.cfg.get('tracking', {}).get('target_stream_source'),
            'coverage_samples': self.coverage_samples,
            'static_residual_routes': self.static_residual_routes,
            'static_pair_state': self.static_pair_state(),
            'static_strategy_history': self.static_strategy_history,
            'static_verify_complete_vehicle': self.static_verify_complete_vehicle,
            'target_localization_reports': self.target_localization_reports,
            'detection_validation_events': self.detection_validation_events,
            'detection_validation_summaries': {str(k): v for k, v in self.detection_validation_summaries.items()},
        })
        competition_results = self.build_competition_final_results()
        dump_json(self.run_dir / 'competition_final_results.json', competition_results)
        self.competition_results_pub.publish(
            String(data=json.dumps(competition_results, ensure_ascii=False)))
        rospy.logwarn(
            'V6.7.19 official result bridge payload published: static=%d dynamic=%d',
            len(competition_results['static_entries']),
            len(competition_results['dynamic_entries']))
        rospy.sleep(float(self.cfg.get('competition_output', {}).get(
            'bridge_delivery_wait_seconds', 1.0)))

        complete = {'mission_id': self.mission_id, 'evaluation_path': str(self.run_dir / 'evaluation.json')}
        self.complete_pub.publish(String(data=json.dumps(complete, ensure_ascii=False)))
        rospy.loginfo('V6 mission complete %s', json.dumps(complete, ensure_ascii=False))
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        MissionManager().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal('V6 manager failed: %s\n%s', exc, traceback.format_exc())
        raise
