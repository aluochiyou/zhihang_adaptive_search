#!/usr/bin/env python3
from __future__ import annotations

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
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandVtolTransition, SetMode
from std_msgs.msg import Bool, String
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_from_matrix, quaternion_matrix)

from zhihang_adaptive_search_v6.common import dump_json, validate_packet, wrap_pi
from zhihang_adaptive_search_v6.tracking_recovery import (
    DynamicTargetFilter, SuvMotionGate,
    generate_square_spiral_reacquisition_waypoints, possible_target_radius,
    static_hover_point, trajectory_yaw, weighted_position_fusion,
    yaw_rate_command)

NS = '/zhihang/search_v6'
PARAM_ROOT = '/zhihang_search_v6'
VTOL_MC = 3
VTOL_FW = 4


def norm_xy(v) -> float:
    return float(np.linalg.norm(np.asarray(v, dtype=float)[:2]))


def yaw_from_pose(pose: Pose) -> float:
    q = pose.orientation
    return float(euler_from_quaternion([q.x, q.y, q.z, q.w])[2])


def pose_with_yaw(x: float, y: float, z: float, yaw: float) -> Pose:
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = float(x), float(y), float(z)
    q = quaternion_from_euler(0.0, 0.0, float(yaw))
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
    return msg


def limit_vector_xy(v: np.ndarray, maximum: float) -> np.ndarray:
    out = np.asarray(v, dtype=float).copy()
    n = float(np.linalg.norm(out[:2]))
    if n > maximum > 0:
        out[:2] *= maximum / n
    return out


class VehicleFlightAgent:
    def __init__(self) -> None:
        rospy.init_node('vehicle_flight_agent_v6')
        self.id = int(rospy.get_param('~vehicle_id'))
        self.name = f'standard_vtol_{self.id}'
        self.local_cfg = rospy.get_param(PARAM_ROOT)
        self.lock = threading.RLock()
        self.task_event = threading.Event()
        self.assignment_event = threading.Event()
        self.complete_event = threading.Event()
        self.task: Optional[dict] = None
        self.assignment: Optional[dict] = None
        self.assignment_sequence = 0
        self.mission_id = ''
        self.start_epoch: Optional[float] = None
        self.last_manager_heartbeat = time.monotonic()
        self.abort_requested = False
        self.command_mode = 'none'
        self.position_sp = Pose()
        self.velocity_sp = Twist()
        self.phase = 'WAIT_TASK'
        self.state: Optional[State] = None
        self.ext: Optional[ExtendedState] = None
        self.local_pose: Optional[PoseStamped] = None
        self.local_vel = np.zeros(3)
        self.world_positions: Dict[str, np.ndarray] = {}
        self.matrix = np.eye(3)
        self.offset = np.zeros(3)
        self.home_world: Optional[np.ndarray] = None
        self.control = {}
        self.return_cfg = {}
        self.mission_params = {}
        self.tracking_cfg = {}
        self.static_cfg = {}
        self.gate_cfg = {}
        self.result = {}
        self.local_run_dir: Optional[Path] = None
        self.current_route_id = ''
        self.route_waypoint_index = 0
        self.route_waypoint_total = 0
        self.detection_valid = False
        self.detection_segment_type = ''
        self.detection_leg_id = ''
        self.cross_track_error = None
        self.heading_error_deg = None
        self.segment_along_m = None
        self.segment_length_m = None
        self.latest_target: Optional[dict] = None
        self.latest_localization_reports: Dict[str, List[dict]] = {}
        self.tracking_target = ''
        self.tracking_started_ros: Optional[float] = None
        self.tracking_points = 0
        self.static_confirmed = set()
        # V6.7.19 resolves every spatial candidate independently. A target name
        # is finalized only after all of its candidate groups have been checked.
        self.static_candidate_resolved = set()
        self.static_precise_candidates: Dict[str, List[dict]] = {}
        self.static_verify_target = ''
        self.static_verify_candidate_id = ''
        self.static_verify_attempt = 0
        self.last_velocity_world = np.zeros(3)
        self.last_velocity_wall = time.monotonic()
        # V6.3 startup diagnostics and retry state. A single temporary PX4
        # arming rejection must not fail the whole distributed mission.
        self.last_offboard_request_wall = 0.0
        self.arming_attempts = 0
        self.last_arming_result = None
        self.startup_cancelled = False

        prefix = f'{NS}/vehicle_{self.id}'
        self.ack_pub = rospy.Publisher(f'{prefix}/task_ack/flight', String, queue_size=2, latch=True)
        self.status_pub = rospy.Publisher(f'{prefix}/flight_status', String, queue_size=20, latch=True)
        self.event_pub = rospy.Publisher(f'{prefix}/event', String, queue_size=100)
        self.result_pub = rospy.Publisher(f'{prefix}/result', String, queue_size=2, latch=True)

        xt = f'/xtdrone/{self.name}'
        mav = f'/{self.name}/mavros'
        self.pose_pub = rospy.Publisher(f'{xt}/cmd_pose_enu', Pose, queue_size=20)
        self.vel_pub = rospy.Publisher(f'{xt}/cmd_vel_enu', Twist, queue_size=20)
        self.cmd_pub = rospy.Publisher(f'{xt}/cmd', String, queue_size=10)

        rospy.Subscriber(f'{NS}/manager/task/vehicle_{self.id}', String, self.task_cb, queue_size=1)
        rospy.Subscriber(f'{NS}/manager/assignment/vehicle_{self.id}', String, self.assignment_cb, queue_size=5)
        rospy.Subscriber(f'{NS}/manager/target/vehicle_{self.id}', String, self.target_cb, queue_size=20)
        rospy.Subscriber(f'{NS}/vehicle_{self.id}/target_localization_report', String,
                         self.localization_cb, queue_size=100)
        rospy.Subscriber(f'{NS}/manager/start', String, self.start_cb, queue_size=1)
        rospy.Subscriber(f'{NS}/manager/heartbeat', String, self.heartbeat_cb, queue_size=10)
        rospy.Subscriber(f'{NS}/manager/abort', Bool, self.abort_cb, queue_size=1)
        rospy.Subscriber(f'{NS}/manager/mission_complete', String, self.complete_cb, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.world_cb, queue_size=1)
        rospy.Subscriber(f'{mav}/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber(f'{mav}/extended_state', ExtendedState, self.ext_cb, queue_size=1)
        rospy.Subscriber(f'{mav}/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(f'{mav}/local_position/velocity_local', TwistStamped, self.vel_cb, queue_size=1)

        self.arm_srv = rospy.ServiceProxy(f'{mav}/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy(f'{mav}/set_mode', SetMode)
        self.vtol_srv = rospy.ServiceProxy(f'{mav}/cmd/vtol_transition', CommandVtolTransition)
        hz = float(self.local_cfg['flight_control']['control_rate_hz'])
        status_hz = float(self.local_cfg['flight_control']['status_rate_hz'])
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / hz), self.control_cb)
        self.status_timer = rospy.Timer(rospy.Duration(1.0 / status_hz), self.status_tick)

    # ---------------- subscriptions and task binding ----------------
    def publish_task_ack(self, packet: dict, accepted: bool, reason: str = '') -> None:
        row = {'mission_id': packet.get('mission_id', ''), 'vehicle_id': self.id,
               'component': 'flight', 'accepted': bool(accepted),
               'checksum': packet.get('checksum', ''), 'reason': reason, 'phase': self.phase}
        self.ack_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))

    def task_cb(self, msg: String) -> None:
        try:
            packet = json.loads(msg.data)
            validate_packet(packet, self.id)
            if self.task is not None and packet['mission_id'] == self.mission_id:
                self.publish_task_ack(packet, True, 'duplicate acknowledged'); return
            if self.task is not None and self.is_armed():
                self.publish_task_ack(packet, False, 'aircraft already armed'); return
            self.task = packet
            self.mission_id = str(packet['mission_id'])
            self.control = dict(packet['flight_control'])
            self.return_cfg = dict(packet['return_strategy'])
            self.mission_params = dict(packet['mission_parameters'])
            self.tracking_cfg = dict(packet['tracking'])
            self.static_cfg = dict(packet['static_verify'])
            self.gate_cfg = dict(packet['straight_detection_gate'])
            self.matrix = np.asarray(packet['coordinate_transform']['world_to_local_matrix'], dtype=float)
            root = Path(os.path.expanduser(self.mission_params['vehicle_output_root']))
            self.local_run_dir = root / self.mission_id / f'vehicle_{self.id}'
            self.local_run_dir.mkdir(parents=True, exist_ok=True)
            dump_json(self.local_run_dir / 'task_packet.json', packet)
            self.result = {'schema_version': 6, 'mission_id': self.mission_id, 'vehicle_id': self.id,
                           'ok': False, 'error': '', 'events': {}, 'assignments': [],
                           'static_confirmed': [], 'tracking': {}}
            self.task_event.set()
            self.publish_task_ack(packet, True, 'accepted')
            rospy.loginfo('v%d V6 flight task accepted mission=%s', self.id, self.mission_id)
        except Exception as exc:
            rospy.logerr('v%d task rejected: %s', self.id, exc)

    def assignment_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') != self.mission_id or int(row.get('vehicle_id', -1)) != self.id:
                return
            seq = int(row.get('sequence', 0))
            with self.lock:
                if seq <= self.assignment_sequence:
                    return
                self.assignment = row
                self.assignment_sequence = seq
                self.result.setdefault('assignments', []).append(row)
                self.assignment_event.set()
            self.event('ASSIGNMENT_RECEIVED', sequence=seq, mode=row.get('mode'), reason=row.get('reason'))
        except Exception as exc:
            rospy.logerr('v%d invalid assignment: %s', self.id, exc)

    def target_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') == self.mission_id:
                with self.lock: self.latest_target = row
        except Exception:
            pass

    def localization_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') != self.mission_id:
                return
            name = str(row.get('target_name', ''))
            if not name:
                return
            with self.lock:
                rows = self.latest_localization_reports.setdefault(name, [])
                rows.append(row)
                if len(rows) > 200:
                    del rows[:-200]
        except Exception:
            pass

    def start_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') == self.mission_id:
                self.start_epoch = float(row['ros_time'])
        except Exception:
            pass

    def heartbeat_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if not self.mission_id or row.get('mission_id') == self.mission_id:
                self.last_manager_heartbeat = time.monotonic()
        except Exception:
            pass

    def abort_cb(self, msg: Bool) -> None:
        if msg.data: self.abort_requested = True

    def complete_cb(self, msg: String) -> None:
        try:
            if json.loads(msg.data).get('mission_id') == self.mission_id:
                self.complete_event.set()
        except Exception:
            pass

    def state_cb(self, msg):
        with self.lock: self.state = msg

    def ext_cb(self, msg):
        with self.lock: self.ext = msg

    def pose_cb(self, msg):
        with self.lock: self.local_pose = msg

    def vel_cb(self, msg):
        with self.lock:
            self.local_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=float)

    def world_cb(self, msg):
        positions = {}
        for name, pose in zip(msg.name, msg.pose):
            positions[name] = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        with self.lock: self.world_positions = positions

    # ---------------- low-level helpers ----------------
    def control_cb(self, _event=None) -> None:
        if rospy.is_shutdown() or self.abort_requested: return
        try:
            if self.command_mode == 'position': self.pose_pub.publish(self.position_sp)
            elif self.command_mode == 'velocity': self.vel_pub.publish(self.velocity_sp)
        except rospy.ROSException:
            pass

    def is_armed(self) -> bool:
        with self.lock: return bool(self.state is not None and self.state.armed)

    def vtol_state(self) -> Optional[int]:
        with self.lock: return None if self.ext is None else int(self.ext.vtol_state)

    def current_world(self) -> Optional[np.ndarray]:
        with self.lock:
            p = self.world_positions.get(self.name)
            return None if p is None else p.copy()

    def current_local(self) -> np.ndarray:
        with self.lock:
            if self.local_pose is None: raise RuntimeError('local pose unavailable')
            p = self.local_pose.pose.position
            return np.array([p.x, p.y, p.z], dtype=float)

    def current_attitude(self):
        """Return body attitude expressed in the same world frame as world_position.

        MAVROS local pose is expressed in the calibrated local ENU frame while
        current_world() is in the task/Gazebo world frame.  The configured
        world_to_local rotation is therefore removed before publishing the
        quaternion used by camera geolocation.
        """
        with self.lock:
            if self.local_pose is None:
                return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
            q = self.local_pose.pose.orientation
            local_q = [float(q.x), float(q.y), float(q.z), float(q.w)]
            local_h = quaternion_matrix(local_q)
            world_h = np.eye(4, dtype=float)
            world_h[:3, :3] = self.matrix.T.dot(local_h[:3, :3])
            world_q = [float(x) for x in quaternion_from_matrix(world_h)]
            roll, pitch, yaw = euler_from_quaternion(world_q)
            return [float(roll), float(pitch), float(yaw)], world_q

    def current_yaw(self) -> float:
        return float(self.current_attitude()[0][2])

    def ground_speed(self) -> float:
        with self.lock: return float(np.linalg.norm(self.local_vel[:2]))

    def current_world_velocity(self) -> np.ndarray:
        with self.lock:
            local = self.local_vel.copy()
        return self.matrix.T.dot(local)

    def world_to_local(self, point: np.ndarray) -> np.ndarray:
        return self.matrix.dot(np.asarray(point, dtype=float)) + self.offset

    def world_vector_to_local(self, vector: np.ndarray) -> np.ndarray:
        current = self.current_world()
        if current is None: return np.asarray(vector, dtype=float)
        return self.world_to_local(current + np.asarray(vector, dtype=float)) - self.world_to_local(current)


    def _safe_fw_position_hold(self, reason: str = '') -> bool:
        """Install a finite forward pass-through setpoint while still in FW.

        Publishing a zero Twist or the XTDrone HOVER command in fixed-wing mode
        can produce PX4's `Invalid offboard setpoint` and a FW HOLD/LOITER mode.
        This helper is therefore the only velocity-to-position handover used in
        FW recovery paths.
        """
        current = self.current_world()
        if current is None:
            return False
        distance = float(self.static_cfg.get(
            'hold_recovery_forward_distance_m', 120.0))
        yaw = self.current_yaw()
        desired = current.copy()
        desired[0] += max(30.0, distance) * math.cos(yaw)
        desired[1] += max(30.0, distance) * math.sin(yaw)
        desired[2] = max(
            float(current[2]),
            float(self.home_world[2] + self.mission_params.get(
                'search_altitude_m', 40.0)) if self.home_world is not None
            else float(current[2]))
        local = self.world_to_local(desired)
        if not np.all(np.isfinite(local[:3])):
            return False
        self.position_sp = pose_with_yaw(
            local[0], local[1], local[2], yaw)
        self.command_mode = 'position'
        self.event('FW_ZERO_VELOCITY_HANDOVER_BLOCKED',
                   reason=reason, safe_pass_world=desired.tolist())
        return True

    def switch_to_position_mode(self) -> None:
        if self.command_mode == 'velocity':
            if self.vtol_state() == VTOL_FW:
                if self._safe_fw_position_hold(
                        'velocity_to_position_mode_switch'):
                    return
                # Do not emit a zero-velocity FW command even if pose is not
                # momentarily available. Keep the last position setpoint active.
                self.command_mode = 'position'
                return
            zero = Twist()
            self.velocity_sp = zero
            hold = (float(self.return_cfg.get(
                'command_handover_hold_seconds', 0.35))
                if self.return_cfg else 0.35)
            deadline = time.monotonic() + max(0.12, hold)
            while time.monotonic() < deadline and not rospy.is_shutdown():
                self.vel_pub.publish(zero)
                rospy.sleep(0.04)
            self.cmd_pub.publish(String(data='HOVER'))
            rospy.sleep(0.12)
        self.command_mode = 'position'


    def set_position_world(self, point: np.ndarray, yaw: float) -> None:
        point=np.asarray(point,dtype=float)
        if point.size<3 or not np.all(np.isfinite(point[:3])) or not math.isfinite(float(yaw)):
            raise RuntimeError(f'invalid finite position setpoint point={point.tolist()} yaw={yaw}')
        q=self.world_to_local(point)
        if not np.all(np.isfinite(q[:3])):
            raise RuntimeError(f'invalid transformed position setpoint world={point.tolist()} local={q.tolist()}')
        self.switch_to_position_mode()
        self.position_sp=pose_with_yaw(q[0],q[1],q[2],yaw)


    def set_velocity_world(self, velocity: np.ndarray, yaw_rate: float = 0.0) -> None:
        velocity=np.asarray(velocity,dtype=float)
        if velocity.size<3 or not np.all(np.isfinite(velocity[:3])) or not math.isfinite(float(yaw_rate)):
            rospy.logerr_throttle(1.0,'v%d blocked invalid velocity setpoint velocity=%s yaw_rate=%s',self.id,velocity,yaw_rate)
            velocity=np.zeros(3,dtype=float); yaw_rate=0.0
        local=self.world_vector_to_local(velocity)
        if not np.all(np.isfinite(local[:3])):
            rospy.logerr_throttle(1.0,'v%d blocked invalid transformed velocity local=%s',self.id,local)
            local=np.zeros(3,dtype=float); velocity=np.zeros(3,dtype=float)
        msg=Twist(); msg.linear.x,msg.linear.y,msg.linear.z=[float(x) for x in local[:3]]
        msg.angular.z=float(yaw_rate)
        self.velocity_sp=msg; self.last_velocity_world=velocity.copy(); self.command_mode='velocity'

    def ensure_offboard(self) -> bool:
        with self.lock: mode = '' if self.state is None else self.state.mode
        if mode == 'OFFBOARD':
            return True
        now = time.monotonic()
        if now - self.last_offboard_request_wall >= 1.0:
            response = self.mode_srv(custom_mode='OFFBOARD')
            self.last_offboard_request_wall = now
            if not response.mode_sent:
                rospy.logwarn_throttle(2.0, 'v%d OFFBOARD not accepted; current_mode=%s', self.id, mode)
        return False

    def recover_offboard_hold(self, context: str,
                              timeout: Optional[float] = None) -> bool:
        """Force a safe OFFBOARD recovery without sending invalid FW zero velocity."""
        enabled = bool(self.static_cfg.get('hold_recovery_enabled', True))
        if not enabled:
            return self.ensure_offboard()
        duration = float(
            self.static_cfg.get('hold_recovery_timeout_seconds', 10.0)
            if timeout is None else timeout)
        current = self.current_world()
        if self.vtol_state() == VTOL_FW:
            self._safe_fw_position_hold(context)
        elif current is not None:
            self.set_position_world(current, self.current_yaw())
        deadline = time.monotonic() + max(1.0, duration)
        attempts = 0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                mode = '' if self.state is None else str(self.state.mode)
            if mode == 'OFFBOARD':
                self.event('OFFBOARD_HOLD_RECOVERY_COMPLETE',
                           context=context, attempts=attempts,
                           vtol_state=self.vtol_state())
                return True
            # Pre-stream a valid setpoint before every mode request.
            if self.command_mode == 'position':
                self.pose_pub.publish(self.position_sp)
            elif self.vtol_state() != VTOL_FW:
                self.vel_pub.publish(self.velocity_sp)
            attempts += 1
            try:
                response = self.mode_srv(custom_mode='OFFBOARD')
                self.last_offboard_request_wall = time.monotonic()
                self.event('OFFBOARD_HOLD_RECOVERY_ATTEMPT',
                           context=context, attempt=attempts,
                           previous_mode=mode,
                           accepted=bool(response.mode_sent))
            except Exception as exc:
                rospy.logwarn_throttle(
                    1.0, 'v%d OFFBOARD hold recovery failed: %s',
                    self.id, exc)
            rospy.sleep(0.25)
        self.event('OFFBOARD_HOLD_RECOVERY_TIMEOUT',
                   context=context, attempts=attempts,
                   vtol_state=self.vtol_state())
        return False

    def assignment_mode(self) -> str:
        with self.lock:
            return '' if self.assignment is None else str(self.assignment.get('mode', ''))

    def return_requested(self) -> bool:
        return self.assignment_mode() == 'RETURN'

    def complete_unarmed_return(self, reason: str) -> None:
        self.startup_cancelled = True
        self.result['ok'] = True
        self.result['error'] = ''
        self.result['startup_cancelled_reason'] = reason
        self.event('RETURN_SKIPPED_UNARMED', reason=reason)
        self.set_phase('DONE')

    def wait_offboard_confirmed(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.1, timeout)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.return_requested() and not self.is_armed():
                return False
            self.check_abort()
            if self.ensure_offboard():
                return True
            rospy.sleep(0.1)
        with self.lock:
            mode = '' if self.state is None else self.state.mode
        rospy.logwarn('v%d OFFBOARD confirmation timeout; last_mode=%s', self.id, mode)
        return False

    def check_abort(self) -> None:
        if rospy.is_shutdown() or self.abort_requested: raise RuntimeError('abort requested')
        if self.start_epoch is not None:
            timeout = float(self.task['manager_policy']['heartbeat_timeout_seconds'])
            if time.monotonic() - self.last_manager_heartbeat > timeout:
                raise RuntimeError(f'manager heartbeat lost >{timeout}s')

    def event(self, name: str, **extra) -> None:
        row = {'mission_id': self.mission_id, 'vehicle_id': self.id, 'event': name,
               'ros_time': rospy.Time.now().to_sec(), 'phase': self.phase}
        row.update(extra)
        self.result.setdefault('events', {})[name] = row['ros_time']
        self.event_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))
        if self.local_run_dir:
            with open(self.local_run_dir / 'events.jsonl', 'a', encoding='utf-8') as fp:
                fp.write(json.dumps(row, ensure_ascii=False) + '\n')
        rospy.loginfo('v%d EVENT %s %s', self.id, name, json.dumps(extra, ensure_ascii=False))

    def set_phase(self, phase: str) -> None:
        self.phase = phase

    def status_payload(self) -> dict:
        p = self.current_world()
        attitude_rpy, orientation_xyzw = self.current_attitude()
        tracking_seconds = 0.0 if self.tracking_started_ros is None else max(0.0, rospy.Time.now().to_sec() - self.tracking_started_ros)
        return {
            'mission_id': self.mission_id, 'vehicle_id': self.id, 'component': 'flight',
            'phase': self.phase, 'ready': self.phase == 'READY' or self.phase not in ('WAIT_TASK','WAIT_INPUTS','CALIBRATING'),
            'assignment_mode': None if self.assignment is None else self.assignment.get('mode'),
            'assignment_sequence': self.assignment_sequence,
            'armed': self.is_armed(), 'mode': None if self.state is None else self.state.mode,
            'vtol_state': self.vtol_state(), 'world_position': None if p is None else p.tolist(),
            'world_yaw': float(attitude_rpy[2]),
            'world_attitude_rpy_rad': attitude_rpy,
            'world_attitude_rpy_deg': [math.degrees(x) for x in attitude_rpy],
            'world_orientation_xyzw': orientation_xyzw,
            'ground_speed_mps': self.ground_speed(),
            'route_id': self.current_route_id, 'route_waypoint_index': self.route_waypoint_index,
            'route_waypoint_total': self.route_waypoint_total,
            'detection_valid': self.detection_valid,
            'detection_segment_type': self.detection_segment_type,
            'detection_leg_id': self.detection_leg_id,
            'cross_track_error_m': self.cross_track_error,
            'heading_error_deg': self.heading_error_deg,
            'segment_along_m': self.segment_along_m,
            'segment_length_m': self.segment_length_m,
            'tracking_target': self.tracking_target,
            'tracking_seconds': tracking_seconds,
            'tracking_points': self.tracking_points,
            'static_confirmed': sorted(self.static_confirmed),
            'static_candidate_resolved': sorted(self.static_candidate_resolved),
            'static_verify_target': self.static_verify_target,
            'static_verify_candidate_id': self.static_verify_candidate_id,
            'static_verify_attempt': self.static_verify_attempt,
            'arming_attempts': self.arming_attempts,
            'last_arming_result': self.last_arming_result,
            'startup_cancelled': self.startup_cancelled,
            'manager_heartbeat_age_seconds': time.monotonic() - self.last_manager_heartbeat,
            'ros_time': rospy.Time.now().to_sec(), 'error': self.result.get('error', ''),
        }

    def status_tick(self, _event=None) -> None:
        self.status_pub.publish(String(data=json.dumps(self.status_payload(), ensure_ascii=False)))

    # ---------------- startup ----------------
    def wait_task(self) -> None:
        while not rospy.is_shutdown() and not self.task_event.wait(0.2):
            rospy.loginfo_throttle(2.0, 'v%d waiting V6 task', self.id)

    def wait_inputs(self) -> None:
        self.set_phase('WAIT_INPUTS')
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                ready = self.state is not None and self.state.connected and self.ext is not None and self.local_pose is not None and self.name in self.world_positions
            if ready: return
            rospy.sleep(0.1)
        raise RuntimeError('input timeout')

    def calibrate(self) -> None:
        self.set_phase('CALIBRATING')
        cfg = self.task['coordinate_transform']
        samples = []
        for _ in range(int(cfg['calibration_samples'])):
            world = self.current_world()
            if world is not None: samples.append(self.current_local() - self.matrix.dot(world))
            rospy.sleep(float(cfg['calibration_interval_seconds']))
        if len(samples) < 5: raise RuntimeError('not enough calibration samples')
        arr = np.vstack(samples); self.offset = np.mean(arr, axis=0); std = np.std(arr, axis=0)
        if float(np.max(std)) > float(cfg['maximum_calibration_std_m']):
            raise RuntimeError(f'calibration unstable std={std.tolist()}')
        self.home_world = self.current_world()
        if self.home_world is None: raise RuntimeError('home unavailable')

    def wait_start(self) -> bool:
        self.set_phase('READY'); self.event('READY')
        while not rospy.is_shutdown() and self.start_epoch is None:
            if self.return_requested() and not self.is_armed():
                self.complete_unarmed_return('RETURN received before manager start barrier')
                return False
            self.check_abort(); rospy.sleep(0.05)
        delay = float(self.mission_params['start_delay_seconds'])
        target = float(self.start_epoch) + delay
        while rospy.Time.now().to_sec() < target:
            if self.return_requested() and not self.is_armed():
                self.complete_unarmed_return('RETURN received during takeoff staggering')
                return False
            self.check_abort(); rospy.sleep(0.05)
        return True

    def arm(self) -> bool:
        if self.is_armed():
            return True
        timeout = float(self.control.get('arming_retry_timeout_seconds', 30.0))
        interval = float(self.control.get('arming_retry_interval_seconds', 1.0))
        offboard_timeout = float(self.control.get('offboard_confirm_timeout_seconds', 8.0))
        deadline = time.monotonic() + max(timeout, 1.0)
        last_mode = ''
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.return_requested() and not self.is_armed():
                self.complete_unarmed_return('RETURN received before arming completed')
                return False
            self.check_abort()
            self.wait_offboard_confirmed(min(offboard_timeout, max(0.5, deadline-time.monotonic())))
            if self.return_requested() and not self.is_armed():
                self.complete_unarmed_return('RETURN received while confirming OFFBOARD')
                return False
            with self.lock:
                last_mode = '' if self.state is None else self.state.mode
            self.arming_attempts += 1
            try:
                response = self.arm_srv(True)
                self.last_arming_result = int(response.result)
                rospy.loginfo('v%d arming attempt=%d success=%s result=%s mode=%s',
                              self.id, self.arming_attempts, response.success, response.result, last_mode)
            except Exception as exc:
                self.last_arming_result = None
                rospy.logwarn('v%d arming service attempt=%d failed: %s', self.id, self.arming_attempts, exc)
                response = None
            observe_deadline = min(deadline, time.monotonic() + max(0.8, interval))
            while time.monotonic() < observe_deadline and not rospy.is_shutdown():
                if self.is_armed():
                    self.event('ARMED', attempts=self.arming_attempts, last_result=self.last_arming_result)
                    return True
                if self.return_requested():
                    self.complete_unarmed_return('RETURN received after arming request')
                    return False
                self.check_abort(); rospy.sleep(0.05)
            rospy.logwarn_throttle(1.0,
                'v%d arming not yet accepted; retrying attempt=%d result=%s mode=%s',
                self.id, self.arming_attempts, self.last_arming_result, last_mode)
            rospy.sleep(max(0.05, interval * 0.2))
        raise RuntimeError(
            f'arming timeout after {self.arming_attempts} attempts; '
            f'last_result={self.last_arming_result} last_mode={last_mode}')

    def wait_position(self, desired: np.ndarray, yaw: float, timeout: float, xy_tol: float, z_tol: float,
                      assignment_sequence: Optional[int] = None) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort(); self.ensure_offboard()
            if assignment_sequence is not None and self.assignment_sequence != assignment_sequence: return False
            current = self.current_world()
            if current is None: rospy.sleep(0.05); continue
            if norm_xy(desired-current) <= xy_tol and abs(float(desired[2]-current[2])) <= z_tol: return True
            self.set_position_world(desired, yaw); rospy.sleep(0.05)
        raise RuntimeError(f'position timeout phase={self.phase}')

    def transition(self, desired_state: int) -> None:
        target_name = 'FW' if desired_state == VTOL_FW else 'MC'
        deadline = time.monotonic() + float(self.control['transition_timeout_seconds']); last = 0.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            if self.vtol_state() == desired_state:
                self.event(f'TRANSITION_{target_name}_COMPLETE'); return
            if time.monotonic() - last > 1.0:
                response = self.vtol_srv(state=desired_state)
                rospy.loginfo('v%d transition %s success=%s result=%s', self.id, target_name, response.success, response.result)
                last = time.monotonic()
            self.ensure_offboard(); rospy.sleep(0.05)
        raise RuntimeError(f'transition timeout to {target_name}')

    def takeoff_10m_then_fw(self) -> bool:
        assert self.home_world is not None
        self.set_phase('TAKEOFF_10M'); self.event('TAKEOFF_START')
        target = self.home_world.copy(); target[2] += float(self.mission_params['takeoff_transition_height_m'])
        self.set_position_world(target, 0.0)
        end = time.monotonic() + float(self.control['offboard_prestream_seconds'])
        while time.monotonic() < end:
            if self.return_requested() and not self.is_armed():
                self.complete_unarmed_return('RETURN received during OFFBOARD prestream')
                return False
            self.check_abort(); rospy.sleep(0.05)
        if not self.wait_offboard_confirmed(float(self.control.get('offboard_confirm_timeout_seconds', 8.0))):
            if self.return_requested() and not self.is_armed():
                self.complete_unarmed_return('RETURN received before OFFBOARD confirmation')
                return False
        rospy.sleep(0.4)
        if not self.arm():
            return False
        if self.return_requested():
            self.event('STARTUP_RETURN_AFTER_ARM')
            with self.lock: assignment = dict(self.assignment) if self.assignment else {'mode': 'RETURN', 'sequence': self.assignment_sequence}
            self.execute_return(assignment)
            return False
        takeoff_seq = self.assignment_sequence
        reached = self.wait_position(target, 0.0, float(self.control['position_timeout_seconds']), 2.0, 1.0, takeoff_seq)
        if not reached and self.return_requested():
            self.event('STARTUP_RETURN_DURING_CLIMB')
            with self.lock: assignment = dict(self.assignment) if self.assignment else {'mode': 'RETURN', 'sequence': self.assignment_sequence}
            self.execute_return(assignment)
            return False
        self.event('TAKEOFF_10M_COMPLETE')
        if not bool(self.control['skip_fixed_wing']):
            # The user requirement is transition immediately after reaching 10 m.
            # Before requesting FW, point the position setpoint toward the first
            # assigned search entry at 40 m so the transition accelerates into
            # the mission direction instead of hovering vertically at home.
            hint = None
            with self.lock:
                assignment = None if self.assignment is None else dict(self.assignment)
            if assignment:
                route = assignment.get('route') or {}
                waypoints = route.get('waypoints') or []
                if waypoints:
                    hint = np.asarray(waypoints[0]['point'], dtype=float)
            if hint is not None:
                yaw = math.atan2(hint[1]-target[1], hint[0]-target[0])
                self.set_position_world(hint, yaw)
                rospy.sleep(0.25)
            self.set_phase('INITIAL_TRANSITION_FW'); self.transition(VTOL_FW)
        return True

    # ---------------- search ----------------
    def reset_detection_gate(self) -> None:
        self.detection_valid = False; self.detection_segment_type = ''; self.detection_leg_id = ''
        self.cross_track_error = None; self.heading_error_deg = None
        self.segment_along_m = None; self.segment_length_m = None

    def follow_fw_waypoint(self, waypoint: dict, previous: np.ndarray, seq: int) -> bool:
        target = np.asarray(waypoint['point'], dtype=float)
        start = np.asarray(previous, dtype=float)
        line = target[:2] - start[:2]
        length = max(float(np.linalg.norm(line)), 1e-6)
        unit = line / length; yaw = math.atan2(unit[1], unit[0])
        valid_type = waypoint.get('segment_type') in self.gate_cfg['valid_segment_types'] and bool(waypoint.get('detection_valid', False))
        deadline = time.monotonic() + float(self.control['waypoint_timeout_seconds'])
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            if self.assignment_sequence != seq:
                self.reset_detection_gate(); return False
            self.ensure_offboard(); self.set_position_world(target, yaw)
            current = self.current_world()
            if current is None: rospy.sleep(0.05); continue
            delta = current[:2] - start[:2]
            along = float(np.dot(delta, unit)); cross = float(abs(unit[0]*delta[1] - unit[1]*delta[0]))
            heading_error = abs(math.degrees(wrap_pi(self.current_yaw() - yaw)))
            edge = float(self.gate_cfg['segment_edge_exclusion_m'])
            gate_valid = (valid_type and edge <= along <= max(edge, length-edge)
                          and cross <= float(self.gate_cfg['maximum_cross_track_error_m'])
                          and heading_error <= float(self.gate_cfg['maximum_heading_error_deg'])
                          and self.ground_speed() >= float(self.gate_cfg['minimum_ground_speed_mps'])
                          and self.vtol_state() == VTOL_FW)
            self.detection_valid = bool(gate_valid)
            self.detection_segment_type = str(waypoint.get('segment_type', ''))
            self.detection_leg_id = str(waypoint.get('leg_id', ''))
            self.cross_track_error = cross; self.heading_error_deg = heading_error
            self.segment_along_m = along; self.segment_length_m = length
            dist = float(np.linalg.norm(target[:2] - current[:2])); remaining = length - along
            if dist <= float(self.control['fixed_wing_waypoint_acceptance_m']) or remaining <= float(self.control['fixed_wing_pass_gate_m']):
                self.reset_detection_gate(); return True
            rospy.sleep(float(self.control['search_interrupt_poll_seconds']))
        self.reset_detection_gate(); raise RuntimeError(f'FW waypoint timeout route={self.current_route_id}')

    def ensure_fw(self, target_hint: Optional[np.ndarray] = None) -> None:
        if bool(self.control['skip_fixed_wing']): return
        if self.vtol_state() == VTOL_FW: return
        current = self.current_world()
        if current is None: raise RuntimeError('world position unavailable')
        desired = current.copy(); desired[2] = max(desired[2], float(self.mission_params['takeoff_transition_height_m']))
        if target_hint is not None:
            yaw = math.atan2(target_hint[1]-current[1], target_hint[0]-current[0])
        else: yaw = self.current_yaw()
        self.set_position_world(desired, yaw); rospy.sleep(0.2); self.transition(VTOL_FW)

    def execute_search(self, assignment: dict) -> None:
        seq = int(assignment['sequence']); route = assignment.get('route') or self.task['primary_route']
        waypoints = list(route['waypoints'])
        if not waypoints: return
        self.current_route_id = str(route.get('route_id', assignment['mode']))
        self.route_waypoint_total = len(waypoints); self.route_waypoint_index = 0
        self.set_phase(str(assignment['mode'])); self.event('SEARCH_START', route_id=self.current_route_id)
        self.ensure_fw(np.asarray(waypoints[0]['point'], dtype=float))
        previous = self.current_world()
        if previous is None: raise RuntimeError('current world unavailable')
        for index, wp in enumerate(waypoints, start=1):
            if self.assignment_sequence != seq: return
            self.route_waypoint_index = index
            reached = self.follow_fw_waypoint(wp, previous, seq)
            if not reached: return
            previous = np.asarray(wp['point'], dtype=float)
        self.event('ROUTE_COMPLETE', route_id=self.current_route_id, route_waypoints=len(waypoints), assignment_mode=assignment['mode'])
        self.reset_detection_gate()
        # Maintain the last position setpoint; PX4 will orbit until a new role arrives.
        while self.assignment_sequence == seq and not rospy.is_shutdown():
            self.check_abort(); rospy.sleep(0.1)

    # ---------------- dynamic tracking ----------------
    def target_snapshot(self, name: str) -> Optional[dict]:
        with self.lock:
            row = None if self.latest_target is None else dict(self.latest_target)
        if row is None or row.get('target_name') != name:
            return None
        return row

    def target_age(self, row: dict) -> float:
        stamp = float(row.get('source_ros_time', row.get(
            'manager_ros_time', rospy.Time.now().to_sec())))
        return max(0.0, rospy.Time.now().to_sec() - stamp)

    def fresh_target_snapshot(self, name: str, timeout: Optional[float] = None) -> Optional[dict]:
        row = self.target_snapshot(name)
        if row is None:
            return None
        maximum = float(self.tracking_cfg.get('target_timeout_seconds', 5.0)
                        if timeout is None else timeout)
        return row if self.target_age(row) <= maximum else None

    def predicted_target(self, row: dict) -> np.ndarray:
        p = np.asarray(row['position'], dtype=float)
        v = np.asarray(row.get('velocity', [0, 0, 0]), dtype=float)
        age = self.target_age(row)
        cap = (float(self.tracking_cfg.get('intercept_prediction_cap_seconds', 30.0))
               if self.phase == 'TRACK_INTERCEPT_FW'
               else float(self.tracking_cfg.get('target_timeout_seconds', 5.0)))
        horizon = min(float(self.tracking_cfg.get('prediction_seconds', 1.0)) + age, cap)
        return p + v * horizon

    def tracking_yaw_rate(self, target_yaw: float) -> float:
        return yaw_rate_command(
            self.current_yaw(), target_yaw,
            float(self.tracking_cfg.get('yaw_kp', 1.2)),
            float(self.tracking_cfg.get('maximum_yaw_rate_rad_s', 0.35)),
            float(self.tracking_cfg.get('yaw_deadband_deg', 3.0)))

    def path_yaw_rate(self, current: np.ndarray, desired: np.ndarray,
                      fallback_yaw: float) -> float:
        return self.tracking_yaw_rate(
            trajectory_yaw(current, desired, fallback_yaw))

    def command_yaw_rate(self, command: np.ndarray,
                         fallback_yaw: float) -> float:
        current = self.current_world()
        if current is None or norm_xy(command) < 0.25:
            return self.tracking_yaw_rate(fallback_yaw)
        desired = current.copy()
        desired[:2] += np.asarray(command, dtype=float)[:2]
        return self.path_yaw_rate(current, desired, fallback_yaw)

    def protected_tracking_yaw_rate(
            self, command: np.ndarray, fallback_yaw: float,
            horizontal_distance_m: float, target_age_seconds: float,
            target_visible: bool = True) -> float:
        """Only rotate close to a fresh target; otherwise preserve camera heading."""
        radius = float(self.tracking_cfg.get(
            'yaw_adjust_enable_radius_m', 20.0))
        maximum_age = float(self.tracking_cfg.get(
            'yaw_adjust_max_target_age_seconds', 0.8))
        minimum_speed = float(self.tracking_cfg.get(
            'yaw_adjust_minimum_command_speed_mps', 0.35))
        if (not target_visible or target_age_seconds > maximum_age
                or horizontal_distance_m > radius
                or norm_xy(command) < minimum_speed):
            return 0.0
        return self.command_yaw_rate(command, fallback_yaw)

    def motion_validation_cfg(self) -> dict:
        return dict(self.tracking_cfg.get('motion_validation', {}))

    def build_suv_motion_gate(self) -> SuvMotionGate:
        cfg = self.motion_validation_cfg()
        return SuvMotionGate(
            observation_seconds=float(cfg.get(
                'observation_seconds', 5.0)),
            minimum_unique_samples=int(cfg.get(
                'minimum_unique_samples', 5)),
            minimum_confidence=float(cfg.get(
                'minimum_confidence', 0.60)),
            maximum_horizontal_std_m=float(cfg.get(
                'maximum_horizontal_std_m', 5.0)),
            moving_min_displacement_m=float(cfg.get(
                'moving_min_displacement_m', 2.5)),
            moving_min_speed_mps=float(cfg.get(
                'moving_min_speed_mps', 0.35)),
            stationary_max_displacement_m=float(cfg.get(
                'stationary_max_displacement_m', 1.5)),
            stationary_max_speed_mps=float(cfg.get(
                'stationary_max_speed_mps', 0.20)))

    def add_motion_row(self, gate: SuvMotionGate, row: Optional[dict],
                       default_vehicle: Optional[int] = None) -> None:
        if not row:
            return
        position = row.get('position_world', row.get('position'))
        if not isinstance(position, list) or len(position) < 3:
            return
        stamp = float(row.get(
            'source_ros_time',
            row.get('ros_time', rospy.Time.now().to_sec())))
        source_vehicle = row.get(
            'source_vehicle_id',
            row.get('vehicle_id', default_vehicle))
        measurement_id = str(row.get(
            'measurement_id',
            row.get('report_event_id',
                    f'{stamp:.6f}:v{source_vehicle}')))
        gate.add(
            position, stamp,
            float(row.get('confidence', 0.0)),
            float(row.get('horizontal_std_m', 0.0)),
            None if source_vehicle is None else int(source_vehicle),
            measurement_id)

    def collect_motion_samples(self, gate: SuvMotionGate, name: str,
                               target_row: Optional[dict] = None) -> dict:
        self.add_motion_row(gate, target_row)
        with self.lock:
            rows = list(self.latest_localization_reports.get(name, []))
        for row in rows:
            self.add_motion_row(gate, row, self.id)
        return gate.summary()

    def external_injection_reliable(self, row: Optional[dict]) -> bool:
        if not row:
            return False
        cfg = dict(self.tracking_cfg.get('external_injection', {}))
        if not bool(cfg.get('enabled', True)):
            return False
        age = self.target_age(row)
        source_vehicle = row.get('source_vehicle_id')
        other_vehicle = (
            source_vehicle is not None
            and int(source_vehicle) != int(self.id))
        reliable = bool(
            row.get('external_injection_reliable', False)
            or (
                float(row.get('confidence', 0.0))
                >= float(cfg.get('minimum_confidence', 0.65))
                and float(row.get('horizontal_std_m', float('inf')))
                <= float(cfg.get('maximum_horizontal_std_m', 5.0))
                and age <= float(cfg.get('maximum_age_seconds', 1.0))
            ))
        return reliable and (
            other_vehicle
            or bool(row.get('external_information_injection', False)))

    def report_motion_confirmed(self, name: str, summary: dict,
                                source: str) -> None:
        self.event(
            'DYNAMIC_TARGET_MOTION_CONFIRMED',
            target_name=name,
            validation_source=source,
            sample_count=int(summary.get('sample_count', 0)),
            elapsed_seconds=float(summary.get('elapsed_seconds', 0.0)),
            displacement_m=float(summary.get('displacement_m', 0.0)),
            speed_mps=float(summary.get('speed_mps', 0.0)),
            position_world=summary.get('position_world'),
            source_vehicle_ids=summary.get('source_vehicle_ids', []))

    def report_false_suv_and_wait(self, seq: int, summary: dict) -> None:
        self.set_phase('TRACK_SUV_STATIC_FALSE_POSITIVE')
        position = summary.get('position_world')
        self.event(
            'DYNAMIC_TARGET_STATIC_FALSE_POSITIVE',
            target_name='suv_camo',
            reclassified_target_name='prius_hybrid_camo',
            reclassified_position_world=position,
            sample_count=int(summary.get('sample_count', 0)),
            elapsed_seconds=float(summary.get('elapsed_seconds', 0.0)),
            displacement_m=float(summary.get('displacement_m', 0.0)),
            speed_mps=float(summary.get('speed_mps', 0.0)),
            mean_confidence=float(summary.get('mean_confidence', 0.0)),
            horizontal_std_m=float(summary.get(
                'horizontal_std_m', 0.0)),
            skip_static_verify=True)
        while self.assignment_sequence == seq and not rospy.is_shutdown():
            self.check_abort()
            if self.vtol_state() == VTOL_FW:
                # Preserve the already valid fixed-wing position/pass-through
                # setpoint until the manager publishes the resumed route.
                self.ensure_offboard()
            else:
                self.set_velocity_world(
                    np.zeros(3),
                    self.tracking_yaw_rate(self.current_yaw()))
            rospy.sleep(0.1)

    def fast_external_approach(
            self, seq: int, name: str, row: dict,
            recovery_altitude_m: float, fallback_yaw: float) -> Optional[dict]:
        cfg = dict(self.tracking_cfg.get('external_injection', {}))
        speed = float(cfg.get('fast_approach_speed_mps', 8.0))
        tolerance = float(cfg.get(
            'fast_approach_tolerance_m',
            self.tracking_cfg.get(
                'last_known_arrival_tolerance_m', 6.0)))
        timeout = float(cfg.get('fast_approach_timeout_seconds', 90.0))
        self.set_phase('TRACK_EXTERNAL_FAST_APPROACH')
        self.event(
            'EXTERNAL_TARGET_INFO_ACCEPTED',
            target_name=name,
            source_vehicle_id=row.get('source_vehicle_id'),
            confidence=float(row.get('confidence', 0.0)),
            horizontal_std_m=float(row.get('horizontal_std_m', 0.0)),
            age=self.target_age(row))
        self.event(
            'EXTERNAL_TARGET_FAST_APPROACH_START',
            target_name=name,
            recovery_altitude_m=float(recovery_altitude_m),
            speed_mps=speed)
        latest = dict(row)
        deadline = time.monotonic() + timeout
        filtered = np.zeros(3)
        previous = time.monotonic()
        while self.assignment_sequence == seq and time.monotonic() < deadline:
            self.check_abort()
            candidate = self.fresh_target_snapshot(
                name,
                float(cfg.get('maximum_age_seconds', 1.0)))
            if candidate is not None:
                latest = candidate
            target = self.predicted_target(latest)
            target[2] = float(recovery_altitude_m)
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05)
                continue
            error = target - current
            if norm_xy(error) <= tolerance:
                self.event(
                    'EXTERNAL_TARGET_FAST_APPROACH_REACHED',
                    target_name=name,
                    position_world=current.tolist())
                return latest
            cmd = np.zeros(3)
            cmd[:2] = 0.45 * error[:2]
            cmd = limit_vector_xy(cmd, speed)
            cmd[2] = float(np.clip(
                0.7 * error[2],
                -float(self.tracking_cfg.get(
                    'maximum_z_speed_mps', 2.0)),
                float(self.tracking_cfg.get(
                    'maximum_z_speed_mps', 2.0))))
            now = time.monotonic()
            dt = max(0.01, min(0.2, now - previous))
            previous = now
            allowed = float(self.tracking_cfg.get(
                'maximum_xy_acceleration_mps2', 1.5)) * dt
            delta = cmd - filtered
            dn = norm_xy(delta)
            if dn > allowed > 0.0:
                delta[:2] *= allowed / dn
            delta[2] = float(np.clip(delta[2], -allowed, allowed))
            filtered += delta
            self.set_velocity_world(
                filtered,
                self.protected_tracking_yaw_rate(
                    filtered, fallback_yaw, norm_xy(error),
                    self.target_age(latest), target_visible=True))
            self.detection_valid = True
            self.detection_segment_type = 'MC_EXTERNAL_FAST_APPROACH_YAW_PROTECTED'
            self.detection_leg_id = name
            rospy.sleep(1.0 / max(
                5.0, float(self.control['control_rate_hz'])))
        return self.fresh_target_snapshot(name)

    def fly_mc_to_last_known(
            self, seq: int, name: str, last_row: dict,
            recovery_altitude_m: float,
            fallback_yaw: float) -> Optional[dict]:
        """Fly to the last predicted position without changing MC altitude."""
        self.set_phase('TRACK_MC_LAST_KNOWN_APPROACH')
        target = self.predicted_target(last_row)
        target[2] = float(recovery_altitude_m)
        speed = float(self.tracking_cfg.get(
            'last_known_approach_speed_mps', 5.0))
        tol = float(self.tracking_cfg.get(
            'last_known_arrival_tolerance_m', 6.0))
        max_z = float(self.tracking_cfg.get(
            'maximum_z_speed_mps', 2.0))
        accel = float(self.tracking_cfg.get(
            'maximum_xy_acceleration_mps2', 1.5))
        rate = max(5.0, float(self.control['control_rate_hz']))
        filtered = np.zeros(3)
        previous = time.monotonic()
        deadline = time.monotonic() + float(
            self.tracking_cfg.get(
                'last_known_approach_timeout_seconds', 90.0))
        self.event(
            'TRACK_LAST_KNOWN_APPROACH_START',
            target_name=name,
            predicted_world=target.tolist(),
            source_age=self.target_age(last_row),
            recovery_altitude_m=float(recovery_altitude_m))
        while self.assignment_sequence == seq and time.monotonic() < deadline:
            self.check_abort()
            fresh = self.fresh_target_snapshot(name)
            if fresh is not None:
                if self.external_injection_reliable(fresh):
                    return self.fast_external_approach(
                        seq, name, fresh, recovery_altitude_m, fallback_yaw)
                self.event(
                    'TRACK_TARGET_REACQUIRED_EN_ROUTE',
                    target_name=name,
                    age=self.target_age(fresh),
                    source=fresh.get('source'))
                return fresh
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05)
                continue
            error = target - current
            if norm_xy(error) <= tol and abs(float(error[2])) <= 2.0:
                # Do not hover at the stale point. Coast along the last known
                # target velocity (or the current heading) while recovery starts.
                coast = np.asarray(
                    last_row.get('velocity', [0.0, 0.0, 0.0]),
                    dtype=float)
                coast[2] = 0.0
                coast_speed = float(self.tracking_cfg.get(
                    'loss_continue_speed_mps', 2.5))
                if norm_xy(coast) < 0.25:
                    coast[:2] = [math.cos(self.current_yaw()),
                                 math.sin(self.current_yaw())]
                coast = limit_vector_xy(coast, coast_speed)
                if norm_xy(coast) < 0.25:
                    coast[:2] = [coast_speed * math.cos(self.current_yaw()),
                                 coast_speed * math.sin(self.current_yaw())]
                self.set_velocity_world(coast, 0.0)
                self.event(
                    'TRACK_LAST_KNOWN_POSITION_REACHED_CONTINUE_MOTION',
                    target_name=name,
                    position_world=current.tolist(),
                    coast_velocity_world=coast.tolist(),
                    recovery_altitude_m=float(recovery_altitude_m))
                return None
            cmd = np.zeros(3)
            cmd[:2] = 0.45 * error[:2]
            cmd = limit_vector_xy(cmd, speed)
            cmd[2] = float(np.clip(
                0.7 * error[2], -max_z, max_z))
            now = time.monotonic()
            dt = max(0.01, min(0.2, now - previous))
            previous = now
            delta = cmd - filtered
            allowed = accel * dt
            dn = float(np.linalg.norm(delta[:2]))
            if dn > allowed > 0.0:
                delta[:2] *= allowed / dn
            delta[2] = float(np.clip(
                delta[2], -allowed, allowed))
            filtered += delta
            self.set_velocity_world(filtered, 0.0)
            self.detection_valid = True
            self.detection_segment_type = 'MC_LAST_KNOWN_APPROACH_YAW_FROZEN'
            self.detection_leg_id = name
            rospy.sleep(1.0 / rate)
        return self.fresh_target_snapshot(name)

    def reacquire_dynamic_target(
            self, seq: int, name: str, last_row: dict,
            recovery_altitude_m: float,
            fallback_yaw: float) -> Optional[dict]:
        """Centre-out square-spiral recovery at the current MC altitude."""
        self.set_phase('TRACK_MC_REACQUIRE')
        rate = max(5.0, float(self.control['control_rate_hz']))
        bounds = dict(self.local_cfg['search_area'])
        center_base = np.asarray(last_row['position'], dtype=float)
        target_velocity = np.asarray(
            last_row.get('velocity', [0, 0, 0]), dtype=float)
        last_stamp = float(last_row.get(
            'source_ros_time', rospy.Time.now().to_sec()))
        std = float(last_row.get('horizontal_std_m', 0.0))
        pass_index = 0
        while self.assignment_sequence == seq and not rospy.is_shutdown():
            self.check_abort()
            fresh = self.fresh_target_snapshot(name)
            if fresh is not None:
                if self.external_injection_reliable(fresh):
                    return self.fast_external_approach(
                        seq, name, fresh,
                        recovery_altitude_m, fallback_yaw)
                self.event(
                    'TRACK_TARGET_REACQUIRED',
                    target_name=name,
                    source=fresh.get('source'),
                    age=self.target_age(fresh),
                    reacquisition_pass=pass_index)
                return fresh
            now_ros = rospy.Time.now().to_sec()
            elapsed = max(0.0, now_ros - last_stamp)
            center = center_base + target_velocity * min(
                elapsed,
                float(self.tracking_cfg.get(
                    'reacquisition_prediction_cap_seconds', 120.0)))
            radius = possible_target_radius(
                elapsed,
                float(self.tracking_cfg.get(
                    'reacquisition_target_speed_bound_mps', 1.5)),
                float(self.tracking_cfg.get(
                    'reacquisition_base_radius_m', 20.0)),
                float(self.tracking_cfg.get(
                    'reacquisition_max_radius_m', 250.0)),
                std)
            current = self.current_world()
            waypoints = generate_square_spiral_reacquisition_waypoints(
                center, radius,
                float(self.tracking_cfg.get(
                    'reacquisition_lane_spacing_m', 90.0)),
                float(recovery_altitude_m),
                bounds, start=current)
            pass_index += 1
            self.event(
                'TRACK_REACQUISITION_PASS_START',
                target_name=name,
                pass_index=pass_index,
                pattern='square_spiral',
                center_world=center.tolist(),
                radius_m=radius,
                lane_spacing_m=float(self.tracking_cfg.get(
                    'reacquisition_lane_spacing_m', 90.0)),
                speed_mps=float(self.tracking_cfg.get(
                    'reacquisition_speed_mps', 6.0)),
                recovery_altitude_m=float(recovery_altitude_m),
                waypoint_count=len(waypoints),
                elapsed_since_last_seen=elapsed)
            if not waypoints:
                rospy.sleep(0.2)
                continue
            filtered = np.zeros(3)
            previous = time.monotonic()
            for waypoint in waypoints:
                desired = np.asarray(waypoint, dtype=float)
                desired[2] = float(recovery_altitude_m)
                while (self.assignment_sequence == seq
                       and not rospy.is_shutdown()):
                    self.check_abort()
                    fresh = self.fresh_target_snapshot(name)
                    if fresh is not None:
                        if self.external_injection_reliable(fresh):
                            return self.fast_external_approach(
                                seq, name, fresh,
                                recovery_altitude_m, fallback_yaw)
                        self.event(
                            'TRACK_TARGET_REACQUIRED',
                            target_name=name,
                            source=fresh.get('source'),
                            age=self.target_age(fresh),
                            reacquisition_pass=pass_index)
                        return fresh
                    current = self.current_world()
                    if current is None:
                        rospy.sleep(0.05)
                        continue
                    error = desired - current
                    if norm_xy(error) <= float(
                            self.tracking_cfg.get(
                                'reacquisition_waypoint_tolerance_m',
                                5.0)):
                        break
                    cmd = np.zeros(3)
                    cmd[:2] = 0.40 * error[:2]
                    cmd = limit_vector_xy(
                        cmd,
                        float(self.tracking_cfg.get(
                            'reacquisition_speed_mps', 6.0)))
                    cmd[2] = float(np.clip(
                        0.7 * error[2], -2.0, 2.0))
                    now = time.monotonic()
                    dt = max(
                        0.01, min(0.2, now - previous))
                    previous = now
                    delta = cmd - filtered
                    allowed = float(
                        self.tracking_cfg.get(
                            'maximum_xy_acceleration_mps2',
                            1.5)) * dt
                    dn = float(np.linalg.norm(delta[:2]))
                    if dn > allowed > 0.0:
                        delta[:2] *= allowed / dn
                    delta[2] = float(np.clip(
                        delta[2], -allowed, allowed))
                    filtered += delta
                    self.set_velocity_world(filtered, 0.0)
                    self.detection_valid = True
                    self.detection_segment_type = (
                        'MC_REACQUIRE_SQUARE_SPIRAL_YAW_FROZEN')
                    self.detection_leg_id = name
                    rospy.sleep(1.0 / rate)
        return None

    def validate_suv_motion_mc(
            self, seq: int, last_valid: dict,
            recovery_altitude_m: float, fallback_yaw: float,
            gate: SuvMotionGate) -> Optional[dict]:
        """Keep ~40 m while validating SUV motion and recovering visibility."""
        self.set_phase('TRACK_VALIDATE_SUV_MC')
        bounds = dict(self.local_cfg['search_area'])
        rate = max(5.0, float(self.control['control_rate_hz']))
        filtered = np.zeros(3)
        previous = time.monotonic()
        spiral: List[List[float]] = []
        spiral_index = 0
        pass_index = 0
        last_external_id = ''
        while self.assignment_sequence == seq and not rospy.is_shutdown():
            self.check_abort()
            fresh = self.fresh_target_snapshot('suv_camo')
            if fresh is not None:
                last_valid = fresh
            summary = self.collect_motion_samples(
                gate, 'suv_camo', fresh or last_valid)
            if summary.get('status') == 'moving':
                self.report_motion_confirmed(
                    'suv_camo', summary, 'five_second_motion_gate')
                self.event(
                    'TRACK_ALTITUDE_DESCENT_ENABLED',
                    target_name='suv_camo',
                    from_altitude_m=float(recovery_altitude_m),
                    to_relative_altitude_m=float(
                        self.mission_params['track_altitude_m']))
                return fresh or last_valid
            if summary.get('status') == 'stationary':
                self.report_false_suv_and_wait(seq, summary)
                return None

            current = self.current_world()
            if current is None:
                rospy.sleep(0.05)
                continue
            external = (
                fresh is not None
                and self.external_injection_reliable(fresh))
            if fresh is not None:
                desired = self.predicted_target(fresh)
                desired[2] = float(recovery_altitude_m)
                if external:
                    measurement_id = str(fresh.get(
                        'measurement_id',
                        fresh.get('source_ros_time', '')))
                    if measurement_id != last_external_id:
                        last_external_id = measurement_id
                        self.event(
                            'EXTERNAL_TARGET_INFO_ACCEPTED',
                            target_name='suv_camo',
                            source_vehicle_id=fresh.get(
                                'source_vehicle_id'),
                            confidence=float(
                                fresh.get('confidence', 0.0)),
                            horizontal_std_m=float(
                                fresh.get('horizontal_std_m', 0.0)),
                            validation_pending=True)
                speed = float(
                    self.tracking_cfg.get(
                        'external_injection', {}).get(
                            'fast_approach_speed_mps', 8.0)
                    if external else
                    self.tracking_cfg.get(
                        'last_known_approach_speed_mps', 5.0))
                segment_type = (
                    'MC_SUV_VALIDATE_EXTERNAL_APPROACH'
                    if external else
                    'MC_SUV_VALIDATE_APPROACH')
            else:
                if not spiral or spiral_index >= len(spiral):
                    now_ros = rospy.Time.now().to_sec()
                    last_stamp = float(last_valid.get(
                        'source_ros_time', now_ros))
                    elapsed = max(0.0, now_ros - last_stamp)
                    center = np.asarray(
                        last_valid['position'], dtype=float)
                    velocity = np.asarray(
                        last_valid.get(
                            'velocity', [0.0, 0.0, 0.0]),
                        dtype=float)
                    center += velocity * min(
                        elapsed,
                        float(self.tracking_cfg.get(
                            'reacquisition_prediction_cap_seconds',
                            120.0)))
                    radius = possible_target_radius(
                        elapsed,
                        float(self.tracking_cfg.get(
                            'reacquisition_target_speed_bound_mps',
                            1.5)),
                        float(self.tracking_cfg.get(
                            'reacquisition_base_radius_m', 20.0)),
                        float(self.tracking_cfg.get(
                            'reacquisition_max_radius_m', 250.0)),
                        float(last_valid.get(
                            'horizontal_std_m', 0.0)))
                    spiral = (
                        generate_square_spiral_reacquisition_waypoints(
                            center, radius,
                            float(self.tracking_cfg.get(
                                'reacquisition_lane_spacing_m',
                                90.0)),
                            float(recovery_altitude_m),
                            bounds, start=current))
                    spiral_index = 0
                    pass_index += 1
                    self.event(
                        'TRACK_REACQUISITION_PASS_START',
                        target_name='suv_camo',
                        validation_pending=True,
                        pass_index=pass_index,
                        pattern='square_spiral',
                        center_world=center.tolist(),
                        radius_m=radius,
                        lane_spacing_m=float(
                            self.tracking_cfg.get(
                                'reacquisition_lane_spacing_m',
                                90.0)),
                        speed_mps=float(
                            self.tracking_cfg.get(
                                'reacquisition_speed_mps', 6.0)),
                        recovery_altitude_m=float(
                            recovery_altitude_m),
                        waypoint_count=len(spiral),
                        elapsed_since_last_seen=elapsed)
                    if not spiral:
                        rospy.sleep(0.1)
                        continue
                desired = np.asarray(
                    spiral[spiral_index], dtype=float)
                desired[2] = float(recovery_altitude_m)
                if norm_xy(desired - current) <= float(
                        self.tracking_cfg.get(
                            'reacquisition_waypoint_tolerance_m',
                            5.0)):
                    spiral_index += 1
                    continue
                speed = float(self.tracking_cfg.get(
                    'reacquisition_speed_mps', 6.0))
                segment_type = 'MC_SUV_VALIDATE_SQUARE_SPIRAL'

            error = desired - current
            cmd = np.zeros(3)
            cmd[:2] = 0.42 * error[:2]
            cmd = limit_vector_xy(cmd, speed)
            cmd[2] = float(np.clip(
                0.75 * error[2],
                -float(self.tracking_cfg.get(
                    'maximum_z_speed_mps', 2.0)),
                float(self.tracking_cfg.get(
                    'maximum_z_speed_mps', 2.0))))
            now = time.monotonic()
            dt = max(0.01, min(0.2, now - previous))
            previous = now
            allowed = float(self.tracking_cfg.get(
                'maximum_xy_acceleration_mps2', 1.5)) * dt
            delta = cmd - filtered
            dn = norm_xy(delta)
            if dn > allowed > 0.0:
                delta[:2] *= allowed / dn
            delta[2] = float(np.clip(
                delta[2], -allowed, allowed))
            filtered += delta
            visible = fresh is not None
            target_age = (self.target_age(fresh)
                          if visible else float('inf'))
            self.set_velocity_world(
                filtered,
                self.protected_tracking_yaw_rate(
                    filtered, fallback_yaw, norm_xy(error),
                    target_age, target_visible=visible))
            self.detection_valid = True
            self.detection_segment_type = segment_type + '_YAW_PROTECTED'
            self.detection_leg_id = 'suv_camo'
            rospy.sleep(1.0 / rate)
        return None

    def execute_track(self, assignment: dict) -> None:
        seq = int(assignment['sequence'])
        name = str(assignment['target_name'])
        self.tracking_target = name
        self.tracking_points = 0
        self.tracking_started_ros = None
        self.reset_detection_gate()
        self.set_phase('TRACK_INTERCEPT_FW')
        motion_required = bool(
            assignment.get('motion_lock_required', name == 'suv_camo'))
        motion_confirmed = bool(
            assignment.get(
                'dynamic_lock_confirmed',
                name != 'suv_camo'))
        motion_gate = (
            self.build_suv_motion_gate()
            if motion_required and not motion_confirmed
            else None)
        if name == 'person_red' and not motion_confirmed:
            motion_confirmed = True
        if name == 'person_red':
            self.report_motion_confirmed(
                name,
                {
                    'sample_count': 0,
                    'elapsed_seconds': 0.0,
                    'displacement_m': 0.0,
                    'speed_mps': 0.0,
                    'position_world': (
                        assignment.get('target_estimate') or {}
                    ).get('position'),
                    'source_vehicle_ids': [],
                },
                'person_red_class_lock_bypass')

        deadline = time.monotonic() + 180.0
        last_valid = assignment.get('target_estimate')
        while (self.assignment_sequence == seq
               and time.monotonic() < deadline):
            self.check_abort()
            row = self.target_snapshot(name)
            if row is None:
                rospy.sleep(0.1)
                continue
            age = self.target_age(row)
            if age <= float(self.tracking_cfg.get(
                    'reacquire_timeout_seconds', 20.0)):
                last_valid = row
            else:
                rospy.logwarn_throttle(
                    2.0,
                    'v%d waiting fresh handoff for %s age=%.2fs source=%s',
                    self.id, name, age, row.get('source'))
                rospy.sleep(0.1)
                continue

            if motion_gate is not None and not motion_confirmed:
                summary = self.collect_motion_samples(
                    motion_gate, name, row)
                if summary.get('status') == 'moving':
                    motion_confirmed = True
                    self.report_motion_confirmed(
                        name, summary,
                        'five_second_motion_gate_fw')
                elif summary.get('status') == 'stationary':
                    self.report_false_suv_and_wait(seq, summary)
                    self.reset_detection_gate()
                    self.tracking_target = ''
                    return

            target = self.predicted_target(row)
            desired = target.copy()
            # Fixed-wing interception remains at the aircraft's approximately
            # 40 m search altitude. It never descends toward the 30 m MC
            # tracking height.
            desired[2] = float(
                self.tracking_cfg['intercept_altitude_m'])
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05)
                continue
            if self.vtol_state() != VTOL_FW:
                self.ensure_fw(desired)
            yaw = math.atan2(
                desired[1] - current[1],
                desired[0] - current[0])
            self.set_position_world(desired, yaw)
            self.ensure_offboard()
            heading_error = abs(math.degrees(
                wrap_pi(self.current_yaw() - yaw)))
            self.detection_valid = bool(
                self.vtol_state() == VTOL_FW
                and self.ground_speed() >= float(
                    self.gate_cfg['minimum_ground_speed_mps'])
                and heading_error <= float(
                    self.gate_cfg['maximum_heading_error_deg']))
            self.detection_segment_type = (
                'TRACK_INTERCEPT_STRAIGHT')
            self.detection_leg_id = name
            self.heading_error_deg = heading_error
            transition_radius_m = float(
                self.tracking_cfg.get(
                    'transition_to_mc_radius_m', 30.0))
            horizontal_distance_m = norm_xy(desired - current)
            if horizontal_distance_m <= transition_radius_m:
                self.event(
                    'TRACK_FW_30M_TRANSITION_GATE_REACHED',
                    target_name=name,
                    horizontal_distance_m=horizontal_distance_m,
                    transition_radius_m=transition_radius_m,
                    intercept_altitude_m=float(
                        self.tracking_cfg['intercept_altitude_m']))
                break
            rospy.sleep(0.05)

        if self.assignment_sequence != seq:
            return
        if last_valid is None:
            raise RuntimeError(
                f'no target handoff available for {name}')
        self.event(
            'TRACK_TARGET_HANDOFF_ACQUIRED',
            target_name=name,
            source=last_valid.get('source'),
            source_age=self.target_age(last_valid),
            motion_lock_required=motion_required,
            motion_lock_confirmed=motion_confirmed)

        # Retain the valid fixed-wing setpoint during conversion.
        self.set_phase('TRACK_TRANSITION_MC')
        self.transition(VTOL_MC)
        fallback_yaw = self.current_yaw()
        transition_hold = self.current_world()
        recovery_altitude = (
            float(transition_hold[2])
            if transition_hold is not None
            else float(self.tracking_cfg['intercept_altitude_m']))
        if transition_hold is not None:
            hold = transition_hold.copy()
            hold[2] = recovery_altitude
            self.set_position_world(hold, fallback_yaw)
        self.event(
            'TRACK_MC_ALTITUDE_HOLD_CAPTURED',
            target_name=name,
            recovery_altitude_m=recovery_altitude,
            descent_deferred=not motion_confirmed)
        rospy.sleep(float(
            self.control['post_transition_hold_seconds']))

        if motion_gate is not None and not motion_confirmed:
            # Localization callbacks continue during the blocking VTOL service;
            # collect those reports before deciding stationary versus moving.
            summary = self.collect_motion_samples(
                motion_gate, name,
                self.fresh_target_snapshot(name) or last_valid)
            if summary.get('status') == 'stationary':
                self.report_false_suv_and_wait(seq, summary)
                self.reset_detection_gate()
                self.tracking_target = ''
                return
            if summary.get('status') == 'moving':
                motion_confirmed = True
                self.report_motion_confirmed(
                    name, summary,
                    'five_second_motion_gate_transition')
                self.event(
                    'TRACK_ALTITUDE_DESCENT_ENABLED',
                    target_name=name,
                    from_altitude_m=recovery_altitude,
                    to_relative_altitude_m=float(
                        self.mission_params['track_altitude_m']))
            else:
                validated = self.validate_suv_motion_mc(
                    seq, last_valid, recovery_altitude,
                    fallback_yaw, motion_gate)
                if self.assignment_sequence != seq:
                    self.reset_detection_gate()
                    self.tracking_target = ''
                    return
                if validated is None:
                    self.reset_detection_gate()
                    self.tracking_target = ''
                    return
                last_valid = validated
                motion_confirmed = True

        fresh = self.fresh_target_snapshot(name)
        if fresh is None:
            fresh = self.fly_mc_to_last_known(
                seq, name, last_valid,
                recovery_altitude, fallback_yaw)
        if fresh is None:
            fresh = self.reacquire_dynamic_target(
                seq, name, last_valid,
                recovery_altitude, fallback_yaw)
        if self.assignment_sequence != seq:
            return

        self.set_phase('TRACK_DYNAMIC_MC')
        self.tracking_started_ros = rospy.Time.now().to_sec()
        target_filter = DynamicTargetFilter(
            float(self.tracking_cfg.get(
                'position_filter_time_constant_seconds', 0.8)),
            float(self.tracking_cfg.get(
                'velocity_filter_time_constant_seconds', 1.2)),
            float(self.tracking_cfg.get(
                'measurement_jump_gate_m', 35.0)))
        integral = np.zeros(3)
        filtered_cmd = np.zeros(3)
        previous_t = time.monotonic()
        if fresh is not None:
            target_filter.reset(
                fresh['position'],
                fresh.get('velocity', [0, 0, 0]),
                float(fresh.get(
                    'source_ros_time',
                    rospy.Time.now().to_sec())))
            last_valid = fresh

        while (self.assignment_sequence == seq
               and not rospy.is_shutdown()):
            self.check_abort()
            row = self.fresh_target_snapshot(name)
            now_wall = time.monotonic()
            if row is None:
                stale = self.target_snapshot(name) or last_valid
                age = (
                    self.target_age(stale)
                    if stale is not None else float('inf'))
                self.event(
                    'TRACK_TARGET_STREAM_STALE',
                    target_name=name,
                    age=age,
                    source=(
                        None if stale is None
                        else stale.get('source')))
                current = self.current_world()
                loss_altitude = (
                    float(current[2])
                    if current is not None
                    else recovery_altitude)
                self.event(
                    'TRACK_LOSS_CONTINUE_TO_LAST_KNOWN',
                    target_name=name,
                    loss_altitude_m=loss_altitude,
                    yaw_policy='freeze_current_heading_no_hover')
                recovered = None
                if stale is not None:
                    recovered = self.fly_mc_to_last_known(
                        seq, name, stale,
                        loss_altitude, fallback_yaw)
                    if recovered is None:
                        recovered = self.reacquire_dynamic_target(
                            seq, name, stale,
                            loss_altitude, fallback_yaw)
                if recovered is None:
                    continue
                self.event(
                    'TRACK_TARGET_STREAM_REACQUIRED',
                    target_name=name,
                    age=self.target_age(recovered),
                    source=recovered.get('source'),
                    recovery_altitude_m=loss_altitude)
                target_filter.reset(
                    recovered['position'],
                    recovered.get('velocity', [0, 0, 0]),
                    float(recovered.get(
                        'source_ros_time',
                        rospy.Time.now().to_sec())))
                last_valid = recovered
                integral[:] = 0.0
                filtered_cmd[:] = 0.0
                previous_t = time.monotonic()
                self.set_phase('TRACK_DYNAMIC_MC')
                continue

            last_valid = row
            stamp = float(row.get(
                'source_ros_time',
                rospy.Time.now().to_sec()))
            # A reliable cross-UAV reinitialization may legitimately be much
            # farther away than the local 35 m jump gate. Reset rather than
            # repeatedly steering toward a stale position.
            reset_for_external = bool(
                row.get('track_reinitialized', False)
                or self.external_injection_reliable(row))
            if reset_for_external and target_filter.position is not None:
                jump = norm_xy(
                    np.asarray(row['position'], dtype=float)
                    - target_filter.position)
                if jump > float(self.tracking_cfg.get(
                        'measurement_jump_gate_m', 35.0)):
                    target_filter.reset(
                        row['position'],
                        row.get('velocity', [0, 0, 0]),
                        stamp)
                    target_position = np.asarray(
                        row['position'], dtype=float)
                    target_velocity = np.asarray(
                        row.get('velocity', [0, 0, 0]),
                        dtype=float)
                    accepted = True
                    self.event(
                        'TRACK_FILTER_EXTERNAL_RESET',
                        target_name=name,
                        jump_m=jump,
                        source_vehicle_id=row.get(
                            'source_vehicle_id'))
                else:
                    target_position, target_velocity, accepted = (
                        target_filter.update(
                            row['position'],
                            row.get('velocity', [0, 0, 0]),
                            stamp))
            else:
                target_position, target_velocity, accepted = (
                    target_filter.update(
                        row['position'],
                        row.get('velocity', [0, 0, 0]),
                        stamp))
            if not accepted:
                rospy.sleep(0.05)
                continue

            age = self.target_age(row)
            target_position = (
                target_position
                + target_velocity * min(
                    float(self.tracking_cfg.get(
                        'prediction_seconds', 1.0)) + age,
                    float(self.tracking_cfg.get(
                        'target_timeout_seconds', 5.0))))
            desired = target_position.copy()
            desired[2] = (
                target_position[2]
                + float(self.mission_params[
                    'track_altitude_m']))
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05)
                continue
            dt = max(
                0.01, min(0.2, now_wall - previous_t))
            previous_t = now_wall
            error = desired - current
            if norm_xy(error) <= float(
                    self.tracking_cfg.get(
                        'position_deadband_m', 1.5)):
                error[:2] = 0.0
            integral += error * dt
            limxy = float(self.tracking_cfg.get(
                'integral_limit_xy', 2.0))
            n = float(np.linalg.norm(integral[:2]))
            if n > limxy > 0.0:
                integral[:2] *= limxy / n
            integral[2] = np.clip(
                integral[2],
                -float(self.tracking_cfg.get(
                    'integral_limit_z', 1.5)),
                float(self.tracking_cfg.get(
                    'integral_limit_z', 1.5)))
            own_velocity = self.current_world_velocity()
            relative_velocity = target_velocity - own_velocity
            cmd = np.zeros(3)
            cmd[:2] = (
                float(self.tracking_cfg.get(
                    'position_kp_xy', 0.45)) * error[:2]
                + float(self.tracking_cfg.get(
                    'relative_velocity_damping_gain',
                    0.65)) * relative_velocity[:2]
                + float(self.tracking_cfg.get(
                    'integral_gain_xy',
                    0.02)) * integral[:2]
                + float(self.tracking_cfg.get(
                    'velocity_feedforward_gain',
                    0.75)) * target_velocity[:2])
            cmd[2] = (
                float(self.tracking_cfg.get(
                    'kp_z', 0.7)) * error[2]
                - float(self.tracking_cfg.get(
                    'vertical_velocity_damping_gain',
                    0.35)) * own_velocity[2])
            external = self.external_injection_reliable(row)
            max_xy = float(
                self.tracking_cfg.get(
                    'external_injection', {}).get(
                        'fast_approach_speed_mps', 8.0)
                if external and norm_xy(error) >= float(
                    self.tracking_cfg.get(
                        'external_injection', {}).get(
                            'minimum_position_change_m', 8.0))
                else self.tracking_cfg.get(
                    'maximum_xy_speed_mps', 5.0))
            cmd = limit_vector_xy(cmd, max_xy)
            cmd[2] = np.clip(
                cmd[2],
                -float(self.tracking_cfg.get(
                    'maximum_z_speed_mps', 2.0)),
                float(self.tracking_cfg.get(
                    'maximum_z_speed_mps', 2.0)))
            max_delta = float(
                self.tracking_cfg.get(
                    'maximum_xy_acceleration_mps2',
                    1.5)) * dt
            delta = cmd - filtered_cmd
            dn = float(np.linalg.norm(delta[:2]))
            if dn > max_delta > 0.0:
                delta[:2] *= max_delta / dn
            delta[2] = np.clip(
                delta[2],
                -float(self.tracking_cfg.get(
                    'maximum_z_acceleration_mps2',
                    1.0)) * dt,
                float(self.tracking_cfg.get(
                    'maximum_z_acceleration_mps2',
                    1.0)) * dt)
            tau = float(self.tracking_cfg.get(
                'command_filter_time_constant_seconds',
                0.45))
            alpha = 1.0 - math.exp(
                -dt / max(tau, 1e-3))
            filtered_cmd += alpha * delta
            horizontal_distance = norm_xy(error)
            self.set_velocity_world(
                filtered_cmd,
                self.protected_tracking_yaw_rate(
                    filtered_cmd, fallback_yaw,
                    horizontal_distance,
                    age, target_visible=True))
            self.detection_valid = True
            self.detection_segment_type = 'MC_TRACK'
            self.detection_leg_id = name
            if norm_xy(error) <= float(
                    self.tracking_cfg.get(
                        'center_tolerance_m', 3.0)):
                self.tracking_points += 1
            rospy.sleep(
                1.0 / float(self.control['control_rate_hz']))
        self.reset_detection_gate()
        self.tracking_target = ''

    def order_static_targets(self, rows: List[dict]) -> List[dict]:
        remaining = list(rows); ordered = []; current = self.current_world()
        if current is None: return remaining
        while remaining:
            best = min(remaining, key=lambda r: norm_xy(np.asarray(r.get('target_world', [*r['ground_xy'],0.2]))-current))
            ordered.append(best); current = np.asarray(best.get('target_world', [*best['ground_xy'],0.2]), dtype=float); remaining.remove(best)
        return ordered

    def static_yaw_rate(self, target_yaw: Optional[float] = None) -> float:
        desired = math.radians(float(self.static_cfg.get('hover_yaw_deg', 0.0))) \
            if target_yaw is None else float(target_yaw)
        return yaw_rate_command(
            self.current_yaw(), desired,
            float(self.static_cfg.get('yaw_kp', 1.2)),
            float(self.static_cfg.get('maximum_yaw_rate_rad_s', 0.35)),
            float(self.static_cfg.get('yaw_deadband_deg', 2.0)))

    def fly_mc_static_to(self, desired: np.ndarray, seq: int, phase: str,
                         xy_tol: float, z_tol: float, timeout: float,
                         yaw_target: Optional[float] = None) -> None:
        """Velocity-guided MC transit with explicit yaw control."""
        self.set_phase(phase)
        rate = max(5.0, float(self.control['control_rate_hz']))
        kp_xy = float(self.static_cfg.get('kp_xy', 0.45))
        kp_z = float(self.static_cfg.get('kp_z', 0.8))
        max_xy = float(self.static_cfg.get('maximum_xy_speed_mps', 4.0))
        max_z = float(self.static_cfg.get('maximum_z_speed_mps', 2.0))
        max_acc = float(self.static_cfg.get('maximum_xy_acceleration_mps2', 2.0))
        progress_timeout = float(self.static_cfg.get('progress_timeout_seconds', 20.0))
        progress_min = float(self.static_cfg.get('progress_minimum_m', 2.0))
        deadline = time.monotonic() + timeout
        filtered = np.zeros(3)
        previous = time.monotonic(); best_distance = float('inf'); last_progress = time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            if self.assignment_sequence != seq:
                return
            self.ensure_offboard()
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05); continue
            error = np.asarray(desired, dtype=float) - current
            yaw_error_deg = abs(math.degrees(wrap_pi(
                (math.radians(float(self.static_cfg.get('hover_yaw_deg', 0.0)))
                 if yaw_target is None else float(yaw_target)) - self.current_yaw())))
            if (norm_xy(error) <= xy_tol and abs(float(error[2])) <= z_tol
                    and yaw_error_deg <= float(self.static_cfg.get('hover_yaw_tolerance_deg', 5.0))):
                self.set_velocity_world(np.zeros(3), self.static_yaw_rate(yaw_target))
                self.last_velocity_world[:] = 0.0
                return
            distance = norm_xy(error)
            if distance < best_distance - progress_min:
                best_distance = distance; last_progress = time.monotonic()
            elif time.monotonic() - last_progress > progress_timeout:
                self.event('STATIC_VERIFY_PROGRESS_STALL', phase=phase, distance=distance,
                           desired=desired.tolist(), target_name=self.static_verify_target)
                filtered[:] = 0.0; last_progress = time.monotonic(); best_distance = distance
            cmd = np.zeros(3); cmd[:2] = kp_xy * error[:2]
            cmd = limit_vector_xy(cmd, max_xy)
            cmd[2] = float(np.clip(kp_z * error[2], -max_z, max_z))
            now = time.monotonic(); dt = max(0.01, min(0.2, now - previous)); previous = now
            delta = cmd - filtered; allowed = max_acc * dt
            dn = float(np.linalg.norm(delta[:2]))
            if dn > allowed > 0.0:
                delta[:2] *= allowed / dn
            delta[2] = float(np.clip(delta[2], -allowed, allowed))
            filtered += delta
            self.set_velocity_world(filtered, self.static_yaw_rate(yaw_target))
            self.last_velocity_world = filtered.copy()
            rospy.sleep(1.0 / rate)
        self.set_velocity_world(np.zeros(3), self.static_yaw_rate(yaw_target))
        raise RuntimeError(
            f'static velocity timeout phase={phase} target={self.static_verify_target} desired={desired.tolist()}')

    def localization_reports_since(self, name: str, start_ros: float) -> List[dict]:
        with self.lock:
            rows = [dict(row) for row in self.latest_localization_reports.get(name, [])]
        return [row for row in rows
                if float(row.get('source_ros_time', row.get('ros_time', 0.0))) >= float(start_ros)]

    def hold_static_hover(self, hover: np.ndarray, seq: int, name: str,
                          frozen_target: np.ndarray) -> Optional[dict]:
        """Hold 30 m / yaw 0 / 5 m for 3 s, then fuse hover detections."""
        stable_required = float(self.static_cfg.get('stable_seconds', 3.0))
        xy_tol = float(self.static_cfg.get('hover_xy_tolerance_m', 5.0))
        z_tol = float(self.static_cfg.get('hover_z_tolerance_m', 1.0))
        yaw_tol = float(self.static_cfg.get('hover_yaw_tolerance_deg', 5.0))
        target_yaw = math.radians(float(self.static_cfg.get('hover_yaw_deg', 0.0)))
        timeout = float(self.static_cfg.get('mc_approach_timeout_seconds', 90.0))
        deadline = time.monotonic() + timeout; stable_start = None
        rate = max(5.0, float(self.control['control_rate_hz']))
        kp_xy = min(0.5, float(self.static_cfg.get('kp_xy', 0.45)))
        kp_z = min(0.8, float(self.static_cfg.get('kp_z', 0.8)))
        report_start_ros = rospy.Time.now().to_sec()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            if self.assignment_sequence != seq:
                return None
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05); continue
            error = hover - current
            yaw_error_deg = abs(math.degrees(wrap_pi(target_yaw - self.current_yaw())))
            inside = (norm_xy(error) <= xy_tol and abs(float(error[2])) <= z_tol
                      and yaw_error_deg <= yaw_tol)
            if inside:
                if stable_start is None:
                    stable_start = time.monotonic(); report_start_ros = rospy.Time.now().to_sec()
                    self.event('STATIC_HOVER_STABLE_START', target_name=name,
                               frozen_target_world=frozen_target.tolist(),
                               hover_world=hover.tolist(), target_yaw_deg=math.degrees(target_yaw))
                if time.monotonic() - stable_start >= stable_required:
                    self.set_velocity_world(np.zeros(3), self.static_yaw_rate(target_yaw))
                    self.last_velocity_world[:] = 0.0
                    reports = self.localization_reports_since(name, report_start_ros)
                    fusion = weighted_position_fusion(
                        reports, name,
                        int(self.static_cfg.get('hover_refinement_minimum_reports', 2)),
                        float(self.static_cfg.get('hover_refinement_maximum_std_m', 12.0)))
                    return fusion
            else:
                stable_start = None
            cmd = np.zeros(3); cmd[:2] = kp_xy * error[:2]
            cmd = limit_vector_xy(cmd, min(1.5, float(self.static_cfg.get('maximum_xy_speed_mps', 4.0))))
            cmd[2] = float(np.clip(kp_z * error[2], -1.0, 1.0))
            self.set_velocity_world(cmd, self.static_yaw_rate(target_yaw))
            self.last_velocity_world = cmd.copy()
            rospy.sleep(1.0 / rate)
        self.set_velocity_world(np.zeros(3), self.static_yaw_rate(target_yaw))
        raise RuntimeError(f'static hover stability timeout target={name}')

    def fixed_wing_static_intercept(self, target: np.ndarray, seq: int, name: str) -> None:
        """Fly a valid pass-through setpoint and accept safe closest approach."""
        self.set_phase('STATIC_INTERCEPT_FW')
        approach = np.asarray(target, dtype=float).copy()
        approach[2] = self.home_world[2] + float(self.mission_params['search_altitude_m'])
        self.ensure_fw(approach)
        current = self.current_world()
        if current is None:
            raise RuntimeError('world unavailable for static intercept')
        direction = approach[:2] - current[:2]
        length = max(float(np.linalg.norm(direction)), 1.0); direction /= length
        pass_point = approach.copy()
        pass_point[:2] += direction * float(self.static_cfg.get('fixed_wing_pass_through_m', 100.0))
        radius = float(self.static_cfg.get('transition_to_mc_radius_m', 30.0))
        fallback_radius = float(self.static_cfg.get('fixed_wing_fallback_transition_radius_m', 55.0))
        growth = float(self.static_cfg.get('fixed_wing_closest_approach_growth_m', 8.0))
        deadline = time.monotonic() + float(self.static_cfg.get('fixed_wing_intercept_timeout_seconds', 120.0))
        best = float('inf'); best_time = time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            if self.assignment_sequence != seq:
                return
            if not self.ensure_offboard():
                self.recover_offboard_hold('static_fixed_wing_intercept', 6.0)
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05); continue
            distance = norm_xy(approach - current)
            if distance < best:
                best = distance; best_time = time.monotonic()
            if distance <= radius:
                self.event('STATIC_FW_INTERCEPT_REACHED', target_name=name,
                           distance_m=distance, rule='nominal_radius')
                return
            # If the aircraft has already passed the closest point, transition is
            # safer than commanding zero velocity in FW or falling into HOLD.
            if (best <= fallback_radius and distance >= best + growth
                    and time.monotonic() - best_time >= 0.4):
                self.event('STATIC_FW_INTERCEPT_REACHED', target_name=name,
                           distance_m=distance, best_distance_m=best,
                           rule='closest_approach_fallback')
                return
            yaw = math.atan2(pass_point[1] - current[1], pass_point[0] - current[0])
            self.set_position_world(pass_point, yaw)
            rospy.sleep(float(self.control['search_interrupt_poll_seconds']))
        raise RuntimeError(
            f'static fixed-wing intercept timeout target={name} best_distance={best:.1f}m')

    def _static_candidate_key(self, row: dict) -> str:
        candidate = str(row.get('candidate_id', '')).strip()
        if candidate:
            return candidate
        name = str(row.get('target_name', 'unknown'))
        p = row.get('frozen_target_world', row.get('target_world',
                    [*row.get('ground_xy', [0.0, 0.0]), 0.2]))
        return f"{name}@{float(p[0]):.1f},{float(p[1]):.1f}"

    def verify_one_static_target(
            self, row: dict, seq: int,
            next_hint: Optional[np.ndarray]) -> bool:
        name = str(row['target_name'])
        candidate_id = self._static_candidate_key(row)
        frozen_target = np.asarray(
            row.get('frozen_target_world',
                    row.get('target_world', [*row['ground_xy'], 0.2])),
            dtype=float)
        self.static_verify_target = name
        self.static_verify_candidate_id = candidate_id
        offsets = dict(self.static_cfg.get('target_hover_offset_xy_m', {}))
        hover = static_hover_point(
            frozen_target, name,
            float(self.static_cfg.get('hover_altitude_m',
                  self.mission_params.get('static_verify_altitude_m', 30.0))),
            offsets)
        self.event('STATIC_VERIFY_TARGET_START', target_name=name,
                   candidate_id=candidate_id,
                   frozen_target_world=frozen_target.tolist(),
                   hover_world=hover.tolist(), attempt=self.static_verify_attempt,
                   position_lock='frozen_until_hover_refinement')
        intercept_target = frozen_target.copy()
        intercept_target[:2] = hover[:2]
        self.fixed_wing_static_intercept(intercept_target, seq, name)
        if self.assignment_sequence != seq:
            return False

        self.set_phase('STATIC_TRANSITION_MC')
        self.transition(VTOL_MC)
        current = self.current_world()
        target_yaw = math.radians(float(
            self.static_cfg.get('hover_yaw_deg', 0.0)))
        if current is not None:
            self.set_position_world(current, target_yaw)
        rospy.sleep(float(self.control.get('post_transition_hold_seconds', 1.0)))
        self.fly_mc_static_to(
            hover, seq, 'STATIC_MC_FINAL_APPROACH',
            float(self.static_cfg.get('hover_xy_tolerance_m', 5.0)),
            float(self.static_cfg.get('hover_z_tolerance_m', 1.0)),
            float(self.static_cfg.get('mc_approach_timeout_seconds', 90.0)),
            yaw_target=target_yaw)
        self.set_phase('STATIC_HOVER_VERIFY')
        refinement = self.hold_static_hover(
            hover, seq, name, frozen_target)
        if self.assignment_sequence != seq:
            return False

        confirmed = refinement is not None
        if confirmed:
            refined_target = np.asarray(
                refinement['position_world'], dtype=float)
            confidence = float(refinement.get(
                'mean_confidence', row.get('confidence', 0.0)))
            precise = {
                'candidate_id': candidate_id,
                'target_name': name,
                'target_world': refined_target.tolist(),
                'confidence': confidence,
                'refinement': refinement,
                'frozen_target_world': frozen_target.tolist(),
            }
            self.static_precise_candidates.setdefault(name, []).append(precise)
            self.event('STATIC_POSITION_REFINED', target_name=name,
                       candidate_id=candidate_id,
                       frozen_target_world=frozen_target.tolist(),
                       refined_target_world=refined_target.tolist(),
                       hover_world=hover.tolist(), refinement=refinement)
            self.event('STATIC_CANDIDATE_PRECISE_CONFIRMED',
                       target_name=name, candidate_id=candidate_id,
                       frozen_target_world=frozen_target.tolist(),
                       refined_target_world=refined_target.tolist(),
                       target_world=refined_target.tolist(),
                       candidate_confidence=float(row.get('confidence', 0.0)),
                       precise_confidence=confidence,
                       hover_world=hover.tolist())
        else:
            # V6.7.19: an empty hover is evidence that this spatial group was a
            # false detection. Never silently confirm its frozen search point.
            self.event('STATIC_POSITION_REFINEMENT_UNAVAILABLE',
                       target_name=name, candidate_id=candidate_id,
                       frozen_target_world=frozen_target.tolist(),
                       hover_world=hover.tolist())
            self.event('STATIC_CANDIDATE_REJECTED',
                       target_name=name, candidate_id=candidate_id,
                       frozen_target_world=frozen_target.tolist(),
                       candidate_confidence=float(row.get('confidence', 0.0)),
                       reason='no_valid_target_localization_during_mc_hover',
                       save_original_image=True)

        self.static_candidate_resolved.add(candidate_id)
        transition_point = hover.copy()
        transition_height_above_target = max(
            float(hover[2] - frozen_target[2]),
            float(self.static_cfg.get('retransition_altitude_m', 30.0)),
            float(self.mission_params.get('takeoff_transition_height_m', 10.0)))
        transition_point[2] = frozen_target[2] + transition_height_above_target
        self.fly_mc_static_to(
            transition_point, seq, 'STATIC_RETRANSITION_CLIMB',
            5.0, 1.0, 60.0, yaw_target=target_yaw)
        if self.assignment_sequence != seq:
            return confirmed
        hint = next_hint
        if hint is None:
            try:
                hint = self.assigned_gate(self.assigned_landing())
            except Exception:
                hint = transition_point.copy()
        self.set_phase('STATIC_TRANSITION_FW')
        self.ensure_fw(np.asarray(hint, dtype=float))
        self.event('STATIC_CONFIRMATION_LEG_COMPLETE', target_name=name,
                   candidate_id=candidate_id, candidate_confirmed=confirmed,
                   resolved_candidate_count=len(self.static_candidate_resolved))
        return confirmed

    def _recover_static_verify_exception(
            self, seq: int, name: str, candidate_id: str,
            error: str) -> None:
        with self.lock:
            mode = '' if self.state is None else str(self.state.mode)
        self.event('STATIC_VERIFY_HOLD_GUARD_TRIGGERED',
                   target_name=name, candidate_id=candidate_id,
                   error=error, current_mode=mode,
                   vtol_state=self.vtol_state())
        current = self.current_world()
        if self.vtol_state() == VTOL_FW:
            self._safe_fw_position_hold('static_verify_exception')
            self.recover_offboard_hold('static_verify_exception_fw')
            return
        if current is not None:
            self.set_position_world(current, self.current_yaw())
            self.recover_offboard_hold('static_verify_exception_mc')
            climb = current.copy()
            if self.home_world is not None:
                climb[2] = max(
                    climb[2], self.home_world[2] + float(
                        self.static_cfg.get('retransition_altitude_m', 30.0)))
            self.fly_mc_static_to(
                climb, seq, 'STATIC_RECOVERY_CLIMB',
                6.0, 1.5, 60.0, yaw_target=self.current_yaw())
            self.ensure_fw(climb)

    def execute_static_verify(self, assignment: dict) -> None:
        seq = int(assignment['sequence'])
        rows = [row for row in assignment.get('static_targets', [])
                if self._static_candidate_key(row)
                not in self.static_candidate_resolved]
        targets = self.order_static_targets(rows)
        attempts = {self._static_candidate_key(row): 0 for row in targets}
        queue = list(targets)
        max_attempts = max(
            1, int(self.static_cfg.get('per_target_max_attempts', 2)))
        failed_candidates = []
        while (queue and self.assignment_sequence == seq
               and not rospy.is_shutdown()):
            row = queue.pop(0)
            name = str(row.get('target_name', ''))
            candidate_id = self._static_candidate_key(row)
            if not name or candidate_id in self.static_candidate_resolved:
                continue
            attempts[candidate_id] += 1
            self.static_verify_attempt = attempts[candidate_id]
            next_hint = None
            if queue:
                nxt = queue[0]
                next_hint = np.asarray(
                    nxt.get('target_world',
                            [*nxt['ground_xy'], 0.2]), dtype=float)
                next_hint[2] = self.home_world[2] + float(
                    self.mission_params['search_altitude_m'])
            try:
                self.verify_one_static_target(row, seq, next_hint)
            except RuntimeError as exc:
                self.event('STATIC_VERIFY_TARGET_RETRY',
                           target_name=name, candidate_id=candidate_id,
                           attempt=attempts[candidate_id],
                           max_attempts=max_attempts, error=str(exc))
                try:
                    self._recover_static_verify_exception(
                        seq, name, candidate_id, str(exc))
                except Exception as recovery_exc:
                    self.event('STATIC_VERIFY_RECOVERY_WARNING',
                               target_name=name, candidate_id=candidate_id,
                               error=str(recovery_exc))
                if (attempts[candidate_id] < max_attempts
                        and self.assignment_sequence == seq):
                    queue.append(row)
                else:
                    failed_candidates.append(candidate_id)

        self.static_verify_target = ''
        self.static_verify_candidate_id = ''
        self.static_verify_attempt = 0
        if self.assignment_sequence != seq:
            return
        unresolved = [self._static_candidate_key(row) for row in targets
                      if self._static_candidate_key(row)
                      not in self.static_candidate_resolved]
        if unresolved or failed_candidates:
            self.event('STATIC_VERIFY_INCOMPLETE',
                       failed_candidates=sorted(set(
                           unresolved + failed_candidates)),
                       resolved_candidates=sorted(
                           self.static_candidate_resolved),
                       confirmed_targets=sorted(
                           self.static_precise_candidates))
            self.set_phase('STATIC_VERIFY_WAIT_REISSUE')
            return

        # Finalize exactly one precise position per class after every spatial
        # candidate in this assignment has been checked.
        final_rows = []
        assigned_names = sorted(set(
            str(row.get('target_name', '')) for row in targets))
        for name in assigned_names:
            candidates = self.static_precise_candidates.get(name, [])
            if not candidates:
                continue
            best = max(candidates, key=lambda x: float(x.get('confidence', 0.0)))
            self.static_confirmed.add(name)
            final_rows.append(best)
            self.event('STATIC_CONFIRMED_FINAL',
                       target_name=name,
                       candidate_id=best['candidate_id'],
                       target_world=best['target_world'],
                       confidence=float(best.get('confidence', 0.0)),
                       evaluated_candidate_count=len(candidates))
        self.result['static_confirmed'] = sorted(self.static_confirmed)
        missing_after_hover = [
            name for name in assigned_names
            if not self.static_precise_candidates.get(name)]
        if missing_after_hover:
            self.event(
                'STATIC_VERIFY_INCOMPLETE',
                failed_candidates=[],
                missing_static_targets=missing_after_hover,
                reason='all_candidate_groups_rejected_after_empty_mc_hover',
                resolved_candidates=sorted(
                    self.static_candidate_resolved),
                confirmed_targets=sorted(self.static_confirmed),
                final_static_targets=final_rows)
            self.set_phase('STATIC_VERIFY_WAIT_REISSUE')
            return
        self.event('STATIC_VERIFY_COMPLETE',
                   confirmed_targets=sorted(self.static_confirmed),
                   final_static_targets=final_rows,
                   resolved_candidate_ids=sorted(
                       self.static_candidate_resolved),
                   confirmed_count=len(self.static_confirmed),
                   vehicle_id=self.id)
        self.set_phase('STATIC_VERIFY_COMPLETE_FW')
        while self.assignment_sequence == seq and not rospy.is_shutdown():
            self.check_abort()
            rospy.sleep(0.1)

    # ---------------- return ----------------
    def assigned_landing(self) -> np.ndarray:
        assert self.home_world is not None
        p = self.home_world.copy(); off = self.return_cfg['landing_offset_xy_m']; p[0] += float(off[0]); p[1] += float(off[1]); return p

    def assigned_gate(self, landing: np.ndarray) -> np.ndarray:
        p = landing.copy(); off = self.return_cfg['return_gate_offset_xy_m']; p[0] += float(off[0]); p[1] += float(off[1]); p[2] = self.home_world[2] + float(self.mission_params['search_altitude_m']); return p

    def fly_mc_velocity_to(self, desired: np.ndarray, seq: int, phase: str,
                           xy_tol: float, z_tol: float, timeout: float) -> None:
        """Velocity-guided MC transit using the interface already proven by tracking."""
        self.set_phase(phase)
        rate = max(5.0, float(self.control['control_rate_hz']))
        kp_xy = float(self.return_cfg.get('multicopter_return_kp_xy', 0.35))
        kp_z = float(self.return_cfg.get('multicopter_return_kp_z', 0.8))
        max_xy = float(self.return_cfg.get('multicopter_return_max_xy_speed_mps', 8.0))
        max_z = float(self.return_cfg.get('multicopter_return_max_z_speed_mps', 2.5))
        max_acc = float(self.return_cfg.get('multicopter_return_acceleration_mps2', 3.0))
        progress_timeout = float(self.return_cfg.get('multicopter_progress_timeout_seconds', 25.0))
        progress_min = float(self.return_cfg.get('multicopter_progress_minimum_m', 5.0))
        deadline = time.monotonic() + timeout
        filtered = np.asarray(self.last_velocity_world, dtype=float).copy()
        previous = time.monotonic()
        best_distance = float('inf'); last_progress = time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            if self.assignment_sequence != seq:
                return
            self.ensure_offboard()
            current = self.current_world()
            if current is None:
                rospy.sleep(0.05); continue
            error = np.asarray(desired, dtype=float) - current
            distance = norm_xy(error)
            if distance <= xy_tol and abs(float(error[2])) <= z_tol:
                self.set_velocity_world(np.zeros(3))
                self.last_velocity_world[:] = 0.0
                return
            if distance < best_distance - progress_min:
                best_distance = distance; last_progress = time.monotonic()
            elif time.monotonic() - last_progress > progress_timeout:
                self.event('RETURN_MC_PROGRESS_STALL', phase=phase, distance=distance, desired=desired.tolist())
                # Reset the velocity integrator/transport mode without switching
                # to the unreliable long-distance pose-control return.
                filtered[:] = 0.0
                self.set_velocity_world(filtered)
                last_progress = time.monotonic(); best_distance = distance
            cmd = np.zeros(3)
            cmd[:2] = kp_xy * error[:2]
            cmd = limit_vector_xy(cmd, max_xy)
            cmd[2] = float(np.clip(kp_z * error[2], -max_z, max_z))
            now = time.monotonic(); dt = max(0.01, min(0.2, now - previous)); previous = now
            delta = cmd - filtered
            dn = float(np.linalg.norm(delta[:2])); allowed = max_acc * dt
            if dn > allowed > 0.0:
                delta[:2] *= allowed / dn
            delta[2] = float(np.clip(delta[2], -allowed, allowed))
            filtered += delta
            self.set_velocity_world(filtered)
            self.last_velocity_world = filtered.copy()
            rospy.sleep(1.0 / rate)
        self.set_velocity_world(np.zeros(3))
        raise RuntimeError(f'velocity return timeout phase={phase} desired={desired.tolist()}')

    def follow_mc_return_path(self, target_xy: np.ndarray, seq: int) -> None:
        """Return in MC using velocity-controlled bounded segments."""
        current = self.current_world()
        if current is None or self.home_world is None:
            raise RuntimeError('world/home unavailable for MC return')
        altitude = self.home_world[2] + float(self.return_cfg.get('multicopter_return_altitude_m', self.mission_params['track_altitude_m']))
        segment_length = max(40.0, float(self.return_cfg.get('multicopter_segment_length_m', 140.0)))
        base_timeout = float(self.return_cfg.get('multicopter_segment_timeout_seconds', 120.0))
        cruise_start = current.copy(); cruise_start[2] = altitude
        self.fly_mc_velocity_to(cruise_start, seq, 'RETURN_MC_CLIMB', 5.0, 2.0, max(60.0, base_timeout))
        start = self.current_world()
        if start is None:
            raise RuntimeError('world unavailable after MC climb')
        delta = np.asarray(target_xy[:2], dtype=float) - start[:2]
        distance = float(np.linalg.norm(delta))
        count = max(1, int(math.ceil(distance / segment_length)))
        for index in range(1, count + 1):
            fraction = float(index) / float(count)
            point = start.copy()
            point[:2] = start[:2] + fraction * delta
            point[2] = altitude
            segment_origin = self.current_world()
            if segment_origin is None: segment_origin = start
            segment_distance = float(np.linalg.norm(point[:2] - segment_origin[:2]))
            timeout = max(base_timeout, segment_distance / 2.0 + 45.0)
            self.fly_mc_velocity_to(point, seq, 'RETURN_MC_TRANSIT', 8.0, 2.5, timeout)
            self.event('RETURN_MC_SEGMENT_REACHED', index=index, total=count, point=point.tolist())



    def prepare_fixed_wing_return(self, gate: np.ndarray, landing: np.ndarray, seq: int) -> np.ndarray:
        """MC/FW handover matching the proven image-collection return pattern."""
        current=self.current_world()
        if current is None or self.home_world is None: raise RuntimeError('world/home unavailable for FW return')
        altitude=self.home_world[2]+float(self.return_cfg.get('fixed_wing_return_altitude_m',self.mission_params['search_altitude_m']))
        gate=np.asarray(gate,dtype=float).copy(); gate[2]=altitude
        corridor=landing[:2]-gate[:2]
        n=float(np.linalg.norm(corridor))
        if n<1.0:
            corridor=landing[:2]-current[:2]; n=max(float(np.linalg.norm(corridor)),1.0)
        direction=corridor/n
        pass_distance=float(self.return_cfg.get('fixed_wing_pass_through_distance_m',80.0))
        pass_point=np.array([landing[0]+direction[0]*pass_distance,
                             landing[1]+direction[1]*pass_distance,altitude],dtype=float)
        yaw=math.atan2(float(gate[1]-current[1]),float(gate[0]-current[0]))
        self.set_velocity_world(np.zeros(3)); rospy.sleep(0.25)
        self.set_position_world(gate,yaw)
        prestream=float(self.return_cfg.get('fixed_wing_return_prestream_seconds',1.0))
        deadline=time.monotonic()+max(0.3,prestream)
        while time.monotonic()<deadline and not rospy.is_shutdown():
            self.check_abort(); self.ensure_offboard(); rospy.sleep(0.05)
        if self.assignment_sequence!=seq: raise RuntimeError('FW return superseded during prestream')
        self.transition(VTOL_FW)
        self.event('RETURN_FW_TRANSITION_COMPLETE',gate=gate.tolist(),pass_point=pass_point.tolist())
        return pass_point

    def follow_fixed_wing_return(self, gate: np.ndarray, landing: np.ndarray, seq: int) -> None:
        pass_point=self.prepare_fixed_wing_return(gate,landing,seq)
        synthetic={'point':np.asarray(gate,dtype=float).tolist(),'segment_type':'RETURN',
                   'detection_valid':False,'leg_id':'return_fw_gate'}
        previous=self.current_world()
        if previous is None: raise RuntimeError('world unavailable before FW gate')
        if not self.follow_fw_waypoint(synthetic,previous,seq):
            raise RuntimeError('FW return interrupted before gate')
        # Keep a valid forward pass-through setpoint active during back transition,
        # rather than commanding a hover point to a fixed-wing aircraft.
        current=self.current_world()
        yaw=self.current_yaw() if current is None else math.atan2(float(pass_point[1]-current[1]),float(pass_point[0]-current[0]))
        self.set_position_world(pass_point,yaw)
        rospy.sleep(float(self.return_cfg.get('fixed_wing_pass_setpoint_hold_seconds',0.35)))
        self.event('RETURN_FW_GATE_REACHED',gate=np.asarray(gate,dtype=float).tolist(),pass_point=pass_point.tolist())
        self.transition(VTOL_MC)
        self.event('RETURN_FW_BACK_TRANSITION_COMPLETE')


    def execute_return(self, assignment: dict) -> None:
        seq=int(assignment['sequence']); source_phase=str(self.phase); source_vtol=self.vtol_state()
        self.reset_detection_gate(); self.set_phase('RETURN')
        landing=self.assigned_landing(); gate=self.assigned_gate(landing); current=self.current_world()
        self.current_route_id=f'return_v{self.id}'; self.route_waypoint_index=0; self.route_waypoint_total=1
        self.event('RETURN_START',gate=gate.tolist(),landing=landing.tolist(),
                   source_phase=source_phase,source_vtol_state=source_vtol)
        if current is None: raise RuntimeError('world unavailable')
        distance=norm_xy(gate-current)
        tracking_source=(source_phase.startswith('TRACK_') or source_phase.startswith('STATIC_') or source_phase=='VERIFY_STATIC')
        force_fw=bool(self.return_cfg.get('force_fixed_wing_after_tracking',True)) and tracking_source
        minimum=float(self.return_cfg.get('fixed_wing_return_min_distance_m',120.0))
        use_fw=(not bool(self.control['skip_fixed_wing']) and (force_fw or distance>float(self.return_cfg.get('use_fixed_wing_if_distance_above_m',220.0))) and distance>minimum)
        if use_fw:
            reason='force fixed-wing return after dynamic/static MC task' if force_fw else 'long-distance fixed-wing return'
            self.event('RETURN_MODE_SELECTED',mode='FW',reason=reason,distance_m=distance)
            try:
                self.follow_fixed_wing_return(gate,landing,seq)
            except RuntimeError as exc:
                if not bool(self.return_cfg.get('fixed_wing_timeout_fallback_to_multicopter',True)): raise
                self.event('RETURN_FW_FALLBACK_MC',error=str(exc))
                if self.vtol_state()==VTOL_FW: self.transition(VTOL_MC)
                self.follow_mc_return_path(gate,seq)
        else:
            if self.vtol_state()==VTOL_FW: self.transition(VTOL_MC)
            self.event('RETURN_MODE_SELECTED',mode='MC',reason='safety exception: insufficient FW transition distance',distance_m=distance)
            self.follow_mc_return_path(gate,seq)

        if self.vtol_state()==VTOL_FW: self.transition(VTOL_MC)
        hover=landing.copy(); hover[2]=self.home_world[2]+float(self.control['home_hover_height_m'])
        # After the fixed-wing gate and MC back-transition, use the proven pose
        # landing approach. Finite guards and the pass-through handover prevent the
        # QGC "Invalid offboard setpoint" condition seen in V6.4 screenshots.
        self.set_position_world(hover,0.0)
        reached=self.wait_position(hover,0.0,float(self.return_cfg.get('landing_approach_timeout_seconds',240.0)),3.0,1.5,seq)
        if not reached: raise RuntimeError('landing approach interrupted')
        response=self.mode_srv(custom_mode='AUTO.LAND'); self.event('AUTO_LAND_SENT',mode_sent=bool(response.mode_sent))
        deadline=time.monotonic()+float(self.control['landing_timeout_seconds'])
        while time.monotonic()<deadline and not rospy.is_shutdown():
            with self.lock:
                landed=self.ext is not None and self.ext.landed_state==ExtendedState.LANDED_STATE_ON_GROUND
                armed=self.state is not None and self.state.armed
            if landed:
                self.event('LANDED')
                if armed:
                    try:self.arm_srv(False)
                    except Exception:pass
                    disarm_deadline=time.monotonic()+10.0
                    while time.monotonic()<disarm_deadline and self.is_armed() and not rospy.is_shutdown():rospy.sleep(0.1)
                if not self.is_armed():self.event('DISARMED')
                self.result['ok']=not self.is_armed();self.set_phase('DONE');return
            rospy.sleep(0.2)
        raise RuntimeError('landing timeout')

    def emergency_land(self) -> None:
        self.reset_detection_gate(); self.set_phase('EMERGENCY_LAND')
        try: self.vtol_srv(state=VTOL_MC)
        except Exception: pass
        try: self.mode_srv(custom_mode='AUTO.LAND')
        except Exception: pass

    def finish(self) -> None:
        self.result['phase'] = self.phase
        self.result['tracking'][self.tracking_target or 'last'] = {
            'tracking_points': self.tracking_points,
            'tracking_seconds': 0.0 if self.tracking_started_ros is None else rospy.Time.now().to_sec()-self.tracking_started_ros,
        }
        if self.local_run_dir: dump_json(self.local_run_dir / 'vehicle_result.json', self.result)
        try:
            self.result_pub.publish(String(data=json.dumps(self.result, ensure_ascii=False)))
            deadline = time.monotonic() + 60.0
            while not rospy.is_shutdown() and time.monotonic() < deadline and not self.complete_event.is_set():
                self.result_pub.publish(String(data=json.dumps(self.result, ensure_ascii=False)))
                rospy.sleep(1.0)
        except (rospy.ROSInterruptException, rospy.ROSException):
            pass

    def run(self) -> None:
        self.wait_task()
        try:
            self.wait_inputs(); self.calibrate()
            if not self.wait_start():
                return
            if not self.takeoff_10m_then_fw():
                return
            while not rospy.is_shutdown():
                self.check_abort()
                if self.assignment is None:
                    self.assignment_event.wait(0.1); continue
                with self.lock: assignment = dict(self.assignment)
                mode = str(assignment.get('mode', ''))
                if mode.startswith('SEARCH_'): self.execute_search(assignment)
                elif mode == 'TRACK_DYNAMIC': self.execute_track(assignment)
                elif mode == 'VERIFY_STATIC': self.execute_static_verify(assignment)
                elif mode == 'RETURN': self.execute_return(assignment); break
                elif mode == 'HOLD': rospy.sleep(0.1)
                else: rospy.logwarn_throttle(2.0, 'v%d unknown assignment %s', self.id, mode); rospy.sleep(0.1)
            if self.phase != 'DONE': self.result['ok'] = False
        except rospy.ROSInterruptException:
            self.result['error'] = 'ROS shutdown'
            if self.is_armed(): self.emergency_land()
            self.set_phase('FAILED')
        except Exception as exc:
            self.result['error'] = f'{type(exc).__name__}: {exc}'
            rospy.logerr('v%d V6 failed: %s\n%s', self.id, exc, traceback.format_exc())
            if self.is_armed(): self.emergency_land()
            self.set_phase('FAILED')
        finally:
            self.finish()


if __name__ == '__main__':
    VehicleFlightAgent().run()
