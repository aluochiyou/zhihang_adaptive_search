#!/usr/bin/env python3
"""Formal multi-UAV visual target-state estimator for V6.7.19.

Only stable YOLO geolocation reports are consumed.  Gazebo truth is never
subscribed.  The estimator supports reliable cross-aircraft information
injection, stale-track reinitialization and suppression of an SUV detection
that the flight/mission logic has reclassified as ``prius_hybrid_camo``.
"""
from __future__ import annotations

import json
import math
import threading
from typing import Dict, List

import numpy as np
import rospy
from std_msgs.msg import String

from zhihang_adaptive_search_v6.tracking_recovery import DynamicTargetFilter

NS = '/zhihang/search_v6'
PARAM_ROOT = '/zhihang_search_v6'


class VisionTargetStateEstimator:
    def __init__(self) -> None:
        rospy.init_node('vision_target_state_estimator_v6')
        cfg = rospy.get_param(PARAM_ROOT)
        self.dynamic_targets = set(cfg['perception']['dynamic_targets'])
        tcfg = dict(cfg.get('tracking', {}))
        ecfg = dict(tcfg.get('external_injection', {}))
        self.position_tau = float(tcfg.get(
            'position_filter_time_constant_seconds', 0.8))
        self.velocity_tau = float(tcfg.get(
            'velocity_filter_time_constant_seconds', 1.2))
        self.jump_gate = float(tcfg.get('measurement_jump_gate_m', 35.0))
        self.publish_hz = float(tcfg.get('target_update_hz', 5.0))
        self.coast_seconds = float(tcfg.get(
            'visual_estimator_coast_seconds', 3.0))
        self.maximum_prediction = float(tcfg.get(
            'target_timeout_seconds', 5.0))
        self.reinitialize_after = float(ecfg.get(
            'estimator_reinitialize_after_seconds', 4.0))
        self.reliable_confidence = float(ecfg.get(
            'minimum_confidence', 0.65))
        self.reliable_std = float(ecfg.get(
            'maximum_horizontal_std_m', 5.0))
        self.suppression_default_seconds = float(
            tcfg.get('suv_false_positive', {}).get(
                'suppression_seconds', 1800.0))
        self.suppression_default_radius = float(
            tcfg.get('suv_false_positive', {}).get(
                'suppression_radius_m', 20.0))

        self.lock = threading.RLock()
        self.filters: Dict[str, DynamicTargetFilter] = {}
        self.latest_meta: Dict[str, dict] = {}
        self.suppression_regions: List[dict] = []
        self.publisher = rospy.Publisher(
            f'{NS}/tracking/target_state', String, queue_size=100)
        rospy.Subscriber(
            f'{NS}/tracking/reclassification', String,
            self.reclassification_cb, queue_size=20)
        for vehicle_id in cfg['mission']['enabled_vehicle_ids']:
            rospy.Subscriber(
                f'{NS}/vehicle_{int(vehicle_id)}/target_localization_report',
                String, lambda msg, v=int(vehicle_id): self.report_cb(msg, v),
                queue_size=100)
        rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_hz, 1e-3)), self.tick)
        rospy.logwarn(
            'V6.7.19 formal visual target-state estimator active; '
            'Gazebo truth is not subscribed')

    def reclassification_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            source_name = str(row.get('source_target', ''))
            target_name = str(row.get('reclassified_target', ''))
            position = np.asarray(row.get('position', []), dtype=float)
            if (source_name != 'suv_camo'
                    or target_name != 'prius_hybrid_camo'
                    or position.size < 3):
                return
            now = rospy.Time.now().to_sec()
            region = {
                'position': position[:3].copy(),
                'radius_m': float(row.get(
                    'suppression_radius_m',
                    self.suppression_default_radius)),
                'expires_ros_time': float(row.get(
                    'expires_ros_time',
                    now + self.suppression_default_seconds)),
            }
            with self.lock:
                self.suppression_regions.append(region)
                self.filters.pop('suv_camo', None)
                self.latest_meta.pop('suv_camo', None)
            rospy.logwarn(
                'V6.7.19 estimator suppressed false suv_camo at '
                '(%.1f, %.1f), radius=%.1fm',
                position[0], position[1], region['radius_m'])
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, 'invalid target reclassification: %s', exc)

    def suppressed_suv(self, position: np.ndarray, now: float) -> bool:
        active = []
        suppressed = False
        for region in self.suppression_regions:
            if now > float(region['expires_ros_time']):
                continue
            active.append(region)
            distance = float(np.linalg.norm(
                position[:2] - np.asarray(region['position'])[:2]))
            if distance <= float(region['radius_m']):
                suppressed = True
        self.suppression_regions = active
        return suppressed

    def report_cb(self, msg: String, vehicle_id: int) -> None:
        try:
            row = json.loads(msg.data)
            name = str(row.get('target_name', ''))
            if name not in self.dynamic_targets:
                return
            position = np.asarray(row.get('position_world', []), dtype=float)
            if position.size < 3 or not np.all(np.isfinite(position[:3])):
                return
            stamp = float(row.get(
                'source_ros_time',
                row.get('ros_time', rospy.Time.now().to_sec())))
            velocity = np.asarray(
                row.get('velocity', [0.0, 0.0, 0.0]), dtype=float)
            if velocity.size < 3 or not np.all(np.isfinite(velocity[:3])):
                velocity = np.zeros(3)
            confidence = float(row.get('confidence', 0.0))
            horizontal_std = float(row.get('horizontal_std_m', 0.0))
            now = rospy.Time.now().to_sec()
            with self.lock:
                if name == 'suv_camo' and self.suppressed_suv(position, now):
                    rospy.logwarn_throttle(
                        2.0,
                        'V6.7.19 ignored suv_camo inside reclassified '
                        'prius_hybrid_camo region')
                    return
                filt = self.filters.setdefault(
                    name, DynamicTargetFilter(
                        self.position_tau, self.velocity_tau, self.jump_gate))
                stale_gap = (
                    filt.stamp is not None
                    and stamp - float(filt.stamp) >
                    self.reinitialize_after)
                reinitialized = False
                if stale_gap:
                    filt.reset(position[:3], velocity[:3], stamp)
                    filtered_p = position[:3].copy()
                    filtered_v = velocity[:3].copy()
                    accepted = True
                    reinitialized = True
                    rospy.logwarn(
                        'V6.7.19 EXTERNAL_TARGET_STATE_REINITIALIZED '
                        'target=%s vehicle=%d gap=%.2fs',
                        name, vehicle_id,
                        stamp - float(self.latest_meta.get(
                            name, {}).get('measurement_stamp', stamp)))
                else:
                    filtered_p, filtered_v, accepted = filt.update(
                        position[:3], velocity[:3], stamp)
                if not accepted:
                    rospy.logwarn_throttle(
                        1.0,
                        'visual target-state rejected jump target=%s '
                        'vehicle=%d', name, vehicle_id)
                    return
                reliable = (
                    confidence >= self.reliable_confidence
                    and horizontal_std <= self.reliable_std)
                measurement_id = str(row.get(
                    'report_event_id',
                    f'{stamp:.6f}:v{int(vehicle_id)}'))
                self.latest_meta[name] = {
                    'mission_id': str(row.get('mission_id', '')),
                    'vehicle_id': int(vehicle_id),
                    'confidence': confidence,
                    'horizontal_std_m': horizontal_std,
                    'measurement_stamp': stamp,
                    'measurement_id': measurement_id,
                    'position': filtered_p.tolist(),
                    'velocity': filtered_v.tolist(),
                    'external_injection_reliable': bool(reliable),
                    'reinitialized': bool(reinitialized),
                }
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, 'visual target-state report parse failed: %s', exc)

    def tick(self, _event=None) -> None:
        now = rospy.Time.now().to_sec()
        rows = []
        with self.lock:
            for name, filt in self.filters.items():
                if filt.stamp is None:
                    continue
                age = max(0.0, now - float(filt.stamp))
                if age > self.coast_seconds:
                    continue
                position, velocity = filt.predict(
                    now, self.maximum_prediction)
                meta = dict(self.latest_meta.get(name, {}))
                rows.append({
                    'schema_version': 2,
                    'mission_id': meta.get('mission_id', ''),
                    'target_name': name,
                    'position': position.tolist(),
                    'velocity': velocity.tolist(),
                    'source_ros_time': float(filt.stamp),
                    'ros_time': now,
                    'source': 'formal_yolo_multiuav_filtered_state',
                    'source_vehicle_id': meta.get('vehicle_id'),
                    'confidence': meta.get('confidence', 0.0),
                    'horizontal_std_m': meta.get(
                        'horizontal_std_m', 0.0),
                    'measurement_age_seconds': age,
                    'measurement_id': meta.get('measurement_id', ''),
                    'external_injection_reliable': bool(
                        meta.get('external_injection_reliable', False)),
                    'track_reinitialized': bool(
                        meta.get('reinitialized', False)),
                    'truth_used': False,
                })
        for row in rows:
            self.publisher.publish(
                String(data=json.dumps(row, ensure_ascii=False)))


if __name__ == '__main__':
    VisionTargetStateEstimator()
    rospy.spin()
