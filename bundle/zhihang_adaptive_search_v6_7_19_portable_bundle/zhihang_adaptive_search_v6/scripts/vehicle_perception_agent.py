#!/usr/bin/env python3
from __future__ import annotations

import collections
import json
import math
import os
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Sequence

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from zhihang_adaptive_search_v6.common import dump_json, fov_project_nadir, validate_packet
from zhihang_adaptive_search_v6.target_localization import (
    LocalizationEventValidator,
    TargetLocalizationTracker,
)

NS = '/zhihang/search_v6'
PARAM_ROOT = '/zhihang_search_v6'


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        block = sock.recv(size - len(data))
        if not block:
            raise ConnectionError('YOLO worker closed connection')
        data.extend(block)
    return bytes(data)


def send_packet(sock: socket.socket, header: dict, image_bytes: bytes) -> dict:
    raw = json.dumps(header, separators=(',', ':')).encode('utf-8')
    sock.sendall(struct.pack('!I', len(raw)) + raw)
    sock.sendall(struct.pack('!I', len(image_bytes)) + image_bytes)
    size = struct.unpack('!I', recv_exact(sock, 4))[0]
    if size > 20_000_000:
        raise ValueError(f'invalid YOLO response size: {size}')
    return json.loads(recv_exact(sock, size).decode('utf-8'))


@dataclass
class LatestFrame:
    lock: threading.RLock = field(default_factory=threading.RLock)
    sequence: int = 0
    consumed: int = 0
    ros_stamp: float = 0.0
    wall_stamp: float = 0.0
    image: object = None
    replaced: int = 0
    stale: int = 0
    processed_total: int = 0
    source_times: collections.deque = field(default_factory=lambda: collections.deque(maxlen=3000))
    processed_times: collections.deque = field(default_factory=lambda: collections.deque(maxlen=3000))

    def update(self, stamp: float, image) -> None:
        now = time.monotonic()
        with self.lock:
            if self.sequence > self.consumed:
                self.replaced += 1
            self.sequence += 1
            self.ros_stamp = stamp
            self.wall_stamp = now
            self.image = image
            self.source_times.append(now)

    def newest(self):
        with self.lock:
            if self.image is None or self.sequence == self.consumed:
                return None
            return self.sequence, self.ros_stamp, self.wall_stamp, self.image.copy()

    def snapshot(self):
        """Return the newest raw frame even if the YOLO sender consumed it."""
        with self.lock:
            if self.image is None:
                return None
            return self.sequence, self.ros_stamp, self.wall_stamp, self.image.copy()

    def mark_processed(self, sequence: int) -> None:
        with self.lock:
            self.consumed = max(self.consumed, sequence)
            self.processed_total += 1
            self.processed_times.append(time.monotonic())

    def mark_stale(self, sequence: int) -> None:
        with self.lock:
            self.consumed = max(self.consumed, sequence)
            self.stale += 1

    def fps(self, source: bool, window: float) -> float:
        now = time.monotonic()
        with self.lock:
            values = list(self.source_times if source else self.processed_times)
        values = [v for v in values if now - v <= window]
        if len(values) < 2:
            return 0.0
        return (len(values) - 1) / max(values[-1] - values[0], 1e-6)

    def reset_metrics(self) -> None:
        with self.lock:
            self.consumed = self.sequence
            self.replaced = 0
            self.stale = 0
            self.processed_total = 0
            self.source_times.clear()
            self.processed_times.clear()


class YoloDetectionDisplay:
    """Always-on, independent OpenCV window for one aircraft.

    V6.7.6 created and destroyed windows according to whether a target was
    present.  V6.7.12 keeps all three windows open after the first camera frame,
    displays a clear ``NO VALID TASK TARGET`` state when empty, and overlays
    task-target boxes when detections are available.  GUI errors remain isolated
    from inference and flight control.
    """

    def __init__(self, vehicle_id: int, cfg: dict, target_names: Sequence[str],
                 kind_map: dict) -> None:
        self.vehicle_id = int(vehicle_id)
        self.cfg = dict(cfg or {})
        self.target_names = set(map(str, target_names))
        self.kind_map = {str(k): str(v) for k, v in (kind_map or {}).items()}
        self.enabled = bool(self.cfg.get('enabled', True))
        self.minimum_confidence = float(self.cfg.get('minimum_confidence', 0.25))
        self.always_open = bool(self.cfg.get('always_open', True))
        self.close_when_empty = bool(self.cfg.get('close_when_no_valid_target', False))
        self.window_name = f"{self.cfg.get('window_title_prefix', 'YOLO target')} - UAV{self.vehicle_id}"
        self.width = int(self.cfg.get('window_width', 620))
        self.height = int(self.cfg.get('window_height', 350))
        self.auto_tile = bool(self.cfg.get('auto_tile', True))
        self.tile_columns = max(1, int(self.cfg.get('tile_columns', 3)))
        self.window_gap = int(self.cfg.get('window_gap_px', 8))
        self.origin_x = int(self.cfg.get('window_origin_x', 0))
        self.origin_y = int(self.cfg.get('window_origin_y', 0))
        self.lock = threading.RLock()
        self.pending = None
        self.window_open = False
        self.running = True
        if self.enabled and not os.environ.get('DISPLAY'):
            rospy.logwarn('v%d YOLO display disabled: DISPLAY is not set', self.vehicle_id)
            self.enabled = False
        self.thread = None
        if self.enabled:
            self.thread = threading.Thread(target=self._loop,
                                           name=f'yolo_display_v{self.vehicle_id}', daemon=True)
            self.thread.start()

    def _valid(self, detections: Sequence[dict]) -> list:
        rows = []
        for det in detections or []:
            name = str(det.get('class_name', ''))
            score = float(det.get('confidence', 0.0))
            if name in self.target_names and score >= self.minimum_confidence:
                rows.append(det)
        return rows

    def annotate(self, image, detections: Sequence[dict], headline: Optional[str] = None):
        annotated = image.copy()
        valid = self._valid(detections)
        for det in valid:
            box = det.get('xyxy') or []
            if len(box) < 4:
                continue
            x1, y1, x2, y2 = [int(round(float(v))) for v in box[:4]]
            name = str(det.get('class_name', 'target'))
            score = float(det.get('confidence', 0.0))
            kind = self.kind_map.get(name, '')
            colour = (0, 220, 0) if kind == 'static' else (0, 140, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 2)
            label = f'{name} {score:.2f} {kind}'
            cv2.putText(annotated, label, (x1, max(24, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2,
                        cv2.LINE_AA)
        state = headline or (f'VALID TARGETS: {len(valid)}' if valid else 'NO VALID TASK TARGET')
        state_colour = (0, 220, 0) if valid else (80, 80, 255)
        cv2.rectangle(annotated, (0, 0), (min(annotated.shape[1], 610), 42), (20, 20, 20), -1)
        cv2.putText(annotated, f'UAV{self.vehicle_id} | {state}',
                    (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.72,
                    state_colour, 2, cv2.LINE_AA)
        return annotated, valid

    def update(self, image, detections: Sequence[dict]) -> None:
        if not self.enabled:
            return
        annotated, valid = self.annotate(image, detections)
        with self.lock:
            self.pending = (annotated, bool(valid))

    def _open_window(self) -> None:
        if self.window_open:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
        if self.auto_tile:
            tile_x = self.vehicle_id % self.tile_columns
            tile_y = self.vehicle_id // self.tile_columns
            cv2.moveWindow(
                self.window_name,
                self.origin_x + tile_x * (self.width + self.window_gap),
                self.origin_y + tile_y * (self.height + self.window_gap))
        self.window_open = True

    def _loop(self) -> None:
        while self.running and not rospy.is_shutdown():
            item = None
            with self.lock:
                if self.pending is not None:
                    item = self.pending
                    self.pending = None
            if item is None:
                time.sleep(0.02)
                continue
            image, valid = item
            try:
                if image is not None and (self.always_open or valid):
                    self._open_window()
                    cv2.imshow(self.window_name, image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), 27):
                        # A manually closed window is recreated by the next frame
                        # because the mission requirement is to keep all three open.
                        cv2.destroyWindow(self.window_name)
                        self.window_open = False
                elif self.close_when_empty and self.window_open:
                    cv2.destroyWindow(self.window_name)
                    cv2.waitKey(1)
                    self.window_open = False
            except Exception as exc:
                rospy.logwarn('v%d YOLO display disabled after GUI error: %s',
                              self.vehicle_id, exc)
                self.enabled = False
                self.running = False
        if self.window_open:
            try:
                cv2.destroyWindow(self.window_name)
                cv2.waitKey(1)
            except Exception:
                pass
            self.window_open = False

    def close(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)


class DetectionEventValidator:
    """Validation-only, event-level comparison of YOLO against expected FOV truth.

    This evaluator is deliberately isolated inside the perception process.  It
    may read Gazebo model states in validation mode, but its output is used only
    for statistics and image export; it never changes assignments, routes,
    tracking commands or flight control.
    """

    def __init__(self, vehicle_id: int, mission_id: str, cfg: dict,
                 target_names: Sequence[str], kind_map: dict,
                 event_pub, summary_pub, output_dir: Path) -> None:
        self.vehicle_id = int(vehicle_id)
        self.mission_id = str(mission_id)
        self.cfg = dict(cfg or {})
        self.enabled = bool(self.cfg.get('enabled', False))
        self.target_names = [str(x) for x in target_names]
        self.kind_map = {str(k): str(v) for k, v in (kind_map or {}).items()}
        self.aliases = {str(k): str(v) for k, v in self.cfg.get('class_aliases', {}).items()}
        self.event_pub = event_pub
        self.summary_pub = summary_pub
        self.output_dir = Path(output_dir)
        self.event_dir = self.output_dir / 'detection_validation_events'
        self.event_dir.mkdir(parents=True, exist_ok=True)
        self.high_conf = float(self.cfg.get('high_confidence_threshold', 0.60))
        self.min_visibility_seconds = float(self.cfg.get('minimum_expected_visibility_seconds', 0.8))
        self.min_expected_frames = int(self.cfg.get('minimum_expected_frames', 6))
        self.correct_min_consecutive = int(self.cfg.get('correct_minimum_consecutive_frames', 3))
        self.correct_min_ratio = float(self.cfg.get('correct_minimum_frame_ratio', 0.25))
        self.false_min_consecutive = int(self.cfg.get('false_positive_minimum_consecutive_frames', 3))
        self.false_min_seconds = float(self.cfg.get('false_positive_minimum_seconds', 0.35))
        self.event_gap_seconds = float(self.cfg.get('event_gap_seconds', 0.55))
        self.platform_radius = float(self.cfg.get('platform_exclusion_radius_m', 120.0))
        self.minimum_altitude = float(self.cfg.get('minimum_evaluation_altitude_m', 15.0))
        self.fov_margin = float(self.cfg.get('expected_fov_margin_ratio', 0.88))
        self.jpeg_quality = int(self.cfg.get('saved_jpeg_quality', 92))
        self.save_correct = bool(self.cfg.get('save_correct_event_image', False))
        self.lock = threading.RLock()
        self.active_expected: Dict[str, dict] = {}
        self.active_false: Dict[str, dict] = {}
        self.counts = collections.Counter()
        self.per_target: Dict[str, collections.Counter] = {
            name: collections.Counter() for name in self.target_names
        }
        self.event_index = 0
        self.ignored_short = 0
        self.finalized = False

    def _canonical(self, name: str) -> str:
        return self.aliases.get(str(name), str(name))

    def _annotated_image(self, image, detections: Sequence[dict], expected=None,
                         title: str = ''):
        out = image.copy()
        for det in detections or []:
            name = self._canonical(str(det.get('class_name', '')))
            if name not in self.target_names and name not in ('static_target', 'dynamic_target'):
                continue
            box = det.get('xyxy') or []
            if len(box) >= 4:
                x1, y1, x2, y2 = [int(round(float(v))) for v in box[:4]]
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 180, 255), 2)
                cv2.putText(out, f'{name} {float(det.get("confidence", 0.0)):.2f}',
                            (x1, max(22, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.58, (0, 180, 255), 2, cv2.LINE_AA)
        if expected:
            u, v = int(round(float(expected.get('u', 0.0)))), int(round(float(expected.get('v', 0.0))))
            cv2.drawMarker(out, (u, v), (255, 0, 255), cv2.MARKER_CROSS, 24, 3)
            cv2.putText(out, f'EXPECTED {expected.get("target_name", "")}',
                        (max(5, u - 120), max(25, v - 18)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.rectangle(out, (0, 0), (min(out.shape[1], 820), 42), (10, 10, 10), -1)
        cv2.putText(out, title, (10, 29), cv2.FONT_HERSHEY_SIMPLEX,
                    0.68, (255, 255, 255), 2, cv2.LINE_AA)
        return out

    def _save_image(self, event_type: str, target_name: str, event: dict) -> Optional[str]:
        image = event.get('best_image')
        if image is None:
            return None
        self.event_index += 1
        start = float(event.get('start_ros', 0.0))
        confidence = float(event.get('max_confidence', 0.0))
        safe_target = ''.join(ch if ch.isalnum() or ch in ('_', '-') else '_' for ch in target_name)
        name = (f'{event_type.upper()}_v{self.vehicle_id}_{safe_target}_'
                f'{self.event_index:04d}_ros{start:.3f}_conf{confidence:.2f}.jpg')
        path = self.event_dir / name
        expected = event.get('best_expected')
        title = f'{event_type.upper()} | UAV{self.vehicle_id} | {target_name}'
        annotated = self._annotated_image(
            image, event.get('best_detections', []), expected=expected, title=title)
        ok = cv2.imwrite(str(path), annotated,
                         [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return str(path) if ok else None

    def _allowed(self, flight: dict, vehicle: np.ndarray,
                 home_world: Optional[np.ndarray], ground_z: float) -> tuple:
        phase = str(flight.get('phase', ''))
        if phase.startswith('SEARCH_'):
            if not bool(flight.get('detection_valid', False)):
                return False, 'search_not_on_valid_straight_segment'
        elif not (phase.startswith('TRACK_') or phase == 'STATIC_HOVER_VERIFY'):
            return False, f'excluded_phase:{phase}'
        if float(vehicle[2]) - float(ground_z) < self.minimum_altitude:
            return False, 'below_minimum_evaluation_altitude'
        if home_world is not None:
            if float(np.linalg.norm((vehicle - home_world)[:2])) <= self.platform_radius:
                return False, 'inside_takeoff_landing_platform_exclusion'
        return True, ''

    def _task_detections(self, detections: Sequence[dict]) -> list:
        rows = []
        for raw in detections or []:
            name = self._canonical(str(raw.get('class_name', '')))
            kind = self.kind_map.get(name, self.kind_map.get(str(raw.get('class_name', '')), ''))
            if name in self.target_names or kind in ('static', 'dynamic'):
                row = dict(raw)
                row['canonical_name'] = name
                row['kind'] = kind
                rows.append(row)
        return rows

    def _match_confidence(self, target_name: str, expected: Dict[str, dict], detections: list) -> float:
        direct = [float(d.get('confidence', 0.0)) for d in detections
                  if d.get('canonical_name') == target_name]
        if direct:
            return max(direct)
        kind = self.kind_map.get(target_name, '')
        generic = 'static_target' if kind == 'static' else 'dynamic_target'
        same_kind_expected = [name for name in expected if self.kind_map.get(name, '') == kind]
        if len(same_kind_expected) == 1:
            rows = [float(d.get('confidence', 0.0)) for d in detections
                    if d.get('canonical_name') == generic]
            if rows:
                return max(rows)
        return 0.0

    def _new_expected_event(self, name: str, kind: str, now: float) -> dict:
        return {
            'target_name': name, 'kind': kind, 'start_ros': now,
            'last_expected_ros': now, 'expected_frames': 0,
            'high_confidence_match_frames': 0, 'low_confidence_match_frames': 0,
            'current_consecutive': 0, 'maximum_consecutive': 0,
            'max_confidence': 0.0, 'best_image': None,
            'best_expected': None, 'best_detections': [],
            'best_centre_distance_px': float('inf'),
        }

    def _new_false_event(self, name: str, kind: str, now: float) -> dict:
        return {
            'target_name': name, 'kind': kind, 'start_ros': now,
            'last_detection_ros': now, 'frames': 0,
            'current_consecutive': 0, 'maximum_consecutive': 0,
            'max_confidence': 0.0, 'best_image': None,
            'best_expected': None, 'best_detections': [],
        }

    def _publish_event(self, row: dict) -> None:
        row = {'mission_id': self.mission_id, 'vehicle_id': self.vehicle_id, **row}
        self.event_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))
        with open(self.output_dir / 'detection_validation_events.jsonl', 'a', encoding='utf-8') as fp:
            fp.write(json.dumps(row, ensure_ascii=False) + '\n')
        self._publish_summary()

    def _publish_summary(self) -> None:
        row = self.summary()
        self.summary_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))
        dump_json(self.output_dir / 'detection_validation_summary.json', row)

    def _finalize_expected(self, name: str, reason: str) -> None:
        event = self.active_expected.pop(name, None)
        if not event:
            return
        duration = max(0.0, float(event['last_expected_ros']) - float(event['start_ros']))
        frames = int(event['expected_frames'])
        if duration < self.min_visibility_seconds or frames < self.min_expected_frames:
            self.ignored_short += 1
            return
        high = int(event['high_confidence_match_frames'])
        ratio = float(high) / max(1, frames)
        correct = (int(event['maximum_consecutive']) >= self.correct_min_consecutive and
                   ratio >= self.correct_min_ratio)
        event_type = 'correct' if correct else 'miss'
        self.counts[event_type] += 1
        self.per_target.setdefault(name, collections.Counter())[event_type] += 1
        image_path = None
        if event_type == 'miss' or self.save_correct:
            image_path = self._save_image(event_type, name, event)
        self._publish_event({
            'event_type': event_type, 'target_name': name,
            'kind': event['kind'], 'start_ros': event['start_ros'],
            'end_ros': event['last_expected_ros'], 'duration_seconds': duration,
            'expected_frames': frames,
            'high_confidence_match_frames': high,
            'low_confidence_match_frames': int(event['low_confidence_match_frames']),
            'high_confidence_match_ratio': ratio,
            'maximum_consecutive_high_confidence_frames': int(event['maximum_consecutive']),
            'maximum_confidence': float(event['max_confidence']),
            'classification_reason': (
                'reliable target-specific recognition during continuous expected-visibility event'
                if correct else
                'target remained expected in FOV but reliable recognition threshold was not met'
            ),
            'termination_reason': reason,
            'representative_image': image_path,
        })

    def _finalize_false(self, name: str, reason: str) -> None:
        event = self.active_false.pop(name, None)
        if not event:
            return
        duration = max(0.0, float(event['last_detection_ros']) - float(event['start_ros']))
        qualifies = (int(event['maximum_consecutive']) >= self.false_min_consecutive and
                     duration >= self.false_min_seconds)
        if not qualifies:
            self.counts['ignored_false_candidate'] += 1
            return
        self.counts['false_positive'] += 1
        self.per_target.setdefault(name, collections.Counter())['false_positive'] += 1
        image_path = self._save_image('false_positive', name, event)
        self._publish_event({
            'event_type': 'false_positive', 'target_name': name,
            'kind': event['kind'], 'start_ros': event['start_ros'],
            'end_ros': event['last_detection_ros'], 'duration_seconds': duration,
            'detected_frames': int(event['frames']),
            'maximum_consecutive_high_confidence_frames': int(event['maximum_consecutive']),
            'maximum_confidence': float(event['max_confidence']),
            'classification_reason': (
                'no valid task target was expected in the camera FOV, but a task class '
                'was repeatedly reported above the high-confidence threshold'
            ),
            'termination_reason': reason,
            'representative_image': image_path,
        })

    def update(self, image, detections: Sequence[dict], ros_stamp: float,
               sequence: int, flight: dict, positions: Dict[str, np.ndarray],
               camera: dict, home_world: Optional[np.ndarray]) -> None:
        if not self.enabled or self.finalized:
            return
        with self.lock:
            vehicle = positions.get(f'standard_vtol_{self.vehicle_id}')
            if vehicle is None:
                raw = flight.get('world_position')
                if isinstance(raw, list) and len(raw) >= 3:
                    vehicle = np.asarray(raw[:3], dtype=float)
            if vehicle is None:
                return
            now = float(ros_stamp)
            allowed, excluded_reason = self._allowed(
                flight, vehicle, home_world, float(camera.get('ground_z_m', 0.2)))
            if not allowed:
                for name in list(self.active_expected):
                    self._finalize_expected(name, excluded_reason)
                for name in list(self.active_false):
                    self._finalize_false(name, excluded_reason)
                return

            yaw = float(flight.get('world_yaw', 0.0))
            expected: Dict[str, dict] = {}
            for name in self.target_names:
                target = positions.get(name)
                if target is None:
                    continue
                projection = fov_project_nadir(vehicle, yaw, target, camera, self.fov_margin)
                if projection.get('inside', False):
                    expected[name] = {**projection, 'target_name': name,
                                      'kind': self.kind_map.get(name, '')}

            task_detections = self._task_detections(detections)
            image_center = np.asarray([float(camera.get('cx', image.shape[1] / 2.0)),
                                       float(camera.get('cy', image.shape[0] / 2.0))])
            for name, projection in expected.items():
                event = self.active_expected.setdefault(
                    name, self._new_expected_event(name, self.kind_map.get(name, ''), now))
                event['last_expected_ros'] = now
                event['expected_frames'] += 1
                confidence = self._match_confidence(name, expected, task_detections)
                event['max_confidence'] = max(float(event['max_confidence']), confidence)
                if confidence >= self.high_conf:
                    event['high_confidence_match_frames'] += 1
                    event['current_consecutive'] += 1
                    event['maximum_consecutive'] = max(
                        int(event['maximum_consecutive']), int(event['current_consecutive']))
                else:
                    if confidence > 0.0:
                        event['low_confidence_match_frames'] += 1
                    event['current_consecutive'] = 0
                centre_distance = float(np.linalg.norm(
                    np.asarray([projection.get('u', 0.0), projection.get('v', 0.0)]) - image_center))
                if centre_distance < float(event['best_centre_distance_px']):
                    event['best_centre_distance_px'] = centre_distance
                    event['best_image'] = image.copy()
                    event['best_expected'] = dict(projection)
                    event['best_detections'] = [dict(x) for x in task_detections]

            for name, event in list(self.active_expected.items()):
                if name not in expected and now - float(event['last_expected_ros']) > self.event_gap_seconds:
                    self._finalize_expected(name, 'target_left_expected_FOV')

            if expected:
                for name in list(self.active_false):
                    self._finalize_false(name, 'valid_target_entered_expected_FOV')
            else:
                high_rows = [d for d in task_detections
                             if float(d.get('confidence', 0.0)) >= self.high_conf]
                best_by_class = {}
                for det in high_rows:
                    name = str(det.get('canonical_name', ''))
                    if (name not in best_by_class or
                            float(det.get('confidence', 0.0)) >
                            float(best_by_class[name].get('confidence', 0.0))):
                        best_by_class[name] = det
                present = set(best_by_class)
                for name, det in best_by_class.items():
                    event = self.active_false.setdefault(
                        name, self._new_false_event(name, str(det.get('kind', '')), now))
                    event['last_detection_ros'] = now
                    event['frames'] += 1
                    event['current_consecutive'] += 1
                    event['maximum_consecutive'] = max(
                        int(event['maximum_consecutive']), int(event['current_consecutive']))
                    conf = float(det.get('confidence', 0.0))
                    if conf >= float(event['max_confidence']):
                        event['max_confidence'] = conf
                        event['best_image'] = image.copy()
                        event['best_detections'] = [dict(x) for x in task_detections]
                for name, event in list(self.active_false.items()):
                    if name not in present:
                        event['current_consecutive'] = 0
                    if now - float(event['last_detection_ros']) > self.event_gap_seconds:
                        self._finalize_false(name, 'high_confidence_detection_sequence_ended')

    def finalize(self, reason: str = 'mission_complete') -> None:
        if not self.enabled:
            return
        with self.lock:
            if self.finalized:
                return
            for name in list(self.active_expected):
                self._finalize_expected(name, reason)
            for name in list(self.active_false):
                self._finalize_false(name, reason)
            self.finalized = True
            self._publish_summary()

    def summary(self) -> dict:
        with self.lock:
            return {
                'mission_id': self.mission_id, 'vehicle_id': self.vehicle_id,
                'enabled': self.enabled,
                'counts': {
                    'correct': int(self.counts.get('correct', 0)),
                    'miss': int(self.counts.get('miss', 0)),
                    'false_positive': int(self.counts.get('false_positive', 0)),
                    'ignored_false_candidate': int(self.counts.get('ignored_false_candidate', 0)),
                    'ignored_short_visibility': int(self.ignored_short),
                },
                'per_target': {name: dict(rows) for name, rows in self.per_target.items()},
                'active_expected_events': len(self.active_expected),
                'active_false_positive_candidates': len(self.active_false),
                'high_confidence_threshold': self.high_conf,
                'event_directory': str(self.event_dir),
                'event_level_not_frame_level': True,
                'excluded_near_platform_radius_m': self.platform_radius,
            }


class VehiclePerceptionAgent:
    """One independent image/Yolo/FOV process for one aircraft.

    YOLO runs continuously and is evaluated, but the current management result
    is the geometric FOV proxy. No detection topic is subscribed by the flight
    agent, so perception remains outside the flight-control loop.
    """

    def __init__(self) -> None:
        rospy.init_node('vehicle_perception_agent_v6')
        self.id = int(rospy.get_param('~vehicle_id'))
        self.yolo_host = str(rospy.get_param('~yolo_host', '127.0.0.1'))
        self.yolo_port = int(rospy.get_param('~yolo_port'))
        self.local_cfg = rospy.get_param(PARAM_ROOT)
        self.bridge = CvBridge()
        self.slot = LatestFrame()
        self.lock = threading.RLock()
        self.task_event = threading.Event()
        self.complete_event = threading.Event()
        self.task: Optional[dict] = None
        self.mission_id = ''
        self.perception_cfg = {}
        self.camera_topic = ''
        self.camera_info_topic = ''
        self.camera = {}
        self.connected = False
        self.ready = False
        self.last_error = ''
        self.last_worker_fps = 0.0
        self.last_inference_ms = None
        self.last_imgsz = None
        self.consecutive_pass_windows = 0
        self.positions: Dict[str, np.ndarray] = {}
        self.flight_status: dict = {}
        self.detected_static = set()
        self.last_dynamic: Dict[str, float] = {}
        self.mission_active = False
        self.local_run_dir: Optional[Path] = None
        self.image_sub = None
        self.camera_info_sub = None
        self.detection_display: Optional[YoloDetectionDisplay] = None
        self.detection_validator: Optional[LocalizationEventValidator] = None
        self.target_localizer: Optional[TargetLocalizationTracker] = None
        self.localization_report_count = 0
        self.last_localization_report: Optional[dict] = None
        self.validation_home_world: Optional[np.ndarray] = None
        rospy.on_shutdown(self.shutdown_cleanup)

        prefix = f'{NS}/vehicle_{self.id}'
        self.ack_pub = rospy.Publisher(f'{prefix}/task_ack/perception', String, queue_size=2, latch=True)
        self.status_pub = rospy.Publisher(f'{prefix}/perception_status', String, queue_size=10, latch=True)
        self.detection_pub = rospy.Publisher(f'{prefix}/detection_report', String, queue_size=300)
        self.yolo_pub = rospy.Publisher(f'{prefix}/yolo_report', String, queue_size=300)
        self.localization_pub = rospy.Publisher(
            f'{prefix}/target_localization_report', String, queue_size=300)
        self.validation_event_pub = rospy.Publisher(
            f'{prefix}/detection_validation_event', String, queue_size=100)
        self.validation_summary_pub = rospy.Publisher(
            f'{prefix}/detection_validation_summary', String, queue_size=2, latch=True)

        rospy.Subscriber(f'{NS}/manager/task/vehicle_{self.id}', String, self.task_cb, queue_size=1)
        rospy.Subscriber(f'{NS}/manager/start', String, self.start_cb, queue_size=1)
        rospy.Subscriber(f'{NS}/manager/mission_complete', String, self.complete_cb, queue_size=1)
        rospy.Subscriber(f'{prefix}/flight_status', String, self.flight_status_cb, queue_size=20)
        rospy.Subscriber(f'{prefix}/event', String,
                         self.flight_event_cb, queue_size=100)
        self.validation_fov_proxy=bool(self.local_cfg.get('validation',{}).get('fov_proxy_enabled',False))
        validation_cfg = self.local_cfg.get('perception', {}).get('detection_event_validation', {})
        self.validation_event_enabled = bool(validation_cfg.get('enabled', False))
        self.world_sub=None
        if self.validation_fov_proxy or self.validation_event_enabled:
            self.world_sub=rospy.Subscriber('/gazebo/model_states', ModelStates, self.world_cb, queue_size=1)
            rospy.logwarn(
                'v%d VALIDATION ONLY: Gazebo truth enabled for %s', self.id,
                'FOV proxy + event evaluator' if self.validation_fov_proxy and self.validation_event_enabled
                else ('FOV proxy' if self.validation_fov_proxy else 'event evaluator'))
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.status_tick)
        self.proxy_timer = (rospy.Timer(rospy.Duration(1.0 / max(float(self.local_cfg['perception']['proxy_publish_rate_hz']), 1.0)), self.proxy_tick) if self.validation_fov_proxy else None)
        self.sender_thread = threading.Thread(target=self.sender_loop, name=f'perception_sender_v{self.id}', daemon=True)
        self.sender_thread.start()

    def publish_task_ack(self, packet: dict, accepted: bool, reason: str = '') -> None:
        ack = {
            'mission_id': str(packet.get('mission_id', '')), 'vehicle_id': self.id,
            'component': 'perception', 'accepted': bool(accepted),
            'checksum': str(packet.get('checksum', '')), 'node': rospy.get_name(),
            'yolo_endpoint': f'{self.yolo_host}:{self.yolo_port}', 'reason': reason,
        }
        self.ack_pub.publish(String(data=json.dumps(ack, ensure_ascii=False)))

    def task_cb(self, msg: String) -> None:
        try:
            packet = json.loads(msg.data)
            validate_packet(packet, self.id)
            mission_id = str(packet['mission_id'])
            if self.task is not None and mission_id == self.mission_id:
                self.publish_task_ack(packet, True, 'duplicate acknowledged')
                return
            if self.mission_active:
                self.publish_task_ack(packet, False, 'mission already active')
                return
            self.task = packet
            self.mission_id = mission_id
            self.perception_cfg = dict(packet['perception'])
            self.close_display()
            display_cfg = dict(self.perception_cfg.get('detection_display', {}))
            display_kind_map = self.perception_cfg.get('class_kind_map', {})
            display_target_names = list(self.perception_cfg.get('static_targets', [])) + \
                list(self.perception_cfg.get('dynamic_targets', [])) + \
                [str(name) for name, kind in display_kind_map.items()
                 if str(kind) in ('static', 'dynamic')]
            self.detection_display = YoloDetectionDisplay(
                self.id, display_cfg, display_target_names, display_kind_map)
            self.camera_topic = str(packet['camera']['image_topic'])
            self.camera_info_topic = str(packet['camera']['camera_info_topic'])
            fb = packet['camera']['fallback']
            self.camera = {
                'width': int(fb['fallback_width']), 'height': int(fb['fallback_height']),
                'fx': float(fb['fallback_fx']), 'fy': float(fb['fallback_fy']),
                'cx': float(fb['fallback_cx']), 'cy': float(fb['fallback_cy']),
                'u_right_sign': float(fb['image_u_to_body_right_sign']),
                'v_forward_sign': float(fb['image_v_to_body_forward_sign']),
                'ground_z_m': float(fb['ground_z_m']),
                'distortion_model': str(fb.get('distortion_model', 'plumb_bob')),
                'distortion_coefficients': list(fb.get('distortion_coefficients', [])),
            }
            if self.image_sub is None:
                self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1, buff_size=2**26)
                self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1)
            self.detected_static.clear(); self.last_dynamic.clear(); self.slot.reset_metrics()
            self.validation_home_world = None
            self.connected = False; self.ready = False; self.consecutive_pass_windows = 0; self.last_error = ''
            root = Path(os.path.expanduser(self.local_cfg['mission']['vehicle_output_root']))
            self.local_run_dir = root / self.mission_id / f'vehicle_{self.id}'
            self.local_run_dir.mkdir(parents=True, exist_ok=True)
            validation_cfg = dict(self.perception_cfg.get('detection_event_validation', {}))
            localization_cfg = dict(self.perception_cfg.get('target_localization', {}))
            validation_targets = list(self.perception_cfg.get('static_targets', [])) + \
                list(self.perception_cfg.get('dynamic_targets', []))
            self.target_localizer = TargetLocalizationTracker(
                self.id, self.mission_id, localization_cfg, validation_targets,
                display_kind_map,
                selected_result_source=str(
                    self.perception_cfg.get('selected_result_source', 'fov_proxy')))
            self.detection_validator = LocalizationEventValidator(
                self.id, self.mission_id, validation_cfg, validation_targets,
                display_kind_map, self.validation_event_pub,
                self.validation_summary_pub, self.local_run_dir,
                localization_cfg=localization_cfg)
            self.localization_report_count = 0
            self.last_localization_report = None
            dump_json(self.local_run_dir / 'task_packet_perception.json', packet)
            self.task_event.set()
            self.publish_task_ack(packet, True, 'accepted')
            rospy.loginfo('v%d V6 perception accepted camera=%s yolo=%s:%d', self.id, self.camera_topic, self.yolo_host, self.yolo_port)
        except Exception as exc:
            self.last_error = f'task rejected: {exc}'
            rospy.logerr('v%d perception task rejected: %s', self.id, exc)

    def start_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') == self.mission_id:
                self.mission_active = True
        except Exception:
            pass

    def complete_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') == self.mission_id:
                self.complete_event.set()
                if self.detection_validator is not None:
                    self.detection_validator.finalize('management_mission_complete')
                rospy.signal_shutdown('management mission complete')
        except Exception:
            pass

    def flight_status_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if not self.mission_id or row.get('mission_id') == self.mission_id:
                with self.lock:
                    self.flight_status = row
                    position = row.get('world_position')
                    phase = str(row.get('phase', ''))
                    if (self.validation_home_world is None and isinstance(position, list) and
                            len(position) >= 3 and phase in
                            ('READY', 'TAKEOFF_10M', 'INITIAL_TRANSITION_FW')):
                        self.validation_home_world = np.asarray(position[:3], dtype=float)
        except Exception:
            pass

    def flight_event_cb(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
            if row.get('mission_id') not in (None, '', self.mission_id):
                return
            if str(row.get('event', '')) != 'STATIC_CANDIDATE_REJECTED':
                return
            snapshot = self.slot.snapshot()
            if snapshot is None or self.local_run_dir is None:
                rospy.logwarn('v%d no raw frame available for rejected static candidate', self.id)
                return
            sequence, stamp, _wall, image = snapshot
            directory = self.local_run_dir / 'static_rejection_images'
            directory.mkdir(parents=True, exist_ok=True)
            name = str(row.get('target_name', 'unknown'))
            candidate = str(row.get('candidate_id', 'candidate'))
            safe = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_'
                           for ch in f'{name}_{candidate}')
            basename = f'RAW_REJECTED_v{self.id}_{safe}_seq{sequence}_ros{stamp:.3f}'
            image_path = directory / f'{basename}.jpg'
            metadata_path = directory / f'{basename}.json'
            quality = int(self.perception_cfg.get(
                'static_rejection_image_jpeg_quality', 95))
            if not cv2.imwrite(str(image_path), image,
                               [int(cv2.IMWRITE_JPEG_QUALITY), quality]):
                raise RuntimeError(f'cv2.imwrite failed: {image_path}')
            dump_json(metadata_path, {
                **row,
                'raw_image_path': str(image_path),
                'raw_frame_sequence': int(sequence),
                'raw_frame_ros_time': float(stamp),
                'image_is_unannotated_original_camera_frame': True,
            })
            rospy.logwarn(
                'v%d saved rejected static candidate raw image: %s',
                self.id, image_path)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0, 'v%d failed saving static rejection image: %s',
                self.id, exc)

    def image_cb(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            stamp = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
            self.slot.update(stamp, image)
        except Exception as exc:
            self.last_error = f'cv_bridge: {exc}'
            rospy.logwarn_throttle(2.0, 'v%d image conversion failed: %s', self.id, exc)

    def camera_info_cb(self, msg: CameraInfo) -> None:
        if msg.width > 0 and msg.height > 0 and len(msg.K) >= 6 and msg.K[0] > 1 and msg.K[4] > 1:
            with self.lock:
                self.camera.update({
                    'width': int(msg.width), 'height': int(msg.height),
                    'fx': float(msg.K[0]), 'fy': float(msg.K[4]),
                    'cx': float(msg.K[2]), 'cy': float(msg.K[5]),
                    'distortion_model': str(msg.distortion_model),
                    'distortion_coefficients': [float(x) for x in msg.D],
                })

    def world_cb(self, msg: ModelStates) -> None:
        positions = {name: np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
                     for name, pose in zip(msg.name, msg.pose)}
        with self.lock:
            self.positions = positions

    def connect(self) -> socket.socket:
        sock = socket.create_connection((self.yolo_host, self.yolo_port), timeout=float(self.perception_cfg['socket_timeout_seconds']))
        sock.settimeout(float(self.perception_cfg['socket_timeout_seconds']))
        return sock

    def sender_loop(self) -> None:
        sock = None
        next_send = 0.0
        while not rospy.is_shutdown():
            if not self.task_event.is_set():
                time.sleep(0.1); continue
            period = 1.0 / max(float(self.perception_cfg['target_fps']), 0.1)
            now = time.monotonic()
            if now < next_send:
                time.sleep(min(next_send - now, 0.02)); continue
            frame = self.slot.newest()
            if frame is None:
                time.sleep(0.004); continue
            sequence, ros_stamp, wall_stamp, image = frame
            if now - wall_stamp > float(self.perception_cfg['max_frame_age_seconds']):
                self.slot.mark_stale(sequence); continue
            if sock is None:
                try:
                    sock = self.connect(); self.connected = True; self.last_error = ''
                except Exception as exc:
                    self.connected = False; self.last_error = f'connect: {exc}'
                    time.sleep(float(self.perception_cfg['reconnect_seconds'])); continue
            try:
                next_send = time.monotonic() + period
                ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.perception_cfg['jpeg_quality'])])
                if not ok:
                    raise RuntimeError('JPEG encode failed')
                t0 = time.perf_counter()
                response = send_packet(sock, {'mission_id': self.mission_id, 'vehicle_id': self.id,
                                               'stamp': ros_stamp, 'sequence': sequence}, encoded.tobytes())
                roundtrip_ms = (time.perf_counter() - t0) * 1000.0
                self.slot.mark_processed(sequence)
                self.connected = True
                self.last_worker_fps = float(response.get('worker_fps', 0.0))
                self.last_inference_ms = response.get('inference_ms')
                self.last_imgsz = response.get('imgsz')
                self.last_error = str(response.get('error', ''))
                yolo_detections = response.get('detections', [])
                if self.detection_display is not None:
                    self.detection_display.update(image, yolo_detections)
                with self.lock:
                    localization_flight = dict(self.flight_status)
                    localization_camera = dict(self.camera)
                    validation_positions = dict(self.positions)
                    validation_home = (None if self.validation_home_world is None
                                       else self.validation_home_world.copy())
                localization_reports = []
                if self.target_localizer is not None:
                    localization_reports = self.target_localizer.update(
                        image, yolo_detections, ros_stamp, sequence,
                        localization_flight, localization_camera)
                    for localization_row in localization_reports:
                        self.publish_localization_report(localization_row)
                if self.detection_validator is not None and self.detection_validator.enabled:
                    self.detection_validator.update(
                        image, yolo_detections, localization_reports,
                        ros_stamp, sequence, localization_flight,
                        validation_positions, localization_camera, validation_home)
                report = {
                    'mission_id': self.mission_id, 'vehicle_id': self.id, 'source': 'yolo26',
                    'ros_time': rospy.Time.now().to_sec(), 'frame_stamp': ros_stamp, 'sequence': sequence,
                    'worker_fps': self.last_worker_fps, 'inference_ms': self.last_inference_ms,
                    'roundtrip_ms': roundtrip_ms, 'imgsz': self.last_imgsz,
                    'detections': yolo_detections, 'in_flight_control_loop': False,
                }
                self.yolo_pub.publish(String(data=json.dumps(report, ensure_ascii=False)))
                # Management receives only accepted multi-frame localization reports
                # in formal YOLO mode. Validation FOV-proxy operation remains unchanged.
                if self.local_run_dir:
                    with open(self.local_run_dir / 'yolo_metrics.jsonl', 'a', encoding='utf-8') as fp:
                        fp.write(json.dumps(report, ensure_ascii=False) + '\n')
            except Exception as exc:
                self.connected = False; self.last_error = f'{type(exc).__name__}: {exc}'
                try: sock.close()
                except Exception: pass
                sock = None
                time.sleep(float(self.perception_cfg['reconnect_seconds']))


    def publish_localization_report(self, row: dict) -> None:
        """Publish a stable geolocation report and optionally feed management.

        The dedicated localization topic is always published. The normal
        detection_report topic is populated only when the configured management
        source is YOLO localization, preserving the validation FOV-proxy path.
        """
        payload = dict(row)
        payload['mission_id'] = self.mission_id
        payload['vehicle_id'] = self.id
        self.localization_report_count += 1
        self.last_localization_report = dict(payload)
        self.localization_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))
        if bool(payload.get('selected_as_management_result', False)):
            management = {
                **payload,
                'source': 'yolo_localized_stable',
                'selected_as_management_result': True,
            }
            self.detection_pub.publish(
                String(data=json.dumps(management, ensure_ascii=False)))
        if self.local_run_dir:
            with open(self.local_run_dir / 'target_localization_reports.jsonl',
                      'a', encoding='utf-8') as fp:
                fp.write(json.dumps(payload, ensure_ascii=False) + '\n')
        rospy.loginfo(
            'v%d LOCALIZATION target=%s conf=%.2f world=(%.1f,%.1f,%.1f) '
            'std_xy=%.1fm frames=%d selected=%s',
            self.id, payload.get('target_name'), float(payload.get('confidence', 0.0)),
            float(payload.get('position_world', [0, 0, 0])[0]),
            float(payload.get('position_world', [0, 0, 0])[1]),
            float(payload.get('position_world', [0, 0, 0])[2]),
            float(payload.get('horizontal_std_m', 0.0)),
            int(payload.get('consecutive_frames', 0)),
            bool(payload.get('selected_as_management_result', False)))


    def publish_yolo_management_detections(self, detections, ros_stamp: float) -> None:
        """Project trained YOLO detections to the ground in formal mode.

        This uses only the aircraft pose, camera calibration, and image box centre.
        It does not read target truth. Camera roll/pitch calibration should replace
        the near-nadir approximation before formal scoring.
        """
        # Legacy single-frame projection is retained only for source-level
        # comparison and is deliberately disabled. V6.7.12 management
        # uses TargetLocalizationTracker multi-frame reports.
        return
        with self.lock:
            flight=dict(self.flight_status); camera=dict(self.camera)
        vehicle=flight.get('world_position')
        if not isinstance(vehicle,list) or len(vehicle)<3:
            return
        phase=str(flight.get('phase','')); gate=bool(flight.get('detection_valid',False))
        if self.perception_cfg.get('require_straight_gate_for_fw_search',True):
            if not (phase.startswith('TRACK_') or phase == 'STATIC_HOVER_VERIFY') and not gate:
                return
        yaw=float(flight.get('world_yaw',0.0)); height=float(vehicle[2])-float(camera.get('ground_z_m',0.2))
        if height <= 0.5:return
        kind_map=self.perception_cfg.get('class_kind_map',{})
        now=rospy.Time.now().to_sec()
        for det in detections:
            name=str(det.get('class_name','')); kind=str(kind_map.get(name,''))
            if name not in self.perception_cfg.get('static_targets',[]) + self.perception_cfg.get('dynamic_targets',[]):
                continue
            if kind not in ('static','dynamic'):continue
            c=det.get('center');
            if not isinstance(c,list) or len(c)<2:continue
            u,v=float(c[0]),float(c[1])
            right=(u-float(camera['cx']))*height/(float(camera.get('u_right_sign',1.0))*float(camera['fx']))
            forward=(v-float(camera['cy']))*height/(float(camera.get('v_forward_sign',-1.0))*float(camera['fy']))
            cy,sy=math.cos(yaw),math.sin(yaw)
            dx=cy*forward-sy*right;dy=sy*forward+cy*right
            target=[float(vehicle[0])+dx,float(vehicle[1])+dy,float(camera.get('ground_z_m',0.2))]
            if kind=='static':
                if name in self.detected_static:continue
                self.detected_static.add(name)
            else:
                interval=1.0/max(float(self.perception_cfg.get('dynamic_repeat_hz',5.0)),1e-3)
                if now-self.last_dynamic.get(name,-1e9)<interval:continue
                self.last_dynamic[name]=now
            row={'mission_id':self.mission_id,'vehicle_id':self.id,'source':'yolo_projected',
                'selected_as_management_result':True,'ros_time':now,'frame_stamp':ros_stamp,
                'target_name':name,'kind':kind,'class_name':name,
                'confidence':float(det.get('confidence',0.0)),'image_uv':[u,v],
                'xyxy':det.get('xyxy'),'ground_xy':target[:2],'target_world':target,
                'vehicle_world':[float(x) for x in vehicle[:3]],'vehicle_yaw_rad':yaw,
                'flight_phase':phase,'detection_segment_type':flight.get('detection_segment_type'),
                'detection_leg_id':flight.get('detection_leg_id'),
                'projection_rule':'near_nadir_camera_model_no_target_truth'}
            self.detection_pub.publish(String(data=json.dumps(row,ensure_ascii=False)))
            if self.local_run_dir:
                with open(self.local_run_dir/'detections_local.jsonl','a',encoding='utf-8') as fp:fp.write(json.dumps(row,ensure_ascii=False)+'\n')

    def shutdown_cleanup(self) -> None:
        validator = self.detection_validator
        if validator is not None:
            try:
                validator.finalize('perception_node_shutdown')
            except Exception as exc:
                rospy.logwarn('v%d validation finalization failed: %s', self.id, exc)
        self.close_display()

    def close_display(self) -> None:
        display = self.detection_display
        self.detection_display = None
        if display is not None:
            display.close()

    def status_payload(self) -> dict:
        window = float(self.perception_cfg.get('performance_window_seconds', 5.0)) if self.perception_cfg else 5.0
        source_fps = self.slot.fps(True, window)
        processed_fps = self.slot.fps(False, window)
        minimum = float(self.perception_cfg.get('minimum_required_fps', 10.0)) if self.perception_cfg else 10.0
        rate_pass = self.connected and source_fps >= minimum and processed_fps >= minimum and self.last_worker_fps >= minimum
        self.consecutive_pass_windows = self.consecutive_pass_windows + 1 if rate_pass else 0
        required_windows = int(self.perception_cfg.get('readiness_consecutive_windows', 3)) if self.perception_cfg else 3
        required_frames = int(self.perception_cfg.get('readiness_minimum_processed_frames', 40)) if self.perception_cfg else 40
        self.ready = bool(rate_pass and self.consecutive_pass_windows >= required_windows and self.slot.processed_total >= required_frames)
        with self.lock:
            phase = self.flight_status.get('phase')
            gate = bool(self.flight_status.get('detection_valid', False))
        return {
            'mission_id': self.mission_id, 'vehicle_id': self.id, 'component': 'perception',
            'endpoint': f'{self.yolo_host}:{self.yolo_port}', 'connected': self.connected, 'ready': self.ready,
            'source_fps': source_fps, 'processed_fps': processed_fps, 'worker_fps': self.last_worker_fps,
            'minimum_required_fps': minimum, 'rate_pass': rate_pass,
            'consecutive_pass_windows': self.consecutive_pass_windows, 'processed_frame_count': self.slot.processed_total,
            'replaced_frames': self.slot.replaced, 'stale_frames': self.slot.stale,
            'last_inference_ms': self.last_inference_ms, 'last_imgsz': self.last_imgsz,
            'last_error': self.last_error, 'selected_result_source': ('fov_proxy_validation' if self.validation_fov_proxy else 'yolo'),
            'flight_phase': phase, 'straight_detection_gate': gate,
            'target_localization': {
                'enabled': bool(self.target_localizer is not None and
                                self.target_localizer.enabled),
                'report_count': int(self.localization_report_count),
                'last_report': self.last_localization_report,
            },
            'detection_event_validation': (
                None if self.detection_validator is None
                else self.detection_validator.summary()),
            'ros_time': rospy.Time.now().to_sec(),
        }

    def status_tick(self, _event=None) -> None:
        row = self.status_payload()
        self.status_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))
        if self.local_run_dir:
            with open(self.local_run_dir / 'perception_status.jsonl', 'a', encoding='utf-8') as fp:
                fp.write(json.dumps(row, ensure_ascii=False) + '\n')

    def proxy_tick(self, _event=None) -> None:
        if not self.mission_active or not self.task_event.is_set() or rospy.is_shutdown():
            return
        with self.lock:
            positions = dict(self.positions)
            flight = dict(self.flight_status)
            camera = dict(self.camera)
        vehicle = positions.get(f'standard_vtol_{self.id}')
        if vehicle is None:
            return
        phase = str(flight.get('phase', ''))
        search_gate = bool(flight.get('detection_valid', False))
        valid_phase = phase.startswith('TRACK_') or phase == 'STATIC_HOVER_VERIFY' or search_gate
        if bool(self.perception_cfg.get('require_straight_gate_for_fw_search', True)) and not valid_phase:
            return
        yaw = float(flight.get('world_yaw', 0.0))
        now = rospy.Time.now().to_sec()
        dynamic_interval = 1.0 / max(float(self.perception_cfg['dynamic_repeat_hz']), 1e-3)
        static_targets = list(self.perception_cfg['static_targets'])
        dynamic_targets = list(self.perception_cfg['dynamic_targets'])
        margin = float(self.perception_cfg['fov_margin_ratio'])
        for name in static_targets + dynamic_targets:
            target = positions.get(name)
            if target is None:
                continue
            projection = fov_project_nadir(vehicle, yaw, target, camera, margin)
            if not projection.get('inside', False):
                continue
            kind = 'static' if name in static_targets else 'dynamic'
            if kind == 'static':
                if name in self.detected_static:
                    continue
                self.detected_static.add(name)
            else:
                if now - self.last_dynamic.get(name, -1e9) < dynamic_interval:
                    continue
                self.last_dynamic[name] = now
            horizontal = float(np.linalg.norm((vehicle - target)[:2]))
            row = {
                'mission_id': self.mission_id, 'vehicle_id': self.id,
                'source': 'fov_proxy_local_perception', 'selected_as_management_result': True,
                'ros_time': now, 'timestamp_ns': int(rospy.Time.now().to_nsec()),
                'target_name': name, 'kind': kind,
                'class_name': 'static_target' if kind == 'static' else name,
                'confidence': 1.0, 'ground_xy': [float(target[0]), float(target[1])],
                'target_world': target.tolist(), 'vehicle_world': vehicle.tolist(),
                'vehicle_yaw_rad': yaw, 'horizontal_distance_m_diagnostic_only': horizontal,
                'image_uv': [projection.get('u'), projection.get('v')],
                'projection': projection, 'flight_phase': phase,
                'detection_segment_type': flight.get('detection_segment_type'),
                'detection_leg_id': flight.get('detection_leg_id'),
                'proxy_rule': 'target centre projects inside configured camera image FOV margin; no fixed distance threshold',
                'fov_margin_ratio': margin, 'yolo_pipeline_ready': self.ready,
                'yolo_processed_fps': self.slot.fps(False, float(self.perception_cfg['performance_window_seconds'])),
            }
            self.detection_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))
            if self.local_run_dir:
                with open(self.local_run_dir / 'detections_local.jsonl', 'a', encoding='utf-8') as fp:
                    fp.write(json.dumps(row, ensure_ascii=False) + '\n')
            rospy.loginfo('v%d FOV proxy target=%s phase=%s uv=(%.1f,%.1f) range=%.1fm',
                          self.id, name, phase, projection['u'], projection['v'], horizontal)


if __name__ == '__main__':
    VehiclePerceptionAgent()
    rospy.spin()
