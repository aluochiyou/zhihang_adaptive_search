#!/usr/bin/env python3
"""Multi-frame YOLO target geolocation and validation-only evaluation.

The localization path uses only aircraft pose, full attitude quaternion, camera
intrinsics, configurable camera-to-body extrinsics and a ground-plane model.
Gazebo model states are accepted only by :class:`LocalizationEventValidator`
for offline/validation scoring; they are never consumed by the localization
tracker, mission manager, route planner or flight controller.
"""
from __future__ import annotations

import collections
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    from std_msgs.msg import String
except Exception:  # ROS-free unit tests
    class String:
        def __init__(self, data=''):
            self.data = data


DEFAULT_OPTICAL_TO_BODY = np.asarray([
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=float)


def _finite_vector(values: Sequence[float], size: int) -> Optional[np.ndarray]:
    try:
        row = np.asarray(list(values)[:size], dtype=float)
    except Exception:
        return None
    if row.shape != (size,) or not np.all(np.isfinite(row)):
        return None
    return row


def quaternion_to_matrix_xyzw(values: Sequence[float]) -> np.ndarray:
    """Return body-to-world rotation matrix for an ``[x,y,z,w]`` quaternion."""
    q = _finite_vector(values, 4)
    if q is None:
        return np.eye(3, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        return np.eye(3, dtype=float)
    x, y, z, w = q / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def yaw_to_quaternion_xyzw(yaw: float) -> List[float]:
    half = 0.5 * float(yaw)
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def orientation_from_flight(flight: dict) -> Tuple[np.ndarray, List[float]]:
    q = flight.get('world_orientation_xyzw')
    if isinstance(q, list) and len(q) >= 4:
        qv = _finite_vector(q, 4)
        if qv is not None:
            return quaternion_to_matrix_xyzw(qv), qv.tolist()
    fallback = yaw_to_quaternion_xyzw(float(flight.get('world_yaw', 0.0)))
    return quaternion_to_matrix_xyzw(fallback), fallback


def _matrix3(value, fallback: np.ndarray) -> np.ndarray:
    try:
        matrix = np.asarray(value, dtype=float).reshape(3, 3)
    except Exception:
        return fallback.copy()
    if not np.all(np.isfinite(matrix)):
        return fallback.copy()
    # Reject a degenerate matrix but do not require exact orthonormality because
    # field calibration may contain small numerical errors.
    if abs(float(np.linalg.det(matrix))) < 0.2:
        return fallback.copy()
    u, _, vt = np.linalg.svd(matrix)
    return u @ vt


def camera_pose_world(vehicle_world: Sequence[float], flight: dict,
                      localization_cfg: dict) -> Tuple[np.ndarray, np.ndarray, dict]:
    """Return camera origin and optical-to-world rotation.

    Body frame is assumed FLU/ENU-compatible. Camera optical frame follows ROS
    convention: +x right, +y down, +z forward. The default camera points down,
    with image down corresponding to body backward, preserving the historical
    V6 image sign convention.
    """
    vehicle = _finite_vector(vehicle_world, 3)
    if vehicle is None:
        raise ValueError('invalid vehicle world position')
    r_world_body, q = orientation_from_flight(flight)
    r_body_optical = _matrix3(
        localization_cfg.get('optical_to_body_matrix', DEFAULT_OPTICAL_TO_BODY.tolist()),
        DEFAULT_OPTICAL_TO_BODY)
    offset = _finite_vector(localization_cfg.get('camera_position_body_m', [0.0, 0.0, 0.0]), 3)
    if offset is None:
        offset = np.zeros(3, dtype=float)
    origin = vehicle + r_world_body @ offset
    r_world_optical = r_world_body @ r_body_optical
    return origin, r_world_optical, {
        'world_orientation_xyzw': q,
        'camera_position_body_m': offset.tolist(),
        'optical_to_body_matrix': r_body_optical.tolist(),
    }


def camera_matrix(camera: dict) -> np.ndarray:
    return np.asarray([
        [float(camera['fx']), 0.0, float(camera['cx'])],
        [0.0, float(camera['fy']), float(camera['cy'])],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def undistort_pixel(u: float, v: float, camera: dict,
                    use_distortion: bool = True) -> Tuple[float, float]:
    if not use_distortion:
        return ((float(u) - float(camera['cx'])) / float(camera['fx']),
                (float(v) - float(camera['cy'])) / float(camera['fy']))
    distortion = camera.get('distortion_coefficients', [])
    try:
        d = np.asarray(distortion, dtype=float).reshape(-1)
    except Exception:
        d = np.zeros(0, dtype=float)
    if d.size == 0 or not np.any(np.abs(d) > 1e-12):
        return ((float(u) - float(camera['cx'])) / float(camera['fx']),
                (float(v) - float(camera['cy'])) / float(camera['fy']))
    pts = np.asarray([[[float(u), float(v)]]], dtype=np.float64)
    normalized = cv2.undistortPoints(pts, camera_matrix(camera), d)
    return float(normalized[0, 0, 0]), float(normalized[0, 0, 1])


def localize_pixel_to_ground(vehicle_world: Sequence[float], flight: dict,
                             camera: dict, localization_cfg: dict,
                             u: float, v: float,
                             ground_z_m: Optional[float] = None) -> dict:
    """Intersect the calibrated image ray with the configured ground plane."""
    ground = float(camera.get('ground_z_m', 0.2) if ground_z_m is None else ground_z_m)
    origin, r_world_optical, extrinsics = camera_pose_world(
        vehicle_world, flight, localization_cfg)
    nx, ny = undistort_pixel(
        u, v, camera, bool(localization_cfg.get('use_camera_info_distortion', True)))
    ray_optical = np.asarray([nx, ny, 1.0], dtype=float)
    ray_optical /= max(float(np.linalg.norm(ray_optical)), 1e-9)
    ray_world = r_world_optical @ ray_optical
    if not np.all(np.isfinite(ray_world)) or ray_world[2] >= -1e-5:
        return {
            'valid': False,
            'reason': 'camera ray does not intersect ground below aircraft',
            'ray_world': ray_world.tolist(),
            'camera_origin_world': origin.tolist(),
        }
    scale = (ground - float(origin[2])) / float(ray_world[2])
    if not math.isfinite(scale) or scale <= 0.0:
        return {
            'valid': False,
            'reason': 'ground intersection lies behind camera',
            'ray_world': ray_world.tolist(),
            'camera_origin_world': origin.tolist(),
        }
    point = origin + scale * ray_world
    ground_range = float(np.linalg.norm(point[:2] - origin[:2]))
    max_range = float(localization_cfg.get('maximum_ground_range_m', 350.0))
    if ground_range > max_range:
        return {
            'valid': False,
            'reason': f'ground range {ground_range:.1f}m exceeds limit {max_range:.1f}m',
            'position_world': point.tolist(),
            'ground_range_m': ground_range,
            'ray_world': ray_world.tolist(),
            'camera_origin_world': origin.tolist(),
        }
    return {
        'valid': True,
        'position_world': point.tolist(),
        'ground_xy': point[:2].tolist(),
        'ground_range_m': ground_range,
        'camera_origin_world': origin.tolist(),
        'ray_world': ray_world.tolist(),
        'normalized_image_ray': ray_optical.tolist(),
        'extrinsics': extrinsics,
        'projection_rule': 'full_attitude_calibrated_camera_ray_ground_plane_intersection',
    }


def project_world_to_image(vehicle_world: Sequence[float], flight: dict,
                           camera: dict, localization_cfg: dict,
                           target_world: Sequence[float], margin_ratio: float = 1.0) -> dict:
    """Project a known world point into the calibrated camera for validation only."""
    target = _finite_vector(target_world, 3)
    if target is None:
        return {'inside': False, 'reason': 'invalid target world position'}
    origin, r_world_optical, _ = camera_pose_world(
        vehicle_world, flight, localization_cfg)
    optical = r_world_optical.T @ (target - origin)
    if not np.all(np.isfinite(optical)) or optical[2] <= 1e-5:
        return {'inside': False, 'reason': 'target behind camera'}
    x, y, z = [float(v) for v in optical]
    distortion = camera.get('distortion_coefficients', [])
    use_distortion = bool(localization_cfg.get('use_camera_info_distortion', True))
    try:
        d = np.asarray(distortion, dtype=float).reshape(-1)
    except Exception:
        d = np.zeros(0, dtype=float)
    if use_distortion and d.size and np.any(np.abs(d) > 1e-12):
        object_points = np.asarray([[[x, y, z]]], dtype=np.float64)
        pixels, _ = cv2.projectPoints(
            object_points, np.zeros(3), np.zeros(3), camera_matrix(camera), d)
        u, v = float(pixels[0, 0, 0]), float(pixels[0, 0, 1])
    else:
        u = float(camera['fx']) * x / z + float(camera['cx'])
        v = float(camera['fy']) * y / z + float(camera['cy'])
    width, height = float(camera['width']), float(camera['height'])
    margin = max(0.05, min(1.0, float(margin_ratio)))
    x_pad = 0.5 * width * (1.0 - margin)
    y_pad = 0.5 * height * (1.0 - margin)
    inside = x_pad <= u <= width - x_pad and y_pad <= v <= height - y_pad
    return {
        'inside': bool(inside), 'u': u, 'v': v,
        'optical_depth_m': z,
        'camera_origin_world': origin.tolist(),
    }


def detection_anchor(det: dict, mode: str = 'bottom_center') -> Optional[Tuple[float, float]]:
    box = det.get('xyxy')
    if isinstance(box, list) and len(box) >= 4:
        x1, y1, x2, y2 = [float(v) for v in box[:4]]
        if mode == 'center':
            return 0.5 * (x1 + x2), 0.5 * (y1 + y2)
        if mode == 'lower_quarter_center':
            return 0.5 * (x1 + x2), 0.75 * y2 + 0.25 * y1
        return 0.5 * (x1 + x2), y2
    center = det.get('center')
    if isinstance(center, list) and len(center) >= 2:
        return float(center[0]), float(center[1])
    return None


def _jsonable_report(report: dict) -> dict:
    return {key: value for key, value in report.items()
            if not key.startswith('_')}


class TargetLocalizationTracker:
    """Convert stable, target-specific YOLO sequences into world positions."""

    def __init__(self, vehicle_id: int, mission_id: str, cfg: dict,
                 target_names: Sequence[str], kind_map: dict,
                 selected_result_source: str = 'fov_proxy') -> None:
        self.vehicle_id = int(vehicle_id)
        self.mission_id = str(mission_id)
        self.cfg = dict(cfg or {})
        self.enabled = bool(self.cfg.get('enabled', True))
        self.target_names = set(str(x) for x in target_names)
        self.kind_map = {str(k): str(v) for k, v in (kind_map or {}).items()}
        self.aliases = {str(k): str(v) for k, v in self.cfg.get('class_aliases', {}).items()}
        self.minimum_confidence = float(self.cfg.get('minimum_confidence', 0.50))
        self.minimum_consecutive = max(1, int(self.cfg.get('minimum_consecutive_frames', 3)))
        self.maximum_gap = float(self.cfg.get('maximum_frame_gap_seconds', 0.35))
        self.spatial_gate = float(self.cfg.get('spatial_consistency_radius_m', 30.0))
        self.maximum_std = float(self.cfg.get('maximum_horizontal_std_m', 12.0))
        self.sample_window = max(self.minimum_consecutive,
                                 int(self.cfg.get('sample_window_frames', 8)))
        self.anchor_mode = str(self.cfg.get('image_anchor_mode', 'bottom_center'))
        self.dynamic_interval = 1.0 / max(
            float(self.cfg.get('dynamic_report_hz', 5.0)), 1e-3)
        self.static_repeat = float(self.cfg.get('static_repeat_seconds', 9999.0))
        self.static_hover_interval = 1.0 / max(
            float(self.cfg.get('static_hover_report_hz', 2.0)), 1e-3)
        self.require_straight_gate = bool(
            self.cfg.get('require_straight_gate_for_fw_search', True))
        self.selected_result_source = str(selected_result_source)
        self.management_sources = set(self.cfg.get(
            'management_source_names', ['yolo_projected', 'yolo_localized']))
        self.states: Dict[str, dict] = {}
        self.report_index = 0

    def _canonical(self, name: str) -> str:
        return self.aliases.get(str(name), str(name))

    def _allowed(self, flight: dict) -> bool:
        phase = str(flight.get('phase', ''))
        if self.require_straight_gate:
            if not (phase.startswith('TRACK_') or phase == 'STATIC_HOVER_VERIFY') and not bool(
                    flight.get('detection_valid', False)):
                return False
        return phase not in ('READY', 'TAKEOFF_10M', 'RETURN', 'AUTO_LAND',
                             'LANDED', 'DONE', 'FAILED', 'EMERGENCY_LAND')

    def _reset(self, name: str, stamp: float) -> dict:
        state = {
            'target_name': name,
            'start_stamp': float(stamp),
            'last_stamp': float(stamp),
            'consecutive': 0,
            'samples': collections.deque(maxlen=self.sample_window),
            'last_report_stamp': -1e9,
            'reported_in_event': False,
        }
        self.states[name] = state
        return state

    def update(self, image, detections: Sequence[dict], ros_stamp: float,
               sequence: int, flight: dict, camera: dict) -> List[dict]:
        if not self.enabled or not self._allowed(flight):
            self.states.clear()
            return []
        vehicle = _finite_vector(flight.get('world_position', []), 3)
        if vehicle is None:
            return []
        best: Dict[str, dict] = {}
        for raw in detections or []:
            name = self._canonical(str(raw.get('class_name', '')))
            confidence = float(raw.get('confidence', 0.0))
            if name not in self.target_names or confidence < self.minimum_confidence:
                continue
            if name not in best or confidence > float(best[name].get('confidence', 0.0)):
                best[name] = dict(raw)
        reports: List[dict] = []
        present = set()
        for name, det in best.items():
            anchor = detection_anchor(det, self.anchor_mode)
            if anchor is None:
                continue
            projection = localize_pixel_to_ground(
                vehicle, flight, camera, self.cfg, anchor[0], anchor[1])
            if not projection.get('valid', False):
                continue
            point = np.asarray(projection['position_world'], dtype=float)
            state = self.states.get(name)
            if (state is None or float(ros_stamp) - float(state['last_stamp']) > self.maximum_gap):
                state = self._reset(name, ros_stamp)
            elif state['samples']:
                previous = np.asarray(state['samples'][-1]['position_world'], dtype=float)
                if float(np.linalg.norm(point[:2] - previous[:2])) > self.spatial_gate:
                    state = self._reset(name, ros_stamp)
            state['last_stamp'] = float(ros_stamp)
            state['consecutive'] += 1
            sample = {
                'position_world': point.tolist(),
                'confidence': float(det.get('confidence', 0.0)),
                'frame_stamp': float(ros_stamp),
                'sequence': int(sequence),
                'image_uv': [float(anchor[0]), float(anchor[1])],
                'xyxy': det.get('xyxy'),
            }
            state['samples'].append(sample)
            present.add(name)
            if int(state['consecutive']) < self.minimum_consecutive:
                continue
            samples = list(state['samples'])
            xyz = np.asarray([row['position_world'] for row in samples], dtype=float)
            weights = np.asarray([max(float(row['confidence']), 1e-3) for row in samples], dtype=float)
            estimate = np.average(xyz, axis=0, weights=weights)
            deviations = xyz - estimate
            std = np.sqrt(np.average(deviations * deviations, axis=0, weights=weights))
            horizontal_std = float(math.hypot(float(std[0]), float(std[1])))
            if horizontal_std > self.maximum_std:
                continue
            kind = self.kind_map.get(name, '')
            if kind == 'dynamic':
                interval = self.dynamic_interval
            elif str(flight.get('phase', '')) == 'STATIC_HOVER_VERIFY':
                interval = self.static_hover_interval
            else:
                interval = self.static_repeat
            if float(ros_stamp) - float(state['last_report_stamp']) < interval:
                continue
            if (kind == 'static' and state['reported_in_event']
                    and str(flight.get('phase', '')) != 'STATIC_HOVER_VERIFY'):
                continue
            self.report_index += 1
            confidence_values = [float(row['confidence']) for row in samples]
            report = {
                'schema_version': 1,
                'mission_id': self.mission_id,
                'vehicle_id': self.vehicle_id,
                'report_event_id': (
                    f'LOC_v{self.vehicle_id}_{name}_{self.report_index:06d}'),
                'source': 'yolo_multiframe_full_attitude_ray_plane',
                'selected_as_management_result': (
                    self.selected_result_source in self.management_sources),
                'ros_time': float(ros_stamp),
                'source_ros_time': float(ros_stamp),
                'frame_stamp': float(ros_stamp),
                'sequence': int(sequence),
                'target_name': name,
                'class_name': name,
                'kind': kind,
                'confidence': float(np.mean(confidence_values)),
                'minimum_frame_confidence': float(min(confidence_values)),
                'maximum_frame_confidence': float(max(confidence_values)),
                'consecutive_frames': int(state['consecutive']),
                'sample_count': len(samples),
                'position_world': estimate.tolist(),
                'target_world': estimate.tolist(),
                'ground_xy': estimate[:2].tolist(),
                'position_std_m': std.tolist(),
                'horizontal_std_m': horizontal_std,
                'image_uv': sample['image_uv'],
                'xyxy': sample.get('xyxy'),
                'image_anchor_mode': self.anchor_mode,
                'vehicle_world': vehicle.tolist(),
                'vehicle_orientation_xyzw': orientation_from_flight(flight)[1],
                'vehicle_attitude_rpy_rad': flight.get('world_attitude_rpy_rad'),
                'camera_info': {
                    'width': int(camera['width']), 'height': int(camera['height']),
                    'fx': float(camera['fx']), 'fy': float(camera['fy']),
                    'cx': float(camera['cx']), 'cy': float(camera['cy']),
                    'distortion_model': str(camera.get('distortion_model', '')),
                    'distortion_coefficients': list(camera.get('distortion_coefficients', [])),
                },
                'camera_extrinsics': projection.get('extrinsics'),
                'ground_z_m': float(camera.get('ground_z_m', 0.2)),
                'ground_range_m': float(projection.get('ground_range_m', 0.0)),
                'projection_rule': projection.get('projection_rule'),
                'flight_phase': str(flight.get('phase', '')),
                'detection_valid': bool(flight.get('detection_valid', False)),
                'detection_segment_type': flight.get('detection_segment_type'),
                'detection_leg_id': flight.get('detection_leg_id'),
                'multiframe_acceptance': {
                    'minimum_confidence': self.minimum_confidence,
                    'minimum_consecutive_frames': self.minimum_consecutive,
                    'spatial_consistency_radius_m': self.spatial_gate,
                    'maximum_horizontal_std_m': self.maximum_std,
                },
            }
            reports.append(report)
            state['last_report_stamp'] = float(ros_stamp)
            state['reported_in_event'] = True
        for name, state in list(self.states.items()):
            if name not in present and float(ros_stamp) - float(state['last_stamp']) > self.maximum_gap:
                self.states.pop(name, None)
        return reports


class LocalizationEventValidator:
    """Event-level validation of *reported* detections and target positions.

    Classification semantics:
      * ``correct``: an expected target received a stable, target-consistent
        report and its horizontal localization error was within tolerance.
      * ``miss``: an expected visibility event ended without an effective report.
      * ``wrong_report``: a stable report was emitted when that target was not
        expected, the reported class was inconsistent, or localization error
        exceeded the configured tolerance.

    Every saved event contains an untouched raw image, an annotated image and a
    JSON metadata sidecar. Statistics are event-level, never per frame.
    """

    def __init__(self, vehicle_id: int, mission_id: str, cfg: dict,
                 target_names: Sequence[str], kind_map: dict,
                 event_pub, summary_pub, output_dir: Path,
                 localization_cfg: Optional[dict] = None) -> None:
        self.vehicle_id = int(vehicle_id)
        self.mission_id = str(mission_id)
        self.cfg = dict(cfg or {})
        self.localization_cfg = dict(localization_cfg or {})
        self.enabled = bool(self.cfg.get('enabled', False))
        self.target_names = [str(x) for x in target_names]
        self.kind_map = {str(k): str(v) for k, v in (kind_map or {}).items()}
        self.aliases = {str(k): str(v) for k, v in self.cfg.get('class_aliases', {}).items()}
        self.event_pub = event_pub
        self.summary_pub = summary_pub
        self.output_dir = Path(output_dir)
        self.event_dir = self.output_dir / 'detection_validation_events'
        self.event_dir.mkdir(parents=True, exist_ok=True)
        self.min_visibility_seconds = float(
            self.cfg.get('minimum_expected_visibility_seconds', 0.8))
        self.min_expected_frames = int(self.cfg.get('minimum_expected_frames', 6))
        self.event_gap_seconds = float(self.cfg.get('event_gap_seconds', 0.55))
        self.platform_radius = float(self.cfg.get('platform_exclusion_radius_m', 120.0))
        self.minimum_altitude = float(self.cfg.get('minimum_evaluation_altitude_m', 15.0))
        self.fov_margin = float(self.cfg.get('expected_fov_margin_ratio', 0.88))
        self.report_confidence = float(self.cfg.get('report_confidence_threshold', 0.50))
        self.correct_error = float(self.cfg.get('correct_localization_error_m', 20.0))
        self.minimum_correct_reports = int(self.cfg.get('minimum_correct_reports', 1))
        self.wrong_min_reports = int(self.cfg.get('wrong_report_minimum_reports', 1))
        self.wrong_gap = float(self.cfg.get('wrong_report_event_gap_seconds', 0.75))
        self.jpeg_quality = int(self.cfg.get('saved_jpeg_quality', 92))
        self.save_correct = bool(self.cfg.get('save_correct_event_image', True))
        self.save_raw = bool(self.cfg.get('save_raw_and_annotated_images', True))
        self.active_expected: Dict[str, dict] = {}
        self.active_wrong: Dict[str, dict] = {}
        self.counts = collections.Counter()
        self.per_target: Dict[str, collections.Counter] = {
            name: collections.Counter() for name in self.target_names
        }
        self.localization_errors: List[dict] = []
        self.event_index = 0
        self.ignored_short = 0
        self.finalized = False

    def _canonical(self, name: str) -> str:
        return self.aliases.get(str(name), str(name))

    def _allowed(self, flight: dict, vehicle: np.ndarray,
                 home_world: Optional[np.ndarray], ground_z: float) -> Tuple[bool, str]:
        phase = str(flight.get('phase', ''))
        if phase.startswith('SEARCH_'):
            if not bool(flight.get('detection_valid', False)):
                return False, 'search_not_on_valid_straight_segment'
        elif not (phase.startswith('TRACK_') or phase == 'STATIC_HOVER_VERIFY'):
            return False, f'excluded_phase:{phase}'
        if float(vehicle[2]) - float(ground_z) < self.minimum_altitude:
            return False, 'below_minimum_evaluation_altitude'
        if home_world is not None and float(np.linalg.norm(
                (vehicle - home_world)[:2])) <= self.platform_radius:
            return False, 'inside_takeoff_landing_platform_exclusion'
        return True, ''

    def _new_expected(self, name: str, kind: str, now: float) -> dict:
        return {
            'target_name': name, 'kind': kind,
            'start_ros': float(now), 'last_expected_ros': float(now),
            'expected_frames': 0, 'report_count': 0,
            'correct_report_count': 0, 'wrong_report_count': 0,
            'best_error_m': float('inf'), 'best_image': None,
            'best_detections': [], 'best_expected': None,
            'best_report': None, 'best_centre_distance_px': float('inf'),
            'errors_m': [],
        }

    def _new_wrong(self, name: str, kind: str, now: float) -> dict:
        return {
            'target_name': name, 'kind': kind,
            'start_ros': float(now), 'last_report_ros': float(now),
            'report_count': 0, 'best_image': None,
            'best_detections': [], 'best_report': None,
            'maximum_confidence': 0.0,
        }

    def _clean_payload(self, event: dict) -> dict:
        result = {}
        for key, value in event.items():
            if key in ('best_image', 'best_detections'):
                continue
            if isinstance(value, float) and not math.isfinite(value):
                result[key] = None
            else:
                result[key] = value
        return result

    def _annotate(self, image, detections: Sequence[dict], title: str,
                  expected: Optional[dict], report: Optional[dict]) -> np.ndarray:
        out = image.copy()
        for det in detections or []:
            box = det.get('xyxy') or []
            if len(box) < 4:
                continue
            x1, y1, x2, y2 = [int(round(float(x))) for x in box[:4]]
            name = self._canonical(str(det.get('class_name', '')))
            confidence = float(det.get('confidence', 0.0))
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 180, 255), 2)
            cv2.putText(out, f'{name} {confidence:.2f}',
                        (x1, max(22, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.58, (0, 180, 255), 2, cv2.LINE_AA)
        if expected:
            u = int(round(float(expected.get('u', 0.0))))
            v = int(round(float(expected.get('v', 0.0))))
            cv2.drawMarker(out, (u, v), (255, 0, 255), cv2.MARKER_CROSS, 24, 3)
            cv2.putText(out, f'EXPECTED {expected.get("target_name", "")}',
                        (max(5, u - 120), max(25, v - 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                        (255, 0, 255), 2, cv2.LINE_AA)
        if report:
            error = report.get('validation_horizontal_error_m')
            text = f'REPORTED {report.get("target_name", "")}'
            if error is not None:
                text += f' err={float(error):.1f}m'
            cv2.putText(out, text, (12, 66), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (255, 255, 0), 2, cv2.LINE_AA)
        cv2.rectangle(out, (0, 0), (min(out.shape[1], 1000), 42),
                      (10, 10, 10), -1)
        cv2.putText(out, title, (10, 29), cv2.FONT_HERSHEY_SIMPLEX,
                    0.68, (255, 255, 255), 2, cv2.LINE_AA)
        return out

    def _save_pair(self, event_type: str, target_name: str,
                   event: dict) -> dict:
        image = event.get('best_image')
        if image is None:
            return {'raw_image': None, 'annotated_image': None,
                    'metadata_file': None}
        self.event_index += 1
        safe_target = ''.join(
            ch if ch.isalnum() or ch in ('_', '-') else '_'
            for ch in str(target_name))
        stamp = float(event.get('start_ros', 0.0))
        stem = (f'{event_type.upper()}_v{self.vehicle_id}_{safe_target}_'
                f'{self.event_index:04d}_ros{stamp:.3f}')
        raw_path = self.event_dir / f'RAW_{stem}.jpg'
        annotated_path = self.event_dir / f'ANNOTATED_{stem}.jpg'
        metadata_path = self.event_dir / f'{stem}.json'
        if self.save_raw:
            cv2.imwrite(str(raw_path), image,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        title = f'{event_type.upper()} | UAV{self.vehicle_id} | {target_name}'
        annotated = self._annotate(
            image, event.get('best_detections', []), title,
            event.get('best_expected'), event.get('best_report'))
        cv2.imwrite(str(annotated_path), annotated,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        metadata = {
            'event_type': event_type,
            'vehicle_id': self.vehicle_id,
            'target_name': target_name,
            'event': self._clean_payload(event),
            'raw_image': str(raw_path) if self.save_raw else None,
            'annotated_image': str(annotated_path),
        }
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
        return {
            'raw_image': str(raw_path) if self.save_raw else None,
            'annotated_image': str(annotated_path),
            'metadata_file': str(metadata_path),
        }

    def _publish(self, row: dict) -> None:
        self.event_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))
        self._publish_summary()

    def _publish_summary(self) -> None:
        row = self.summary()
        self.summary_pub.publish(String(data=json.dumps(row, ensure_ascii=False)))

    def _localization_summary(self, rows: Iterable[dict]) -> dict:
        values = np.asarray([float(row['horizontal_error_m']) for row in rows], dtype=float)
        if values.size == 0:
            return {
                'evaluated_reports': 0, 'mean_horizontal_error_m': None,
                'median_horizontal_error_m': None,
                'p95_horizontal_error_m': None, 'rmse_horizontal_error_m': None,
                'within_correct_threshold_ratio': None,
            }
        return {
            'evaluated_reports': int(values.size),
            'mean_horizontal_error_m': float(np.mean(values)),
            'median_horizontal_error_m': float(np.median(values)),
            'p95_horizontal_error_m': float(np.percentile(values, 95)),
            'rmse_horizontal_error_m': float(np.sqrt(np.mean(values ** 2))),
            'within_correct_threshold_ratio': float(np.mean(values <= self.correct_error)),
        }

    def _finalize_expected(self, name: str, reason: str) -> None:
        event = self.active_expected.pop(name, None)
        if not event:
            return
        duration = max(0.0, float(event['last_expected_ros']) - float(event['start_ros']))
        if (duration < self.min_visibility_seconds or
                int(event['expected_frames']) < self.min_expected_frames):
            self.ignored_short += 1
            self.counts['ignored_short_visibility'] += 1
            return
        if int(event['correct_report_count']) >= self.minimum_correct_reports:
            event_type = 'correct'
        elif int(event['report_count']) > 0:
            event_type = 'wrong_report'
        else:
            event_type = 'miss'
        self.counts[event_type] += 1
        if event_type == 'wrong_report':
            self.counts['false_positive'] += 1  # backward-compatible alias
        self.per_target.setdefault(name, collections.Counter())[event_type] += 1
        images = {'raw_image': None, 'annotated_image': None, 'metadata_file': None}
        if event_type != 'correct' or self.save_correct:
            images = self._save_pair(event_type, name, event)
        self._publish({
            'mission_id': self.mission_id,
            'vehicle_id': self.vehicle_id,
            'event_type': event_type,
            'target_name': name,
            'kind': event['kind'],
            'start_ros': event['start_ros'],
            'end_ros': event['last_expected_ros'],
            'duration_seconds': duration,
            'expected_frames': int(event['expected_frames']),
            'report_count': int(event['report_count']),
            'correct_report_count': int(event['correct_report_count']),
            'wrong_report_count': int(event['wrong_report_count']),
            'best_horizontal_localization_error_m': (
                None if not math.isfinite(float(event['best_error_m']))
                else float(event['best_error_m'])),
            'classification_reason': {
                'correct': 'stable report matched expected target and localization tolerance',
                'miss': 'valid target was expected but no effective report was emitted',
                'wrong_report': 'report class or localization was inconsistent with expected target',
            }[event_type],
            'termination_reason': reason,
            **images,
        })

    def _finalize_wrong(self, name: str, reason: str) -> None:
        event = self.active_wrong.pop(name, None)
        if not event:
            return
        if int(event['report_count']) < self.wrong_min_reports:
            self.counts['ignored_wrong_report_candidate'] += 1
            return
        self.counts['wrong_report'] += 1
        self.counts['false_positive'] += 1
        self.per_target.setdefault(name, collections.Counter())['wrong_report'] += 1
        images = self._save_pair('wrong_report', name, event)
        self._publish({
            'mission_id': self.mission_id,
            'vehicle_id': self.vehicle_id,
            'event_type': 'wrong_report',
            'target_name': name,
            'kind': event['kind'],
            'start_ros': event['start_ros'],
            'end_ros': event['last_report_ros'],
            'report_count': int(event['report_count']),
            'maximum_confidence': float(event['maximum_confidence']),
            'classification_reason': (
                'aircraft emitted a stable task-target report while that target '
                'was not valid in the evaluated camera FOV'),
            'termination_reason': reason,
            **images,
        })

    def update(self, image, detections: Sequence[dict], reports: Sequence[dict],
               ros_stamp: float, sequence: int, flight: dict,
               positions: Dict[str, np.ndarray], camera: dict,
               home_world: Optional[np.ndarray]) -> None:
        if not self.enabled or self.finalized:
            return
        vehicle = positions.get(f'standard_vtol_{self.vehicle_id}')
        if vehicle is None:
            vehicle = _finite_vector(flight.get('world_position', []), 3)
        if vehicle is None:
            return
        now = float(ros_stamp)
        allowed, excluded = self._allowed(
            flight, vehicle, home_world, float(camera.get('ground_z_m', 0.2)))
        if not allowed:
            for name in list(self.active_expected):
                self._finalize_expected(name, excluded)
            for name in list(self.active_wrong):
                self._finalize_wrong(name, excluded)
            return

        expected: Dict[str, dict] = {}
        for name in self.target_names:
            truth = positions.get(name)
            if truth is None:
                continue
            projection = project_world_to_image(
                vehicle, flight, camera, self.localization_cfg,
                truth, self.fov_margin)
            if projection.get('inside', False):
                expected[name] = {
                    **projection, 'target_name': name,
                    'kind': self.kind_map.get(name, ''),
                    'truth_world': np.asarray(truth, dtype=float).tolist(),
                }

        image_center = np.asarray([
            float(camera.get('cx', image.shape[1] / 2.0)),
            float(camera.get('cy', image.shape[0] / 2.0))])
        reports_by_name: Dict[str, List[dict]] = collections.defaultdict(list)
        for raw in reports or []:
            name = self._canonical(str(raw.get('target_name', '')))
            if name in self.target_names and float(raw.get('confidence', 0.0)) >= self.report_confidence:
                reports_by_name[name].append(dict(raw))

        for name, projection in expected.items():
            event = self.active_expected.setdefault(
                name, self._new_expected(name, self.kind_map.get(name, ''), now))
            event['last_expected_ros'] = now
            event['expected_frames'] += 1
            centre_distance = float(np.linalg.norm(
                np.asarray([projection['u'], projection['v']]) - image_center))
            if centre_distance < float(event['best_centre_distance_px']):
                event['best_centre_distance_px'] = centre_distance
                event['best_image'] = image.copy()
                event['best_detections'] = [dict(x) for x in detections or []]
                event['best_expected'] = dict(projection)
            truth = np.asarray(projection['truth_world'], dtype=float)
            for report in reports_by_name.get(name, []):
                estimated = _finite_vector(report.get('position_world', []), 3)
                if estimated is None:
                    continue
                horizontal = float(np.linalg.norm((estimated - truth)[:2]))
                three_d = float(np.linalg.norm(estimated - truth))
                report['validation_horizontal_error_m'] = horizontal
                report['validation_3d_error_m'] = three_d
                event['report_count'] += 1
                event['errors_m'].append(horizontal)
                if horizontal <= self.correct_error:
                    event['correct_report_count'] += 1
                else:
                    event['wrong_report_count'] += 1
                self.localization_errors.append({
                    'target_name': name, 'vehicle_id': self.vehicle_id,
                    'ros_time': now, 'horizontal_error_m': horizontal,
                    'error_3d_m': three_d,
                    'reported_position_world': estimated.tolist(),
                    'truth_position_world': truth.tolist(),
                    'report_event_id': report.get('report_event_id'),
                })
                if horizontal < float(event['best_error_m']):
                    event['best_error_m'] = horizontal
                    event['best_image'] = image.copy()
                    event['best_detections'] = [dict(x) for x in detections or []]
                    event['best_expected'] = dict(projection)
                    event['best_report'] = dict(report)

        for name, event in list(self.active_expected.items()):
            if name not in expected and now - float(event['last_expected_ros']) > self.event_gap_seconds:
                self._finalize_expected(name, 'target_left_expected_FOV')

        for name, rows in reports_by_name.items():
            if name in expected:
                if name in self.active_wrong:
                    self._finalize_wrong(name, 'target_became_expected')
                continue
            best_report = max(rows, key=lambda row: float(row.get('confidence', 0.0)))
            event = self.active_wrong.setdefault(
                name, self._new_wrong(name, self.kind_map.get(name, ''), now))
            event['last_report_ros'] = now
            event['report_count'] += len(rows)
            confidence = float(best_report.get('confidence', 0.0))
            if confidence >= float(event['maximum_confidence']):
                event['maximum_confidence'] = confidence
                event['best_image'] = image.copy()
                event['best_detections'] = [dict(x) for x in detections or []]
                event['best_report'] = dict(best_report)
        present_wrong = {name for name in reports_by_name if name not in expected}
        for name, event in list(self.active_wrong.items()):
            if name not in present_wrong and now - float(event['last_report_ros']) > self.wrong_gap:
                self._finalize_wrong(name, 'stable_wrong_report_sequence_ended')

    def finalize(self, reason: str = 'mission_complete') -> None:
        if not self.enabled or self.finalized:
            return
        for name in list(self.active_expected):
            self._finalize_expected(name, reason)
        for name in list(self.active_wrong):
            self._finalize_wrong(name, reason)
        self.finalized = True
        self._publish_summary()

    def summary(self) -> dict:
        per_target_metrics = {}
        for name in self.target_names:
            per_target_metrics[name] = {
                **dict(self.per_target.get(name, {})),
                'localization': self._localization_summary(
                    row for row in self.localization_errors
                    if row['target_name'] == name),
            }
        return {
            'mission_id': self.mission_id,
            'vehicle_id': self.vehicle_id,
            'enabled': self.enabled,
            'counts': {
                'correct': int(self.counts.get('correct', 0)),
                'miss': int(self.counts.get('miss', 0)),
                'wrong_report': int(self.counts.get('wrong_report', 0)),
                'false_positive': int(self.counts.get('false_positive', 0)),
                'ignored_wrong_report_candidate': int(
                    self.counts.get('ignored_wrong_report_candidate', 0)),
                'ignored_short_visibility': int(self.ignored_short),
            },
            'localization': self._localization_summary(self.localization_errors),
            'per_target': per_target_metrics,
            'active_expected_events': len(self.active_expected),
            'active_wrong_report_events': len(self.active_wrong),
            'event_level_not_frame_level': True,
            'event_directory': str(self.event_dir),
            'raw_and_annotated_image_pairs': self.save_raw,
            'correct_localization_error_m': self.correct_error,
            'truth_firewall': (
                'truth is used only by this validation evaluator and never by '
                'localization, management planning or flight control'),
        }
