#!/usr/bin/env python3
"""Pure helpers for V6.7.19 dynamic tracking recovery and static verification.

This module deliberately has no ROS dependency so its geometry, filtering and
state-transition rules can be unit-tested without PX4/Gazebo.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


def wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_rate_command(current_yaw: float, target_yaw: float, kp: float,
                     maximum_rate: float, deadband_deg: float = 0.0) -> float:
    """Return bounded yaw-rate command for an ENU/world-yaw target."""
    error = wrap_pi(float(target_yaw) - float(current_yaw))
    if abs(math.degrees(error)) <= float(deadband_deg):
        return 0.0
    return float(np.clip(float(kp) * error, -abs(float(maximum_rate)),
                         abs(float(maximum_rate))))


def first_order_alpha(dt: float, time_constant: float) -> float:
    dt = max(0.0, float(dt))
    tau = max(1e-6, float(time_constant))
    return 1.0 - math.exp(-dt / tau)


@dataclass
class DynamicTargetFilter:
    """Lightweight position/velocity filter with jump rejection.

    Position is exponentially smoothed. Velocity is derived from the filtered
    position and blended with the reported velocity, avoiding the large noisy
    derivative term that previously drove MC attitude oscillations.
    """

    position_tau: float = 0.8
    velocity_tau: float = 1.2
    jump_gate_m: float = 35.0
    position: Optional[np.ndarray] = None
    velocity: Optional[np.ndarray] = None
    stamp: Optional[float] = None

    def reset(self, position: Sequence[float], velocity: Sequence[float],
              stamp: float) -> None:
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = np.asarray(velocity, dtype=float).copy()
        self.stamp = float(stamp)

    def update(self, position: Sequence[float], velocity: Sequence[float],
               stamp: float) -> Tuple[np.ndarray, np.ndarray, bool]:
        p = np.asarray(position, dtype=float)
        v_measured = np.asarray(velocity, dtype=float)
        t = float(stamp)
        if self.position is None or self.velocity is None or self.stamp is None:
            self.reset(p, v_measured, t)
            return self.position.copy(), self.velocity.copy(), True
        dt = max(1e-3, min(2.0, t - float(self.stamp)))
        predicted = self.position + self.velocity * dt
        jump = float(np.linalg.norm((p - predicted)[:2]))
        accepted = jump <= max(1.0, float(self.jump_gate_m))
        if not accepted:
            # A single inconsistent geolocation must not kick the aircraft.
            self.stamp = t
            return self.position.copy(), self.velocity.copy(), False
        a_p = first_order_alpha(dt, self.position_tau)
        previous = self.position.copy()
        self.position = predicted + a_p * (p - predicted)
        derived = (self.position - previous) / dt
        a_v = first_order_alpha(dt, self.velocity_tau)
        blended_measurement = 0.55 * v_measured + 0.45 * derived
        self.velocity = self.velocity + a_v * (blended_measurement - self.velocity)
        self.stamp = t
        return self.position.copy(), self.velocity.copy(), True

    def predict(self, stamp: float, maximum_horizon: float) -> Tuple[np.ndarray, np.ndarray]:
        if self.position is None or self.velocity is None or self.stamp is None:
            raise RuntimeError('dynamic target filter has no state')
        horizon = min(max(0.0, float(stamp) - float(self.stamp)),
                      max(0.0, float(maximum_horizon)))
        return self.position + self.velocity * horizon, self.velocity.copy()


def possible_target_radius(elapsed_seconds: float, target_speed_bound_mps: float,
                           base_uncertainty_m: float, maximum_radius_m: float,
                           localization_std_m: float = 0.0) -> float:
    """Conservative horizontal uncertainty radius after the last valid report."""
    radius = (max(0.0, float(base_uncertainty_m))
              + max(0.0, float(target_speed_bound_mps)) * max(0.0, float(elapsed_seconds))
              + max(0.0, float(localization_std_m)))
    return min(max(0.0, float(maximum_radius_m)), radius)


def clamp_xy(point: Sequence[float], bounds: Dict[str, float]) -> np.ndarray:
    p = np.asarray(point, dtype=float).copy()
    p[0] = np.clip(p[0], float(bounds['safe_x_min']), float(bounds['safe_x_max']))
    p[1] = np.clip(p[1], float(bounds['safe_y_min']), float(bounds['safe_y_max']))
    return p


def generate_mc_reacquisition_waypoints(
        center: Sequence[float], radius_m: float, lane_spacing_m: float,
        altitude_m: float, bounds: Dict[str, float],
        start: Optional[Sequence[float]] = None) -> List[List[float]]:
    """Generate a fast serpentine coverage pattern over a circular uncertainty set.

    Rows are clipped to the uncertainty circle and to the safe search rectangle.
    The first row is chosen near the aircraft to reduce dead transit.  All points
    are at the requested MC altitude.
    """
    c = np.asarray(center, dtype=float).copy()
    c = clamp_xy(c, bounds)
    radius = max(float(lane_spacing_m) * 0.5, float(radius_m))
    spacing = max(5.0, float(lane_spacing_m))
    y_min = max(float(bounds['safe_y_min']), c[1] - radius)
    y_max = min(float(bounds['safe_y_max']), c[1] + radius)
    ys: List[float] = []
    y = y_max
    while y >= y_min - 1e-6:
        ys.append(max(y_min, y))
        y -= spacing
    if not ys or abs(ys[-1] - y_min) > 1e-6:
        ys.append(y_min)
    rows: List[Tuple[float, float, float]] = []
    for y in ys:
        dy = y - c[1]
        half = math.sqrt(max(0.0, radius * radius - dy * dy))
        x0 = max(float(bounds['safe_x_min']), c[0] - half)
        x1 = min(float(bounds['safe_x_max']), c[0] + half)
        if x1 - x0 < 1.0:
            continue
        rows.append((y, x0, x1))
    if start is not None and rows:
        s = np.asarray(start, dtype=float)
        nearest = min(range(len(rows)), key=lambda i: abs(rows[i][0] - s[1]))
        rows = rows[nearest:] + list(reversed(rows[:nearest]))
    points: List[List[float]] = []
    direction_east = True
    if start is not None and rows:
        s = np.asarray(start, dtype=float)
        direction_east = abs(s[0] - rows[0][1]) <= abs(s[0] - rows[0][2])
    for y, x0, x1 in rows:
        a, b = (x0, x1) if direction_east else (x1, x0)
        points.append([float(a), float(y), float(altitude_m)])
        points.append([float(b), float(y), float(altitude_m)])
        direction_east = not direction_east
    return points


def static_hover_point(target_world: Sequence[float], target_name: str,
                       hover_altitude_m: float,
                       offset_map: Optional[Dict[str, Sequence[float]]] = None) -> np.ndarray:
    """Build the frozen hover setpoint, applying per-target horizontal offsets."""
    target = np.asarray(target_world, dtype=float)
    hover = target.copy()
    offset = (offset_map or {}).get(str(target_name), [0.0, 0.0])
    if len(offset) >= 2:
        hover[0] += float(offset[0])
        hover[1] += float(offset[1])
    hover[2] = float(target[2]) + float(hover_altitude_m)
    return hover


def weighted_position_fusion(reports: Iterable[dict], target_name: str,
                             minimum_reports: int, maximum_std_m: float) -> Optional[dict]:
    """Fuse post-hover YOLO geolocation reports; return None when unreliable."""
    rows = [dict(r) for r in reports
            if str(r.get('target_name', '')) == str(target_name)
            and isinstance(r.get('position_world'), list)
            and len(r.get('position_world')) >= 3]
    if len(rows) < max(1, int(minimum_reports)):
        return None
    xyz = np.asarray([r['position_world'][:3] for r in rows], dtype=float)
    weights = np.asarray([max(1e-3, float(r.get('confidence', 0.0))) for r in rows], dtype=float)
    mean = np.average(xyz, axis=0, weights=weights)
    std = np.sqrt(np.average((xyz - mean) ** 2, axis=0, weights=weights))
    horizontal_std = float(math.hypot(float(std[0]), float(std[1])))
    if horizontal_std > float(maximum_std_m):
        return None
    return {
        'position_world': mean.tolist(),
        'position_std_m': std.tolist(),
        'horizontal_std_m': horizontal_std,
        'report_count': len(rows),
        'mean_confidence': float(np.average(
            np.asarray([float(r.get('confidence', 0.0)) for r in rows]), weights=weights)),
        'report_event_ids': [r.get('report_event_id') for r in rows],
    }

@dataclass
class SuvMotionGate:
    """Five-second motion validation for the visually ambiguous ``suv_camo``.

    Only reliable, localized and timestamp-unique measurements are admitted.
    The result is intentionally conservative: a target is locked as dynamic only
    after the configured displacement or speed threshold is exceeded.  After the
    observation window, anything below those moving thresholds is classified as
    stationary so the static ``prius_hybrid_camo`` false-positive cannot consume
    a dynamic-tracker slot for the rest of the mission.
    """

    observation_seconds: float = 5.0
    minimum_unique_samples: int = 5
    minimum_confidence: float = 0.60
    maximum_horizontal_std_m: float = 5.0
    moving_min_displacement_m: float = 2.5
    moving_min_speed_mps: float = 0.35
    stationary_max_displacement_m: float = 1.5
    stationary_max_speed_mps: float = 0.20
    samples: Optional[List[dict]] = None

    def __post_init__(self) -> None:
        if self.samples is None:
            self.samples = []

    def add(self, position: Sequence[float], stamp: float, confidence: float,
            horizontal_std_m: float, source_vehicle_id: Optional[int] = None,
            measurement_id: Optional[str] = None) -> bool:
        p = np.asarray(position, dtype=float)
        if p.size < 3 or not np.all(np.isfinite(p[:3])):
            return False
        if float(confidence) < float(self.minimum_confidence):
            return False
        if float(horizontal_std_m) > float(self.maximum_horizontal_std_m):
            return False
        key = str(measurement_id or f'{float(stamp):.6f}:{source_vehicle_id}')
        if any(str(row['measurement_id']) == key for row in self.samples or []):
            return False
        self.samples.append({
            'position': p[:3].copy(),
            'stamp': float(stamp),
            'confidence': float(confidence),
            'horizontal_std_m': float(horizontal_std_m),
            'source_vehicle_id': source_vehicle_id,
            'measurement_id': key,
        })
        self.samples.sort(key=lambda row: float(row['stamp']))
        if len(self.samples) > 200:
            del self.samples[:-200]
        return True

    def elapsed(self) -> float:
        if not self.samples:
            return 0.0
        return max(0.0, float(self.samples[-1]['stamp']) -
                   float(self.samples[0]['stamp']))

    def summary(self) -> dict:
        rows = list(self.samples or [])
        if not rows:
            return {
                'status': 'pending', 'sample_count': 0, 'elapsed_seconds': 0.0,
                'displacement_m': 0.0, 'speed_mps': 0.0,
                'position_world': None,
            }
        xyz = np.asarray([row['position'] for row in rows], dtype=float)
        stamps = np.asarray([row['stamp'] for row in rows], dtype=float)
        weights = np.asarray([max(1e-3, row['confidence']) for row in rows],
                             dtype=float)
        fused = np.average(xyz, axis=0, weights=weights)

        group = max(1, min(len(rows) // 3, 5))
        first = np.median(xyz[:group, :2], axis=0)
        last = np.median(xyz[-group:, :2], axis=0)
        displacement = float(np.linalg.norm(last - first))
        elapsed = max(0.0, float(stamps[-1] - stamps[0]))

        speed = 0.0
        if len(rows) >= 2 and elapsed > 1e-3:
            # Robust linear slope for x and y.  Centering prevents poor numerical
            # conditioning when ROS simulation time is large.
            t = stamps - float(np.mean(stamps))
            denom = float(np.dot(t, t))
            if denom > 1e-9:
                vx = float(np.dot(t, xyz[:, 0] - float(np.mean(xyz[:, 0]))) / denom)
                vy = float(np.dot(t, xyz[:, 1] - float(np.mean(xyz[:, 1]))) / denom)
                speed = math.hypot(vx, vy)

        enough = (len(rows) >= max(2, int(self.minimum_unique_samples))
                  and elapsed >= float(self.observation_seconds))
        moving = (displacement >= float(self.moving_min_displacement_m)
                  or speed >= float(self.moving_min_speed_mps))
        clearly_stationary = (
            displacement <= float(self.stationary_max_displacement_m)
            and speed <= float(self.stationary_max_speed_mps))
        status = 'pending'
        if enough:
            # Only positive motion evidence locks a dynamic tracker.  An
            # ambiguous five-second history is treated as stationary.
            status = 'moving' if moving else 'stationary'
        return {
            'status': status,
            'sample_count': len(rows),
            'elapsed_seconds': elapsed,
            'displacement_m': displacement,
            'speed_mps': speed,
            'clearly_stationary': clearly_stationary,
            'position_world': fused.tolist(),
            'mean_confidence': float(np.average(
                np.asarray([row['confidence'] for row in rows]), weights=weights)),
            'horizontal_std_m': float(math.hypot(
                float(np.std(xyz[:, 0])), float(np.std(xyz[:, 1])))),
            'first_stamp': float(stamps[0]),
            'last_stamp': float(stamps[-1]),
            'source_vehicle_ids': sorted({
                int(row['source_vehicle_id']) for row in rows
                if row.get('source_vehicle_id') is not None
            }),
        }


def generate_square_spiral_reacquisition_waypoints(
        center: Sequence[float], radius_m: float, spacing_m: float,
        altitude_m: float, bounds: Dict[str, float],
        start: Optional[Sequence[float]] = None) -> List[List[float]]:
    """Generate a centre-out, equal-spacing square spiral.

    Consecutive straight legs form an Archimedean-like square spiral whose
    adjacent arms are separated by ``spacing_m``.  The spiral reaches at least
    ``radius_m`` in both horizontal axes, is clipped to the safe rectangle and
    keeps a continuous point order suitable for tangent-yaw guidance.
    """
    c = clamp_xy(center, bounds)
    spacing = max(5.0, float(spacing_m))
    radius = max(0.5 * spacing, float(radius_m))
    layers = max(1, int(math.ceil(radius / spacing)))

    # Select the first direction that points most nearly from the spiral centre
    # toward the aircraft.  This minimizes the heading reversal at the centre
    # without changing the 90 m arm spacing.
    directions = [
        np.asarray([1.0, 0.0]), np.asarray([0.0, 1.0]),
        np.asarray([-1.0, 0.0]), np.asarray([0.0, -1.0]),
    ]
    if start is not None:
        delta = np.asarray(start, dtype=float)[:2] - c[:2]
        if float(np.linalg.norm(delta)) > 1e-6:
            first = int(np.argmax([float(np.dot(delta, d)) for d in directions]))
            directions = directions[first:] + directions[:first]

    raw: List[np.ndarray] = [np.asarray([c[0], c[1], float(altitude_m)], dtype=float)]
    current = raw[0].copy()
    step_units = 1
    direction_index = 0
    # Two legs at each length: 1,1,2,2,3,3,...  Four extra legs close the
    # outermost square even when the target radius is not a spacing multiple.
    maximum_leg_units = 2 * layers + 1
    while step_units <= maximum_leg_units:
        for _ in range(2):
            d = directions[direction_index % 4]
            direction_index += 1
            current = current.copy()
            current[0] += d[0] * step_units * spacing
            current[1] += d[1] * step_units * spacing
            current[2] = float(altitude_m)
            raw.append(current.copy())
        step_units += 1

    points: List[List[float]] = []
    for point in raw:
        clipped = clamp_xy(point, bounds)
        clipped[2] = float(altitude_m)
        if points and np.linalg.norm(clipped[:2] - np.asarray(points[-1])[:2]) < 1.0:
            continue
        points.append([float(clipped[0]), float(clipped[1]), float(clipped[2])])

    # Safe-boundary clipping can collapse outer corners.  Remove immediate
    # A-B-A reversals, which otherwise waste time without adding coverage.
    cleaned: List[List[float]] = []
    for point in points:
        if len(cleaned) >= 2 and np.linalg.norm(
                np.asarray(point)[:2] - np.asarray(cleaned[-2])[:2]) < 1.0:
            cleaned.pop()
            continue
        cleaned.append(point)
    return cleaned


def trajectory_yaw(current: Sequence[float], desired: Sequence[float],
                   fallback_yaw: float = 0.0) -> float:
    """Heading tangent to the current world-frame trajectory segment."""
    delta = np.asarray(desired, dtype=float)[:2] - np.asarray(current, dtype=float)[:2]
    if float(np.linalg.norm(delta)) < 0.25:
        return float(fallback_yaw)
    return math.atan2(float(delta[1]), float(delta[0]))

