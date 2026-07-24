#!/usr/bin/env python3
from pathlib import Path
import importlib.util
import math
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'zhihang_adaptive_search_v6/src/zhihang_adaptive_search_v6/tracking_recovery.py'
spec = importlib.util.spec_from_file_location('v6715_tracking_recovery', MODULE)
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)

bounds = {
    'safe_x_min': 100.0, 'safe_x_max': 1900.0,
    'safe_y_min': -900.0, 'safe_y_max': 900.0,
}
points = mod.generate_square_spiral_reacquisition_waypoints(
    [1000.0, 0.0, 0.2], 250.0, 90.0, 40.0, bounds,
    start=[1000.0, 0.0, 40.0])
assert len(points) >= 12
assert all(abs(p[2] - 40.0) < 1e-9 for p in points)
assert all(bounds['safe_x_min'] <= p[0] <= bounds['safe_x_max'] for p in points)
assert all(bounds['safe_y_min'] <= p[1] <= bounds['safe_y_max'] for p in points)
# Each generated segment is axis-aligned, proving square rather than serpentine
# diagonal geometry.
for a, b in zip(points, points[1:]):
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    assert dx < 1e-6 or dy < 1e-6
assert max(abs(p[0] - 1000.0) for p in points) >= 250.0
assert max(abs(p[1] - 0.0) for p in points) >= 250.0

moving = mod.SuvMotionGate()
for i in range(6):
    assert moving.add(
        [1000.0 + i, 0.0, 0.2], float(i), 0.8, 0.2, 1, f'm{i}')
moving_summary = moving.summary()
assert moving_summary['status'] == 'moving'
assert moving_summary['elapsed_seconds'] >= 5.0
assert moving_summary['speed_mps'] > 0.35

stationary = mod.SuvMotionGate()
for i in range(6):
    assert stationary.add(
        [1176.2 + 0.02 * i, -69.6, 0.2], float(i), 0.8, 0.2, 1, f's{i}')
stationary_summary = stationary.summary()
assert stationary_summary['status'] == 'stationary'
assert stationary_summary['displacement_m'] < 1.5
assert stationary_summary['speed_mps'] < 0.2

# Unreliable samples cannot force either result.
unreliable = mod.SuvMotionGate()
for i in range(10):
    assert not unreliable.add(
        [1000.0 + i, 0.0, 0.2], float(i), 0.4, 20.0, 2, f'u{i}')
assert unreliable.summary()['status'] == 'pending'

flight = (ROOT / 'zhihang_adaptive_search_v6/scripts/vehicle_flight_agent.py').read_text()
manager = (ROOT / 'zhihang_adaptive_search_v6/scripts/mission_manager.py').read_text()
estimator = (ROOT / 'zhihang_adaptive_search_v6/scripts/vision_target_state_estimator.py').read_text()

assert 'TRACK_MC_ALTITUDE_HOLD_CAPTURED' in flight
assert 'DYNAMIC_TARGET_MOTION_CONFIRMED' in flight
assert 'DYNAMIC_TARGET_STATIC_FALSE_POSITIVE' in flight
assert 'EXTERNAL_TARGET_FAST_APPROACH_START' in flight
assert 'MC_REACQUIRE_SQUARE_SPIRAL' in flight
assert 'generate_square_spiral_reacquisition_waypoints' in flight
assert 'self.command_yaw_rate' in flight

assert 'dynamic_assignment_confirmed' in manager
assert 'restore_dynamic_search_after_false_suv' in manager
assert "'skip_static_verify': True" in manager
assert "self.static_confirmed['prius_hybrid_camo']" in manager
assert 'tracking/reclassification' in manager

assert 'estimator_reinitialize_after_seconds' in estimator
assert 'tracking/reclassification' in estimator
assert 'external_injection_reliable' in estimator
assert '/gazebo/model_states' not in estimator

print('V6.7.15 MOTION/SPIRAL/ROLLBACK TEST PASSED')
