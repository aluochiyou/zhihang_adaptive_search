#!/usr/bin/env python3
from pathlib import Path
import ast
import yaml

root = Path(__file__).resolve().parents[1]
pkg = root / 'zhihang_adaptive_search_v6'
flight_path = pkg / 'scripts/vehicle_flight_agent.py'
flight = flight_path.read_text(encoding='utf-8')
ast.parse(flight)

for name in ('adaptive_search.yaml', 'adaptive_search_formal.yaml'):
    cfg = yaml.safe_load((pkg / 'config' / name).read_text(encoding='utf-8'))
    assert float(cfg['tracking']['intercept_altitude_m']) == 40.0
    assert float(cfg['tracking']['transition_to_mc_radius_m']) == 30.0
    # Static verification remains independently configured at 30 m.
    assert float(cfg['static_verify']['transition_to_mc_radius_m']) == 30.0

assert "'TRACK_FW_30M_TRANSITION_GATE_REACHED'" in flight
assert "'transition_to_mc_radius_m', 30.0" in flight
assert 'horizontal_distance_m <= transition_radius_m' in flight
print('V6.7.16 DYNAMIC 30M TRANSITION GATE TEST PASSED')
