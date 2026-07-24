#!/usr/bin/env python3
from pathlib import Path
import sys
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'zhihang_adaptive_search_v6'
sys.path.insert(0, str(PKG / 'src'))
from zhihang_adaptive_search_v6.common import build_plan

cfg = yaml.safe_load((PKG / 'config/adaptive_search.yaml').read_text(encoding='utf-8'))
with tempfile.TemporaryDirectory() as td:
    cfg['mission']['output_root'] = td
    plan, routes = build_plan(cfg)

role = plan['role_policy']
assert role['dynamic_inspection_vehicle_ids'] == [0, 1]
assert role['maneuver_inspection_vehicle_id'] == 2
assert role['first_dynamic_tracker'] == 'maneuver_inspection_vehicle'
assert role['second_dynamic_tracker'] == 'detecting_free_dynamic_inspection_vehicle'
assert role['static_search_vehicle'] == 'remaining_dynamic_inspection_vehicle'

v2 = routes[2]
assert v2['role'] == 'MANEUVER_INSPECTION'
assert v2['route_id'] == 'maneuver_initial_east_west_north_to_south_then_square_outward'
stage = v2['initial_stage']
rows = stage['lane_y']
assert rows[0] == 650.0 and rows[-1] == -650.0
assert all(b < a for a, b in zip(rows, rows[1:]))
for i, leg in enumerate(stage['legs']):
    assert {leg['x_start'], leg['x_end']} == {350.0, 1650.0}
    assert leg['heading'] == ('east' if i % 2 == 0 else 'west')
assert all(
    not wp['detection_valid'] or wp['segment_type'] == 'SEARCH_STRAIGHT'
    for wp in v2['waypoints']
)
print('V6.7.11 ROLE POLICY AND MANEUVER ROUTE TEST PASSED')
