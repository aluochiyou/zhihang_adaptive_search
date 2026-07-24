#!/usr/bin/env python3
from __future__ import annotations

import ast
import hashlib
import json
import math
import os
import random
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def dump_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    with open(tmp, 'w', encoding='utf-8') as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def canonical_json(payload: dict) -> str:
    return json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(',', ':'))


def payload_checksum(payload: dict) -> str:
    clean = dict(payload)
    clean.pop('checksum', None)
    return hashlib.sha256(canonical_json(clean).encode('utf-8')).hexdigest()


def validate_packet(packet: dict, vehicle_id: Optional[int] = None) -> None:
    if not isinstance(packet, dict):
        raise ValueError('task packet must be a dictionary')
    if int(packet.get('schema_version', 0)) != 1:
        raise ValueError(f"unsupported task packet schema={packet.get('schema_version')}")
    if vehicle_id is not None and int(packet.get('vehicle_id', -1)) != int(vehicle_id):
        raise ValueError(f"vehicle mismatch {packet.get('vehicle_id')} != {vehicle_id}")
    expected = str(packet.get('checksum', ''))
    actual = payload_checksum(packet)
    if not expected or expected != actual:
        raise ValueError(f'task packet checksum mismatch expected={expected} actual={actual}')


def _eval_number(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return float(node.value)
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return -_eval_number(node.operand)
    if isinstance(node, ast.BinOp):
        a, b = _eval_number(node.left), _eval_number(node.right)
        if isinstance(node.op, ast.Add): return a + b
        if isinstance(node.op, ast.Sub): return a - b
        if isinstance(node.op, ast.Mult): return a * b
        if isinstance(node.op, ast.Div): return a / b
    raise ValueError(ast.dump(node))


def parse_model_state(path: str) -> dict:
    path = os.path.expanduser(path)
    text = Path(path).read_text(encoding='utf-8')
    tree = ast.parse(text)
    values = {}
    proximity = []
    for node in tree.body:
        if not (isinstance(node, ast.Assign) and len(node.targets) == 1):
            continue
        target = node.targets[0]
        # model_state.py uses both NAME=value and tuple unpacking such as
        # X_MIN, X_MAX = 0, 2000. Support both without executing the file.
        if isinstance(target, ast.Name):
            key = target.id
            try:
                values[key] = _eval_number(node.value)
            except Exception:
                if key == 'PROXIMITY_REQUIREMENTS' and isinstance(node.value, (ast.List, ast.Tuple)):
                    for item in node.value.elts:
                        if isinstance(item, (ast.Tuple, ast.List)) and len(item.elts) == 3:
                            try:
                                proximity.append([ast.literal_eval(item.elts[0]), ast.literal_eval(item.elts[1]),
                                                  float(_eval_number(item.elts[2]))])
                            except Exception:
                                pass
        elif isinstance(target, (ast.Tuple, ast.List)) and isinstance(node.value, (ast.Tuple, ast.List)):
            if len(target.elts) == len(node.value.elts):
                for left, right in zip(target.elts, node.value.elts):
                    if isinstance(left, ast.Name):
                        try:
                            values[left.id] = _eval_number(right)
                        except Exception:
                            pass
    values['PROXIMITY_REQUIREMENTS'] = proximity
    return values



def derive_model_region(cfg: dict) -> dict:
    area = cfg['search_area']
    path = os.path.expanduser(str(area.get('model_state_path', '')))
    fallback_safe = {
        'x_min': float(area['safe_x_min']), 'x_max': float(area['safe_x_max']),
        'y_min': float(area['safe_y_min']), 'y_max': float(area['safe_y_max']),
    }
    fallback_start = {
        'x_min': float(area['initial_x_min']), 'x_max': float(area['initial_x_max']),
        'y_min': float(area['initial_y_min']), 'y_max': float(area['initial_y_max']),
    }
    report = {
        'model_state_path': path,
        'used_fallback': True,
        'target_speed_mps': float(cfg['dynamic_search']['target_speed_mps']),
        'safe_bounds': fallback_safe,
        'dynamic_start_bbox': fallback_start,
        'maximum_static_offset_m': 300.0,
        'curve_target_length_m': 2100.0,
        'grid_definition': {
            'x_min': 0.0, 'x_max': 2000.0, 'y_min': -1000.0, 'y_max': 1000.0,
            'cols': 8, 'rows': 8, 'min_col': 1, 'max_col': 6,
            'min_row': 1, 'max_row': 6, 'inner_margin_m': 100.0,
            'min_gap_cells': 3,
        },
        'proximity_requirements': [
            ['person_red','car_lexus',200.0], ['person_red','prius_hybrid_camo',300.0],
            ['suv_camo','person_white',200.0], ['suv_camo','car_opel',300.0],
            ['suv','prius_hybrid',200.0], ['suv','fire_truck',300.0],
        ],
    }
    if not path or not os.path.isfile(path):
        report['reason'] = 'model_state.py unavailable; configured fallback used'
        return report
    try:
        c = parse_model_state(path)
        required = ['X_MIN','X_MAX','Y_MIN','Y_MAX','BOUNDARY_MARGIN','GRID_COLS','GRID_ROWS',
                    'MIN_COL','MAX_COL','MIN_ROW','MAX_ROW','SPEED']
        missing = [k for k in required if k not in c]
        if missing:
            raise ValueError(f'missing constants {missing}')
        cell_x = (c['X_MAX'] - c['X_MIN']) / c['GRID_COLS']
        cell_y = (c['Y_MAX'] - c['Y_MIN']) / c['GRID_ROWS']
        margin = c['BOUNDARY_MARGIN']
        safe = {
            'x_min': c['X_MIN'] + margin,
            'x_max': c['X_MAX'] - margin,
            'y_min': c['Y_MIN'] + margin,
            'y_max': c['Y_MAX'] - margin,
        }
        start = {
            'x_min': c['X_MIN'] + c['MIN_COL'] * cell_x + margin,
            'x_max': c['X_MIN'] + (c['MAX_COL'] + 1) * cell_x - margin,
            'y_min': c['Y_MIN'] + c['MIN_ROW'] * cell_y + margin,
            'y_max': c['Y_MIN'] + (c['MAX_ROW'] + 1) * cell_y - margin,
        }
        proximity = c.get('PROXIMITY_REQUIREMENTS', [])
        max_offset = max([float(x[2]) for x in proximity] or [300.0])
        report.update({
            'used_fallback': False,
            'target_speed_mps': float(c['SPEED']),
            'safe_bounds': safe,
            'dynamic_start_bbox': start,
            'maximum_static_offset_m': max_offset,
            'proximity_requirements': proximity,
            'grid_definition': {
                'x_min': float(c['X_MIN']), 'x_max': float(c['X_MAX']),
                'y_min': float(c['Y_MIN']), 'y_max': float(c['Y_MAX']),
                'cols': int(c['GRID_COLS']), 'rows': int(c['GRID_ROWS']),
                'min_col': int(c['MIN_COL']), 'max_col': int(c['MAX_COL']),
                'min_row': int(c['MIN_ROW']), 'max_row': int(c['MAX_ROW']),
                'inner_margin_m': float(margin),
                'min_gap_cells': int(c.get('GRID_MIN_GAP', 3)),
                'cell_size_x_m': float(cell_x), 'cell_size_y_m': float(cell_y),
            },
            'reason': 'derived from model_state constants, candidate start-cell cores, and exact static offsets',
        })
    except Exception as exc:
        report['reason'] = f'parse failed: {type(exc).__name__}: {exc}; fallback used'
    return report


def _initial_start_core_rectangles(model_report: dict) -> List[dict]:
    """Return all public candidate start-cell cores from model_state constants.

    This is a mission prior, not a target-state observation. Each core is the
    cell interior left after BOUNDARY_MARGIN is applied by model_state.py.
    """
    grid = model_report['grid_definition']
    x0, y0 = float(grid['x_min']), float(grid['y_min'])
    sx = float(grid.get('cell_size_x_m',
                        (float(grid['x_max']) - x0) / int(grid['cols'])))
    sy = float(grid.get('cell_size_y_m',
                        (float(grid['y_max']) - y0) / int(grid['rows'])))
    margin = float(grid['inner_margin_m'])
    rows: List[dict] = []
    for col in range(int(grid['min_col']), int(grid['max_col']) + 1):
        for row in range(int(grid['min_row']), int(grid['max_row']) + 1):
            rows.append({
                'col': col, 'row': row,
                'x_min': x0 + col * sx + margin,
                'x_max': x0 + (col + 1) * sx - margin,
                'y_min': y0 + row * sy + margin,
                'y_max': y0 + (row + 1) * sy - margin,
            })
    return rows


def _distance_to_rectangle_xy(x: float, y: float, rect: dict) -> float:
    dx = max(float(rect['x_min']) - x, 0.0, x - float(rect['x_max']))
    dy = max(float(rect['y_min']) - y, 0.0, y - float(rect['y_max']))
    return math.hypot(dx, dy)


def derive_search_region_model(model_report: dict) -> dict:
    """Define simulation, initial, moving-distribution and static-risk regions.

    Definitions come only from public model_state.py constants and constraints:
    * simulation region: X_MIN..X_MAX, Y_MIN..Y_MAX;
    * initial region: the union of legal start-cell cores;
    * distribution region: safe bounds within which every translated curve is
      checked by check_curve_in_bounds();
    * static high-risk support: union of legal start cores dilated by the
      maximum configured static offset (300 m in the supplied task).

    The high-risk support is graded: the 200 m dilation is primary risk and the
    200..300 m fringe is secondary risk. This preserves the complete possible
    static support without pretending the whole bounding rectangle is equally
    likely.
    """
    grid = model_report['grid_definition']
    safe = dict(model_report['safe_bounds'])
    initial = dict(model_report['dynamic_start_bbox'])
    cores = _initial_start_core_rectangles(model_report)
    max_offset = float(model_report.get('maximum_static_offset_m', 300.0))
    offsets = sorted({float(row[2]) for row in model_report.get('proximity_requirements', [])})
    primary_offset = min(offsets) if offsets else min(200.0, max_offset)
    return {
        'simulation_region': {
            'x_min': float(grid['x_min']), 'x_max': float(grid['x_max']),
            'y_min': float(grid['y_min']), 'y_max': float(grid['y_max']),
            'source': 'model_state_world_bounds',
        },
        'dynamic_initial_region': {
            **initial, 'shape': 'union_of_start_cell_cores',
            'core_count': len(cores), 'cores': cores,
            'source': 'model_state_candidate_start_cells_after_margin',
        },
        'dynamic_distribution_region': {
            **safe, 'shape': 'safe_rectangle',
            'source': 'model_state_check_curve_in_bounds',
        },
        'static_high_risk_region': {
            **safe, 'shape': 'union_of_dilated_start_cell_cores',
            'primary_offset_m': primary_offset,
            'maximum_offset_m': max_offset,
            'source': 'model_state_start_core_union_plus_static_offsets',
        },
    }


def _static_risk_priority_at_xy(model_report: dict, x: float, y: float) -> Tuple[float, str]:
    region_model = model_report.get('search_region_model') or derive_search_region_model(model_report)
    risk = region_model['static_high_risk_region']
    primary = float(risk['primary_offset_m'])
    maximum = float(risk['maximum_offset_m'])
    distance = min(_distance_to_rectangle_xy(float(x), float(y), rect)
                   for rect in region_model['dynamic_initial_region']['cores'])
    if distance <= primary + 1e-9:
        return 1.0, 'derived_static_primary_support'
    if distance <= maximum + 1e-9:
        return 0.72, 'derived_static_outer_support'
    return 0.0, ''


def _static_primary_half_span(model_report: dict) -> float:
    region_model = model_report.get('search_region_model') or derive_search_region_model(model_report)
    initial = region_model['dynamic_initial_region']
    distribution = region_model['dynamic_distribution_region']
    offset = float(region_model['static_high_risk_region']['primary_offset_m'])
    return min(max(abs(float(distribution['y_min'])), abs(float(distribution['y_max']))),
               max(abs(float(initial['y_min']) - offset),
                   abs(float(initial['y_max']) + offset)))


def _quarter_turn_points(corner: Sequence[float], incoming: str, outgoing: str,
                         radius: float, altitude: float, samples: int,
                         role: str, leg_id: str) -> List[dict]:
    """Small rounded-corner guide with only the configured intermediate points."""
    cx, cy = float(corner[0]), float(corner[1])
    mapping = {
        ('E', 'S'): (math.pi / 2.0, 0.0, cx - radius, cy + radius),
        ('S', 'W'): (0.0, -math.pi / 2.0, cx - radius, cy - radius),
        ('W', 'N'): (-math.pi / 2.0, -math.pi, cx + radius, cy - radius),
        ('N', 'E'): (math.pi, math.pi / 2.0, cx + radius, cy + radius),
        ('E', 'N'): (-math.pi / 2.0, 0.0, cx - radius, cy - radius),
        ('N', 'W'): (0.0, math.pi / 2.0, cx - radius, cy + radius),
        ('W', 'S'): (math.pi / 2.0, math.pi, cx + radius, cy + radius),
        ('S', 'E'): (math.pi, 3.0 * math.pi / 2.0, cx + radius, cy - radius),
    }
    if (incoming, outgoing) not in mapping or radius <= 0.0:
        return []
    a0, a1, ox, oy = mapping[(incoming, outgoing)]
    angles = np.linspace(a0, a1, max(1, int(samples)) + 2)[1:-1]
    return [_wp([ox + radius * math.cos(float(a)),
                 oy + radius * math.sin(float(a)), altitude],
                'TURN_GUIDE', False, leg_id, role) for a in angles]

def camera_footprint(camera: dict, altitude_m: float, ground_z_m: Optional[float] = None) -> dict:
    ground = float(camera['ground_z_m'] if ground_z_m is None else ground_z_m)
    h = max(0.1, float(altitude_m) - ground)
    half_right = h * (float(camera['width']) / 2.0) / float(camera['fx'])
    half_forward = h * (float(camera['height']) / 2.0) / float(camera['fy'])
    return {
        'altitude_m': float(altitude_m),
        'height_above_ground_m': h,
        'half_right_m': half_right,
        'half_forward_m': half_forward,
        'width_m': 2.0 * half_right,
        'length_m': 2.0 * half_forward,
        'minor_dimension_m': 2.0 * min(half_right, half_forward),
    }


def fov_project_nadir(vehicle_xyz: Sequence[float], vehicle_yaw: float,
                      target_xyz: Sequence[float], camera: dict,
                      margin_ratio: float = 0.90) -> dict:
    """Approximate target projection for a near-nadir camera.

    Search detections are accepted only on straight, near-level FW legs, so roll
    and pitch are intentionally ignored in this proxy. The real YOLO projection
    remains a separate calibrated component.
    """
    vehicle = np.asarray(vehicle_xyz, dtype=float)
    target = np.asarray(target_xyz, dtype=float)
    height = float(vehicle[2] - target[2])
    if height <= 0.5:
        return {'inside': False, 'reason': 'nonpositive camera height'}
    dx, dy = float(target[0] - vehicle[0]), float(target[1] - vehicle[1])
    cy, sy = math.cos(vehicle_yaw), math.sin(vehicle_yaw)
    forward = cy * dx + sy * dy
    right = -sy * dx + cy * dy
    u = float(camera['cx']) + float(camera.get('u_right_sign', 1.0)) * right * float(camera['fx']) / height
    v = float(camera['cy']) + float(camera.get('v_forward_sign', -1.0)) * forward * float(camera['fy']) / height
    width, image_h = float(camera['width']), float(camera['height'])
    mx = (1.0 - float(margin_ratio)) * width / 2.0
    my = (1.0 - float(margin_ratio)) * image_h / 2.0
    inside = mx <= u <= width - mx and my <= v <= image_h - my
    return {
        'inside': bool(inside), 'u': u, 'v': v, 'height_m': height,
        'forward_m': forward, 'right_m': right,
        'pixel_bounds': [mx, width - mx, my, image_h - my],
    }


def _wp(point, segment_type: str, valid: bool, leg_id: str, role: str) -> dict:
    return {
        'point': [float(x) for x in point],
        'segment_type': str(segment_type),
        'detection_valid': bool(valid),
        'leg_id': str(leg_id),
        'role': str(role),
    }


def _semicircle_vertical_turn(x0: float, x1: float, y: float, direction: int,
                              altitude: float, samples: int, role: str, leg_id: str) -> List[dict]:
    """Connect two vertical parallel legs with a tangent semicircle.

    direction=+1 means the current leg ended while flying north; the arc bulges
    north and arrives at x1 heading south. direction=-1 is the mirrored case.
    """
    center_x = (x0 + x1) / 2.0
    radius = abs(x1 - x0) / 2.0
    if radius < 1e-6:
        return []
    if x1 > x0:
        angles = np.linspace(math.pi, 0.0, max(1, samples) + 2)[1:]
    else:
        angles = np.linspace(0.0, math.pi, max(1, samples) + 2)[1:]
    out = []
    for a in angles:
        yy = y + direction * radius * math.sin(float(a))
        xx = center_x + radius * math.cos(float(a))
        out.append(_wp([xx, yy, altitude], 'TURN_GUIDE', False, leg_id, role))
    return out


def _semicircle_horizontal_turn(y0: float, y1: float, x: float, direction: int,
                                altitude: float, samples: int, role: str, leg_id: str) -> List[dict]:
    """Connect two horizontal parallel legs with a tangent semicircle.

    ``direction=+1`` means the current leg ended while flying east and the arc
    bulges east before arriving on the next row heading west. ``direction=-1``
    is the mirrored west-bulging manoeuvre.  The helper is used by the
    manoeuvre-inspection aircraft so its initial east-west raster remains
    orthogonal to the two north-south dynamic-inspection routes.
    """
    center_y = (y0 + y1) / 2.0
    radius = abs(y1 - y0) / 2.0
    if radius < 1e-6:
        return []
    if y1 < y0:
        angles = (
            np.linspace(math.pi / 2.0, -math.pi / 2.0, max(1, samples) + 2)[1:]
            if direction > 0 else
            np.linspace(math.pi / 2.0, 3.0 * math.pi / 2.0, max(1, samples) + 2)[1:]
        )
    else:
        angles = (
            np.linspace(-math.pi / 2.0, math.pi / 2.0, max(1, samples) + 2)[1:]
            if direction > 0 else
            np.linspace(3.0 * math.pi / 2.0, math.pi / 2.0, max(1, samples) + 2)[1:]
        )
    out = []
    for a in angles:
        xx = x + direction * radius * math.cos(float(a))
        yy = center_y + radius * math.sin(float(a))
        out.append(_wp([xx, yy, altitude], 'TURN_GUIDE', False, leg_id, role))
    return out


def _paired_span(index: int, first: float, growth: float, maximum: float) -> float:
    return min(maximum, first + (index // 2) * growth)


def _vehicle_search_altitude(cfg: dict, vehicle_id: int) -> float:
    """Return the configured fixed-wing cruise altitude for one aircraft.

    V6.7 keeps a scalar fallback for compatibility with older task packets, but
    prefers mission.search_altitudes_m=[v0,v1,v2].
    """
    values = cfg.get('mission', {}).get('search_altitudes_m')
    if isinstance(values, (list, tuple)) and 0 <= int(vehicle_id) < len(values):
        return float(values[int(vehicle_id)])
    return float(cfg['mission']['search_altitude_m'])


def generate_side_dynamic_route(cfg: dict, side: str, model_report: dict,
                                vehicle_id: Optional[int] = None) -> dict:
    """Initial side route: centre outward at 90 m with progressive leg length.

    The innermost straight leg covers the primary static high-risk support. Each
    following lane moves outward by ``lane_spacing_m`` and increases its length
    until the complete moving-target distribution region is covered. Only long
    straight legs are detection-valid; turn guides are deliberately minimal.
    """
    ds = cfg['dynamic_search']
    region_model = model_report.get('search_region_model') or derive_search_region_model(model_report)
    distribution = region_model['dynamic_distribution_region']
    initial = region_model['dynamic_initial_region']
    vid = int(vehicle_id if vehicle_id is not None else (0 if side == 'left' else 1))
    altitude = _vehicle_search_altitude(cfg, vid)
    center_x = 0.5 * (float(initial['x_min']) + float(initial['x_max']))
    center_gap = float(ds.get('center_gap_m', 50.0))
    inner_x = center_x - 0.5 * center_gap if side == 'left' else center_x + 0.5 * center_gap
    outer_x = float(distribution['x_min'] if side == 'left' else distribution['x_max'])
    spacing = float(ds['lane_spacing_m'])
    if side == 'left':
        lane_count = max(2, int(math.floor((inner_x - outer_x) / spacing)) + 1)
        lanes = [max(outer_x, inner_x - i * spacing) for i in range(lane_count)]
        if lanes[-1] > outer_x + 0.25 * spacing:
            lanes.append(outer_x)
    else:
        lane_count = max(2, int(math.floor((outer_x - inner_x) / spacing)) + 1)
        lanes = [min(outer_x, inner_x + i * spacing) for i in range(lane_count)]
        if lanes[-1] < outer_x - 0.25 * spacing:
            lanes.append(outer_x)

    edge_exclusion = float(cfg.get('straight_detection_gate', {}).get('segment_edge_exclusion_m', 35.0))
    primary_half = _static_primary_half_span(model_report)
    configured_first = float(ds.get('first_leg_half_span_m', primary_half))
    first_span = max(configured_first, primary_half + edge_exclusion)
    distribution_half = min(
        max(abs(float(distribution['y_min'])), abs(float(distribution['y_max']))),
        float(ds.get('maximum_leg_half_span_m', 900.0)),
    )
    first_span = min(first_span, distribution_half)
    spans: List[float] = []
    denominator = max(1, len(lanes) - 1)
    for index in range(len(lanes)):
        progress = index / denominator
        span = first_span + progress * (distribution_half - first_span)
        spans.append(float(span))

    points: List[dict] = []
    direction = 1 if side == 'left' else -1
    turn_samples = int(ds.get('turn_guide_samples', 2))
    for i, (x, span) in enumerate(zip(lanes, spans)):
        y_start = -span if direction > 0 else span
        y_end = span if direction > 0 else -span
        leg_id = f'{side}_dynamic_initial_leg_{i:02d}'
        role = f'DYNAMIC_{side.upper()}_INITIAL'
        if not points:
            points.append(_wp([x, y_start, altitude], 'TRANSIT_ENTRY', False, leg_id, role))
        else:
            previous = points[-1]['point']
            if math.hypot(previous[0] - x, previous[1] - y_start) > 1.0:
                points.append(_wp([x, y_start, altitude], 'TURN_GUIDE', False, leg_id, role))
        points.append(_wp([x, y_end, altitude], 'SEARCH_STRAIGHT', True, leg_id, role))
        if i + 1 < len(lanes):
            next_span = spans[i + 1]
            turn_y = next_span if direction > 0 else -next_span
            if abs(turn_y - y_end) > 1.0:
                points.append(_wp([x, turn_y, altitude], 'TURN_GUIDE', False, leg_id, role))
            points.extend(_semicircle_vertical_turn(
                x, lanes[i + 1], turn_y, direction, altitude,
                turn_samples, role, leg_id,
            ))
        direction *= -1

    return {
        'route_id': f'dynamic_{side}_initial_inner_to_outer',
        'role': f'DYNAMIC_{side.upper()}_INITIAL', 'side': side,
        'vehicle_id': vid, 'altitude_m': altitude, 'waypoints': points,
        'lane_x': [float(x) for x in lanes], 'lane_half_spans': spans,
        'inner_start_x_m': float(inner_x), 'outer_limit_x_m': float(outer_x),
        'primary_high_risk_half_span_m': float(primary_half),
        'distribution_half_span_m': float(distribution_half),
        'description': 'inner shortest leg covers primary static-risk support; 90m outward lanes progressively cover the moving distribution region',
    }


def generate_dynamic_distribution_inward_route(
        cfg: dict, side: str, model_report: dict, pass_index: int = 0,
        vehicle_id: Optional[int] = None) -> dict:
    """Own-side dynamic continuation from the outer edge toward the centre.

    Left and right aircraft receive disjoint half-regions. No route chunk from
    the opposite aircraft is copied. Later passes use a half-lane offset, so a
    repeated pass covers previous gaps without placing both aircraft on the same
    path.
    """
    ds = cfg['dynamic_search']
    region_model = model_report.get('search_region_model') or derive_search_region_model(model_report)
    distribution = region_model['dynamic_distribution_region']
    vid = int(vehicle_id if vehicle_id is not None else (0 if side == 'left' else 1))
    altitude = _vehicle_search_altitude(cfg, vid)
    spacing = float(ds['lane_spacing_m'])
    center_x = 0.5 * (float(distribution['x_min']) + float(distribution['x_max']))
    separation = float(ds.get('continuation_center_separation_m', spacing))
    offset = 0.5 * spacing if int(pass_index) % 2 else 0.0
    if side == 'left':
        outer = float(distribution['x_min']) + offset
        inner = center_x - 0.5 * separation
        lanes = list(np.arange(outer, inner + 0.25 * spacing, spacing))
        lanes = [float(min(x, inner)) for x in lanes]
    else:
        outer = float(distribution['x_max']) - offset
        inner = center_x + 0.5 * separation
        lanes = list(np.arange(outer, inner - 0.25 * spacing, -spacing))
        lanes = [float(max(x, inner)) for x in lanes]
    # Remove duplicated clipped terminal lane.
    clean: List[float] = []
    for value in lanes:
        if not clean or abs(value - clean[-1]) > 1.0:
            clean.append(value)
    lanes = clean
    ymin, ymax = float(distribution['y_min']), float(distribution['y_max'])
    points: List[dict] = []
    direction = 1 if side == 'left' else -1
    role = f'DYNAMIC_{side.upper()}_DISTRIBUTION_INWARD'
    turn_samples = int(ds.get('continuation_turn_guide_samples', ds.get('turn_guide_samples', 2)))
    for i, x in enumerate(lanes):
        y_start = ymin if direction > 0 else ymax
        y_end = ymax if direction > 0 else ymin
        leg_id = f'{side}_distribution_inward_p{int(pass_index):02d}_leg{i:02d}'
        if not points:
            points.append(_wp([x, y_start, altitude], 'TRANSIT_ENTRY', False, leg_id, role))
        else:
            last = points[-1]['point']
            if math.hypot(last[0] - x, last[1] - y_start) > 1.0:
                points.append(_wp([x, y_start, altitude], 'TURN_GUIDE', False, leg_id, role))
        points.append(_wp([x, y_end, altitude], 'SEARCH_STRAIGHT', True, leg_id, role))
        if i + 1 < len(lanes):
            points.extend(_semicircle_vertical_turn(
                x, lanes[i + 1], y_end, direction, altitude,
                turn_samples, role, leg_id,
            ))
        direction *= -1
    return {
        'route_id': f'dynamic_{side}_distribution_outer_to_center_pass{int(pass_index):02d}',
        'role': role, 'side': side, 'vehicle_id': vid,
        'pass_index': int(pass_index), 'altitude_m': altitude,
        'waypoints': points, 'lane_x': lanes,
        'distribution_bounds': dict(distribution),
        'center_separation_m': separation,
        'description': 'disjoint own-side moving-distribution coverage from outer edge toward centre; never reuses opposite aircraft route',
    }

def _center_out_lane_sequence(xmin: float, xmax: float, spacing: float) -> List[float]:
    center = (xmin + xmax) / 2.0
    values = [center]
    k = 1
    while True:
        added = False
        left, right = center - k * spacing, center + k * spacing
        if left >= xmin - 1e-6:
            values.append(left); added = True
        if right <= xmax + 1e-6:
            values.append(right); added = True
        if not added: break
        k += 1
    return values


def generate_center_out_route(cfg: dict, model_report: dict, full_residual: bool = False) -> dict:
    sc = cfg['static_search']
    safe = model_report['safe_bounds']
    initial = model_report['dynamic_start_bbox']
    altitude = float(cfg['mission']['search_altitude_m'])
    spacing = float(sc.get('residual_lane_spacing_m', sc['lane_spacing_m']) if full_residual else sc.get('initial_lane_spacing_m', sc['lane_spacing_m']))
    if full_residual:
        xmin, xmax = safe['x_min'], safe['x_max']
        ymin, ymax = safe['y_min'], safe['y_max']
        role, route_id = 'STATIC_RESIDUAL', 'static_residual_full_safe'
    else:
        center = (initial['x_min'] + initial['x_max']) / 2.0
        half_width = float(sc.get('primary_center_half_width_m', (initial['x_max']-initial['x_min'])/2.0))
        xmin, xmax = max(initial['x_min'], center-half_width), min(initial['x_max'], center+half_width)
        ymin, ymax = initial['y_min'], initial['y_max']
        role, route_id = 'STATIC_CENTER_OUT', 'static_center_out_primary'
    lanes = _center_out_lane_sequence(xmin, xmax, spacing)
    first_half = float(sc['first_leg_half_span_m'])
    max_half = min(max(abs(ymin), abs(ymax)), float(sc['maximum_leg_half_span_m']))
    growth = float(sc['half_span_growth_per_lane_m'])
    points: List[dict] = []
    direction = 1
    for i, x in enumerate(lanes):
        distance_rank = int(math.ceil(i / 2.0))
        span = min(max_half, first_half + distance_rank * growth)
        if full_residual:
            span = max_half
        y_start = -span if direction > 0 else span
        y_end = span if direction > 0 else -span
        leg_id = f'{role.lower()}_leg_{i:02d}'
        if not points:
            points.append(_wp([x, y_start, altitude], 'TRANSIT_ENTRY', False, leg_id, role))
        else:
            # Long center-out connectors are straight and useful for static observation,
            # but the first/last edge fractions remain invalid in the flight agent.
            points.append(_wp([x, y_start, altitude], 'SEARCH_CONNECTOR_STRAIGHT', True, leg_id, role))
        points.append(_wp([x, y_end, altitude], 'SEARCH_STRAIGHT', True, leg_id, role))
        direction *= -1
    return {
        'route_id': route_id, 'role': role, 'altitude_m': altitude,
        'waypoints': points, 'lane_x': lanes,
        'description': 'centre-out alternating coverage; residual route expands to full safe bounds',
    }



def generate_outer_in_route(cfg: dict, model_report: dict, vehicle_id: int = 2) -> dict:
    """Build the manoeuvre-inspection aircraft initial route for V6.7.12.

    The two dynamic-inspection aircraft use north-south lanes and expand from
    the centre toward the west/east sides.  To avoid duplicating that geometry,
    vehicle 2 first covers the complete public dynamic-initial region with
    east-west straight legs ordered strictly from north to south.  The raster is
    therefore orthogonal to the two dynamic-inspection routes.

    After the initial region is complete, the existing outward square stage is
    retained unchanged as a low-priority continuation over the area between the
    initial box and the complete moving-target distribution boundary.  All
    target observations are accepted only on long ``SEARCH_STRAIGHT`` legs;
    transfers and the minimal rounded turn guides are never detection-valid.

    No runtime target truth is used.  Bounds come only from the public
    ``model_state.py`` generation rules represented by ``model_report``.
    """
    sc = cfg['static_search']
    region_model = model_report.get('search_region_model') or derive_search_region_model(model_report)
    initial = region_model['dynamic_initial_region']
    distribution = region_model['dynamic_distribution_region']
    altitude = _vehicle_search_altitude(cfg, vehicle_id)
    spacing = float(sc.get('initial_lane_spacing_m', 90.0))
    role_initial = 'MANEUVER_INSPECTION_INITIAL'
    role_square = 'MANEUVER_INSPECTION_OUTER_SQUARE'
    turn_samples = int(sc.get('initial_square_corner_guide_samples', 1))
    turn_radius = min(float(sc.get('initial_square_corner_radius_m', 40.0)), 0.45 * spacing)

    ix0, ix1 = float(initial['x_min']), float(initial['x_max'])
    iy0, iy1 = float(initial['y_min']), float(initial['y_max'])
    dx0, dx1 = float(distribution['x_min']), float(distribution['x_max'])
    dy0, dy1 = float(distribution['y_min']), float(distribution['y_max'])

    # Strict north-to-south row order.  Include the exact southern boundary even
    # when the 90 m lattice does not divide the 1300 m initial-region height.
    lane_y: List[float] = []
    value = iy1
    while value >= iy0 - 1e-6:
        lane_y.append(float(value))
        value -= spacing
    if not lane_y or abs(lane_y[-1] - iy0) > 1.0:
        lane_y.append(float(iy0))

    points: List[dict] = []
    initial_leg_rows: List[dict] = []
    direction = 1  # first northern row flies west -> east
    for row_index, y in enumerate(lane_y):
        x_start = ix0 if direction > 0 else ix1
        x_end = ix1 if direction > 0 else ix0
        leg_id = f'manoeuvre_initial_north_to_south_row_{row_index:02d}'
        if not points:
            points.append(_wp([x_start, y, altitude], 'TRANSIT_ENTRY', False, leg_id, role_initial))
        else:
            previous = points[-1]['point']
            previous_y = float(lane_y[row_index - 1])
            # ``direction`` denotes the new row.  The prior row flew in the
            # opposite direction and ended at the corresponding x boundary.
            prior_direction = -direction
            previous_end_x = ix1 if prior_direction > 0 else ix0
            if abs(float(previous[0]) - previous_end_x) > 1.0 or abs(float(previous[1]) - previous_y) > 1.0:
                points.append(_wp([previous_end_x, previous_y, altitude], 'TURN_GUIDE', False, leg_id, role_initial))
            points.extend(_semicircle_horizontal_turn(
                previous_y, y, previous_end_x, prior_direction,
                altitude, max(1, turn_samples), role_initial, leg_id,
            ))
        points.append(_wp([x_end, y, altitude], 'SEARCH_STRAIGHT', True, leg_id, role_initial))
        initial_leg_rows.append({
            'index': row_index,
            'y': float(y),
            'x_start': float(x_start),
            'x_end': float(x_end),
            'heading': 'east' if direction > 0 else 'west',
        })
        direction *= -1

    # Preserve the proven outward-square continuation from V6.7.9.  It begins
    # only after the orthogonal initial-region raster is complete.
    rectangles: List[dict] = []
    max_rings_cfg = int(sc.get('initial_square_ring_count', 8))
    ring = 1
    last_rect = None
    while ring <= max_rings_cfg:
        rect = {
            'ring': ring - 1,
            'x_min': max(dx0, ix0 - ring * spacing),
            'x_max': min(dx1, ix1 + ring * spacing),
            'y_min': max(dy0, iy0 - ring * spacing),
            'y_max': min(dy1, iy1 + ring * spacing),
        }
        key = tuple(round(float(rect[k]), 6) for k in ('x_min', 'x_max', 'y_min', 'y_max'))
        if last_rect == key:
            break
        rectangles.append(rect)
        last_rect = key
        if (abs(rect['x_min'] - dx0) < 1e-6 and abs(rect['x_max'] - dx1) < 1e-6 and
                abs(rect['y_min'] - dy0) < 1e-6 and abs(rect['y_max'] - dy1) < 1e-6):
            break
        ring += 1

    current_xy = np.asarray(points[-1]['point'][:2], dtype=float)
    for rect in rectangles:
        x0, x1 = float(rect['x_min']), float(rect['x_max'])
        y0, y1 = float(rect['y_min']), float(rect['y_max'])
        base_corners = [
            np.asarray([x0, y0], dtype=float),
            np.asarray([x0, y1], dtype=float),
            np.asarray([x1, y1], dtype=float),
            np.asarray([x1, y0], dtype=float),
        ]
        start_index = int(np.argmin([np.linalg.norm(c - current_xy) for c in base_corners]))
        corners = base_corners[start_index:] + base_corners[:start_index]
        corners.append(corners[0])
        headings = []
        for a, b in zip(corners, corners[1:]):
            delta = b - a
            if abs(delta[0]) >= abs(delta[1]):
                headings.append('E' if delta[0] > 0 else 'W')
            else:
                headings.append('N' if delta[1] > 0 else 'S')
        for side_index, (a, b, heading) in enumerate(zip(corners, corners[1:], headings)):
            leg_id = f'manoeuvre_square_outward_ring{int(rect["ring"]):02d}_side{side_index}'
            if np.linalg.norm(current_xy - a) > 1.0:
                points.append(_wp([a[0], a[1], altitude], 'TRANSIT_ENTRY', False, leg_id, role_square))
            points.append(_wp([b[0], b[1], altitude], 'SEARCH_STRAIGHT', True, leg_id, role_square))
            current_xy = b.copy()
            if side_index < 3:
                points.extend(_quarter_turn_points(
                    b, heading, headings[side_index + 1], turn_radius,
                    altitude, max(1, turn_samples), role_square, leg_id,
                ))

    return {
        'route_id': 'maneuver_initial_east_west_north_to_south_then_square_outward',
        'role': 'MANEUVER_INSPECTION',
        'functional_role': 'MANEUVER_INSPECTION',
        'vehicle_id': int(vehicle_id),
        'altitude_m': altitude,
        'waypoints': points,
        'initial_stage': {
            'bounds': {'x_min': ix0, 'x_max': ix1, 'y_min': iy0, 'y_max': iy1},
            'lane_y': lane_y,
            'legs': initial_leg_rows,
            'direction': 'east_west_rows_ordered_north_to_south',
            'orthogonal_to_dynamic_inspection_routes': True,
        },
        'outer_square_stage': {
            'rectangles': rectangles,
            'direction': 'initial_boundary_to_distribution_boundary',
            'inset_or_expansion_m': spacing,
            'corner_guide_samples': turn_samples,
        },
        'description': (
            'manoeuvre-inspection aircraft first covers the entire initial region '
            'with east-west straight rows from north to south, orthogonal to the '
            'two dynamic-inspection aircraft; only then does it retain the existing '
            'nested-square outward continuation over the surrounding distribution area'
        ),
    }

def _static_possible_bounds(cfg: dict, model_report: dict) -> dict:
    """Return the guaranteed static-target distribution bounds.

    Static targets are generated around the dynamic-model starting points by up
    to maximum_static_offset_m. The expanded box is clipped by the safe world
    boundary. In the supplied competition model this evaluates to the complete
    safe rectangle, but keeping the derivation explicit avoids hard-coding it.
    """
    safe = model_report['safe_bounds']
    initial = model_report['dynamic_start_bbox']
    offset = float(model_report.get('maximum_static_offset_m', 300.0))
    return {
        'x_min': max(float(safe['x_min']), float(initial['x_min']) - offset),
        'x_max': min(float(safe['x_max']), float(initial['x_max']) + offset),
        'y_min': max(float(safe['y_min']), float(initial['y_min']) - offset),
        'y_max': min(float(safe['y_max']), float(initial['y_max']) + offset),
    }


def _covered_by_oriented_fov(point_xy: Sequence[float], samples: Sequence[dict],
                              half_forward_m: float, half_right_m: float) -> bool:
    px, py = float(point_xy[0]), float(point_xy[1])
    for sample in samples:
        pos = sample.get('position') or sample.get('world_position')
        if not pos or len(pos) < 2:
            continue
        dx, dy = px - float(pos[0]), py - float(pos[1])
        yaw = float(sample.get('yaw', sample.get('world_yaw', 0.0)))
        cy, sy = math.cos(yaw), math.sin(yaw)
        forward = cy * dx + sy * dy
        right = -sy * dx + cy * dy
        if abs(forward) <= half_forward_m and abs(right) <= half_right_m:
            return True
    return False


def filter_uncovered_static_prior_points(
        cfg: dict,
        coverage_samples: Sequence[dict],
        points: Sequence[dict],
        assigned_altitude_m: Optional[float] = None) -> List[dict]:
    """Return public static-prior cells not yet covered by actual FOV samples.

    This is a side-effect-free helper used by the conditional static-route
    preservation logic.  It uses the same conservative camera-footprint model
    as :func:`generate_static_gap_route`, but does not generate a route, consume
    a planner pass index, or write result files.
    """
    camera_cfg = cfg['camera']
    camera = {
        'width': float(camera_cfg['fallback_width']),
        'height': float(camera_cfg['fallback_height']),
        'fx': float(camera_cfg['fallback_fx']),
        'fy': float(camera_cfg['fallback_fy']),
        'cx': float(camera_cfg['fallback_cx']),
        'cy': float(camera_cfg['fallback_cy']),
        'ground_z_m': float(camera_cfg['ground_z_m']),
    }
    altitude = float(assigned_altitude_m if assigned_altitude_m is not None
                     else cfg['mission']['search_altitude_m'])
    footprint = camera_footprint(camera, altitude)
    margin = float(cfg['perception'].get('fov_margin_ratio', 0.88))
    conservative = float(cfg['static_search'].get('coverage_conservative_ratio', 0.92))
    half_right = footprint['half_right_m'] * margin * conservative
    half_forward = footprint['half_forward_m'] * margin * conservative
    return [
        dict(point) for point in points
        if not _covered_by_oriented_fov(
            (float(point['x']), float(point['y'])),
            coverage_samples, half_forward, half_right)
    ]


def _static_region_contains(region: dict, x: float, y: float) -> bool:
    """Return whether one public-prior region contains a grid point."""
    shape = str(region.get('shape', 'rectangle'))
    if shape == 'circle':
        cx = float(region['center_x'])
        cy = float(region['center_y'])
        radius = max(0.0, float(region['radius_m']))
        return math.hypot(float(x) - cx, float(y) - cy) <= radius + 1e-9
    if shape == 'rectangle':
        return (
            float(region['x_min']) - 1e-9 <= float(x) <= float(region['x_max']) + 1e-9
            and float(region['y_min']) - 1e-9 <= float(y) <= float(region['y_max']) + 1e-9
        )
    raise ValueError(f'unsupported static risk-region shape: {shape}')


def generate_static_prior_grid_points(
        cfg: dict,
        model_report: dict,
        minimum_priority: float = 0.0,
        grid_step_m: Optional[float] = None) -> List[dict]:
    """Build the truth-free public static-risk grid.

    The grid is derived only from the pre-mission ``high_risk_regions``
    configuration and the public safe bounds parsed from ``model_state.py``.
    It never consumes a target position, dynamic-target start point, or Gazebo
    truth. Nested prior regions are merged by taking the maximum priority at
    each cell.
    """
    sc = cfg['static_search']
    guaranteed = _static_possible_bounds(cfg, model_report)
    step = float(grid_step_m if grid_step_m is not None else
                 sc.get('risk_grid_resolution_m', 35.0))
    if step <= 0.0:
        raise ValueError('static prior grid step must be positive')
    use_derived = bool(sc.get('use_model_derived_high_risk', True))
    regions = list(sc.get('high_risk_regions', []))
    if not regions and not use_derived:
        regions = [{
            'shape': 'rectangle',
            'name': 'safe_fallback',
            'x_min': guaranteed['x_min'], 'x_max': guaranteed['x_max'],
            'y_min': guaranteed['y_min'], 'y_max': guaranteed['y_max'],
            'priority': 1.0, 'source': 'configured_safe_region',
        }]
    gx0 = float(guaranteed['x_min'])
    gy0 = float(guaranteed['y_min'])
    points: List[dict] = []
    for x in np.arange(gx0, float(guaranteed['x_max']) + 0.25 * step, step):
        for y in np.arange(gy0, float(guaranteed['y_max']) + 0.25 * step, step):
            if use_derived:
                priority, source = _static_risk_priority_at_xy(model_report, float(x), float(y))
                if priority <= 0.0 or priority + 1e-9 < float(minimum_priority):
                    continue
                points.append({
                    'x': float(x), 'y': float(y), 'priority': float(priority),
                    'sources': [source],
                    'region_names': ['model_derived_static_high_risk'],
                    'target_names': [],
                })
                continue
            matched = [r for r in regions if _static_region_contains(r, float(x), float(y))]
            if not matched:
                continue
            priority = max(float(r.get('priority', 1.0)) for r in matched)
            if priority + 1e-9 < float(minimum_priority):
                continue
            points.append({
                'x': float(x), 'y': float(y), 'priority': float(priority),
                'sources': sorted({str(r.get('source', r.get('name', 'public_prior'))) for r in matched}),
                'region_names': sorted({str(r.get('name', 'risk')) for r in matched}),
                'target_names': [],
            })
    return points


def filter_static_prior_points_by_regions(
        prior_points: Sequence[dict],
        regions: Sequence[dict]) -> List[dict]:
    """Intersect public-prior cells with one or more local focus regions."""
    selected: List[dict] = []
    for point in prior_points:
        x = float(point['x'])
        y = float(point['y'])
        matched = [r for r in regions if _static_region_contains(r, x, y)]
        if not matched:
            continue
        row = dict(point)
        row['sources'] = sorted(set(row.get('sources', [])) | {
            str(r.get('source', r.get('name', 'focus_region'))) for r in matched
        })
        row['target_names'] = sorted(set(row.get('target_names', [])) | {
            str(name) for r in matched for name in r.get('target_names', [])
        })
        # A detected static partner makes this local region more important, but
        # the cell must still belong to the public static-prior grid.
        row['priority'] = max(
            float(row.get('priority', 0.0)),
            max(float(r.get('priority', 1.0)) for r in matched),
        )
        selected.append(row)
    return selected


def _point_to_segment_distance_xy(point: Sequence[float], start: Sequence[float],
                                  end: Sequence[float]) -> float:
    p = np.asarray(point[:2], dtype=float)
    a = np.asarray(start[:2], dtype=float)
    b = np.asarray(end[:2], dtype=float)
    delta = b - a
    denom = float(np.dot(delta, delta))
    if denom <= 1e-9:
        return float(np.linalg.norm(p - a))
    t = clamp(float(np.dot(p - a, delta) / denom), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * delta)))


def filter_static_prior_points_by_route_remainder(
        prior_points: Sequence[dict],
        route: dict,
        start_waypoint_index: int,
        corridor_half_width_m: float) -> Tuple[List[dict], List[dict]]:
    """Intersect static-prior cells with the *unflown* detection corridors.

    This replaces the old behaviour of copying the remaining dynamic-search
    waypoints verbatim. Only public static-risk cells near still-unflown valid
    search segments are retained. The returned segment metadata is useful for
    diagnostics and run artefacts.
    """
    waypoints = list(route.get('waypoints', []))
    start_index = max(1, int(start_waypoint_index))
    half_width = max(1.0, float(corridor_half_width_m))
    segments: List[dict] = []
    for index in range(start_index, len(waypoints)):
        endpoint = waypoints[index]
        if not bool(endpoint.get('detection_valid', False)):
            continue
        if str(endpoint.get('segment_type', '')) not in (
                'SEARCH_STRAIGHT', 'SEARCH_CONNECTOR_STRAIGHT'):
            continue
        previous = waypoints[index - 1]
        start = previous.get('point')
        end = endpoint.get('point')
        if not isinstance(start, list) or not isinstance(end, list) or len(start) < 2 or len(end) < 2:
            continue
        segments.append({
            'waypoint_index': int(index),
            'start': [float(start[0]), float(start[1])],
            'end': [float(end[0]), float(end[1])],
            'segment_type': str(endpoint.get('segment_type', '')),
            'leg_id': str(endpoint.get('leg_id', '')),
        })
    if not segments:
        return [], []
    selected: List[dict] = []
    for point in prior_points:
        p = [float(point['x']), float(point['y'])]
        if any(_point_to_segment_distance_xy(p, s['start'], s['end']) <= half_width
               for s in segments):
            row = dict(point)
            row['sources'] = sorted(set(row.get('sources', [])) | {'unflown_dynamic_route_corridor'})
            selected.append(row)
    return selected, segments


def generate_static_gap_route(cfg: dict, model_report: dict, coverage_samples: Sequence[dict],
                              current_position: Sequence[float], pass_index: int = 0,
                              focus_regions: Optional[Sequence[dict]] = None,
                              assigned_altitude_m: Optional[float] = None,
                              focus_grid_points: Optional[Sequence[dict]] = None,
                              priority_floor_override: Optional[float] = None,
                              max_route_length_override_m: Optional[float] = None) -> dict:
    """Generate an efficient static-only route over uncovered high-risk cells.

    Candidate cells come from pre-mission rectangles/circles or from an
    explicitly supplied subset of the public static-prior grid. The latter is
    used for precise own/opposite unflown-corridor searches and pair-guided
    searches. The oriented FOV footprints actually flown by all three aircraft
    are subtracted before connected components are swept. The planner accepts
    no target truth and performs no start-position inference.
    """
    sc = cfg['static_search']
    camera_cfg = cfg['camera']
    camera = {
        'width': float(camera_cfg['fallback_width']),
        'height': float(camera_cfg['fallback_height']),
        'fx': float(camera_cfg['fallback_fx']),
        'fy': float(camera_cfg['fallback_fy']),
        'cx': float(camera_cfg['fallback_cx']),
        'cy': float(camera_cfg['fallback_cy']),
        'ground_z_m': float(camera_cfg['ground_z_m']),
    }
    altitude = float(assigned_altitude_m if assigned_altitude_m is not None else cfg['mission']['search_altitude_m'])
    footprint = camera_footprint(camera, altitude)
    margin = float(cfg['perception'].get('fov_margin_ratio', 0.88))
    conservative = float(sc.get('coverage_conservative_ratio', 0.92))
    half_right = footprint['half_right_m'] * margin * conservative
    half_forward = footprint['half_forward_m'] * margin * conservative
    guaranteed = _static_possible_bounds(cfg, model_report)
    grid_step = float(sc.get('risk_grid_resolution_m', sc.get('residual_grid_resolution_m', 25.0)))
    lane_spacing = float(sc.get('risk_lane_spacing_m', 82.0))
    cross_half = max(0.5 * lane_spacing, min(half_right * 0.92, 0.75 * lane_spacing))
    min_segment = float(sc.get('risk_min_segment_length_m', 120.0))
    padding = float(sc.get('risk_segment_padding_m', 60.0))
    edge = float(cfg['straight_detection_gate'].get('segment_edge_exclusion_m', 35.0))
    max_runs = int(sc.get('risk_max_scan_runs', 80))

    regions = list(focus_regions or [{**guaranteed, 'shape': 'rectangle',
                                      'source': 'guaranteed_static_region', 'target_names': []}])
    candidate_meta: Dict[Tuple[int,int], dict] = {}
    gx0, gy0 = float(guaranteed['x_min']), float(guaranteed['y_min'])

    def add_candidate(x: float, y: float, region: dict) -> None:
        if not (guaranteed['x_min'] <= x <= guaranteed['x_max'] and
                guaranteed['y_min'] <= y <= guaranteed['y_max']):
            return
        ix = int(round((x - gx0) / grid_step)); iy = int(round((y - gy0) / grid_step))
        key = (ix, iy)
        row = candidate_meta.setdefault(key, {
            'x': gx0 + ix * grid_step, 'y': gy0 + iy * grid_step,
            'target_names': set(), 'sources': set(), 'priority': 0.0,
        })
        row['target_names'].update(region.get('target_names', []))
        row['sources'].add(str(region.get('source', 'unknown')))
        row['priority'] = max(float(row['priority']), float(region.get('priority', 1.0)))

    floors = [float(v) for v in sc.get('pass_priority_floors', [0.72, 0.40, 0.0])]
    priority_floor = (float(priority_floor_override)
                      if priority_floor_override is not None
                      else (floors[min(max(int(pass_index), 0), len(floors)-1)] if floors else 0.0))
    for region in regions:
        shape = str(region.get('shape', 'rectangle'))
        if shape not in ('rectangle', 'circle'):
            raise ValueError(f'unsupported static risk-region shape: {shape}')
        if float(region.get('priority', 1.0)) + 1e-9 < priority_floor:
            continue
        if shape == 'circle':
            cx = float(region['center_x']); cy = float(region['center_y'])
            radius = max(0.0, float(region['radius_m']))
            x_lo = max(guaranteed['x_min'], cx - radius)
            x_hi = min(guaranteed['x_max'], cx + radius)
            y_lo = max(guaranteed['y_min'], cy - radius)
            y_hi = min(guaranteed['y_max'], cy + radius)
        else:
            x_lo = max(guaranteed['x_min'], float(region.get('x_min', guaranteed['x_min'])))
            x_hi = min(guaranteed['x_max'], float(region.get('x_max', guaranteed['x_max'])))
            y_lo = max(guaranteed['y_min'], float(region.get('y_min', guaranteed['y_min'])))
            y_hi = min(guaranteed['y_max'], float(region.get('y_max', guaranteed['y_max'])))
        for x in np.arange(gx0 + math.ceil((x_lo-gx0)/grid_step)*grid_step,
                           x_hi + 0.25*grid_step, grid_step):
            for y in np.arange(gy0 + math.ceil((y_lo-gy0)/grid_step)*grid_step,
                               y_hi + 0.25*grid_step, grid_step):
                if shape == 'circle' and math.hypot(float(x)-cx, float(y)-cy) > radius + 1e-9:
                    continue
                add_candidate(float(x), float(y), region)

    if focus_grid_points is not None:
        # Explicit points are already the intersection of the public static
        # prior with the relevant unflown corridor or pair-guided neighbourhood.
        # Replace, rather than union with, the broad region candidates.
        candidate_meta = {}
        for point in focus_grid_points:
            priority = float(point.get('priority', 1.0))
            if priority + 1e-9 < priority_floor:
                continue
            region = {
                'priority': priority,
                'source': ','.join(map(str, point.get('sources', ['explicit_static_prior_grid']))),
                'target_names': list(point.get('target_names', [])),
            }
            add_candidate(float(point['x']), float(point['y']), region)

    candidate_keys = set(candidate_meta)
    uncovered = {
        key for key, row in candidate_meta.items()
        if not _covered_by_oriented_fov((row['x'], row['y']), coverage_samples, half_forward, half_right)
    }
    fallback_verification = False
    pair_guided_full_revisit = False
    if (focus_grid_points is None and not uncovered and candidate_keys
            and int(pass_index) >= max(0, len(floors)-1)):
        # Only on the final outer-area pass, perform a stricter
        # edge-verification pass. This intentionally occurs after, not during,
        # the normal gap fill and therefore does not create early duplicates.
        fallback_verification = True
        shrink = float(sc.get('risk_verification_coverage_ratio', 0.68))
        uncovered = {
            key for key, row in candidate_meta.items()
            if not _covered_by_oriented_fov((row['x'], row['y']), coverage_samples,
                                            half_forward*shrink, half_right*shrink)
        }
    pair_guided = any(str(r.get('source', '')).startswith('detected_static_pair:') for r in regions)
    if (not uncovered and candidate_keys and pair_guided
            and bool(sc.get('pair_guided_force_revisit_when_empty', True))):
        # A target may have been missed even though the nominal FOV grid says the
        # partner-centred area was covered. Revisit only this small observed-pair
        # region; never restart the whole global search.
        pair_guided_full_revisit = True
        uncovered = set(candidate_keys)

    # Connected components in grid-index space.
    components: List[set] = []
    pending = set(uncovered)
    while pending:
        seed = pending.pop(); comp = {seed}; stack = [seed]
        while stack:
            ix, iy = stack.pop()
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    if dx == 0 and dy == 0: continue
                    nxt = (ix+dx, iy+dy)
                    if nxt in pending:
                        pending.remove(nxt); comp.add(nxt); stack.append(nxt)
        components.append(comp)
    minimum_component_cells=int(sc.get('minimum_component_cells',2))
    if int(pass_index) < max(0,len(floors)-1):
        components=[c for c in components if len(c)>=minimum_component_cells]
    components.sort(key=lambda c: -sum(candidate_meta[k]['priority'] for k in c))

    raw_runs = []
    for component_index, comp in enumerate(components):
        remaining = set(comp)
        xs = [candidate_meta[k]['x'] for k in comp]
        ys = [candidate_meta[k]['y'] for k in comp]
        # Sweep along the longer component axis to minimise the number of turns.
        vertical = (max(ys)-min(ys)) >= (max(xs)-min(xs))
        while remaining and len(raw_runs) < max_runs:
            candidate_lines = sorted({candidate_meta[k]['x' if vertical else 'y'] for k in remaining})
            best_line = None; best_cover = set(); best_score = -1.0
            for line in candidate_lines:
                cover = {
                    k for k in remaining
                    if abs(candidate_meta[k]['x' if vertical else 'y'] - line) <= cross_half
                }
                score = sum(candidate_meta[k]['priority'] for k in cover)
                if score > best_score:
                    best_line, best_cover, best_score = float(line), cover, score
            if not best_cover:
                break
            along = sorted((candidate_meta[k]['y' if vertical else 'x'], k) for k in best_cover)
            groups: List[List[Tuple[float,Tuple[int,int]]]] = [[along[0]]]
            for item in along[1:]:
                if item[0] - groups[-1][-1][0] <= 2.5 * grid_step:
                    groups[-1].append(item)
                else:
                    groups.append([item])
            for group in groups:
                a0 = group[0][0] - padding - edge
                a1 = group[-1][0] + padding + edge
                if a1-a0 < min_segment:
                    centre = 0.5*(a0+a1); a0=centre-0.5*min_segment; a1=centre+0.5*min_segment
                if vertical:
                    p0=[best_line, max(guaranteed['y_min'],a0), altitude]
                    p1=[best_line, min(guaranteed['y_max'],a1), altitude]
                else:
                    p0=[max(guaranteed['x_min'],a0), best_line, altitude]
                    p1=[min(guaranteed['x_max'],a1), best_line, altitude]
                raw_runs.append({
                    'start': p0, 'end': p1, 'orientation': 'vertical' if vertical else 'horizontal',
                    'component': component_index, 'covered_high_risk_cells': len(group),
                    'priority': float(sum(candidate_meta[k]['priority'] for _,k in group)),
                })
            remaining -= best_cover

    current = np.asarray(current_position if current_position is not None else [0.0,0.0,altitude], dtype=float)
    remaining_runs = list(raw_runs); ordered=[]
    while remaining_runs:
        best=None
        for run in remaining_runs:
            for reverse in (False,True):
                start = np.asarray(run['end' if reverse else 'start'], dtype=float)
                d = float(np.linalg.norm(start[:2]-current[:2]))
                metric = d - 4.0*float(run.get('priority',0.0))
                candidate=(metric,d,run,reverse)
                if best is None or candidate[:2] < best[:2]: best=candidate
        _,_,run,reverse=best
        remaining_runs.remove(run)
        start=list(run['end' if reverse else 'start']); end=list(run['start' if reverse else 'end'])
        ordered.append({**run,'start':start,'end':end,'reversed':bool(reverse)})
        current=np.asarray(end,dtype=float)

    max_route_length=(float(max_route_length_override_m)
                      if max_route_length_override_m is not None
                      else float(sc.get('max_route_length_m',12500.0)))
    if max_route_length > 0.0:
        limited=[]; total_length=0.0
        for run in ordered:
            length=float(np.linalg.norm(np.asarray(run['end'])[:2]-np.asarray(run['start'])[:2]))
            if limited and total_length+length > max_route_length: break
            limited.append(run); total_length += length
        ordered=limited
    else:
        total_length=sum(float(np.linalg.norm(np.asarray(r['end'])[:2]-np.asarray(r['start'])[:2])) for r in ordered)

    waypoints: List[dict] = []
    for index, run in enumerate(ordered):
        leg_id=f'static_risk_pass{int(pass_index):02d}_leg{index:03d}'
        waypoints.append(_wp(run['start'],'TRANSIT_ENTRY',False,leg_id,'STATIC_RESIDUAL'))
        waypoints.append(_wp(run['end'],'SEARCH_STRAIGHT',True,leg_id,'STATIC_RESIDUAL'))

    serialisable_regions=[]
    for r in regions:
        serialisable_regions.append({k:v for k,v in r.items() if k not in ('_internal',)})
    return {
        'route_id': f'static_high_risk_pass{int(pass_index):02d}_{int(time.time())}',
        'role':'STATIC_RESIDUAL','altitude_m':altitude,'waypoints':waypoints,
        'planner':'global_prior_grid_minus_three_uav_fov', 'pass_index':int(pass_index),
        'static_possible_bounds':guaranteed,'focus_regions':serialisable_regions,
        'coverage_sample_count':len(coverage_samples),
        'candidate_grid_points':len(candidate_keys),'uncovered_grid_points':len(uncovered),
        'estimated_covered_ratio':1.0-float(len(uncovered))/max(1.0,float(len(candidate_keys))),
        'effective_fov_half_forward_m':half_forward,'effective_fov_half_right_m':half_right,
        'fallback_full_static_pass':fallback_verification,'pair_guided_full_revisit':pair_guided_full_revisit,
        'high_risk_component_count':len(components),'scan_runs':ordered,
        'grid_resolution_m':grid_step,'lane_spacing_m':lane_spacing,'priority_floor':priority_floor,'planned_route_length_m':total_length,
        'candidate_source_mode':'explicit_public_prior_grid' if focus_grid_points is not None else 'focus_regions',
        'nominal_cross_track_coverage_ratio': min(1.0, (2.0*half_right)/max(lane_spacing,1e-6)),
        'planning_profile': str(sc.get('residual_planning_profile','fast_static_discovery')),
        'description':'staged static discovery over actual uncovered coverage; optional pair-guided circles use only observed static detections, never target truth',
    }

def build_plan(cfg: dict) -> Tuple[dict, Dict[int, dict]]:
    report = derive_model_region(cfg)
    report['search_region_model'] = derive_search_region_model(report)
    left = generate_side_dynamic_route(cfg, 'left', report, vehicle_id=0)
    right = generate_side_dynamic_route(cfg, 'right', report, vehicle_id=1)
    outer_in = generate_outer_in_route(cfg, report, vehicle_id=2)
    residual = generate_center_out_route(cfg, report, full_residual=True)
    continuation_preview = {
        '0': generate_dynamic_distribution_inward_route(cfg, 'left', report, 0, vehicle_id=0),
        '1': generate_dynamic_distribution_inward_route(cfg, 'right', report, 0, vehicle_id=1),
    }
    routes = {0: left, 1: right, 2: outer_in}
    seed = int(cfg['mission'].get('random_seed', -1))
    if seed < 0:
        seed = random.SystemRandom().randint(1, 2**31 - 1)
    root = Path(os.path.expanduser(cfg['mission']['output_root']))
    run_dir = root / f"{cfg['mission']['scene_id']}_{time.strftime('%Y%m%d_%H%M%S')}_seed{seed}"
    run_dir.mkdir(parents=True, exist_ok=True)
    camera = cfg['camera']
    cam = {
        'width': camera['fallback_width'], 'height': camera['fallback_height'],
        'fx': camera['fallback_fx'], 'fy': camera['fallback_fy'],
        'cx': camera['fallback_cx'], 'cy': camera['fallback_cy'],
        'u_right_sign': camera['image_u_to_body_right_sign'],
        'v_forward_sign': camera['image_v_to_body_forward_sign'],
        'ground_z_m': camera['ground_z_m'],
    }
    altitude_map = {str(v): _vehicle_search_altitude(cfg, v) for v in (0, 1, 2)}
    plan = {
        'schema_version': 10,
        'scene_id': cfg['mission']['scene_id'], 'seed': seed, 'run_dir': str(run_dir),
        'architecture': 'manager_plus_three_independent_adaptive_vehicle_terminals',
        'model_state_analysis': report,
        'region_definitions': report['search_region_model'],
        'dynamic_distribution_continuation_preview': continuation_preview,
        'search_altitudes_m': altitude_map,
        'camera_footprint_search_per_vehicle': {
            str(v): camera_footprint(cam, altitude_map[str(v)]) for v in (0, 1, 2)
        },
        'camera_footprint_track_30m': camera_footprint(cam, cfg['mission']['track_altitude_m']),
        'initial_assignments': {
            '0': 'SEARCH_DYNAMIC_LEFT_INITIAL_INNER_OUT',
            '1': 'SEARCH_DYNAMIC_RIGHT_INITIAL_INNER_OUT',
            '2': 'SEARCH_MANEUVER_INITIAL_EAST_WEST_NORTH_TO_SOUTH',
        },
        'role_policy': {
            'dynamic_inspection_vehicle_ids': [0, 1],
            'maneuver_inspection_vehicle_id': 2,
            'first_dynamic_tracker': 'maneuver_inspection_vehicle',
            'second_dynamic_tracker': 'detecting_free_dynamic_inspection_vehicle',
            'static_search_vehicle': 'remaining_dynamic_inspection_vehicle',
        },
        'routes': {str(k): v for k, v in routes.items()},
        'static_residual_route': residual,
        'task_completion': {
            'dynamic_targets_required': cfg['perception']['dynamic_targets'],
            'static_targets_required': cfg['perception']['static_targets'],
            'dynamic_min_track_seconds': cfg['tracking']['minimum_track_seconds'],
            'dynamic_min_track_points': cfg['tracking']['minimum_track_points'],
        },
    }
    return plan, routes

