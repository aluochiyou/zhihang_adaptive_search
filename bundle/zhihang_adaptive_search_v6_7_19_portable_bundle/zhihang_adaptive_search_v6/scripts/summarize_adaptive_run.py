#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8')) if Path(path).is_file() else None


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: summarize_adaptive_run.py RUN_DIR')
    run = Path(sys.argv[1]).expanduser().resolve()
    plan = load(run/'search_plan.json') or {}
    ev = load(run/'evaluation.json') or {}
    final = load(run/'final_results.json') or {}
    print('RUN:', run)
    print('mission:', ev.get('mission_id', plan.get('mission_id')))
    print('duration first arm -> last land:', ev.get('mission_duration_seconds_first_arm_to_last_land'))
    print('dynamic assignments:', (ev.get('dynamic_targets') or {}).get('assignments'))
    print('dynamic first detection:', (ev.get('dynamic_targets') or {}).get('first_detection_ros_time'))
    print('dynamic track counts:', (ev.get('dynamic_targets') or {}).get('track_counts'))
    print('static detected:', (ev.get('static_targets') or {}).get('detected'))
    print('static confirmed:', (ev.get('static_targets') or {}).get('confirmed'))
    print('YOLO out of flight loop:', (ev.get('yolo26') or {}).get('in_flight_control_loop') is False)
    for vid, row in sorted(((ev.get('yolo26') or {}).get('per_vehicle') or {}).items()):
        print(f'YOLO v{vid}: reports={row.get("reports")} median_worker_fps={row.get("worker_fps_median"):.3f}')
    print('selected source:', ev.get('selected_detection_source'))
    print('proxy rule:', ev.get('proxy_rule'))
    missing = []
    for name in plan.get('task_completion', {}).get('static_targets_required', []):
        if name not in (final.get('static_confirmed') or {}): missing.append(name)
    print('missing static confirmations:', missing)


if __name__ == '__main__': main()
