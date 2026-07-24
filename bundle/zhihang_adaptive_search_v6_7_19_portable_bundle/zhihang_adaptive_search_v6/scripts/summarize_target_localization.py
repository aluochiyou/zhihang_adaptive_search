#!/usr/bin/env python3
"""Summarize V6.7.12 target localization and validation records."""
import collections
import json
import statistics
import sys
from pathlib import Path


def read_jsonl(path: Path):
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding='utf-8').splitlines():
        try:
            rows.append(json.loads(line))
        except Exception:
            pass
    return rows


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: summarize_target_localization.py RUN_DIR')
    run = Path(sys.argv[1]).expanduser().resolve()
    rows = read_jsonl(run / 'target_localization_reports.jsonl')
    grouped = collections.defaultdict(list)
    for row in rows:
        grouped[str(row.get('target_name', 'unknown'))].append(row)
    print('RUN:', run)
    print('localization reports:', len(rows))
    for name, values in sorted(grouped.items()):
        confidences = [float(x.get('confidence', 0.0)) for x in values]
        stds = [float(x.get('horizontal_std_m', 0.0)) for x in values]
        selected = sum(bool(x.get('selected_as_management_result', False)) for x in values)
        latest = values[-1].get('position_world')
        print(
            f'{name}: reports={len(values)} selected={selected} '
            f'confidence_median={statistics.median(confidences):.3f} '
            f'std_xy_median={statistics.median(stds):.3f} latest={latest}')
    summaries = sorted(run.glob('detection_validation_summary_v*.json'))
    for path in summaries:
        try:
            row = json.loads(path.read_text(encoding='utf-8'))
        except Exception:
            continue
        print(path.name, 'counts=', row.get('counts'),
              'localization=', row.get('localization'))


if __name__ == '__main__':
    main()
