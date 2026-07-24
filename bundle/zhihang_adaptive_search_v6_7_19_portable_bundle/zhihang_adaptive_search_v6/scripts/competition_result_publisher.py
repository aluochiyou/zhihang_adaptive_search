#!/usr/bin/env python3
"""Bridge finalized V6 JSON results to the competition custom ROS message.

The competition message package is supplied separately in ~/zhihang_ws and its
package/type names are not hard-coded in the player manual. This node therefore
supports either an explicit outer message type or safe runtime discovery by
field structure:
  outer: header_timestamp, categories[]
  item:  timestamp, model_name, x, y
Only the first finalized payload is published, matching the scoring rule that
the first message received on each official topic is authoritative.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import List, Optional, Tuple, Type

import rospy
import roslib.message
from std_msgs.msg import String

NS = '/zhihang/search_v6'
PARAM_ROOT = '/zhihang_search_v6'


def list_message_types() -> List[str]:
    result = subprocess.run(
        ['rosmsg', 'list'], text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=False, timeout=30.0)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or 'rosmsg list failed')
    return [line.strip() for line in result.stdout.splitlines() if '/' in line]


def class_slots(cls: Type[object]) -> Tuple[List[str], List[str]]:
    return list(getattr(cls, '__slots__', [])), list(getattr(cls, '_slot_types', []))


def compatible_outer(type_name: str):
    cls = roslib.message.get_message_class(type_name)
    if cls is None:
        return None
    slots, slot_types = class_slots(cls)
    if 'header_timestamp' not in slots or 'categories' not in slots:
        return None
    index = slots.index('categories')
    if index >= len(slot_types):
        return None
    category_type = slot_types[index]
    if not category_type.endswith('[]'):
        return None
    item_name = category_type[:-2]
    item_cls = roslib.message.get_message_class(item_name)
    if item_cls is None:
        return None
    item_slots, _ = class_slots(item_cls)
    if not {'timestamp', 'model_name', 'x', 'y'}.issubset(set(item_slots)):
        return None
    return cls, item_cls, item_name


def discover_message(explicit: str = ''):
    if explicit:
        found = compatible_outer(explicit)
        if found is None:
            raise RuntimeError(
                f'explicit message type is incompatible: {explicit}; expected '
                'header_timestamp + categories[] with timestamp/model_name/x/y items')
        return explicit, *found
    matches = []
    for type_name in list_message_types():
        try:
            found = compatible_outer(type_name)
        except Exception:
            continue
        if found is not None:
            matches.append((type_name, *found))
    if not matches:
        raise RuntimeError(
            'no compatible competition message type was found. Source '
            '~/zhihang_ws/devel/setup.bash through ZHIHANG_MESSAGE_WS_SETUP.')
    # Prefer a type whose package/name hints at the supplied competition workspace.
    matches.sort(key=lambda row: (
        0 if any(token in row[0].lower() for token in ('zhihang', 'target', 'pose')) else 1,
        row[0]))
    return matches[0]


class CompetitionResultPublisher:
    def __init__(self) -> None:
        rospy.init_node('competition_result_publisher_v6')
        cfg = rospy.get_param(PARAM_ROOT, {})
        out_cfg = dict(cfg.get('competition_output', {}))
        self.enabled = bool(out_cfg.get('enabled', True))
        self.static_topic = str(out_cfg.get(
            'static_topic', '/zhihang2026/static_targets/pose'))
        self.dynamic_topic = str(out_cfg.get(
            'dynamic_topic', '/zhihang2026/dynamic_targets/pose'))
        explicit = str(out_cfg.get('outer_message_type', '')).strip()
        self.published = False
        self.error_path: Optional[Path] = None
        if not self.enabled:
            rospy.logwarn('competition official-topic output is disabled by config')
            return
        type_name, outer_cls, item_cls, item_type = discover_message(explicit)
        self.outer_type_name = type_name
        self.outer_cls = outer_cls
        self.item_cls = item_cls
        self.item_type_name = item_type
        self.static_pub = rospy.Publisher(
            self.static_topic, self.outer_cls, queue_size=1, latch=True)
        self.dynamic_pub = rospy.Publisher(
            self.dynamic_topic, self.outer_cls, queue_size=1, latch=True)
        rospy.Subscriber(
            f'{NS}/manager/competition_final_results', String,
            self.final_results_cb, queue_size=1)
        rospy.logwarn(
            'official result bridge ready type=%s item=%s static=%s dynamic=%s',
            self.outer_type_name, self.item_type_name,
            self.static_topic, self.dynamic_topic)

    def build_message(self, header_ns: int, rows: List[dict]):
        msg = self.outer_cls()
        msg.header_timestamp = int(header_ns)
        items = []
        for row in rows[:20]:
            item = self.item_cls()
            item.timestamp = int(row['timestamp_ns'])
            item.model_name = str(row['model_name'])
            item.x = float(row['x'])
            item.y = float(row['y'])
            items.append(item)
        msg.categories = items
        return msg

    def final_results_cb(self, msg: String) -> None:
        if self.published or not self.enabled:
            return
        try:
            payload = json.loads(msg.data)
            header_ns = int(payload.get(
                'header_timestamp_ns', rospy.Time.now().to_nsec()))
            static_rows = list(payload.get('static_entries', []))
            dynamic_rows = list(payload.get('dynamic_entries', []))
            # The first official message is authoritative.  Publish the best
            # available partial result at mission completion so one incomplete
            # component does not suppress the other component's score.  Reject
            # only a completely empty payload, which normally indicates a
            # wiring or serialization fault rather than a legitimate mission.
            if not static_rows and not dynamic_rows:
                raise ValueError('final payload is completely empty; official topics not published')
            self.static_pub.publish(self.build_message(header_ns, static_rows))
            self.dynamic_pub.publish(self.build_message(header_ns, dynamic_rows))
            self.published = True
            run_dir = payload.get('run_dir')
            if run_dir:
                evidence = {
                    'published': True,
                    'outer_message_type': self.outer_type_name,
                    'item_message_type': self.item_type_name,
                    'static_topic': self.static_topic,
                    'dynamic_topic': self.dynamic_topic,
                    'static_count': len(static_rows[:20]),
                    'dynamic_count': len(dynamic_rows[:20]),
                    'header_timestamp_ns': header_ns,
                }
                Path(run_dir, 'competition_topic_publish_evidence.json').write_text(
                    json.dumps(evidence, ensure_ascii=False, indent=2),
                    encoding='utf-8')
            rospy.logwarn(
                'OFFICIAL_TOPICS_PUBLISHED first-and-final static=%d dynamic=%d type=%s',
                len(static_rows[:20]), len(dynamic_rows[:20]),
                self.outer_type_name)
        except Exception as exc:
            rospy.logerr('official result publication rejected: %s', exc)


if __name__ == '__main__':
    try:
        node = CompetitionResultPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal('competition result publisher failed: %s', exc)
        raise
