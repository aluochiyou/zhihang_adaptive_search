#!/usr/bin/env python3
import argparse
import json
import sys
import time

import rospy
from std_msgs.msg import Bool, String


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=360.0)
    args = ap.parse_args()
    rospy.init_node('wait_v6_7_application_ready', anonymous=True, disable_signals=True)
    state = {'ready': False, 'status': {}}

    def ready_cb(msg: Bool):
        state['ready'] = bool(msg.data)

    def status_cb(msg: String):
        try:
            state['status'] = json.loads(msg.data)
        except Exception:
            pass

    rospy.Subscriber('/zhihang/search_v6/manager/application_ready', Bool, ready_cb, queue_size=1)
    rospy.Subscriber('/zhihang/search_v6/manager/status', String, status_cb, queue_size=1)
    deadline = time.monotonic() + args.timeout
    last_print = 0.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if state['ready'] or bool(state['status'].get('application_ready', False)):
            row = state['status']
            print('[OK] manager application ready: three flight agents and three YOLO pipelines passed')
            print('[INFO] target motion may now start; manager is held at the start-authorization barrier')
            if row:
                print('[STATUS]', json.dumps({
                    'mission_id': row.get('mission_id'),
                    'flight_ready': row.get('flight_ready'),
                    'perception_ready': row.get('perception_ready'),
                    'start_authorized': row.get('start_authorized'),
                }, ensure_ascii=False))
            return 0
        now = time.monotonic()
        if now - last_print >= 2.0:
            row = state['status']
            print('[WAIT] application_ready=false', json.dumps({
                'flight_ready': row.get('flight_ready'),
                'perception_ready': row.get('perception_ready'),
            }, ensure_ascii=False))
            last_print = now
        rospy.sleep(0.2)
    print('[ERROR] manager did not reach application_ready before timeout', file=sys.stderr)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
