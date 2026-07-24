#!/usr/bin/env python3
import argparse
import json
import threading
import time

import rospy
from std_msgs.msg import String

NS = '/zhihang/search_v6'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=300.0)
    args = parser.parse_args()
    rospy.init_node('wait_v6_manager_start', anonymous=True, disable_signals=True)
    lock = threading.RLock()
    state = {'status': None, 'start': None}

    def status_cb(msg):
        try:
            row = json.loads(msg.data)
        except Exception:
            return
        with lock:
            state['status'] = row

    def start_cb(msg):
        try:
            row = json.loads(msg.data)
        except Exception:
            return
        with lock:
            state['start'] = row

    rospy.Subscriber(f'{NS}/manager/status', String, status_cb, queue_size=5)
    rospy.Subscriber(f'{NS}/manager/start', String, start_cb, queue_size=1)
    deadline = time.monotonic() + args.timeout
    last_print = 0.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        with lock:
            status = state['status']
            start = state['start']
        if start:
            print('[OK] manager start barrier published')
            print(json.dumps(start, ensure_ascii=False, indent=2))
            return 0
        now = time.monotonic()
        if now - last_print >= 2.0:
            if status:
                print('[WAIT] ACK flight={flight_ack} perception={perception_ack} | ready flight={flight_ready} perception={perception_ready}'.format(**status))
                p = status.get('perception_ready', {})
                app_ready = bool(status.get('application_ready', False))
                authorized = bool(status.get('start_authorized', False))
                if not all(bool(p.get(str(i), p.get(i, False))) for i in (0, 1, 2)):
                    print('[INFO] A flight agent at READY is healthy; manager is still waiting for all three YOLO pipelines >=10 Hz.')
                elif app_ready and not authorized:
                    print('[INFO] All application nodes are ready; manager is intentionally waiting for model_state.py and explicit start authorization.')
                elif authorized:
                    print('[INFO] Start authorization received; manager is preparing the synchronized start epoch.')
            else:
                print('[WAIT] manager status topic not received yet')
            last_print = now
        time.sleep(0.2)
    print(f'[ERROR] manager did not publish start within {args.timeout:.0f}s')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
