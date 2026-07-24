#!/usr/bin/env python3
import argparse
import json
import os
import time

import rospy
from std_msgs.msg import String


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--reason', default='model_state_started_after_application_ready')
    ap.add_argument('--marker', default='/tmp/zhihang_v6_7_model_state_started.json')
    args = ap.parse_args()
    rospy.init_node('authorize_v6_7_manager_start', anonymous=True, disable_signals=True)
    mission = ''
    status = rospy.wait_for_message('/zhihang/search_v6/manager/status', String, timeout=10.0)
    try:
        mission = str(json.loads(status.data).get('mission_id', ''))
    except Exception:
        pass
    marker = {}
    if os.path.isfile(args.marker):
        try:
            marker = json.load(open(args.marker, encoding='utf-8'))
        except Exception:
            marker = {}
    payload = {
        'authorized': True,
        'mission_id': mission,
        'reason': args.reason,
        'ros_time': rospy.Time.now().to_sec(),
        'wall_time': time.time(),
        'target_motion_marker': marker,
    }
    pub = rospy.Publisher('/zhihang/search_v6/manager/start_authorization', String,
                          queue_size=1, latch=True)
    deadline = time.monotonic() + 1.5
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        rate.sleep()
    print('[OK] manager start authorized:', json.dumps(payload, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
