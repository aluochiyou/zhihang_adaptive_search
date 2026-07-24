#!/usr/bin/env python3
"""Measure one ROS topic with wall-clock time and exit normally.

This replaces the fragile shell pattern `timeout rostopic hz`, whose expected
SIGTERM exit status is 124 and was incorrectly treated as a preflight failure.
"""
from __future__ import print_function

import argparse
import threading
import time

import rospy
import rostopic


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('topic')
    parser.add_argument('--minimum-hz', type=float, default=10.0)
    parser.add_argument('--sample-seconds', type=float, default=3.0)
    parser.add_argument('--topic-timeout-seconds', type=float, default=12.0)
    parser.add_argument('--vehicle-id', type=int, default=-1)
    args = parser.parse_args()

    rospy.init_node(
        'zhihang_v5_2_topic_rate_probe_{}'.format(args.vehicle_id),
        anonymous=True,
        disable_signals=True,
    )

    deadline = time.monotonic() + args.topic_timeout_seconds
    msg_class = None
    real_topic = args.topic
    while time.monotonic() < deadline and not rospy.is_shutdown():
        msg_class, resolved, _ = rostopic.get_topic_class(args.topic, blocking=False)
        if msg_class is not None:
            real_topic = resolved or args.topic
            break
        time.sleep(0.1)
    if msg_class is None:
        raise SystemExit('[ERROR] topic type unavailable: {}'.format(args.topic))

    wall_times = []
    lock = threading.Lock()

    def callback(_msg):
        with lock:
            wall_times.append(time.monotonic())

    sub = rospy.Subscriber(real_topic, msg_class, callback, queue_size=1)
    try:
        first_deadline = time.monotonic() + args.topic_timeout_seconds
        while time.monotonic() < first_deadline and not rospy.is_shutdown():
            with lock:
                if wall_times:
                    break
            time.sleep(0.02)
        with lock:
            has_first = bool(wall_times)
        if not has_first:
            raise SystemExit('[ERROR] no message received from {}'.format(real_topic))

        sample_deadline = time.monotonic() + args.sample_seconds
        while time.monotonic() < sample_deadline and not rospy.is_shutdown():
            time.sleep(0.02)
    finally:
        sub.unregister()

    with lock:
        samples = list(wall_times)
    if len(samples) < 2 or samples[-1] <= samples[0]:
        raise SystemExit('[ERROR] insufficient samples on {}: {}'.format(real_topic, len(samples)))

    rate = float(len(samples) - 1) / float(samples[-1] - samples[0])
    print('[CHECK] {} wall-clock rate = {:.3f} Hz ({} frames/{:.2f}s)'.format(
        real_topic, rate, len(samples), samples[-1] - samples[0]))
    if rate < args.minimum_hz:
        raise SystemExit('[ERROR] camera source rate {:.3f} Hz below {:.3f} Hz'.format(
            rate, args.minimum_hz))
    print('[OK] topic rate requirement passed')


if __name__ == '__main__':
    main()
