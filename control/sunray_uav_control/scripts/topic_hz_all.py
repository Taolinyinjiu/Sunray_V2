#!/usr/bin/env python3
"""Report frequency for all ROS topics from `rostopic list`.

Usage:
  rosrun uav_control topic_hz_all.py
  ./topic_hz_all.py --window 5

Notes:
- Uses rostopic.ROSTopicHz to sample all topics for a window.
- Default: 5 seconds sampling, 100 msg cap per topic.
"""

import argparse
import sys
import time

import rospy
import rostopic


def parse_args():
    parser = argparse.ArgumentParser(description="Show Hz for all ROS topics")
    parser.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Sampling window in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--max-messages",
        type=int,
        default=100,
        help="Max messages to keep per topic (default: 100)",
    )
    parser.add_argument(
        "--include",
        type=str,
        default="",
        help="Only include topics containing this substring",
    )
    parser.add_argument(
        "--exclude",
        type=str,
        default="",
        help="Exclude topics containing this substring",
    )
    parser.add_argument(
        "--sort",
        type=str,
        default="name",
        choices=["name", "hz"],
        help="Sort by topic name or Hz (default: name)",
    )
    parser.add_argument(
        "--use-wall-time",
        action="store_true",
        help="Use wall time instead of ROS time",
    )
    return parser.parse_args()


def should_keep(name, include, exclude):
    if include and include not in name:
        return False
    if exclude and exclude in name:
        return False
    return True


def main():
    args = parse_args()
    rospy.init_node("topic_hz_all", anonymous=True, disable_signals=True)

    try:
        topic_entries = rostopic.get_topic_list()[0]
    except Exception as exc:
        print(f"[topic_hz_all] failed to list topics: {exc}", file=sys.stderr)
        return 1

    # rostopic.get_topic_list() can return a list of [name, type] pairs.
    topic_list = []
    for entry in topic_entries:
        if isinstance(entry, (list, tuple)):
            if not entry:
                continue
            name = entry[0]
        else:
            name = entry
        if isinstance(name, str) and should_keep(name, args.include, args.exclude):
            topic_list.append(name)
    if not topic_list:
        print("[topic_hz_all] no topics to sample")
        return 0

    # Auto-select wall time if sim time is enabled but /clock is not present.
    use_wall_time = args.use_wall_time
    if not use_wall_time:
        try:
            use_sim_time = rospy.get_param("/use_sim_time", False)
        except Exception:
            use_sim_time = False
        if use_sim_time and "/clock" not in topic_list:
            use_wall_time = True

    # Create Hz trackers per topic and subscribe.
    trackers = {}
    subscribers = []
    for topic in topic_list:
        try:
            msg_class, real_topic, msg_eval = rostopic.get_topic_class(
                topic, blocking=False
            )
        except Exception as exc:
            print(f"[topic_hz_all] type lookup failed: {topic} ({exc})", file=sys.stderr)
            continue

        if msg_class is None or real_topic is None:
            print(f"[topic_hz_all] unknown type: {topic}", file=sys.stderr)
            continue

        hz = rostopic.ROSTopicHz(args.max_messages, use_wtime=use_wall_time)
        trackers[real_topic] = hz

        def _cb(msg, _hz=hz, _topic=real_topic, _eval=msg_eval):
            if _eval is not None:
                try:
                    msg = _eval(msg)
                except Exception:
                    return
            _hz.callback_hz(msg, topic=_topic)

        try:
            subscribers.append(
                rospy.Subscriber(real_topic, msg_class, _cb, queue_size=10)
            )
        except Exception as exc:
            print(
                f"[topic_hz_all] subscribe failed: {real_topic} ({exc})",
                file=sys.stderr,
            )

    # Sample for the given window.
    end_time = time.time() + args.window
    while time.time() < end_time and not rospy.is_shutdown():
        rospy.sleep(0.05)

    rows = []
    for topic, hz in trackers.items():
        stats = hz.get_hz()
        if stats is None:
            hz_val = 0.0
            min_dt = max_dt = std_dt = 0.0
        else:
            hz_val, min_dt, max_dt, std_dt = stats
        rows.append((topic, hz_val, min_dt, max_dt, std_dt))

    if args.sort == "hz":
        rows.sort(key=lambda r: r[1], reverse=True)
    else:
        rows.sort(key=lambda r: r[0])

    print(
        f"{'topic':<50} {'Hz':>8} {'min_dt':>8} {'max_dt':>8} {'std_dt':>8}"
    )
    for topic, hz_val, min_dt, max_dt, std_dt in rows:
        print(
            f"{topic:<50} {hz_val:>8.3f} {min_dt:>8.4f} {max_dt:>8.4f} {std_dt:>8.4f}"
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
