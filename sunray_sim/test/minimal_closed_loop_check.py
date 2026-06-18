#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import tempfile
import time

import roslaunch
import rospy
import rospkg
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray


class TopicMonitor:
    def __init__(self, agent_prefix, check_sensing):
        self.agent_prefix = agent_prefix
        self.check_sensing = check_sensing
        self.odom_samples = []
        self.imu_count = 0
        self.state_count = 0
        self.cloud_count = 0
        self.visualization_count = 0
        self.cmd_rpm_count = 0
        self.max_avg_rpm = 0.0

        rospy.Subscriber(agent_prefix + "/sunray_sim/odom", Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(agent_prefix + "/sunray_sim/imu", Imu, self.imu_cb, queue_size=50)
        rospy.Subscriber(agent_prefix + "/mavros/state", State, self.state_cb, queue_size=50)
        rospy.Subscriber(agent_prefix + "/sunray_sim/cmd_RPM", Float32MultiArray, self.cmd_rpm_cb, queue_size=50)
        rospy.Subscriber(agent_prefix + "/sunray_sim/visualization", MarkerArray, self.visualization_cb, queue_size=10)
        if check_sensing:
            rospy.Subscriber(agent_prefix + "/sunray_sim/cloud_world_frame", PointCloud2, self.cloud_cb, queue_size=10)

    def odom_cb(self, msg):
        stamp = rospy.get_time()
        self.odom_samples.append((stamp, msg.pose.pose.position.z, msg.twist.twist.linear.z))
        if len(self.odom_samples) > 2000:
            self.odom_samples = self.odom_samples[-2000:]

    def imu_cb(self, _msg):
        self.imu_count += 1

    def state_cb(self, _msg):
        self.state_count += 1

    def cloud_cb(self, _msg):
        self.cloud_count += 1

    def visualization_cb(self, _msg):
        self.visualization_count += 1

    def cmd_rpm_cb(self, msg):
        self.cmd_rpm_count += 1
        if msg.data:
            self.max_avg_rpm = max(self.max_avg_rpm, sum(msg.data) / float(len(msg.data)))

    def has_basic_topics(self):
        if not self.odom_samples or self.imu_count <= 0 or self.state_count <= 0:
            return False
        if self.check_sensing and self.cloud_count <= 0:
            return False
        return True

    def latest_z(self):
        if not self.odom_samples:
            return float("nan")
        return self.odom_samples[-1][1]


def make_position_target(frame_id, target_z):
    msg = PositionTarget()
    msg.header.frame_id = frame_id
    msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    msg.type_mask = (
        PositionTarget.IGNORE_VX
        | PositionTarget.IGNORE_VY
        | PositionTarget.IGNORE_VZ
        | PositionTarget.IGNORE_AFX
        | PositionTarget.IGNORE_AFY
        | PositionTarget.IGNORE_AFZ
        | PositionTarget.IGNORE_YAW
        | PositionTarget.IGNORE_YAW_RATE
    )
    msg.position.x = 0.0
    msg.position.y = 0.0
    msg.position.z = target_z
    return msg


def has_launch_arg(extra_launch_args, name):
    prefix = name + ":="
    return any(arg.startswith(prefix) for arg in extra_launch_args)


def create_node_config(args):
    config_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="sunray_sim_test_node_",
        suffix=".yaml",
        delete=False,
    )
    config_file.write("# minimal_closed_loop_check.py 自动生成的临时主节点配置。\n")
    config_file.write("agent_name: %s\n" % args.agent_name)
    config_file.write("agent_ids: [%d]\n" % args.agent_id)
    config_file.write("global_frame_id: %s\n" % args.global_frame)
    config_file.write("enable_sensing: true\n")
    config_file.write("enable_status_print: false\n")
    config_file.write("status_print_hz: 1.0\n")
    config_file.close()
    return config_file.name


def create_single_uav_config(args):
    vehicle_name = args.agent_name + str(args.agent_id)
    config_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="sunray_sim_test_vehicle_",
        suffix=".yaml",
        delete=False,
    )
    config_file.write("# minimal_closed_loop_check.py 自动生成的临时无人机初始位姿配置。\n")
    config_file.write("vehicles:\n")
    config_file.write("  %s:\n" % vehicle_name)
    config_file.write("    init_x: 0.0\n")
    config_file.write("    init_y: 0.0\n")
    config_file.write("    init_z: 0.0\n")
    config_file.write("    init_yaw: 0.0\n")
    config_file.close()
    return config_file.name


def start_launch(args):
    package_path = rospkg.RosPack().get_path("sunray_sim")
    launch_file = os.path.join(package_path, "launch", "sunray_sim.launch")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    temp_node_config = None
    temp_single_uav_config = None
    launch_args = ["rviz:=false"] + args.launch_arg
    if not args.keep_status_print and not has_launch_arg(launch_args, "node_config"):
        temp_node_config = create_node_config(args)
        launch_args.append("node_config:=" + temp_node_config)
    if not has_launch_arg(launch_args, "single_uav_simulator_config"):
        temp_single_uav_config = create_single_uav_config(args)
        launch_args.append("single_uav_simulator_config:=" + temp_single_uav_config)
    parent = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
    parent.start()
    return parent, temp_node_config, temp_single_uav_config


def wait_for_odom(monitor, timeout):
    deadline = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < deadline:
        if monitor.odom_samples:
            return True
        rospy.sleep(0.05)
    return False


def evaluate(monitor, args, start_z):
    failures = []
    if not monitor.odom_samples:
        failures.append("没有收到 odom")
        return failures

    final_z = monitor.odom_samples[-1][1]
    max_z = max(sample[1] for sample in monitor.odom_samples)
    rise = max_z - start_z
    final_error = abs(final_z - args.target_z)

    recent_start = monitor.odom_samples[-1][0] - args.settle_window
    recent = [sample for sample in monitor.odom_samples if sample[0] >= recent_start]
    recent_z_values = [sample[1] for sample in recent]
    recent_vz_values = [abs(sample[2]) for sample in recent]
    stable_range = max(recent_z_values) - min(recent_z_values) if recent_z_values else float("inf")
    max_recent_vz = max(recent_vz_values) if recent_vz_values else float("inf")

    if monitor.imu_count <= 0:
        failures.append("没有收到 imu")
    if monitor.state_count <= 0:
        failures.append("没有收到 mavros/state")
    if args.check_sensing and monitor.cloud_count <= 0:
        failures.append("没有收到局部点云 cloud_world_frame")
    if args.check_visualization and monitor.visualization_count <= 0:
        failures.append("没有收到 visualization MarkerArray")
    if monitor.cmd_rpm_count <= 0 or monitor.max_avg_rpm < args.min_avg_rpm:
        failures.append("没有收到有效 cmd_RPM 输出")
    if rise < args.min_rise:
        failures.append("odom z 没有明显上升")
    if final_error > args.max_final_error:
        failures.append("最终高度误差过大")
    if stable_range > args.max_stable_range and max_recent_vz > args.max_stable_vz:
        failures.append("最近窗口高度仍不稳定")

    print("")
    print("================ sunray_sim 最小闭环验证结果 ================")
    print("odom 样本数: %d" % len(monitor.odom_samples))
    print("imu/state/cloud/visualization/cmd_RPM: %d / %d / %d / %d / %d" % (
        monitor.imu_count,
        monitor.state_count,
        monitor.cloud_count,
        monitor.visualization_count,
        monitor.cmd_rpm_count,
    ))
    print("起始 z: %.3f m  最高 z: %.3f m  最终 z: %.3f m  目标 z: %.3f m" % (
        start_z,
        max_z,
        final_z,
        args.target_z,
    ))
    print("上升量: %.3f m  最终误差: %.3f m" % (rise, final_error))
    print("最近 %.1f s 高度范围: %.3f m  最大 |vz|: %.3f m/s" % (
        args.settle_window,
        stable_range,
        max_recent_vz,
    ))
    print("最大平均 RPM: %.1f" % monitor.max_avg_rpm)
    if failures:
        print("结论: 失败")
        for failure in failures:
            print("  - " + failure)
    else:
        print("结论: 通过")
    print("===================================================================")
    return failures


def parse_args():
    parser = argparse.ArgumentParser(description="sunray_sim 最小闭环验证脚本")
    parser.add_argument("--no-launch", action="store_true", help="不自动启动 launch，只检查已经运行的仿真节点")
    parser.add_argument("--launch-arg", action="append", default=[], help="传给 roslaunch 的额外参数，例如 node_config:=xxx.yaml")
    parser.add_argument("--keep-status-print", action="store_true", help="自动启动 launch 时保留仿真节点状态面板打印")
    parser.add_argument("--agent-name", default="uav", help="无人机名称前缀")
    parser.add_argument("--agent-id", type=int, default=1, help="无人机编号")
    parser.add_argument("--global-frame", default="map", help="setpoint header.frame_id")
    parser.add_argument("--target-z", type=float, default=1.0, help="悬停目标高度，单位 m")
    parser.add_argument("--timeout", type=float, default=18.0, help="发布 setpoint 和采样总时长，单位 s")
    parser.add_argument("--startup-timeout", type=float, default=10.0, help="等待首个 odom 的最长时间，单位 s")
    parser.add_argument("--publish-rate", type=float, default=30.0, help="setpoint 发布频率，单位 Hz")
    parser.add_argument("--settle-window", type=float, default=3.0, help="稳定性检查窗口，单位 s")
    parser.add_argument("--min-rise", type=float, default=0.35, help="认为起飞成功所需的最小 z 上升量，单位 m")
    parser.add_argument("--max-final-error", type=float, default=0.45, help="最终高度允许误差，单位 m")
    parser.add_argument("--max-stable-range", type=float, default=0.35, help="稳定窗口允许高度范围，单位 m")
    parser.add_argument("--max-stable-vz", type=float, default=0.35, help="稳定窗口允许最大垂直速度，单位 m/s")
    parser.add_argument("--min-avg-rpm", type=float, default=100.0, help="认为电机输出有效的最小平均 RPM")
    parser.add_argument("--no-sensing-check", dest="check_sensing", action="store_false", help="不检查局部点云")
    parser.add_argument("--no-visualization-check", dest="check_visualization", action="store_false", help="不检查 RViz MarkerArray")
    parser.set_defaults(check_sensing=True, check_visualization=True)
    return parser.parse_args()


def main():
    args = parse_args()
    args.agent_id = max(args.agent_id, 1)
    agent_prefix = "/" + args.agent_name + str(args.agent_id)

    launch_parent = None
    temp_node_config = None
    temp_single_uav_config = None
    try:
        if not args.no_launch:
            print("[sunray_sim test] 启动仿真 launch...")
            launch_parent, temp_node_config, temp_single_uav_config = start_launch(args)
            time.sleep(1.0)

        rospy.init_node("sunray_sim_minimal_closed_loop_check", anonymous=True)
        monitor = TopicMonitor(agent_prefix, args.check_sensing)
        pub = rospy.Publisher(agent_prefix + "/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        print("[sunray_sim test] 等待 odom...")
        if not wait_for_odom(monitor, args.startup_timeout):
            print("[sunray_sim test] 失败：启动后没有收到 odom")
            return 1

        start_z = monitor.odom_samples[0][1]
        target = make_position_target(args.global_frame, args.target_z)
        rate = rospy.Rate(max(args.publish_rate, 1.0))
        deadline = rospy.Time.now() + rospy.Duration(args.timeout)

        print("[sunray_sim test] 发布悬停 setpoint: z=%.2f m, topic=%s" % (
            args.target_z,
            agent_prefix + "/mavros/setpoint_raw/local",
        ))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            target.header.stamp = rospy.Time.now()
            pub.publish(target)
            rate.sleep()

        failures = evaluate(monitor, args, start_z)
        return 1 if failures else 0
    finally:
        if launch_parent is not None:
            print("[sunray_sim test] 关闭仿真 launch...")
            launch_parent.shutdown()
        if temp_node_config and os.path.exists(temp_node_config):
            os.unlink(temp_node_config)
        if temp_single_uav_config and os.path.exists(temp_single_uav_config):
            os.unlink(temp_single_uav_config)


if __name__ == "__main__":
    sys.exit(main())
