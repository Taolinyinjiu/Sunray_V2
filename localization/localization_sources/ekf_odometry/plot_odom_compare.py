#!/usr/bin/env python3

import argparse
import os

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion


DEFAULT_BAG_FILE = "/home/yundrone/compare_all_ekf.bag"
MOCAP_TOPIC = "/uav1/sunray/mocap_odometry_relative"
METHODS = [
    ("FAST-LIO", "/Odometry", True),
    ("ekf_odometry", "/ekf_odometry", False),
    ("ekf_pose", "/ekf/ekf_odom", False),
]


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def parse_odom(bag, topic):
    rows = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        q = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ], dtype=float)
        roll, pitch, yaw = np.rad2deg(euler_from_quaternion(q, axes="sxyz"))
        rows.append([
            msg.header.stamp.to_sec(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            roll,
            pitch,
            yaw,
        ])

    if not rows:
        raise RuntimeError("No messages found on topic {}".format(topic))

    data = np.array(rows, dtype=float)
    order = np.argsort(data[:, 0])
    data = data[order]
    _, unique_idx = np.unique(data[:, 0], return_index=True)
    data = data[np.sort(unique_idx)]
    return {
        "topic": topic,
        "t": data[:, 0],
        "pos": data[:, 1:4],
        "vel": data[:, 4:7],
        "rpy": data[:, 7:10],
        "vel_source": "twist",
    }


def add_diff_velocity(data):
    vel = data["vel"].copy()
    speed_norm = np.linalg.norm(vel, axis=1)
    if len(data["t"]) >= 2 and np.percentile(speed_norm, 95) < 1.0e-9:
        t = data["t"]
        pos = data["pos"]
        diff_vel = np.zeros_like(pos)
        diff_vel[1:-1] = (pos[2:] - pos[:-2]) / (t[2:] - t[:-2])[:, None]
        diff_vel[0] = (pos[1] - pos[0]) / (t[1] - t[0])
        diff_vel[-1] = (pos[-1] - pos[-2]) / (t[-1] - t[-2])
        data["vel"] = diff_vel
        data["vel_source"] = "position_diff"
    return data


def interp_columns(t_src, values, t_dst):
    out = np.empty((len(t_dst), values.shape[1]))
    for i in range(values.shape[1]):
        out[:, i] = np.interp(t_dst, t_src, values[:, i])
    return out


def interp_angles_deg(t_src, values, t_dst):
    unwrapped = np.rad2deg(np.unwrap(np.deg2rad(values), axis=0))
    return interp_columns(t_src, unwrapped, t_dst)


def vector_stats(values):
    values = np.asarray(values)
    return {
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "mae": float(np.mean(np.abs(values))),
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        "max_abs": float(np.max(np.abs(values))),
    }


def norm_stats(err):
    norm = np.linalg.norm(err, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(norm ** 2))),
        "mae": float(np.mean(np.abs(norm))),
        "p95": float(np.percentile(norm, 95)),
        "max": float(np.max(norm)),
    }


def compute_publish_stats(t):
    dt = np.diff(t)
    if len(dt) == 0:
        raise RuntimeError("Need at least two messages to compute publish interval")
    median_dt = float(np.median(dt))
    large_threshold = max(0.02, 2.0 * median_dt)
    return {
        "count": int(len(t)),
        "start": float(t[0]),
        "end": float(t[-1]),
        "duration": float(t[-1] - t[0]),
        "hz": float((len(t) - 1) / (t[-1] - t[0])),
        "dt": dt,
        "dt_mean": float(np.mean(dt)),
        "dt_std": float(np.std(dt)),
        "dt_min": float(np.min(dt)),
        "dt_max": float(np.max(dt)),
        "dt_p50": float(np.percentile(dt, 50)),
        "dt_p95": float(np.percentile(dt, 95)),
        "dt_p99": float(np.percentile(dt, 99)),
        "large_threshold": large_threshold,
        "large_count": int(np.sum(dt > large_threshold)),
    }


def trim_to_overlap(data, mocap):
    mask = (data["t"] >= mocap["t"][0]) & (data["t"] <= mocap["t"][-1])
    if not np.any(mask):
        raise RuntimeError("{} has no time overlap with mocap".format(data["topic"]))
    return {k: (v[mask] if isinstance(v, np.ndarray) and len(v) == len(data["t"]) else v)
            for k, v in data.items()}


def align_yaw_pos_vel(est, mocap_interp):
    yaw0 = np.deg2rad(mocap_interp["rpy"][0, 2] - est["rpy"][0, 2])
    rot = np.array([
        [np.cos(yaw0), -np.sin(yaw0), 0.0],
        [np.sin(yaw0), np.cos(yaw0), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return {
        "pos": (rot @ (est["pos"] - est["pos"][0]).T).T + mocap_interp["pos"][0],
        "vel": (rot @ est["vel"].T).T,
        "rpy": np.column_stack([
            est["rpy"][:, 0],
            est["rpy"][:, 1],
            est["rpy"][:, 2] + np.rad2deg(yaw0),
        ]),
    }


def compute_errors(data, mocap):
    est = trim_to_overlap(data, mocap)
    mocap_interp = {
        "t": est["t"],
        "pos": interp_columns(mocap["t"], mocap["pos"], est["t"]),
        "vel": interp_columns(mocap["t"], mocap["vel"], est["t"]),
        "rpy": interp_angles_deg(mocap["t"], mocap["rpy"], est["t"]),
    }
    direct = {"pos": est["pos"], "vel": est["vel"], "rpy": est["rpy"]}
    aligned = align_yaw_pos_vel(est, mocap_interp)
    errors = {}
    for key, source in (("direct", direct), ("aligned", aligned)):
        pos_err = source["pos"] - mocap_interp["pos"]
        vel_err = source["vel"] - mocap_interp["vel"]
        rpy_err = wrap_deg(source["rpy"] - mocap_interp["rpy"])
        errors[key] = {
            "pos_err": pos_err,
            "vel_err": vel_err,
            "rpy_err": rpy_err,
            "pos_norm": np.linalg.norm(pos_err, axis=1),
            "vel_norm": np.linalg.norm(vel_err, axis=1),
            "rpy_norm": np.linalg.norm(rpy_err, axis=1),
            "pos_stats": norm_stats(pos_err),
            "vel_stats": norm_stats(vel_err),
            "rpy_stats": norm_stats(rpy_err),
            "pos_axis_stats": [vector_stats(pos_err[:, i]) for i in range(3)],
            "vel_axis_stats": [vector_stats(vel_err[:, i]) for i in range(3)],
            "rpy_axis_stats": [vector_stats(rpy_err[:, i]) for i in range(3)],
        }
    return est, mocap_interp, errors


def shifted_time(series):
    t0 = min(item["data"]["t"][0] for item in series)
    return {item["name"]: item["data"]["t"] - t0 for item in series}


def plot_timeseries(out_dir, methods, mocap):
    fig, axes = plt.subplots(3, 3, figsize=(17, 12))
    labels = [
        ("Position X (m)", "pos", 0, 0, 0),
        ("Position Y (m)", "pos", 1, 0, 1),
        ("Position Z (m)", "pos", 2, 0, 2),
        ("Velocity X (m/s)", "vel", 0, 1, 0),
        ("Velocity Y (m/s)", "vel", 1, 1, 1),
        ("Velocity Z (m/s)", "vel", 2, 1, 2),
        ("Roll (deg)", "rpy", 0, 2, 0),
        ("Pitch (deg)", "rpy", 1, 2, 1),
        ("Yaw (deg)", "rpy", 2, 2, 2),
    ]
    series = [{"name": "MOCAP", "data": mocap}] + methods
    times = shifted_time(series)
    for title, key, axis_idx, row, col in labels:
        ax = axes[row][col]
        ax.plot(times["MOCAP"], mocap[key][:, axis_idx], label="MOCAP", linewidth=1.0, linestyle="--")
        for item in methods:
            ax.plot(times[item["name"]], item["data"][key][:, axis_idx], label=item["name"], linewidth=1.0)
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.grid(True)
        ax.legend()
    fig.tight_layout()
    path = os.path.join(out_dir, "compare_timeseries.png")
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_trajectory(out_dir, results):
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    views = [("X (m)", "Y (m)", 0, 1), ("X (m)", "Z (m)", 0, 2), ("Y (m)", "Z (m)", 1, 2)]
    for ax, (xlabel, ylabel, i, j) in zip(axes, views):
        first = True
        for result in results:
            if first:
                mocap = result["mocap_interp"]
                ax.plot(mocap["pos"][:, i], mocap["pos"][:, j], label="MOCAP", linewidth=1.2, linestyle="--")
                first = False
            est = result["est_trim"]
            ax.plot(est["pos"][:, i], est["pos"][:, j], label=result["name"], linewidth=1.0)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
    fig.tight_layout()
    path = os.path.join(out_dir, "compare_trajectory.png")
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_publish_dt(out_dir, results):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    data = []
    labels = []
    for result in results:
        dt_ms = result["publish"]["dt"] * 1000.0
        axes[0].hist(dt_ms, bins=60, alpha=0.45, label=result["name"])
        data.append(dt_ms)
        labels.append(result["name"])
    axes[0].set_title("Publish Interval Histogram")
    axes[0].set_xlabel("dt (ms)")
    axes[0].set_ylabel("Count")
    axes[0].grid(True)
    axes[0].legend()
    for idx, values in enumerate(data, start=1):
        axes[1].boxplot(np.asarray(values), positions=[idx], widths=0.6, showfliers=True)
    axes[1].set_xticks(range(1, len(labels) + 1))
    axes[1].set_xticklabels(labels)
    axes[1].set_title("Publish Interval Boxplot")
    axes[1].set_ylabel("dt (ms)")
    axes[1].grid(True)
    fig.tight_layout()
    path = os.path.join(out_dir, "publish_dt_distribution.png")
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_error_distribution(out_dir, results, key, title, unit, filename):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    hist_values = []
    labels = []
    for result in results:
        norm = result["errors"]["aligned"][key + "_norm"]
        axes[0].hist(norm, bins=60, alpha=0.45, label=result["name"])
        hist_values.append(norm)
        labels.append(result["name"])
    axes[0].set_title("{} Norm Histogram".format(title))
    axes[0].set_xlabel(unit)
    axes[0].set_ylabel("Count")
    axes[0].grid(True)
    axes[0].legend()
    for idx, values in enumerate(hist_values, start=1):
        axes[1].boxplot(np.asarray(values), positions=[idx], widths=0.6, showfliers=True)
    axes[1].set_xticks(range(1, len(labels) + 1))
    axes[1].set_xticklabels(labels)
    axes[1].set_title("{} Norm Boxplot".format(title))
    axes[1].set_ylabel(unit)
    axes[1].grid(True)
    fig.tight_layout()
    path = os.path.join(out_dir, filename)
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def print_summary(results):
    print("\n========== 发布连续性 ==========")
    print("{:<13} {:>8} {:>9} {:>12} {:>12} {:>10}".format(
        "method", "count", "hz", "dt_p99_ms", "dt_max_ms", "large"))
    for result in results:
        s = result["publish"]
        print("{:<13} {:>8d} {:>9.2f} {:>12.3f} {:>12.3f} {:>10d}".format(
            result["name"], s["count"], s["hz"], s["dt_p99"] * 1000.0, s["dt_max"] * 1000.0, s["large_count"]))

    print("\n========== 对齐后误差汇总 ==========")
    print("{:<13} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}".format(
        "method", "pos_rmse", "pos_p95", "pos_max", "vel_rmse", "vel_p95", "vel_max", "rpy_rmse", "rpy_p95", "rpy_max"))
    for result in results:
        e = result["errors"]["aligned"]
        ps, vs, rs = e["pos_stats"], e["vel_stats"], e["rpy_stats"]
        print("{:<13} {:>11.4f} {:>11.4f} {:>11.4f} {:>11.4f} {:>11.4f} {:>11.4f} {:>11.4f} {:>11.4f} {:>11.4f}".format(
            result["name"],
            ps["rmse"], ps["p95"], ps["max"],
            vs["rmse"], vs["p95"], vs["max"],
            rs["rmse"], rs["p95"], rs["max"],
        ))

    print("\n========== 速度来源 ==========")
    for result in results:
        print("{}: {}".format(result["name"], result["data"]["vel_source"]))

    print("\n========== 直接相减误差检查 ==========")
    for result in results:
        e = result["errors"]["direct"]
        print("{}: pos RMSE {:.4f} m, vel RMSE {:.4f} m/s, rpy RMSE {:.4f} deg".format(
            result["name"], e["pos_stats"]["rmse"], e["vel_stats"]["rmse"], e["rpy_stats"]["rmse"]))


def main():
    parser = argparse.ArgumentParser(description="Compare FAST-LIO, ekf_odometry and ekf_pose against mocap.")
    parser.add_argument("--bag", default=DEFAULT_BAG_FILE, help="Result bag with all comparison topics.")
    parser.add_argument("--out-dir", default=None, help="Directory for figures.")
    args = parser.parse_args()

    out_dir = args.out_dir or os.path.join(os.path.dirname(args.bag), "all_ekf_compare_results")
    os.makedirs(out_dir, exist_ok=True)

    methods = []
    results = []
    with rosbag.Bag(args.bag, "r") as bag:
        mocap = parse_odom(bag, MOCAP_TOPIC)
        for name, topic, diff_velocity_if_empty in METHODS:
            try:
                data = parse_odom(bag, topic)
            except RuntimeError as exc:
                print("WARN: {}".format(exc))
                continue
            if diff_velocity_if_empty:
                data = add_diff_velocity(data)
            methods.append({"name": name, "topic": topic, "data": data})

    if not methods:
        raise RuntimeError("No estimation topics found")

    for item in methods:
        publish = compute_publish_stats(item["data"]["t"])
        est_trim, mocap_interp, errors = compute_errors(item["data"], mocap)
        result = dict(item)
        result["publish"] = publish
        result["est_trim"] = est_trim
        result["mocap_interp"] = mocap_interp
        result["errors"] = errors
        results.append(result)

    paths = [
        plot_timeseries(out_dir, methods, mocap),
        plot_trajectory(out_dir, results),
        plot_publish_dt(out_dir, results),
        plot_error_distribution(out_dir, results, "pos", "Position Error", "m", "position_error_distribution.png"),
        plot_error_distribution(out_dir, results, "vel", "Velocity Error", "m/s", "velocity_error_distribution.png"),
        plot_error_distribution(out_dir, results, "rpy", "Attitude Error", "deg", "attitude_error_distribution.png"),
    ]

    print_summary(results)
    print("\n========== 输出图表 ==========")
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()
