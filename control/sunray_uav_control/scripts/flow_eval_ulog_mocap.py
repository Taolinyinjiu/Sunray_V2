#!/usr/bin/env python3
"""Evaluate PX4 optical-flow performance using ULog + mocap CSV.

This script targets indoor tests where external vision fusion is disabled, and
provides KPI + pass/fail checks for:
- optical-flow fusion activity and fault indicators
- innovation test ratios
- trajectory tracking error against mocap truth

Dependencies:
- numpy
- pyulog (for .ulg parsing)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    from pyulog import ULog
except Exception:  # pragma: no cover - runtime dependency check
    ULog = None


# Bits from PX4 EstimatorStatus.msg (checked against PX4-Autopilot main)
CS_OPT_FLOW_BIT = 3
FS_BAD_OPTFLOW_X_BIT = 7
FS_BAD_OPTFLOW_Y_BIT = 8


@dataclass
class CheckResult:
    name: str
    status: str
    value: Optional[float]
    threshold: Optional[float]
    comparator: str
    note: str = ""


class EvalError(RuntimeError):
    """User-facing evaluation error."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ULog + mocap optical-flow evaluator with KPI and pass/fail"
    )
    parser.add_argument("--ulog", required=True, help="Path to PX4 .ulg file")
    parser.add_argument("--mocap-csv", required=True, help="Path to mocap CSV file")
    parser.add_argument(
        "--output-json",
        default="",
        help="Optional output JSON report path (default: print summary only)",
    )

    # Mocap CSV mapping
    parser.add_argument(
        "--mocap-time-col",
        default="",
        help="Mocap time column (default: auto-detect, e.g. %%time/timestamp)",
    )
    parser.add_argument(
        "--mocap-x-col",
        default="",
        help="Mocap x column (default: auto-detect)",
    )
    parser.add_argument(
        "--mocap-y-col",
        default="",
        help="Mocap y column (default: auto-detect)",
    )
    parser.add_argument(
        "--mocap-z-col",
        default="",
        help="Mocap z column (default: auto-detect)",
    )
    parser.add_argument(
        "--mocap-time-unit",
        choices=["auto", "s", "ms", "us", "ns"],
        default="auto",
        help="Mocap timestamp unit",
    )
    parser.add_argument(
        "--mocap-frame",
        choices=["enu", "ned"],
        default="enu",
        help="Mocap position frame (default: enu). ULog local position is assumed NED.",
    )

    # Time sync
    parser.add_argument(
        "--manual-time-shift-sec",
        type=float,
        default=None,
        help=(
            "Manual time shift applied to mocap timeline (est_time = mocap_time + shift). "
            "If unset, auto-search is used."
        ),
    )
    parser.add_argument("--time-shift-min-sec", type=float, default=-3.0)
    parser.add_argument("--time-shift-max-sec", type=float, default=3.0)
    parser.add_argument("--time-shift-step-sec", type=float, default=0.02)
    parser.add_argument(
        "--align-dt-sec",
        type=float,
        default=0.02,
        help="Resampling interval for alignment and KPI computation",
    )

    # Optional hover window in aligned timeline (seconds from overlap start)
    parser.add_argument(
        "--hover-start-sec",
        type=float,
        default=0.0,
        help="Hover window start in aligned timeline",
    )
    parser.add_argument(
        "--hover-end-sec",
        type=float,
        default=None,
        help="Hover window end in aligned timeline (default: full overlap)",
    )

    # Flow/estimator thresholds
    parser.add_argument("--flow-quality-min", type=float, default=100.0)
    parser.add_argument("--min-flow-quality-ratio", type=float, default=0.90)
    parser.add_argument("--min-flow-active-ratio", type=float, default=0.95)
    parser.add_argument("--max-flow-fault-ratio", type=float, default=0.01)
    parser.add_argument("--max-flow-test-ratio-p95", type=float, default=1.0)
    parser.add_argument("--max-vel-test-ratio-p95", type=float, default=1.0)
    parser.add_argument("--max-pos-test-ratio-p95", type=float, default=1.0)
    parser.add_argument("--max-hgt-test-ratio-p95", type=float, default=1.0)
    parser.add_argument("--max-hagl-test-ratio-p95", type=float, default=1.0)

    # Task-level performance thresholds
    parser.add_argument("--min-overlap-sec", type=float, default=20.0)
    parser.add_argument("--max-xy-rmse", type=float, default=0.15)
    parser.add_argument("--max-xy-p95", type=float, default=0.35)
    parser.add_argument("--max-drift-speed", type=float, default=0.05)

    return parser.parse_args()


def require_pyulog() -> None:
    if ULog is None:
        raise EvalError(
            "pyulog is not available. Install with: pip3 install pyulog"
        )


def as_float_array(values: np.ndarray) -> np.ndarray:
    arr = np.asarray(values)
    if arr.dtype == np.bool_:
        return arr.astype(np.float64)
    if np.issubdtype(arr.dtype, np.integer):
        return arr.astype(np.float64)
    return arr.astype(np.float64, copy=False)


def get_topic_entries(ulog: ULog, topic_name: str):
    entries = [d for d in ulog.data_list if d.name == topic_name]
    entries.sort(key=lambda d: d.multi_id)
    return entries


def get_first_topic(ulog: ULog, topic_name: str):
    entries = get_topic_entries(ulog, topic_name)
    return entries[0] if entries else None


def field_exists(data: Dict[str, np.ndarray], name: str) -> bool:
    return name in data and len(data[name]) > 0


def get_field(
    data: Dict[str, np.ndarray],
    candidates: Sequence[str],
    *,
    required: bool = False,
) -> Optional[np.ndarray]:
    for name in candidates:
        if field_exists(data, name):
            return as_float_array(data[name])
    if required:
        raise EvalError(f"Missing required field. Tried: {', '.join(candidates)}")
    return None


def get_array_components(data: Dict[str, np.ndarray], base: str) -> List[np.ndarray]:
    comps: List[Tuple[int, np.ndarray]] = []
    for key, values in data.items():
        if not key.startswith(base + "["):
            continue
        suffix = key[len(base) + 1 :]
        if not suffix.endswith("]"):
            continue
        idx_str = suffix[:-1]
        if not idx_str.isdigit():
            continue
        comps.append((int(idx_str), as_float_array(values)))
    comps.sort(key=lambda item: item[0])
    return [v for _, v in comps]


def finite_mask(*arrays: np.ndarray) -> np.ndarray:
    if not arrays:
        return np.array([], dtype=bool)
    mask = np.ones_like(arrays[0], dtype=bool)
    for arr in arrays:
        mask &= np.isfinite(arr)
    return mask


def convert_mocap_time_unit(raw_t: np.ndarray, unit: str) -> np.ndarray:
    if raw_t.size == 0:
        raise EvalError("Mocap CSV is empty")

    if unit == "s":
        scale = 1.0
    elif unit == "ms":
        scale = 1e-3
    elif unit == "us":
        scale = 1e-6
    elif unit == "ns":
        scale = 1e-9
    else:
        # Basic heuristic for common timestamps.
        max_abs = float(np.nanmax(np.abs(raw_t)))
        if max_abs > 1e16:
            scale = 1e-9
        elif max_abs > 1e13:
            scale = 1e-6
        elif max_abs > 1e10:
            scale = 1e-3
        else:
            scale = 1.0
    return raw_t * scale


def pick_mocap_column(
    fieldnames: Sequence[str],
    user_col: str,
    candidates: Sequence[str],
    role: str,
) -> str:
    available = list(fieldnames)

    if user_col:
        if user_col in available:
            return user_col
        raise EvalError(
            f"Configured mocap {role} column '{user_col}' not found. "
            f"Available columns: {available}"
        )

    # Exact match first.
    for name in candidates:
        if name in available:
            return name

    # Case-insensitive fallback.
    lower_map = {name.lower(): name for name in available}
    for name in candidates:
        hit = lower_map.get(name.lower())
        if hit is not None:
            return hit

    raise EvalError(
        f"Unable to auto-detect mocap {role} column. "
        f"Tried candidates: {list(candidates)}. Available columns: {available}"
    )


def load_mocap_csv(args: argparse.Namespace) -> Tuple[np.ndarray, np.ndarray]:
    csv_path = Path(args.mocap_csv)
    if not csv_path.exists():
        raise EvalError(f"Mocap CSV not found: {csv_path}")

    rows_t: List[float] = []
    rows_x: List[float] = []
    rows_y: List[float] = []
    rows_z: List[float] = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise EvalError("Mocap CSV has no header row")

        time_col = pick_mocap_column(
            reader.fieldnames,
            args.mocap_time_col,
            ("%time", "timestamp", "time", "stamp"),
            "time",
        )
        x_col = pick_mocap_column(
            reader.fieldnames,
            args.mocap_x_col,
            (
                "x",
                "field.pose.position.x",
                "field.pose.pose.position.x",
                "pose.position.x",
                "pose.pose.position.x",
            ),
            "x",
        )
        y_col = pick_mocap_column(
            reader.fieldnames,
            args.mocap_y_col,
            (
                "y",
                "field.pose.position.y",
                "field.pose.pose.position.y",
                "pose.position.y",
                "pose.pose.position.y",
            ),
            "y",
        )
        z_col = pick_mocap_column(
            reader.fieldnames,
            args.mocap_z_col,
            (
                "z",
                "field.pose.position.z",
                "field.pose.pose.position.z",
                "pose.position.z",
                "pose.pose.position.z",
            ),
            "z",
        )

        for row in reader:
            try:
                rows_t.append(float(row[time_col]))
                rows_x.append(float(row[x_col]))
                rows_y.append(float(row[y_col]))
                rows_z.append(float(row[z_col]))
            except (ValueError, TypeError):
                continue

    if not rows_t:
        raise EvalError("No valid mocap rows parsed from CSV")

    t = np.asarray(rows_t, dtype=np.float64)
    x = np.asarray(rows_x, dtype=np.float64)
    y = np.asarray(rows_y, dtype=np.float64)
    z = np.asarray(rows_z, dtype=np.float64)

    mask = finite_mask(t, x, y, z)
    t, x, y, z = t[mask], x[mask], y[mask], z[mask]
    if t.size < 10:
        raise EvalError("Too few valid mocap samples (<10)")

    order = np.argsort(t)
    t, x, y, z = t[order], x[order], y[order], z[order]

    t_sec = convert_mocap_time_unit(t, args.mocap_time_unit)
    t_sec = t_sec - t_sec[0]

    if args.mocap_frame == "enu":
        # ENU -> NED
        mocap_xyz = np.column_stack((y, x, -z))
    else:
        mocap_xyz = np.column_stack((x, y, z))

    return t_sec, mocap_xyz


def extract_estimated_xyz(ulog: ULog) -> Tuple[np.ndarray, np.ndarray, str, float]:
    # Prefer vehicle_local_position for optical-flow indoor evaluation.
    source_order = ["vehicle_local_position", "vehicle_odometry"]
    for topic in source_order:
        entry = get_first_topic(ulog, topic)
        if entry is None:
            continue

        data = entry.data
        ts = get_field(data, ["timestamp"], required=True)
        x = get_field(data, ["x"], required=True)
        y = get_field(data, ["y"], required=True)
        z = get_field(data, ["z"], required=True)

        mask = finite_mask(ts, x, y, z)
        ts = ts[mask]
        xyz = np.column_stack((x[mask], y[mask], z[mask]))

        if ts.size < 10:
            continue

        order = np.argsort(ts)
        ts = ts[order]
        xyz = xyz[order]

        origin_us = float(ts[0])
        t_sec = (ts - origin_us) * 1e-6
        return t_sec, xyz, topic, origin_us

    raise EvalError(
        "Unable to extract estimated position from ULog. "
        "Expected topic: vehicle_local_position or vehicle_odometry"
    )


def speed_norm(t_sec: np.ndarray, xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if t_sec.size < 3:
        return np.array([]), np.array([])
    dt = np.diff(t_sec)
    dxy = np.diff(xy, axis=0)
    valid = dt > 1e-6
    if not np.any(valid):
        return np.array([]), np.array([])

    t_mid = 0.5 * (t_sec[:-1] + t_sec[1:])
    v = np.full_like(t_mid, np.nan, dtype=np.float64)
    v[valid] = np.linalg.norm(dxy[valid], axis=1) / dt[valid]
    mask = np.isfinite(v)
    return t_mid[mask], v[mask]


def interpolate_columns(
    t_src: np.ndarray,
    values_src: np.ndarray,
    t_query: np.ndarray,
) -> np.ndarray:
    out = np.zeros((t_query.size, values_src.shape[1]), dtype=np.float64)
    for i in range(values_src.shape[1]):
        out[:, i] = np.interp(t_query, t_src, values_src[:, i])
    return out


def find_best_time_shift(
    est_t: np.ndarray,
    est_xy: np.ndarray,
    mocap_t: np.ndarray,
    mocap_xy: np.ndarray,
    shift_min: float,
    shift_max: float,
    shift_step: float,
) -> Tuple[float, float]:
    est_v_t, est_v = speed_norm(est_t, est_xy)
    moc_v_t, moc_v = speed_norm(mocap_t, mocap_xy)
    if est_v_t.size < 20 or moc_v_t.size < 20:
        return 0.0, float("inf")

    if shift_step <= 0.0:
        raise EvalError("time-shift-step-sec must be > 0")

    best_shift = 0.0
    best_rmse = float("inf")

    n_steps = int(math.floor((shift_max - shift_min) / shift_step)) + 1
    for i in range(max(n_steps, 1)):
        shift = shift_min + i * shift_step
        moc_shifted_t = moc_v_t + shift
        mask = (moc_shifted_t >= est_v_t[0]) & (moc_shifted_t <= est_v_t[-1])
        if np.count_nonzero(mask) < 50:
            continue
        est_interp = np.interp(moc_shifted_t[mask], est_v_t, est_v)
        diff = est_interp - moc_v[mask]
        rmse = float(np.sqrt(np.mean(diff * diff)))
        if rmse < best_rmse:
            best_rmse = rmse
            best_shift = shift

    return best_shift, best_rmse


def build_aligned_pairs(
    est_t: np.ndarray,
    est_xyz: np.ndarray,
    mocap_t: np.ndarray,
    mocap_xyz: np.ndarray,
    shift_sec: float,
    align_dt: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if align_dt <= 0.0:
        raise EvalError("align-dt-sec must be > 0")

    start_t = max(float(est_t[0]), float(mocap_t[0] + shift_sec))
    end_t = min(float(est_t[-1]), float(mocap_t[-1] + shift_sec))
    if end_t - start_t < align_dt * 5:
        raise EvalError("Not enough overlap between ULog and mocap after time alignment")

    t_grid = np.arange(start_t, end_t, align_dt, dtype=np.float64)
    if t_grid.size < 20:
        raise EvalError("Too few aligned samples (<20)")

    est_interp = interpolate_columns(est_t, est_xyz, t_grid)
    mocap_interp = interpolate_columns(mocap_t, mocap_xyz, t_grid - shift_sec)
    return t_grid, est_interp, mocap_interp


def fit_rigid_2d(src_xy: np.ndarray, dst_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # Solve src_xy @ R^T + t ~= dst_xy
    src_c = np.mean(src_xy, axis=0)
    dst_c = np.mean(dst_xy, axis=0)

    src0 = src_xy - src_c
    dst0 = dst_xy - dst_c

    h = src0.T @ dst0
    u, _s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T

    t = dst_c - src_c @ r.T
    return r, t


def percentile(values: np.ndarray, p: float) -> float:
    if values.size == 0:
        return float("nan")
    return float(np.percentile(values, p))


def safe_ratio(condition: np.ndarray) -> float:
    if condition.size == 0:
        return float("nan")
    return float(np.mean(condition.astype(np.float64)))


def topic_time_rel_seconds(
    topic_data: Dict[str, np.ndarray], origin_us: float
) -> Optional[np.ndarray]:
    ts = get_field(topic_data, ["timestamp"])
    if ts is None or ts.size == 0:
        return None
    return (ts - origin_us) * 1e-6


def window_mask(t_sec: np.ndarray, start_t: float, end_t: float) -> np.ndarray:
    return (t_sec >= start_t) & (t_sec <= end_t)


def extract_flow_quality(
    ulog: ULog, origin_us: float, start_t: float, end_t: float
) -> Optional[np.ndarray]:
    topic_candidates = ["sensor_optical_flow", "optical_flow", "vehicle_optical_flow"]
    for topic in topic_candidates:
        entry = get_first_topic(ulog, topic)
        if entry is None:
            continue
        data = entry.data
        quality = get_field(data, ["quality", "quality[0]"])
        t_sec = topic_time_rel_seconds(data, origin_us)
        if quality is None or t_sec is None:
            continue

        n = min(quality.size, t_sec.size)
        quality = quality[:n]
        t_sec = t_sec[:n]

        mask = np.isfinite(quality) & window_mask(t_sec, start_t, end_t)
        if np.count_nonzero(mask) < 5:
            continue
        return quality[mask]
    return None


def extract_flow_active_fault(
    ulog: ULog,
    origin_us: float,
    start_t: float,
    end_t: float,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
    # Preferred: estimator_status_flags (explicit booleans)
    entry_flags = get_first_topic(ulog, "estimator_status_flags")
    if entry_flags is not None:
        d = entry_flags.data
        t = topic_time_rel_seconds(d, origin_us)
        if t is not None:
            active = get_field(d, ["cs_opt_flow"])
            fs_x = get_field(d, ["fs_bad_optflow_x"])
            fs_y = get_field(d, ["fs_bad_optflow_y"])
            if active is not None:
                n = min(active.size, t.size)
                active_b = active[:n] > 0.5
                t_n = t[:n]
                mask = window_mask(t_n, start_t, end_t)
                active_b = active_b[mask]
            else:
                active_b = None

            fault_b = None
            if fs_x is not None and fs_y is not None:
                n = min(fs_x.size, fs_y.size, t.size)
                fs = (fs_x[:n] > 0.5) | (fs_y[:n] > 0.5)
                t_n = t[:n]
                mask = window_mask(t_n, start_t, end_t)
                fault_b = fs[mask]

            if active_b is not None or fault_b is not None:
                return active_b, fault_b, "estimator_status_flags"

    # Fallback: estimator_status bitmasks
    entry_status = get_first_topic(ulog, "estimator_status")
    if entry_status is not None:
        d = entry_status.data
        t = topic_time_rel_seconds(d, origin_us)
        if t is None:
            return None, None, "estimator_status"

        control = get_field(d, ["control_mode_flags"])
        faults = get_field(d, ["filter_fault_flags"])

        active_b = None
        if control is not None:
            n = min(control.size, t.size)
            control_u = control[:n].astype(np.uint64)
            active = ((control_u >> CS_OPT_FLOW_BIT) & 1) != 0
            mask = window_mask(t[:n], start_t, end_t)
            active_b = active[mask]

        fault_b = None
        if faults is not None:
            n = min(faults.size, t.size)
            faults_u = faults[:n].astype(np.uint64)
            bad_x = ((faults_u >> FS_BAD_OPTFLOW_X_BIT) & 1) != 0
            bad_y = ((faults_u >> FS_BAD_OPTFLOW_Y_BIT) & 1) != 0
            mask = window_mask(t[:n], start_t, end_t)
            fault_b = (bad_x | bad_y)[mask]

        return active_b, fault_b, "estimator_status"

    return None, None, "missing"


def extract_flow_test_ratio(
    ulog: ULog,
    origin_us: float,
    start_t: float,
    end_t: float,
) -> Optional[np.ndarray]:
    entry = get_first_topic(ulog, "estimator_innovation_test_ratios")
    if entry is None:
        return None

    d = entry.data
    t = topic_time_rel_seconds(d, origin_us)
    if t is None:
        return None

    flow_comps = get_array_components(d, "flow")
    if not flow_comps:
        flow_scalar = get_field(d, ["flow"])
        if flow_scalar is None:
            return None
        n = min(flow_scalar.size, t.size)
        vals = np.abs(flow_scalar[:n])
        mask = np.isfinite(vals) & window_mask(t[:n], start_t, end_t)
        if np.count_nonzero(mask) < 5:
            return None
        return vals[mask]

    n = min([comp.size for comp in flow_comps] + [t.size])
    arr = np.column_stack([comp[:n] for comp in flow_comps])
    vals = np.linalg.norm(arr, axis=1)
    mask = np.isfinite(vals) & window_mask(t[:n], start_t, end_t)
    if np.count_nonzero(mask) < 5:
        return None
    return vals[mask]


def extract_flow_innovation_normalized(
    ulog: ULog,
    origin_us: float,
    start_t: float,
    end_t: float,
) -> Optional[np.ndarray]:
    entry_innov = get_first_topic(ulog, "estimator_innovations")
    if entry_innov is None:
        return None
    entry_var = get_first_topic(ulog, "estimator_innovation_variances")
    if entry_var is None:
        return None

    di = entry_innov.data
    dv = entry_var.data
    ti = topic_time_rel_seconds(di, origin_us)
    tv = topic_time_rel_seconds(dv, origin_us)
    if ti is None or tv is None:
        return None

    innov = get_array_components(di, "flow")
    var = get_array_components(dv, "flow")
    if not innov or not var:
        return None

    n_i = min([c.size for c in innov] + [ti.size])
    n_v = min([c.size for c in var] + [tv.size])
    if n_i < 10 or n_v < 10:
        return None

    t_common_start = max(float(ti[0]), float(tv[0]), start_t)
    t_common_end = min(float(ti[n_i - 1]), float(tv[n_v - 1]), end_t)
    if t_common_end - t_common_start < 0.5:
        return None

    dt = 0.02
    t_grid = np.arange(t_common_start, t_common_end, dt)
    if t_grid.size < 10:
        return None

    innov_arr = interpolate_columns(ti[:n_i], np.column_stack([c[:n_i] for c in innov]), t_grid)
    var_arr = interpolate_columns(tv[:n_v], np.column_stack([c[:n_v] for c in var]), t_grid)

    sigma = np.sqrt(np.maximum(var_arr, 1e-12))
    nis = np.linalg.norm(innov_arr / sigma, axis=1)
    nis = nis[np.isfinite(nis)]
    if nis.size < 5:
        return None
    return nis


def extract_estimator_status_ratios(
    ulog: ULog,
    origin_us: float,
    start_t: float,
    end_t: float,
) -> Dict[str, np.ndarray]:
    entry = get_first_topic(ulog, "estimator_status")
    if entry is None:
        return {}

    data = entry.data
    t = topic_time_rel_seconds(data, origin_us)
    if t is None:
        return {}

    fields = {
        "vel_test_ratio": ["vel_test_ratio"],
        "pos_test_ratio": ["pos_test_ratio"],
        "hgt_test_ratio": ["hgt_test_ratio"],
        "hagl_test_ratio": ["hagl_test_ratio"],
    }

    out: Dict[str, np.ndarray] = {}
    for k, candidates in fields.items():
        arr = get_field(data, candidates)
        if arr is None:
            continue
        n = min(arr.size, t.size)
        vals = np.abs(arr[:n])
        mask = np.isfinite(vals) & window_mask(t[:n], start_t, end_t)
        if np.count_nonzero(mask) < 5:
            continue
        out[k] = vals[mask]
    return out


def make_check_max(
    name: str,
    value: Optional[float],
    threshold: float,
    note: str = "",
) -> CheckResult:
    if value is None or not math.isfinite(value):
        return CheckResult(name, "NA", value, threshold, "<=", note)
    status = "PASS" if value <= threshold else "FAIL"
    return CheckResult(name, status, float(value), threshold, "<=", note)


def make_check_min(
    name: str,
    value: Optional[float],
    threshold: float,
    note: str = "",
) -> CheckResult:
    if value is None or not math.isfinite(value):
        return CheckResult(name, "NA", value, threshold, ">=", note)
    status = "PASS" if value >= threshold else "FAIL"
    return CheckResult(name, status, float(value), threshold, ">=", note)


def check_window(
    t_rel: np.ndarray,
    start: float,
    end: Optional[float],
) -> np.ndarray:
    if end is None:
        end_val = float(t_rel[-1])
    else:
        end_val = end
    if end_val <= start:
        raise EvalError("hover-end-sec must be greater than hover-start-sec")
    return (t_rel >= start) & (t_rel <= end_val)


def summarize_checks(checks: Sequence[CheckResult]) -> str:
    has_fail = any(c.status == "FAIL" for c in checks)
    has_na = any(c.status == "NA" for c in checks)
    if has_fail:
        return "FAIL"
    if has_na:
        return "INCOMPLETE"
    return "PASS"


def to_builtin_number(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, (float, np.floating)):
        if not math.isfinite(float(value)):
            return None
        return float(value)
    return float(value)


def eval_once(args: argparse.Namespace) -> Dict[str, object]:
    require_pyulog()

    ulog_path = Path(args.ulog)
    if not ulog_path.exists():
        raise EvalError(f"ULog file not found: {ulog_path}")

    ulog = ULog(str(ulog_path))

    est_t, est_xyz, est_source_topic, origin_us = extract_estimated_xyz(ulog)
    mocap_t, mocap_xyz = load_mocap_csv(args)

    if args.manual_time_shift_sec is None:
        shift_sec, shift_cost = find_best_time_shift(
            est_t,
            est_xyz[:, :2],
            mocap_t,
            mocap_xyz[:, :2],
            args.time_shift_min_sec,
            args.time_shift_max_sec,
            args.time_shift_step_sec,
        )
    else:
        shift_sec = float(args.manual_time_shift_sec)
        shift_cost = float("nan")

    t_grid, est_interp, mocap_interp = build_aligned_pairs(
        est_t, est_xyz, mocap_t, mocap_xyz, shift_sec, args.align_dt_sec
    )

    overlap_sec = float(t_grid[-1] - t_grid[0])
    t_rel = t_grid - t_grid[0]

    # 2D rigid alignment (mocap -> estimate) to absorb world-frame yaw/origin mismatch.
    r2, t2 = fit_rigid_2d(mocap_interp[:, :2], est_interp[:, :2])
    mocap_xy_aligned = mocap_interp[:, :2] @ r2.T + t2

    # Z offset only (keep scale/sign from frame conversion choice).
    z_offset = float(np.median(est_interp[:, 2] - mocap_interp[:, 2]))
    mocap_z_aligned = mocap_interp[:, 2] + z_offset

    xy_err = est_interp[:, :2] - mocap_xy_aligned
    z_err = est_interp[:, 2] - mocap_z_aligned

    xy_err_norm = np.linalg.norm(xy_err, axis=1)
    xy_rmse = float(np.sqrt(np.mean(np.sum(xy_err * xy_err, axis=1))))
    xy_p95 = percentile(xy_err_norm, 95.0)
    z_rmse = float(np.sqrt(np.mean(z_err * z_err)))

    hover_mask = check_window(t_rel, args.hover_start_sec, args.hover_end_sec)
    if np.count_nonzero(hover_mask) < 10:
        raise EvalError("Hover window has too few samples (<10)")

    hover_t = t_rel[hover_mask]
    hover_duration = float(hover_t[-1] - hover_t[0])
    if hover_duration <= 0.0:
        raise EvalError("Invalid hover window duration")

    hover_xy_err = xy_err_norm[hover_mask]
    hover_xy_rmse = float(np.sqrt(np.mean(np.sum(xy_err[hover_mask] ** 2, axis=1))))
    hover_xy_p95 = percentile(hover_xy_err, 95.0)

    # Drift speed from mocap truth trajectory (aligned frame)
    hover_truth_xy = mocap_xy_aligned[hover_mask]
    hover_drift = float(
        np.linalg.norm(hover_truth_xy[-1] - hover_truth_xy[0]) / max(hover_duration, 1e-6)
    )

    overlap_start = float(t_grid[0])
    overlap_end = float(t_grid[-1])

    quality = extract_flow_quality(ulog, origin_us, overlap_start, overlap_end)
    flow_quality_ratio = (
        safe_ratio(quality >= args.flow_quality_min) if quality is not None else float("nan")
    )

    flow_active_b, flow_fault_b, flow_status_source = extract_flow_active_fault(
        ulog, origin_us, overlap_start, overlap_end
    )
    flow_active_ratio = safe_ratio(flow_active_b) if flow_active_b is not None else float("nan")
    flow_fault_ratio = safe_ratio(flow_fault_b) if flow_fault_b is not None else float("nan")

    flow_test_ratio = extract_flow_test_ratio(ulog, origin_us, overlap_start, overlap_end)
    flow_test_ratio_p95 = (
        percentile(flow_test_ratio, 95.0) if flow_test_ratio is not None else float("nan")
    )

    flow_nis = extract_flow_innovation_normalized(ulog, origin_us, overlap_start, overlap_end)
    flow_nis_p95 = percentile(flow_nis, 95.0) if flow_nis is not None else float("nan")

    est_ratios = extract_estimator_status_ratios(ulog, origin_us, overlap_start, overlap_end)
    vel_test_ratio_p95 = percentile(est_ratios.get("vel_test_ratio", np.array([])), 95.0)
    pos_test_ratio_p95 = percentile(est_ratios.get("pos_test_ratio", np.array([])), 95.0)
    hgt_test_ratio_p95 = percentile(est_ratios.get("hgt_test_ratio", np.array([])), 95.0)
    hagl_test_ratio_p95 = percentile(est_ratios.get("hagl_test_ratio", np.array([])), 95.0)

    checks: List[CheckResult] = [
        make_check_min("overlap_sec", overlap_sec, args.min_overlap_sec),
        make_check_min(
            "flow_quality_ratio",
            flow_quality_ratio,
            args.min_flow_quality_ratio,
            note=f"quality >= {args.flow_quality_min}",
        ),
        make_check_min("flow_active_ratio", flow_active_ratio, args.min_flow_active_ratio),
        make_check_max("flow_fault_ratio", flow_fault_ratio, args.max_flow_fault_ratio),
        make_check_max(
            "flow_test_ratio_p95", flow_test_ratio_p95, args.max_flow_test_ratio_p95
        ),
        make_check_max("vel_test_ratio_p95", vel_test_ratio_p95, args.max_vel_test_ratio_p95),
        make_check_max("pos_test_ratio_p95", pos_test_ratio_p95, args.max_pos_test_ratio_p95),
        make_check_max("hgt_test_ratio_p95", hgt_test_ratio_p95, args.max_hgt_test_ratio_p95),
        make_check_max(
            "hagl_test_ratio_p95", hagl_test_ratio_p95, args.max_hagl_test_ratio_p95
        ),
        make_check_max("hover_xy_rmse_m", hover_xy_rmse, args.max_xy_rmse),
        make_check_max("hover_xy_p95_m", hover_xy_p95, args.max_xy_p95),
        make_check_max("hover_drift_speed_mps", hover_drift, args.max_drift_speed),
    ]

    overall = summarize_checks(checks)

    report: Dict[str, object] = {
        "overall_status": overall,
        "inputs": {
            "ulog": str(ulog_path),
            "mocap_csv": str(Path(args.mocap_csv)),
            "mocap_frame": args.mocap_frame,
            "mocap_time_unit": args.mocap_time_unit,
        },
        "sync": {
            "time_shift_sec": to_builtin_number(shift_sec),
            "time_shift_search_cost": to_builtin_number(shift_cost),
            "aligned_dt_sec": to_builtin_number(args.align_dt_sec),
            "overlap_sec": to_builtin_number(overlap_sec),
        },
        "kpi": {
            "overall_xy_rmse_m": to_builtin_number(xy_rmse),
            "overall_xy_p95_m": to_builtin_number(xy_p95),
            "overall_z_rmse_m": to_builtin_number(z_rmse),
            "hover_xy_rmse_m": to_builtin_number(hover_xy_rmse),
            "hover_xy_p95_m": to_builtin_number(hover_xy_p95),
            "hover_drift_speed_mps": to_builtin_number(hover_drift),
            "flow_quality_ratio": to_builtin_number(flow_quality_ratio),
            "flow_active_ratio": to_builtin_number(flow_active_ratio),
            "flow_fault_ratio": to_builtin_number(flow_fault_ratio),
            "flow_test_ratio_p95": to_builtin_number(flow_test_ratio_p95),
            "flow_nis_p95": to_builtin_number(flow_nis_p95),
            "vel_test_ratio_p95": to_builtin_number(vel_test_ratio_p95),
            "pos_test_ratio_p95": to_builtin_number(pos_test_ratio_p95),
            "hgt_test_ratio_p95": to_builtin_number(hgt_test_ratio_p95),
            "hagl_test_ratio_p95": to_builtin_number(hagl_test_ratio_p95),
        },
        "meta": {
            "estimate_source_topic": est_source_topic,
            "flow_status_source": flow_status_source,
            "hover_window_sec": {
                "start": to_builtin_number(args.hover_start_sec),
                "end": (
                    to_builtin_number(args.hover_end_sec)
                    if args.hover_end_sec is not None
                    else to_builtin_number(float(t_rel[-1]))
                ),
            },
            "align_transform": {
                "rotation_2x2": [[float(r2[0, 0]), float(r2[0, 1])], [float(r2[1, 0]), float(r2[1, 1])]],
                "translation_xy": [float(t2[0]), float(t2[1])],
                "z_offset": to_builtin_number(z_offset),
            },
        },
        "checks": [
            {
                "name": c.name,
                "status": c.status,
                "value": to_builtin_number(c.value),
                "comparator": c.comparator,
                "threshold": to_builtin_number(c.threshold),
                "note": c.note,
            }
            for c in checks
        ],
    }

    return report


def print_console_report(report: Dict[str, object]) -> None:
    print("=" * 72)
    print("Optical Flow Indoor Evaluation (ULog + Mocap)")
    print("=" * 72)
    print(f"Overall: {report['overall_status']}")

    sync = report["sync"]
    print(
        "Sync: shift={:.3f}s overlap={:.2f}s dt={:.3f}s".format(
            float(sync.get("time_shift_sec") or 0.0),
            float(sync.get("overlap_sec") or 0.0),
            float(sync.get("aligned_dt_sec") or 0.0),
        )
    )

    kpi = report["kpi"]
    print("KPI:")
    print(
        "  hover_xy_rmse={:.3f} m, hover_xy_p95={:.3f} m, hover_drift_speed={:.3f} m/s".format(
            float(kpi.get("hover_xy_rmse_m") or float("nan")),
            float(kpi.get("hover_xy_p95_m") or float("nan")),
            float(kpi.get("hover_drift_speed_mps") or float("nan")),
        )
    )
    print(
        "  flow_active_ratio={:.3f}, flow_fault_ratio={:.3f}, flow_quality_ratio={:.3f}".format(
            float(kpi.get("flow_active_ratio") or float("nan")),
            float(kpi.get("flow_fault_ratio") or float("nan")),
            float(kpi.get("flow_quality_ratio") or float("nan")),
        )
    )
    print(
        "  flow_test_ratio_p95={:.3f}, vel/pos/hgt/hagl_p95={:.3f}/{:.3f}/{:.3f}/{:.3f}".format(
            float(kpi.get("flow_test_ratio_p95") or float("nan")),
            float(kpi.get("vel_test_ratio_p95") or float("nan")),
            float(kpi.get("pos_test_ratio_p95") or float("nan")),
            float(kpi.get("hgt_test_ratio_p95") or float("nan")),
            float(kpi.get("hagl_test_ratio_p95") or float("nan")),
        )
    )

    print("Checks:")
    for c in report["checks"]:
        value = c["value"]
        if value is None:
            value_str = "NA"
        else:
            value_str = f"{value:.4f}"
        print(
            f"  [{c['status']:<10}] {c['name']:<24} {value_str:<10} {c['comparator']} {c['threshold']}"
        )


def main() -> int:
    args = parse_args()
    try:
        report = eval_once(args)
    except EvalError as exc:
        print(f"[flow_eval] ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("[flow_eval] Interrupted", file=sys.stderr)
        return 130

    print_console_report(report)

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        print(f"Report saved: {output_path}")

    if report["overall_status"] == "FAIL":
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
