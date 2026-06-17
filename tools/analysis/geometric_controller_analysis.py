#!/usr/bin/env python3
"""Sunray Geometric Controller ULog analysis.

Loads sunray_geometric_controller_{param,input,debug,output,runtime} topics,
merges the control topics on timestamp, then produces PNG plots, a console summary
and a markdown diagnostic report covering position tracking, velocity
noise, vel-error preprocessing chain, a_fb decomposition, thrust + hover
estimator, attitude tracking, loop timing, and AccFF landing runtime state.
"""

from __future__ import annotations

import argparse
import io
import os
import sys
import warnings
from contextlib import redirect_stdout
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog
from scipy import signal


AXES = ("x", "y", "z")
ATT_AXES = ("roll", "pitch", "yaw")
TIMESTAMP_TOL_S = 1e-3


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
@dataclass
class LogData:
    path: Path
    df: pd.DataFrame
    params: dict
    nominal_dt: float
    runtime_df: Optional[pd.DataFrame] = None

    @property
    def has(self):
        return lambda col: col in self.df.columns


def _topic_to_df(ulog: ULog, name: str) -> Optional[pd.DataFrame]:
    for d in ulog.data_list:
        if d.name == name:
            return pd.DataFrame({k: np.asarray(v) for k, v in d.data.items()})
    return None


def _params_dict(df_param: Optional[pd.DataFrame]) -> dict:
    if df_param is None or df_param.empty:
        return {}
    row = df_param.iloc[0].to_dict()
    return row


def _resolve_ulg(path: Path) -> Path:
    if path.is_file():
        return path
    if not path.is_dir():
        raise FileNotFoundError(f"Not a file or directory: {path}")
    candidates = sorted(path.glob("*.ulg"))
    if not candidates:
        raise FileNotFoundError(f"No .ulg files in {path}")
    best, best_rows = None, -1
    for cand in candidates:
        try:
            u = ULog(str(cand))
        except Exception:
            continue
        rows = 0
        for d in u.data_list:
            if d.name.startswith("sunray_geometric_controller_"):
                rows = max(rows, len(d.data.get("timestamp", [])))
        if rows > best_rows:
            best, best_rows = cand, rows
    if best is None:
        raise FileNotFoundError(f"No usable controller logs in {path}")
    return best


def load_log(path: Path, dt_max: float) -> LogData:
    ulog = ULog(str(path))
    df_p = _topic_to_df(ulog, "sunray_geometric_controller_param")
    df_i = _topic_to_df(ulog, "sunray_geometric_controller_input")
    df_d = _topic_to_df(ulog, "sunray_geometric_controller_debug")
    df_o = _topic_to_df(ulog, "sunray_geometric_controller_output")
    df_r = _topic_to_df(ulog, "sunray_geometric_controller_runtime")

    params = _params_dict(df_p)

    frames = []
    for tag, df in (("input", df_i), ("debug", df_d), ("output", df_o)):
        if df is None:
            warnings.warn(f"missing topic: {tag}")
            continue
        df = df.copy()
        df["t"] = df["timestamp"].astype(np.int64) / 1e6
        rename = {c: f"{tag}.{c}" for c in df.columns if c not in ("timestamp", "t")}
        df = df.rename(columns=rename)
        frames.append(df)

    if not frames:
        raise RuntimeError("No data topics found in log")

    base = frames[0].sort_values("t").reset_index(drop=True)
    for extra in frames[1:]:
        extra = extra.sort_values("t").reset_index(drop=True)
        base = pd.merge_asof(
            base, extra, on="t", direction="nearest", tolerance=TIMESTAMP_TOL_S,
            suffixes=("", "_dup"),
        )
        base = base[[c for c in base.columns if not c.endswith("_dup")]]
    df = base.dropna(subset=[c for c in base.columns if c.startswith("debug.") or c.startswith("input.")]).reset_index(drop=True)
    if df.empty:
        raise RuntimeError("merge produced empty dataframe")

    t0_abs = float(df["t"].iloc[0])
    df["t"] = df["t"] - t0_abs

    if "debug.current_dt" in df.columns and dt_max > 0:
        mask = df["debug.current_dt"] <= dt_max
        df = df[mask].reset_index(drop=True)

    runtime_df = None
    if df_r is not None and not df_r.empty and "timestamp" in df_r.columns:
        runtime_df = df_r.copy()
        runtime_df["t"] = runtime_df["timestamp"].astype(np.int64) / 1e6 - t0_abs
        runtime_df = runtime_df.sort_values("t").reset_index(drop=True)

    controller_hz = float(params.get("controller_hz", 100.0)) or 100.0
    nominal_dt = 1.0 / controller_hz
    return LogData(path=path, df=df, params=params, nominal_dt=nominal_dt,
                   runtime_df=runtime_df)


def _vec3(params, key, default=0.0):
    return np.array([params.get(f"{key}[{i}]", default) for i in range(3)], dtype=float)


def print_load_summary(log: LogData):
    df = log.df
    p = log.params
    duration = df["t"].iloc[-1] - df["t"].iloc[0] if len(df) > 1 else 0.0
    pos_kp = _vec3(p, "pos_kp")
    pos_ki = _vec3(p, "pos_ki")
    pos_kd = _vec3(p, "pos_kd")
    vel_kp = _vec3(p, "vel_kp")
    vel_ki = _vec3(p, "vel_ki")
    vel_kd = _vec3(p, "vel_kd")
    print("=" * 64)
    print(f"Log:       {log.path}")
    print(f"Duration:  {duration:.2f} s   Samples: {len(df)}   Rate: ~{(len(df)/max(duration,1e-6)):.1f} Hz")
    print(f"Nominal dt: {log.nominal_dt*1000:.2f} ms (controller_hz={p.get('controller_hz', 'NA')})")
    print(f"hover_thrust_init: {p.get('hover_thrust_init', 'NA')}")
    print(f"pos_kp = {pos_kp}    pos_ki = {pos_ki}    pos_kd = {pos_kd}")
    print(f"vel_kp = {vel_kp}    vel_ki = {vel_ki}    vel_kd = {vel_kd}")
    if "vel_error_filter_tau[0]" in p:
        print(f"vel_error_filter_tau = {_vec3(p,'vel_error_filter_tau')}")
        print(f"vel_error_gate_threshold = {_vec3(p,'vel_error_gate_threshold')}")
    print("=" * 64)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _save(fig, out_dir: Path, stem: str, suffix: str):
    p = out_dir / f"{stem}_{suffix}.png"
    fig.tight_layout()
    fig.savefig(p, dpi=110)
    plt.close(fig)
    print(f"  saved: {p}")
    return p


def _has_cols(df: pd.DataFrame, cols) -> bool:
    return all(c in df.columns for c in cols)


def _cols3(prefix: str):
    return [f"{prefix}[{i}]" for i in range(3)]


# ---------------------------------------------------------------------------
# Section 2: Position tracking
# ---------------------------------------------------------------------------
def analyze_position(log: LogData, out_dir: Path, stem: str):
    print("\n[2] Position tracking")
    df = log.df
    ref_cols = [f"input.reference_position[{i}]" for i in range(3)]
    odom_cols = [f"input.odom_position[{i}]" for i in range(3)]
    err_cols = [f"debug.position_error[{i}]" for i in range(3)]
    if not _has_cols(df, ref_cols + odom_cols + err_cols):
        print("  skip: missing position columns")
        return None

    fig, axs = plt.subplots(6, 1, figsize=(11, 13), sharex=True)
    rms = np.zeros(3)
    mx = np.zeros(3)
    for i, ax in enumerate(AXES):
        axs[i].plot(df["t"], df[ref_cols[i]], label=f"ref {ax}", color="tab:blue", lw=1.0)
        axs[i].plot(df["t"], df[odom_cols[i]], label=f"odom {ax}", color="tab:orange", lw=1.0)
        axs[i].set_ylabel(f"pos {ax} [m]")
        axs[i].grid(True)
        axs[i].legend(loc="upper right", fontsize=8)
        e = df[err_cols[i]].to_numpy()
        axs[3 + i].plot(df["t"], e, color="tab:red", lw=1.0)
        axs[3 + i].axhline(0, color="k", lw=0.5)
        axs[3 + i].set_ylabel(f"err {ax} [m]")
        axs[3 + i].grid(True)
        rms[i] = float(np.sqrt(np.mean(e * e)))
        mx[i] = float(np.max(np.abs(e)))
    axs[-1].set_xlabel("t [s]")
    fig.suptitle("Position tracking & error")
    _save(fig, out_dir, stem, "01_position")
    print(f"  RMS  err: x={rms[0]:.3f}  y={rms[1]:.3f}  z={rms[2]:.3f}  (m)")
    print(f"  Max |err|: x={mx[0]:.3f}  y={mx[1]:.3f}  z={mx[2]:.3f}  (m)")
    return {"rms": rms, "max": mx}


# ---------------------------------------------------------------------------
# Section 3: Velocity noise
# ---------------------------------------------------------------------------
def analyze_velocity_noise(log: LogData, out_dir: Path, stem: str):
    print("\n[3] Velocity noise")
    df = log.df
    odom_cols = _cols3("input.odom_velocity")
    ref_cols = _cols3("input.reference_velocity")
    err_cols = _cols3("debug.velocity_error")
    if not _has_cols(df, odom_cols):
        print("  skip: missing odom_velocity")
        return None

    t = df["t"].to_numpy()
    fs = 1.0 / max(np.median(np.diff(t)), 1e-3) if len(t) > 1 else 1.0 / log.nominal_dt
    fig, axs = plt.subplots(2, 3, figsize=(15, 7))

    std = np.zeros(3); rms_err = np.zeros(3); peak_freq = np.full(3, np.nan)
    for i, ax in enumerate(AXES):
        v = df[odom_cols[i]].to_numpy()
        axs[0, i].plot(t, df[ref_cols[i]] if ref_cols[i] in df.columns else np.zeros_like(v),
                       label="ref", color="tab:blue", lw=1.0)
        axs[0, i].plot(t, v, label="odom", color="tab:orange", lw=1.0)
        axs[0, i].set_title(f"velocity {ax}")
        axs[0, i].set_xlabel("t [s]"); axs[0, i].set_ylabel("m/s")
        axs[0, i].grid(True); axs[0, i].legend(fontsize=8)

        nperseg = min(256, max(32, len(v) // 4))
        f, pxx = signal.welch(v - np.mean(v), fs=fs, nperseg=nperseg)
        axs[1, i].semilogy(f, pxx)
        axs[1, i].set_title(f"PSD odom_v {ax}")
        axs[1, i].set_xlabel("Hz"); axs[1, i].set_ylabel("(m/s)^2/Hz")
        axs[1, i].grid(True, which="both")

        std[i] = float(np.std(v))
        if err_cols[i] in df.columns:
            e = df[err_cols[i]].to_numpy()
            rms_err[i] = float(np.sqrt(np.mean(e * e)))
        mask = f >= 2.0
        if mask.any():
            idx = np.argmax(pxx[mask])
            peak_freq[i] = float(f[mask][idx])
    fig.suptitle("Velocity vs reference + PSD")
    _save(fig, out_dir, stem, "02_velocity_noise")
    print(f"  std(odom_v)  x={std[0]:.3f}  y={std[1]:.3f}  z={std[2]:.3f}  (m/s)")
    print(f"  RMS vel_err  x={rms_err[0]:.3f}  y={rms_err[1]:.3f}  z={rms_err[2]:.3f}  (m/s)")
    print(f"  dominant noise freq (>2Hz)  x={peak_freq[0]:.2f}  y={peak_freq[1]:.2f}  z={peak_freq[2]:.2f}  (Hz)")
    return {"std": std, "rms_err": rms_err, "peak_freq": peak_freq}


# ---------------------------------------------------------------------------
# Section 4: Velocity error preprocessing chain
# ---------------------------------------------------------------------------
def analyze_vel_preproc(log: LogData, out_dir: Path, stem: str):
    print("\n[4] Vel error preprocessing chain")
    df = log.df
    raw_cols = _cols3("debug.velocity_error")
    lim_cols = _cols3("debug.velocity_error_limited")
    flt_cols = _cols3("debug.velocity_error_filtered")
    prc_cols = _cols3("debug.velocity_error_processed")
    gate_cols = _cols3("debug.velocity_error_gate_active")

    if not _has_cols(df, raw_cols):
        print("  skip: missing velocity_error")
        return None
    has_chain = all(_has_cols(df, c) for c in [lim_cols, flt_cols, prc_cols])
    if not has_chain:
        print("  skip: log lacks limited/filtered/processed fields (older log version)")
        return None

    t = df["t"].to_numpy()
    fig, axs = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    labels = ("raw", "limited", "filtered", "processed")
    series = (raw_cols[0], lim_cols[0], flt_cols[0], prc_cols[0])
    for ax, col, lbl in zip(axs, series, labels):
        ax.plot(t, df[col], lw=1.0)
        ax.set_title(f"velocity_error_{lbl} (x)")
        ax.set_ylabel("m/s"); ax.grid(True)
    axs[-1].set_xlabel("t [s]")
    fig.suptitle("X-axis velocity error preprocessing pipeline")
    _save(fig, out_dir, stem, "03_vel_preproc")

    raw = df[raw_cols].to_numpy(); flt = df[flt_cols].to_numpy()
    raw_std = raw.std(axis=0); flt_std = flt.std(axis=0)
    reduction = np.where(raw_std > 1e-9, flt_std / raw_std, np.nan)
    print(f"  std raw     : {raw_std}")
    print(f"  std filtered: {flt_std}")
    print(f"  ratio filt/raw: {reduction}")

    if _has_cols(df, gate_cols):
        gate = df[gate_cols].to_numpy()
        rate = gate.mean(axis=0)
        print(f"  gate active rate (frac): x={rate[0]:.3f}  y={rate[1]:.3f}  z={rate[2]:.3f}")
    return {"std_raw": raw_std, "std_filt": flt_std, "ratio": reduction}


# ---------------------------------------------------------------------------
# Section 5: a_fb decomposition
# ---------------------------------------------------------------------------
def analyze_acc_decomp(log: LogData, out_dir: Path, stem: str):
    print("\n[5] a_fb decomposition")
    df = log.df
    pe_cols = _cols3("debug.position_error")
    afb_cols = _cols3("debug.pid_feedback_acceleration")
    if not _has_cols(df, pe_cols + afb_cols):
        print("  skip: missing pos error or a_fb")
        return None

    pos_kp = _vec3(log.params, "pos_kp")
    posP = df[pe_cols].to_numpy() * pos_kp[None, :]
    afb = df[afb_cols].to_numpy()

    velP = (df[_cols3("debug.velocity_feedback_term")].to_numpy()
            if _has_cols(df, _cols3("debug.velocity_feedback_term"))
            else None)
    dterm = (df[_cols3("debug.derivative_term")].to_numpy()
             if _has_cols(df, _cols3("debug.derivative_term"))
             else None)
    iterm = (df[_cols3("debug.integral_term")].to_numpy()
             if _has_cols(df, _cols3("debug.integral_term"))
             else None)

    if velP is None:
        # older log: synthesize velocity P feedback from logged vel error and gain
        ve_cols = _cols3("debug.velocity_error")
        if _has_cols(df, ve_cols):
            vel_kp = _vec3(log.params, "vel_kp")
            velP = df[ve_cols].to_numpy() * vel_kp[None, :]
            print("  note: velocity_feedback_term absent; synthesized as Kp_vel * velocity_error")
        else:
            velP = np.zeros_like(afb)

    if dterm is None:
        dterm = np.zeros_like(afb)
    if iterm is None:
        iterm = np.zeros_like(afb)

    t = df["t"].to_numpy()
    fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i, ax in enumerate(AXES):
        axs[i].plot(t, posP[:, i], label="pos_P (Kp*e_p)", lw=1.0)
        axs[i].plot(t, velP[:, i], label="vel_P term", lw=1.0)
        axs[i].plot(t, dterm[:, i], label="D term", lw=1.0)
        axs[i].plot(t, iterm[:, i], label="I term", lw=1.0)
        axs[i].plot(t, afb[:, i], label="a_fb total", color="k", lw=1.2, ls="--")
        axs[i].set_ylabel(f"{ax} [m/s^2]")
        axs[i].grid(True); axs[i].legend(fontsize=8, loc="upper right")
    axs[-1].set_xlabel("t [s]")
    fig.suptitle("a_fb term decomposition")
    _save(fig, out_dir, stem, "04_acc_decomp")

    # Time-avg magnitude fraction (NaN-safe)
    mags = {
        "pos_P": np.linalg.norm(np.nan_to_num(posP), axis=1),
        "vel_P": np.linalg.norm(np.nan_to_num(velP), axis=1),
        "D":     np.linalg.norm(np.nan_to_num(dterm), axis=1),
        "I":     np.linalg.norm(np.nan_to_num(iterm), axis=1),
    }
    sums = sum(np.nanmean(m) for m in mags.values())
    fracs = {k: (float(np.nanmean(v)) / sums if sums > 1e-9 else 0.0)
             for k, v in mags.items()}
    print(f"  Time-avg magnitude fractions:")
    for k, v in fracs.items():
        print(f"    {k:6s}: {v*100:5.1f} %   (mean |term| = {float(np.nanmean(mags[k])):.3f} m/s^2)")
    return fracs


# ---------------------------------------------------------------------------
# Section 6: Thrust + hover
# ---------------------------------------------------------------------------
def analyze_thrust(log: LogData, out_dir: Path, stem: str):
    print("\n[6] Thrust & hover estimator")
    df = log.df
    have_mt = "output.mavros_thrust" in df.columns
    have_h = "debug.hover_thrust_estimate" in df.columns
    have_acc = "debug.accepted_hover_thrust" in df.columns
    have_sat = "debug.pid_accel_saturated" in df.columns

    if not have_mt:
        print("  skip: missing mavros_thrust")
        return None

    t = df["t"].to_numpy()
    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    thrust_raw = df["output.mavros_thrust"].to_numpy(dtype=float)
    thrust = thrust_raw[np.isfinite(thrust_raw)]
    axs[0].plot(t, thrust_raw, label="mavros_thrust", color="tab:blue", lw=1.0)
    axs[0].set_ylabel("thrust [norm]")
    axs[0].grid(True); axs[0].legend(fontsize=8)

    if have_h:
        axs[1].plot(t, df["debug.hover_thrust_estimate"], label="hover_thrust_estimate",
                    color="tab:green", lw=1.0)
    if have_acc:
        axs[1].plot(t, df["debug.accepted_hover_thrust"], label="accepted_hover_thrust",
                    color="tab:purple", lw=1.0)
    axs[1].set_ylabel("hover thrust"); axs[1].set_xlabel("t [s]")
    axs[1].grid(True); axs[1].legend(fontsize=8)

    sat_rate = 0.0
    if have_sat:
        sat = df["debug.pid_accel_saturated"].to_numpy().astype(bool)
        sat_rate = sat.mean() * 100
        # mark saturation events
        sat_t = t[sat]
        if len(sat_t) > 0 and len(sat_t) < 500:
            for st in sat_t:
                axs[0].axvline(st, color="r", lw=0.5, alpha=0.4)
        elif len(sat_t) >= 500:
            axs[0].axvspan(sat_t[0], sat_t[-1], color="r", alpha=0.05,
                           label=f"sat ({sat.sum()} frames)")

    fig.suptitle("Thrust & hover thrust estimator")
    _save(fig, out_dir, stem, "05_thrust")

    print(f"  thrust mean={np.nanmean(thrust_raw):.3f}  std={np.nanstd(thrust_raw):.3f}")
    print(f"  saturation rate: {sat_rate:.2f} %")
    if have_acc:
        a = df["debug.accepted_hover_thrust"].to_numpy(dtype=float)
        a = a[np.isfinite(a)]
        if len(a):
            tail = a[int(0.8 * len(a)):]
            print(f"  hover estimate std (last 20%): {tail.std():.4f}")
    return {"mean": float(np.nanmean(thrust_raw)),
            "std": float(np.nanstd(thrust_raw)),
            "sat_rate": sat_rate}


# ---------------------------------------------------------------------------
# Section 7: Attitude
# ---------------------------------------------------------------------------
def analyze_attitude(log: LogData, out_dir: Path, stem: str):
    print("\n[7] Attitude tracking")
    df = log.df
    err_cols = _cols3("debug.attitude_error")
    rate_cols = _cols3("output.desired_bodyrate")
    if not _has_cols(df, err_cols):
        print("  skip: missing attitude_error")
        return None

    t = df["t"].to_numpy()
    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    rms_err = np.zeros(3)
    for i, ax in enumerate(ATT_AXES):
        e = df[err_cols[i]].to_numpy()
        axs[0].plot(t, e, label=f"err {ax}", lw=1.0)
        rms_err[i] = float(np.sqrt(np.mean(e * e)))
    axs[0].set_ylabel("attitude err [rad]"); axs[0].grid(True); axs[0].legend(fontsize=8)

    rms_cmd = np.zeros(3); max_cmd = np.zeros(3)
    if _has_cols(df, rate_cols):
        for i, ax in enumerate(AXES):
            r = df[rate_cols[i]].to_numpy(dtype=float)
            axs[1].plot(t, r, label=f"omega {ax}", lw=1.0)
            r_finite = r[np.isfinite(r)]
            if len(r_finite):
                rms_cmd[i] = float(np.sqrt(np.mean(r_finite * r_finite)))
                max_cmd[i] = float(np.max(np.abs(r_finite)))
    axs[1].set_ylabel("desired bodyrate [rad/s]"); axs[1].set_xlabel("t [s]")
    axs[1].grid(True); axs[1].legend(fontsize=8)
    fig.suptitle("Attitude error & bodyrate command")
    _save(fig, out_dir, stem, "06_attitude")

    print(f"  RMS att err: roll={rms_err[0]:.4f}  pitch={rms_err[1]:.4f}  yaw={rms_err[2]:.4f}  (rad)")
    print(f"  RMS bodyrate cmd: x={rms_cmd[0]:.3f}  y={rms_cmd[1]:.3f}  z={rms_cmd[2]:.3f}  (rad/s)")
    print(f"  max |bodyrate|  : x={max_cmd[0]:.3f}  y={max_cmd[1]:.3f}  z={max_cmd[2]:.3f}  (rad/s)")
    return {"rms_err": rms_err, "rms_cmd": rms_cmd, "max_cmd": max_cmd}


# ---------------------------------------------------------------------------
# Section 8: Loop timing
# ---------------------------------------------------------------------------
def analyze_timing(log: LogData, out_dir: Path, stem: str):
    print("\n[8] Loop timing")
    df = log.df
    if "debug.current_dt" not in df.columns:
        print("  skip: missing current_dt")
        return None
    dt = df["debug.current_dt"].to_numpy()
    t = df["t"].to_numpy()

    fig, axs = plt.subplots(1, 2, figsize=(13, 5))
    axs[0].plot(t, dt * 1000.0, lw=0.8)
    axs[0].axhline(log.nominal_dt * 1000.0, color="r", ls="--", lw=0.8, label="nominal")
    axs[0].set_xlabel("t [s]"); axs[0].set_ylabel("dt [ms]"); axs[0].grid(True)
    axs[0].legend(fontsize=8); axs[0].set_title("current_dt vs time")

    axs[1].hist(dt * 1000.0, bins=60, color="tab:gray")
    axs[1].axvline(log.nominal_dt * 1000.0, color="r", ls="--", lw=0.8)
    axs[1].set_xlabel("dt [ms]"); axs[1].set_ylabel("count")
    axs[1].set_title("current_dt histogram")
    fig.suptitle("Control loop timing")
    _save(fig, out_dir, stem, "07_timing")

    overrun = (dt > 1.5 * log.nominal_dt).mean() * 100
    print(f"  mean dt = {dt.mean()*1000:.3f} ms   std = {dt.std()*1000:.3f} ms   overrun (>1.5*nominal) = {overrun:.2f} %")
    return {"mean": dt.mean(), "std": dt.std(), "overrun": overrun}


# ---------------------------------------------------------------------------
# Section 9: AccFF landing runtime
# ---------------------------------------------------------------------------
def analyze_landing_runtime(log: LogData, out_dir: Path, stem: str):
    print("\n[9] AccFF landing runtime")
    df = log.runtime_df
    if df is None or df.empty:
        print("  skip: missing runtime topic")
        return None

    required = [
        "landing_phase",
        "landing_height_above_ground",
        "landing_release_progress",
        "landing_setpoint_thrust",
        "landing_thrust_release_limit",
        "landing_xy_error",
        "landing_ref_vz",
        "landing_odom_vz",
    ]
    if not _has_cols(df, required):
        print("  skip: runtime topic lacks landing fields (older log version)")
        return None

    if "early_return_reason" in df.columns:
        landing_mask = (
            (df["landing_phase"].astype(float) > 0.0) |
            (df["early_return_reason"].astype(float).isin([13.0, 14.0, 15.0]))
        )
    else:
        landing_mask = df["landing_phase"].astype(float) > 0.0
    landing = df[landing_mask].copy()
    if landing.empty:
        print("  skip: no landing samples in runtime topic")
        return None

    t = landing["t"].to_numpy(dtype=float)
    fig, axs = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    axs[0].step(t, landing["landing_phase"], where="post", label="phase", lw=1.0)
    if "landing_near_ground_trigger" in landing.columns:
        axs[0].step(t, landing["landing_near_ground_trigger"], where="post",
                    label="trigger", lw=1.0)
    axs[0].set_ylabel("phase / trigger")
    axs[0].grid(True); axs[0].legend(fontsize=8)

    axs[1].plot(t, landing["landing_height_above_ground"], label="height", lw=1.0)
    if "landing_near_ground_h" in landing.columns:
        axs[1].plot(t, landing["landing_near_ground_h"], label="near_h", lw=0.9, ls="--")
    if "landing_ground_effect_release_h" in landing.columns:
        axs[1].plot(t, landing["landing_ground_effect_release_h"],
                    label="ground_effect_h", lw=0.9, ls=":")
    axs[1].set_ylabel("height [m]")
    axs[1].grid(True); axs[1].legend(fontsize=8)

    axs[2].plot(t, landing["landing_release_progress"], label="release_u", lw=1.0)
    if "landing_a_ff" in landing.columns:
        axs[2].plot(t, landing["landing_a_ff"], label="a_ff", lw=1.0)
    axs[2].set_ylabel("release / a_ff")
    axs[2].grid(True); axs[2].legend(fontsize=8)

    axs[3].plot(t, landing["landing_setpoint_thrust"], label="setpoint", lw=1.0)
    axs[3].plot(t, landing["landing_thrust_release_limit"], label="limit", lw=1.0)
    if "landing_anchor_thrust" in landing.columns:
        axs[3].plot(t, landing["landing_anchor_thrust"], label="anchor", lw=0.9, ls="--")
    axs[3].set_ylabel("thrust")
    axs[3].grid(True); axs[3].legend(fontsize=8)

    axs[4].plot(t, landing["landing_xy_error"], label="xy_error", lw=1.0)
    axs[4].plot(t, landing["landing_ref_vz"], label="ref_vz", lw=1.0)
    axs[4].plot(t, landing["landing_odom_vz"], label="odom_vz", lw=1.0)
    axs[4].set_ylabel("xy [m] / vz [m/s]")
    axs[4].set_xlabel("t [s]")
    axs[4].grid(True); axs[4].legend(fontsize=8)

    fig.suptitle("AccFF landing runtime")
    _save(fig, out_dir, stem, "08_landing_runtime")

    phases = sorted(int(v) for v in landing["landing_phase"].dropna().unique())
    triggers = []
    if "landing_near_ground_trigger" in landing.columns:
        triggers = sorted(int(v) for v in landing["landing_near_ground_trigger"].dropna().unique())
    min_release = float(np.nanmin(landing["landing_release_progress"]))
    min_height = float(np.nanmin(landing["landing_height_above_ground"]))
    max_xy = float(np.nanmax(landing["landing_xy_error"]))
    min_thrust_margin = float(np.nanmin(
        landing["landing_thrust_release_limit"] - landing["landing_setpoint_thrust"]
    ))
    print(f"  phases: {phases}  triggers: {triggers}")
    print(f"  min height={min_height:.3f}m  min release_u={min_release:.3f}  max xy_error={max_xy:.3f}m")
    print(f"  min thrust_limit-setpoint={min_thrust_margin:.3f}")
    return {
        "phases": phases,
        "triggers": triggers,
        "min_release": min_release,
        "min_height": min_height,
        "max_xy": max_xy,
        "min_thrust_margin": min_thrust_margin,
        "samples": int(len(landing)),
    }


# ---------------------------------------------------------------------------
# Diagnosis & Markdown report
# ---------------------------------------------------------------------------
LEVEL_BADGE = {"OK": "🟢 OK", "WARN": "🟡 WARN", "ERROR": "🔴 ERROR", "INFO": "ℹ️ INFO"}


def _level_max(a: str, b: str) -> str:
    order = {"OK": 0, "INFO": 0, "WARN": 1, "ERROR": 2}
    return a if order[a] >= order[b] else b


def diagnose_position(res):
    if res is None:
        return {"level": "INFO", "lines": ["位置数据缺失，跳过。"], "suggestions": []}
    rms, mx = res["rms"], res["max"]
    level = "OK"
    obs, sug = [], []
    if max(rms[0], rms[1]) > 0.30:
        level = "ERROR"
        obs.append(f"xy 轴位置 RMS 误差 {max(rms[0],rms[1]):.2f} m，跟踪偏差严重。")
    elif max(rms[0], rms[1]) > 0.15:
        level = _level_max(level, "WARN")
        obs.append(f"xy 轴位置 RMS 误差 {max(rms[0],rms[1]):.2f} m，跟踪精度偏弱。")
    if rms[2] > 0.10:
        level = _level_max(level, "WARN")
        obs.append(f"z 轴位置 RMS 误差 {rms[2]:.2f} m，高度环跟踪偏弱。")
    if max(mx) > 1.0:
        level = _level_max(level, "WARN")
        obs.append(f"位置误差最大值 {max(mx):.2f} m，存在显著瞬态偏差。")
    if level == "OK":
        obs.append(f"位置跟踪正常：RMS xy={rms[0]:.2f}/{rms[1]:.2f} m, z={rms[2]:.2f} m。")
    else:
        sug.append("若伴随姿态误差正常 → 位置环增益不足，可适度提升 `pos_kp`。")
        sug.append("若伴随姿态误差也大 → 应优先排查内环（attitude_tau / 推力归一化）。")
    return {"level": level, "lines": obs, "suggestions": sug,
            "metrics": {"rms_xy_m": float(max(rms[0], rms[1])), "rms_z_m": float(rms[2]),
                        "max_xy_m": float(max(mx[0], mx[1]))}}


def diagnose_velocity(res, acc_res):
    if res is None:
        return {"level": "INFO", "lines": ["速度数据缺失，跳过。"], "suggestions": []}
    s, e, f = res["std"], res["rms_err"], res["peak_freq"]
    vel_P_frac = (acc_res or {}).get("vel_P", 0.0)
    level = "OK"
    obs, sug = [], []
    if max(s[0], s[1]) > 0.20:
        level = "ERROR"
        obs.append(f"xy 速度估计 std {max(s[0],s[1]):.3f} m/s，噪声严重。")
    elif max(s[0], s[1]) > 0.10:
        level = _level_max(level, "WARN")
        obs.append(f"xy 速度估计 std {max(s[0],s[1]):.3f} m/s，噪声偏大。")
    finite_f = [v for v in f if np.isfinite(v)]
    if finite_f:
        max_f = float(max(finite_f))
        obs.append(f"PSD 主峰频率（>2 Hz）最高 {max_f:.2f} Hz；该带宽进入控制律的速度反馈项。")
    if vel_P_frac > 0.30:
        level = _level_max(level, "WARN")
        obs.append(f"vel_P 项占 a_fb 总幅值 {vel_P_frac*100:.1f}%（速度通道明显主导），噪声会按此比例放大进入加速度指令。")
        sug.append(f"开启速度误差低通：`vel_error_filter_tau ≈ 0.05~0.08 s`，可衰减 >3 Hz 的噪声。")
        sug.append("若仍振荡 → 适当降低 `vel_kp`（例如 2.0 → 1.5），同时保持 `pos_kp` 不变以维持响应速度，但 `ζ = vel_kp/(2√pos_kp)` 会下降。")
    if max(s[0], s[1]) > 0.15 and finite_f and max(finite_f) < 5.0:
        sug.append("速度噪声主频较低（<5 Hz），可考虑在里程计源头加滤波，而非仅依赖控制器侧低通。")
    if level == "OK":
        obs.append("速度估计噪声水平正常。")
    return {"level": level, "lines": obs, "suggestions": sug,
            "metrics": {"std_xy_mps": float(max(s[0], s[1])), "rms_err_xy_mps": float(max(e[0], e[1])),
                        "peak_freq_xy_hz": float(max(finite_f)) if finite_f else float("nan")}}


def diagnose_acc_decomp(res):
    if res is None:
        return {"level": "INFO", "lines": ["a_fb 分解缺失，跳过。"], "suggestions": []}
    obs, sug = [], []
    level = "OK"
    pos_P, vel_P, D, I = res["pos_P"], res["vel_P"], res["D"], res["I"]
    obs.append(f"a_fb 占比：pos_P={pos_P*100:.1f}%  vel_P={vel_P*100:.1f}%  D={D*100:.1f}%  I={I*100:.1f}%。")
    if vel_P > 0.50:
        level = _level_max(level, "WARN")
        obs.append("速度反馈占比超过一半，控制律对速度噪声非常敏感。")
    if D > 0.20:
        level = _level_max(level, "WARN")
        obs.append("D 项贡献偏高，常因速度信号差分放大噪声而出现，建议核查 `vel_kd` 或速度采样质量。")
    if I > 0.50:
        level = _level_max(level, "WARN")
        obs.append("I 项主导，可能存在长期偏置（载荷/电池/校准）或饱和后 anti-windup 触发，注意是否伴随推力饱和。")
    if pos_P + vel_P + D + I < 0.05:
        level = _level_max(level, "INFO")
        obs.append("各项幅值都很小，控制器处于低强度跟踪状态（悬停或慢速段）。")
    return {"level": level, "lines": obs, "suggestions": sug,
            "metrics": {"pos_P_frac": float(pos_P), "vel_P_frac": float(vel_P),
                        "D_frac": float(D), "I_frac": float(I)}}


def diagnose_thrust(res, df):
    if res is None:
        return {"level": "INFO", "lines": ["推力数据缺失，跳过。"], "suggestions": []}
    level = "OK"
    obs, sug = [], []
    mean, std, sat = res["mean"], res["std"], res["sat_rate"]
    obs.append(f"推力均值 {mean:.3f}，std {std:.3f}，加速度饱和率 {sat:.2f}%。")
    if sat > 5.0:
        level = "ERROR"
        sug.append("加速度长期饱和 → 提升 `max_acc` 或检查动力余量。")
    elif sat > 1.0:
        level = _level_max(level, "WARN")
        sug.append("偶发加速度饱和 → 关注大机动段，必要时收紧轨迹生成的速度/加速度上限。")
    if mean > 0.70:
        level = _level_max(level, "WARN")
        obs.append("推力均值贴近上限，动力余量不足。")
        sug.append("减载、提升电池电压或检查 `hover_thrust_init` 是否设小（导致 vel_P/pos_P 推力放大不够）。")
    elif mean < 0.20:
        level = _level_max(level, "WARN")
        obs.append("推力均值偏低，`hover_thrust_init` 可能偏大，估计器未在线收敛时控制律会偏弱。")

    a = df.get("debug.accepted_hover_thrust")
    if a is not None:
        arr = a.to_numpy(dtype=float)
        arr = arr[np.isfinite(arr)]
        if len(arr) > 0:
            tail = arr[int(0.8 * len(arr)):]
            tail_std = float(tail.std())
            obs.append(f"accepted_hover_thrust 末段 std={tail_std:.4f}，均值={float(tail.mean()):.3f}。")
            if tail_std < 1e-6:
                obs.append("估计器无更新（可能未飞起或始终用 fixed anchor）。")
            elif tail_std > 0.02:
                level = _level_max(level, "WARN")
                sug.append("悬停推力估计未收敛，建议核查推力估计器开关、IMU 数据有效性，必要时换 EKF 估计器。")
    if level == "OK" and not sug:
        obs.append("推力执行正常。")
    return {"level": level, "lines": obs, "suggestions": sug,
            "metrics": {"mean": float(mean), "std": float(std), "sat_rate_pct": float(sat)}}


def diagnose_attitude(res):
    if res is None:
        return {"level": "INFO", "lines": ["姿态数据缺失，跳过。"], "suggestions": []}
    e = res["rms_err"]; mx = res["max_cmd"]
    level = "OK"
    obs, sug = [], []
    rp_max = float(max(e[0], e[1]))
    if rp_max > 0.10:
        level = "ERROR"
        obs.append(f"roll/pitch RMS 误差 {rp_max:.3f} rad（≈{np.degrees(rp_max):.1f}°），姿态环跟踪差。")
        sug.append("姿态环时间常数过大，尝试减小 `attitude_tau`（例如 0.30 → 0.20）。")
    elif rp_max > 0.05:
        level = _level_max(level, "WARN")
        obs.append(f"roll/pitch RMS 误差 {rp_max:.3f} rad（≈{np.degrees(rp_max):.1f}°），可优化。")
    if max(mx) > 5.0:
        level = _level_max(level, "WARN")
        obs.append(f"bodyrate 命令最大值 {max(mx):.2f} rad/s，控制激进，可能由 a_des 噪声引起。")
        sug.append("追溯到位置环 a_fb 是否抖动严重，结合速度噪声诊断处理。")
    if level == "OK":
        obs.append(f"姿态跟踪正常：roll/pitch RMS={e[0]:.3f}/{e[1]:.3f} rad，yaw RMS={e[2]:.4f} rad。")
    return {"level": level, "lines": obs, "suggestions": sug,
            "metrics": {"rp_rms_rad": rp_max, "yaw_rms_rad": float(e[2]),
                        "max_bodyrate_radps": float(max(mx)) if len(mx) else 0.0}}


def diagnose_timing(res):
    if res is None:
        return {"level": "INFO", "lines": ["时序数据缺失，跳过。"], "suggestions": []}
    overrun = res["overrun"]
    mean_dt, std_dt = res["mean"], res["std"]
    level = "OK"
    obs, sug = [], []
    if overrun > 10.0:
        level = "ERROR"
        obs.append(f"超时帧 {overrun:.2f}%（dt > 1.5×nominal），控制器没有稳定运行在标称周期。")
        sug.append("排查 ROS 调度（节点优先级、CPU 占用）、odom 频率是否稳定。")
    elif overrun > 5.0:
        level = _level_max(level, "WARN")
        obs.append(f"超时帧 {overrun:.2f}%，时序略有抖动。")
    if std_dt > 0.5 * mean_dt and overrun <= 5.0:
        level = _level_max(level, "WARN")
        obs.append(f"dt 抖动较大 (std/mean={std_dt/max(mean_dt,1e-6):.2f})，PID 差分稳定性下降。")
    obs.append(f"mean dt={mean_dt*1000:.2f} ms, std={std_dt*1000:.2f} ms, overrun={overrun:.2f}%。")
    if level == "OK":
        obs.append("控制周期稳定。")
    return {"level": level, "lines": obs, "suggestions": sug,
            "metrics": {"mean_dt_ms": float(mean_dt*1000), "std_dt_ms": float(std_dt*1000),
                        "overrun_pct": float(overrun)}}


def diagnose_landing(res):
    if res is None:
        return {"level": "INFO", "lines": ["AccFF 降落 runtime 数据缺失或无降落样本。"], "suggestions": []}
    level = "OK"
    obs, sug = [], []
    phases = res["phases"]
    triggers = res["triggers"]
    obs.append(f"降落 runtime 样本 {res['samples']} 帧，phase={phases}，trigger={triggers}。")
    obs.append(
        f"最低高度 {res['min_height']:.3f} m，最小 release_u={res['min_release']:.3f}，"
        f"最大 XY 误差 {res['max_xy']:.3f} m。"
    )
    if 1 not in phases and 2 not in phases:
        level = _level_max(level, "WARN")
        obs.append("没有进入 NearGround 或 TouchdownRelease，降落可能仍停留在 HighDescent。")
        sug.append("检查 `landing_height_above_ground` 是否低于 near_h，或是否满足地效卡滞触发条件。")
    if 2 not in triggers and 1 not in triggers and 1 in phases:
        level = _level_max(level, "WARN")
        obs.append("进入 NearGround 但 trigger 字段未记录有效来源。")
    if 1 in phases and res["min_release"] > 0.25:
        level = _level_max(level, "WARN")
        obs.append("NearGround 已进入，但 release_u 没有充分下降，推力释放可能仍不足。")
        sug.append("检查 `ramp_time_s`、`landing_release_progress` 和 `landing_thrust_release_limit`。")
    if res["max_xy"] > 0.25:
        level = _level_max(level, "WARN")
        obs.append("降落期间 XY 误差超过 hold 阈值，下降速度可能被主动压低。")
    if level == "OK":
        obs.append("AccFF 降落阶段、释放进度和 XY 误差记录正常。")
    return {"level": level, "lines": obs, "suggestions": sug,
            "metrics": {"min_height_m": res["min_height"], "min_release": res["min_release"],
                        "max_xy_m": res["max_xy"]}}


def generate_diagnostic_report(log: LogData, results: dict, out_dir: Path,
                               stem: str, console_text: str) -> Path:
    diag = {
        "position": diagnose_position(results.get("position")),
        "velocity": diagnose_velocity(results.get("velocity"), results.get("acc")),
        "acc":      diagnose_acc_decomp(results.get("acc")),
        "thrust":   diagnose_thrust(results.get("thrust"), log.df),
        "attitude": diagnose_attitude(results.get("attitude")),
        "timing":   diagnose_timing(results.get("timing")),
        "landing":  diagnose_landing(results.get("landing")),
    }

    overall = "OK"
    for d in diag.values():
        overall = _level_max(overall, d["level"])

    df = log.df
    duration = float(df["t"].iloc[-1] - df["t"].iloc[0]) if len(df) > 1 else 0.0
    rate = len(df) / max(duration, 1e-6)
    p = log.params

    md = []
    md.append(f"# Geometric Controller 诊断报告\n")
    md.append(f"**日志文件：** `{log.path.name}`  ")
    md.append(f"**生成时间：** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  ")
    md.append(f"**总体状态：** {LEVEL_BADGE[overall]}\n")

    md.append("## 1. 数据概览\n")
    md.append(f"- 时长：{duration:.2f} s")
    md.append(f"- 样本数：{len(df)} 帧")
    md.append(f"- 实测速率：{rate:.1f} Hz（标称 {p.get('controller_hz', 'NA')} Hz）")
    md.append(f"- 控制类型：{int(p.get('control_type', -1))}（0=Attitude, 1=BodyRate）")
    md.append(f"- hover_thrust_init：{p.get('hover_thrust_init', 'NA')}")
    md.append(f"- 整机质量：{p.get('mass_kg', 'NA')} kg\n")

    md.append("## 2. 关键参数\n")
    md.append("| 参数 | x | y | z |")
    md.append("|---|---|---|---|")
    for key in ("pos_kp", "pos_ki", "pos_kd", "vel_kp", "vel_ki", "vel_kd"):
        v = _vec3(p, key)
        md.append(f"| `{key}` | {v[0]:.3f} | {v[1]:.3f} | {v[2]:.3f} |")
    if "vel_error_filter_tau[0]" in p:
        v = _vec3(p, "vel_error_filter_tau")
        md.append(f"| `vel_error_filter_tau` | {v[0]:.3f} | {v[1]:.3f} | {v[2]:.3f} |")
        v = _vec3(p, "vel_error_gate_threshold")
        md.append(f"| `vel_error_gate_threshold` | {v[0]:.2f} | {v[1]:.2f} | {v[2]:.2f} |")
    md.append("")

    md.append("## 3. 诊断速览\n")
    md.append("| 领域 | 状态 |")
    md.append("|---|---|")
    name_map = {"position": "位置跟踪", "velocity": "速度噪声", "acc": "a_fb 构成",
                "thrust": "推力 / 估计器", "attitude": "姿态跟踪", "timing": "控制周期",
                "landing": "AccFF 降落"}
    for k, label in name_map.items():
        md.append(f"| {label} | {LEVEL_BADGE[diag[k]['level']]} |")
    md.append("")

    md.append("## 4. 各领域详细诊断\n")
    sections = [
        ("position", "位置跟踪", "01_position"),
        ("velocity", "速度噪声", "02_velocity_noise"),
        ("acc",      "a_fb 加速度反馈构成", "04_acc_decomp"),
        ("thrust",   "推力与悬停估计器", "05_thrust"),
        ("attitude", "姿态跟踪", "06_attitude"),
        ("timing",   "控制周期", "07_timing"),
        ("landing",  "AccFF 降落 runtime", "08_landing_runtime"),
    ]
    for key, label, png_suffix in sections:
        d = diag[key]
        md.append(f"### 4.{sections.index((key, label, png_suffix))+1} {label}  {LEVEL_BADGE[d['level']]}\n")
        for line in d["lines"]:
            md.append(f"- {line}")
        if d["suggestions"]:
            md.append("\n**建议：**")
            for s in d["suggestions"]:
                md.append(f"- {s}")
        png_path = out_dir / f"{stem}_{png_suffix}.png"
        if png_path.exists():
            md.append(f"\n![{label}]({png_path.name})")
        md.append("")

    # 预处理链单独成一节（如果存在）
    preproc_png = out_dir / f"{stem}_03_vel_preproc.png"
    if preproc_png.exists():
        md.append("### 4.7 速度误差预处理链（参考）\n")
        md.append("展示 raw → limited → filtered → processed 四阶段，用于核实速度误差预处理是否生效。")
        md.append(f"\n![预处理链]({preproc_png.name})\n")

    md.append("## 5. 调参建议汇总\n")
    all_sug = []
    for k in ("position", "velocity", "acc", "thrust", "attitude", "timing", "landing"):
        for s in diag[k]["suggestions"]:
            all_sug.append(f"- [{name_map[k]}] {s}")
    if all_sug:
        md.extend(all_sug)
    else:
        md.append("- 无建议（各项状态正常）。")
    md.append("")

    md.append("## 6. 原始数字附录\n")
    md.append("```")
    md.append(console_text.rstrip())
    md.append("```")

    report_path = out_dir / f"{stem}_diagnose.md"
    report_path.write_text("\n".join(md), encoding="utf-8")
    print(f"\n[Report] saved: {report_path}")
    return report_path


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
def print_summary(log: LogData, results: dict):
    df = log.df
    duration = float(df["t"].iloc[-1] - df["t"].iloc[0]) if len(df) > 1 else 0.0
    print()
    print("=== Geometric Controller Analysis Summary ===")
    print(f"File: {log.path}")
    print(f"Duration: {duration:.2f} s   Samples: {len(df)}   Rate: {(len(df)/max(duration,1e-6)):.1f} Hz")

    pos = results.get("position")
    if pos is not None:
        rms, mx = pos["rms"], pos["max"]
        print("[Position Tracking]")
        print(f"  RMS error  x={rms[0]:.3f}  y={rms[1]:.3f}  z={rms[2]:.3f}  (m)")
        print(f"  Max error  x={mx[0]:.3f}  y={mx[1]:.3f}  z={mx[2]:.3f}  (m)")
    vel = results.get("velocity")
    if vel is not None:
        s, e, f = vel["std"], vel["rms_err"], vel["peak_freq"]
        print("[Velocity Noise]")
        print(f"  Odom vel std  x={s[0]:.3f}  y={s[1]:.3f}  z={s[2]:.3f}  (m/s)")
        print(f"  Vel error RMS x={e[0]:.3f}  y={e[1]:.3f}  z={e[2]:.3f}  (m/s)")
        print(f"  Dominant noise freq  x={f[0]:.2f}  y={f[1]:.2f}  z={f[2]:.2f}  (Hz)")
    fr = results.get("acc")
    if fr is not None:
        print("[PID Term Contributions (time-avg magnitude fraction)]")
        print(f"  pos_P={fr['pos_P']*100:.1f}%  vel_P={fr['vel_P']*100:.1f}%  D={fr['D']*100:.1f}%  I={fr['I']*100:.1f}%")
    th = results.get("thrust")
    if th is not None:
        a = df.get("debug.accepted_hover_thrust")
        if a is not None:
            arr = a.to_numpy(dtype=float)
            arr = arr[np.isfinite(arr)]
            hstd = float(arr[int(0.8*len(arr)):].std()) if len(arr) > 0 else float("nan")
        else:
            hstd = float("nan")
        print("[Thrust]")
        print(f"  Mean={th['mean']:.3f}  Std={th['std']:.3f}  Saturation rate={th['sat_rate']:.2f} %")
        print(f"  Hover estimate std (last 20%) = {hstd:.4f}")
    at = results.get("attitude")
    if at is not None:
        e = at["rms_err"]
        print("[Attitude]")
        print(f"  RMS attitude error  roll={e[0]:.4f}  pitch={e[1]:.4f}  yaw={e[2]:.4f}  (rad)")
    tm = results.get("timing")
    if tm is not None:
        print("[Timing]")
        print(f"  Mean dt={tm['mean']*1000:.3f} ms  Std={tm['std']*1000:.3f} ms  Overrun rate={tm['overrun']:.2f} %")
    print("=" * 64)


class _Tee:
    """Forward writes to multiple streams. Used to capture stdout while still
    showing it live in the terminal."""
    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for s in self._streams:
            s.write(data)
        return len(data)

    def flush(self):
        for s in self._streams:
            s.flush()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Sunray geometric controller ULog analyzer")
    p.add_argument("path", type=Path, help=".ulg file or directory containing .ulg files")
    p.add_argument("--output-dir", type=Path, default=None,
                   help="output base dir (default: ulg parent dir). "
                        "A subdirectory named after the .ulg stem will be created inside.")
    p.add_argument("--dt-max", type=float, default=0.05,
                   help="exclude frames with current_dt above this (s); 0 to disable")
    args = p.parse_args()

    ulg = _resolve_ulg(args.path)
    base_dir = (args.output_dir or ulg.parent).resolve()
    stem = ulg.stem
    out_dir = base_dir / stem
    out_dir.mkdir(parents=True, exist_ok=True)

    log = load_log(ulg, args.dt_max)

    buf = io.StringIO()
    tee = _Tee(sys.stdout, buf)
    with redirect_stdout(tee):
        print_load_summary(log)

        results = {}
        results["position"] = analyze_position(log, out_dir, stem)
        results["velocity"] = analyze_velocity_noise(log, out_dir, stem)
        analyze_vel_preproc(log, out_dir, stem)
        results["acc"]      = analyze_acc_decomp(log, out_dir, stem)
        results["thrust"]   = analyze_thrust(log, out_dir, stem)
        results["attitude"] = analyze_attitude(log, out_dir, stem)
        results["timing"]   = analyze_timing(log, out_dir, stem)
        results["landing"]  = analyze_landing_runtime(log, out_dir, stem)
        print_summary(log, results)

    console_text = buf.getvalue()
    generate_diagnostic_report(log, results, out_dir, stem, console_text)
    print(f"\nAll outputs in: {out_dir}")


if __name__ == "__main__":
    main()
