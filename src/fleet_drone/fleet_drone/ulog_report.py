"""wyvern_ulog: analyse a PX4 ULog of a wyvern flight (pyulog) and save plots.

    ros2 run fleet_drone wyvern_ulog <file.ulg | px4 working dir> [--tag keep] [--out docs/img] [--json m.json]

Given a directory (e.g. <ws>/.px4/keep) the newest log/**/*.ulg in it is used.
Metrics (armed + offboard part of the flight):
  position tracking error  |vehicle_local_position - vehicle_local_position_setpoint|  rms / p95 / max
  attitude                 roll/pitch tracking error vs vehicle_attitude_setpoint, max tilt
  motors                   actuator_motors (normalised thrust 0..1): mean/min/max, saturated fraction
Plots: drone_<tag>_tracking.png, drone_<tag>_path.png, drone_<tag>_attitude.png, drone_<tag>_motors.png
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
NAV_OFFBOARD, ARMED = 14, 2


def newest_ulog(path):
    if os.path.isfile(path):
        return path
    files = glob.glob(os.path.join(path, "log", "**", "*.ulg"), recursive=True) + \
        glob.glob(os.path.join(path, "**", "*.ulg"), recursive=True)
    if not files:
        raise SystemExit("no .ulg under %s" % path)
    return max(set(files), key=os.path.getmtime)


def topic(ulog, name, multi=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi:
            return d.data
    return None


def euler(q):
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def windows(t, flag):
    """[(t0, t1)] where flag is true (flag sampled at t)."""
    out, start = [], None
    for ti, f in zip(t, flag):
        if f and start is None:
            start = ti
        elif not f and start is not None:
            out.append((start, ti))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def mask(t, wins):
    m = np.zeros(len(t), bool)
    for a, b in wins:
        m |= (t >= a) & (t <= b)
    return m


def hold(t_src, v_src, t):
    """Sample-and-hold interpolation (setpoints are piecewise constant between messages)."""
    i = np.clip(np.searchsorted(t_src, t, side="right") - 1, 0, len(t_src) - 1)
    return v_src[i]


def lerp(t_src, v_src, t):
    """Linear interpolation per column. The logger keeps vehicle_local_position_setpoint at only 10 Hz while the
    mission streams 50 Hz, so a sample-and-hold would add a speed-proportional sawtooth that is not real."""
    return np.stack([np.interp(t, t_src, v_src[:, j]) for j in range(v_src.shape[1])], 1)


def analyse(path):
    from pyulog import ULog
    want = ["vehicle_local_position", "vehicle_local_position_setpoint", "vehicle_attitude", "vehicle_attitude_setpoint",
            "actuator_motors", "actuator_outputs", "vehicle_status", "vehicle_land_detected"]
    u = ULog(path, want)
    st = topic(u, "vehicle_status")
    lp = topic(u, "vehicle_local_position")
    if st is None or lp is None:
        raise SystemExit("log lacks vehicle_status / vehicle_local_position")
    ts = st["timestamp"] / 1e6
    armed = windows(ts, st["arming_state"] == ARMED)
    offb = windows(ts, (st["arming_state"] == ARMED) & (st["nav_state"] == NAV_OFFBOARD))
    t0 = armed[0][0] if armed else ts[0]
    R = dict(file=path, armed_s=float(sum(b - a for a, b in armed)), offboard_s=float(sum(b - a for a, b in offb)))

    t = lp["timestamp"] / 1e6
    pos = np.stack([lp["x"], lp["y"], lp["z"]], 1)
    D = dict(t0=t0, t=t, pos=pos, armed=armed, offb=offb)
    ld = topic(u, "vehicle_land_detected")
    t_air = None
    if ld is not None and armed:
        tl = ld["timestamp"] / 1e6
        up = np.where((tl > armed[0][0]) & (ld["landed"] == 0))[0]
        t_air = float(tl[up[0]]) if len(up) else None
    R["airborne_after_arm_s"] = None if t_air is None else round(t_air - t0, 2)
    sp = topic(u, "vehicle_local_position_setpoint")
    if sp is not None and len(sp["timestamp"]):
        tsp = sp["timestamp"] / 1e6
        spos = np.stack([sp["x"], sp["y"], sp["z"]], 1)
        m = mask(t, offb) & (t >= tsp[0])
        sph = lerp(tsp, spos, t)
        ok = m & np.all(np.isfinite(sph), 1)
        e = np.linalg.norm(pos - sph, axis=1)
        eh = np.linalg.norm((pos - sph)[:, :2], axis=1)
        if ok.any():
            R["track_err_rms_m"] = float(np.sqrt(np.mean(e[ok] ** 2)))
            R["track_err_p95_m"] = float(np.percentile(e[ok], 95))
            R["track_err_max_m"] = float(e[ok].max())
            R["track_err_horiz_rms_m"] = float(np.sqrt(np.mean(eh[ok] ** 2)))
            R["track_err_vert_rms_m"] = float(np.sqrt(np.mean((pos - sph)[ok, 2] ** 2)))
        # same, excluding lift-off (until 3 s after the land detector reports airborne)
        okc = ok & (t >= (t_air if t_air is not None else t0) + 3.0)
        if okc.any():
            R["track_err_airborne_rms_m"] = float(np.sqrt(np.mean(e[okc] ** 2)))
            R["track_err_airborne_p95_m"] = float(np.percentile(e[okc], 95))
            R["track_err_airborne_max_m"] = float(e[okc].max())
        D.update(tsp=tsp, spos=spos, sph=sph, err=e, ok=ok)
        speed = np.linalg.norm(np.stack([lp["vx"], lp["vy"], lp["vz"]], 1), axis=1)
        R["max_speed_mps"] = float(speed[mask(t, offb)].max()) if mask(t, offb).any() else 0.0
        R["max_alt_m"] = float((-pos[mask(t, armed), 2]).max()) if mask(t, armed).any() else 0.0

    att = topic(u, "vehicle_attitude")
    if att is not None:
        ta = att["timestamp"] / 1e6
        q = np.stack([att["q[%d]" % i] for i in range(4)])
        roll, pitch, yaw = euler(q)
        ma = mask(ta, armed)
        R["max_tilt_deg"] = float(np.degrees(np.arccos(np.clip(np.cos(roll) * np.cos(pitch), -1, 1)))[ma].max()) if ma.any() else 0.0
        D.update(ta=ta, rpy=np.degrees(np.stack([roll, pitch, yaw], 1)))
        asp = topic(u, "vehicle_attitude_setpoint")
        if asp is not None and len(asp["timestamp"]):
            tas = asp["timestamp"] / 1e6
            qd = np.stack([asp["q_d[%d]" % i] for i in range(4)])
            r_d, p_d, y_d = euler(qd)
            rpy_d = np.degrees(np.stack([r_d, p_d, y_d], 1))
            h = hold(tas, rpy_d, ta)
            mo = mask(ta, offb) & (ta >= tas[0])
            if mo.any():
                R["roll_err_rms_deg"] = float(np.sqrt(np.mean((D["rpy"][mo, 0] - h[mo, 0]) ** 2)))
                R["pitch_err_rms_deg"] = float(np.sqrt(np.mean((D["rpy"][mo, 1] - h[mo, 1]) ** 2)))
            D.update(tas=tas, rpy_d=rpy_d)

    mot = topic(u, "actuator_motors")
    if mot is not None:
        tm = mot["timestamp"] / 1e6
        c = np.stack([mot["control[%d]" % i] for i in range(4)], 1)
        mm = mask(tm, armed) & np.all(np.isfinite(c), 1)
        if mm.any():
            R["motor_mean"] = float(c[mm].mean())
            R["motor_min"] = float(c[mm].min())
            R["motor_max"] = float(c[mm].max())
            R["motor_sat_frac"] = float((c[mm] >= 0.99).any(1).mean())
            R["motor_hover_mean"] = [float(v) for v in c[mm].mean(0)]
        D.update(tm=tm, motors=c)
    if ld is not None:
        R["landed_at_end"] = bool(ld["landed"][-1])
    R["disarmed_at_end"] = bool(st["arming_state"][-1] != ARMED)
    return R, D


def _style(ax, ylabel):
    ax.set_facecolor(SURF)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)


def _shade(ax, D):
    for a, b in D["offb"]:
        ax.axvspan(a - D["t0"], b - D["t0"], color="#6250d6", alpha=0.06, lw=0)


def _break_wraps(a):
    """Angle series [deg] with NaN at the +-180 wrap so the plot shows no vertical jump lines."""
    a = np.asarray(a, float).copy()
    jump = np.abs(np.diff(a)) > 180.0
    a[1:][jump] = np.nan
    return a


def plots(R, D, out, tag):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(out, exist_ok=True)
    files = []
    t = D["t"] - D["t0"]
    enu = np.stack([D["pos"][:, 1], D["pos"][:, 0], -D["pos"][:, 2]], 1)
    title_kw = dict(loc="left", color=INK, fontsize=11)

    if "sph" in D:
        sp = D["sph"]
        senu = np.stack([sp[:, 1], sp[:, 0], -sp[:, 2]], 1)
        ok = D["ok"]
        fig, axs = plt.subplots(4, 1, figsize=(10, 8.2), sharex=True, facecolor=SURF)
        for i, lab in enumerate(("east [m]", "north [m]", "up [m]")):
            ax = axs[i]
            _style(ax, lab)
            _shade(ax, D)
            ax.plot(t[ok], senu[ok, i], color=INK2, lw=1.4, ls="--", label="setpoint")
            ax.plot(t, enu[:, i], color=SERIES[0], lw=1.6, label="estimated position")
        axs[0].legend(loc="upper right", fontsize=8, frameon=False)
        ax = axs[3]
        _style(ax, "error [m]")
        _shade(ax, D)
        ax.plot(t[ok], D["err"][ok], color=SERIES[1], lw=1.4)
        ax.set_xlabel("time since arming [s] (shaded: offboard)", color=INK2, fontsize=9)
        axs[0].set_title("wyvern %s: position tracking  (rms %.2f m, p95 %.2f m; after lift-off rms %.2f m, max %.2f m)" % (
            tag, R.get("track_err_rms_m", float("nan")), R.get("track_err_p95_m", float("nan")),
            R.get("track_err_airborne_rms_m", float("nan")), R.get("track_err_airborne_max_m", float("nan"))), **title_kw)
        axs[3].set_xlim(max(t[0], -5.0), t[-1])
        fig.tight_layout()
        f = os.path.join(out, "drone_%s_tracking.png" % tag)
        fig.savefig(f, dpi=110, facecolor=SURF)
        plt.close(fig)
        files.append(f)

        fig, ax = plt.subplots(figsize=(7.5, 7), facecolor=SURF)
        wp = D.get("mission")
        o = np.array([wp["takeoff"]["x"], wp["takeoff"]["y"], 0.0]) if wp else np.zeros(3)
        _style(ax, "y / north [m] (world frame)" if wp else "north [m] (PX4 local frame)")
        ma = mask(D["t"], D["armed"])
        ax.plot(senu[ok, 0] + o[0], senu[ok, 1] + o[1], color=INK2, lw=1.4, ls="--", label="setpoint")
        ax.plot(enu[ma, 0] + o[0], enu[ma, 1] + o[1], color=SERIES[0], lw=1.6, label="estimated path")
        ax.plot(enu[ma, 0][:1] + o[0], enu[ma, 1][:1] + o[1], "o", color=SERIES[2], ms=8, label="takeoff")
        if wp:
            xs = [w["x"] for w in wp["waypoints"]]
            ys = [w["y"] for w in wp["waypoints"]]
            ax.plot(xs, ys, "D", color=SERIES[1], ms=6, mfc="none", mew=1.5, label="waypoints")
            for w in wp["waypoints"]:
                ax.annotate(w["name"], (w["x"], w["y"]), xytext=(5, 5), textcoords="offset points", fontsize=7,
                            color=INK2)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x / east [m] (world frame)" if wp else "east [m]", color=INK2, fontsize=9)
        ax.legend(loc="best", fontsize=8, frameon=False)
        ax.set_title("wyvern %s: flown path (top view)" % tag, **title_kw)
        fig.tight_layout()
        f = os.path.join(out, "drone_%s_path.png" % tag)
        fig.savefig(f, dpi=110, facecolor=SURF)
        plt.close(fig)
        files.append(f)

    if "rpy" in D:
        ta = D["ta"] - D["t0"]
        fig, axs = plt.subplots(3, 1, figsize=(10, 6.6), sharex=True, facecolor=SURF)
        for i, lab in enumerate(("roll [deg]", "pitch [deg]", "yaw [deg] (NED)")):
            ax = axs[i]
            _style(ax, lab)
            _shade(ax, D)
            if "rpy_d" in D:
                ax.plot(D["tas"] - D["t0"], _break_wraps(D["rpy_d"][:, i]), color=INK2, lw=1.2, ls="--", label="setpoint")
            ax.plot(ta, _break_wraps(D["rpy"][:, i]), color=SERIES[0], lw=1.4, label="attitude")
        axs[0].legend(loc="upper right", fontsize=8, frameon=False)
        axs[2].set_xlabel("time since arming [s] (shaded: offboard)", color=INK2, fontsize=9)
        axs[0].set_title("wyvern %s: attitude  (roll err rms %.2f deg, pitch err rms %.2f deg, max tilt %.1f deg)" % (
            tag, R.get("roll_err_rms_deg", float("nan")), R.get("pitch_err_rms_deg", float("nan")),
            R.get("max_tilt_deg", float("nan"))), **title_kw)
        lo = max(ta[0], -5.0)
        axs[2].set_xlim(lo, ta[-1])
        fig.tight_layout()
        f = os.path.join(out, "drone_%s_attitude.png" % tag)
        fig.savefig(f, dpi=110, facecolor=SURF)
        plt.close(fig)
        files.append(f)

    if "motors" in D:
        tm = D["tm"] - D["t0"]
        fig, ax = plt.subplots(figsize=(10, 4.2), facecolor=SURF)
        _style(ax, "motor command [0..1]")
        _shade(ax, D)
        names = ["motor 1 (front right, ccw)", "motor 2 (rear left, ccw)", "motor 3 (front left, cw)",
                 "motor 4 (rear right, cw)"]
        for i in range(4):
            ax.plot(tm, D["motors"][:, i], color=SERIES[i], lw=1.2, label=names[i])
        ax.set_ylim(-0.02, 1.02)
        ax.set_xlim(max(tm[0], -5.0), tm[-1])
        ax.legend(loc="upper right", fontsize=8, frameon=False, ncol=2)
        ax.set_xlabel("time since arming [s] (shaded: offboard)", color=INK2, fontsize=9)
        ax.set_title("wyvern %s: motor outputs  (mean %.2f, max %.2f, saturated %.1f %% of the time)" % (
            tag, R.get("motor_mean", float("nan")), R.get("motor_max", float("nan")),
            100 * R.get("motor_sat_frac", float("nan"))), **title_kw)
        fig.tight_layout()
        f = os.path.join(out, "drone_%s_motors.png" % tag)
        fig.savefig(f, dpi=110, facecolor=SURF)
        plt.close(fig)
        files.append(f)
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ulog", help=".ulg file, or a PX4 working directory (newest log is used)")
    ap.add_argument("--tag", default="flight")
    ap.add_argument("--out", default="docs/img")
    ap.add_argument("--json", default="")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--world", default="", help="draw the path in world coordinates with this world's waypoints "
                    "(assumes the default spawn at the mission takeoff point)")
    a = ap.parse_args()
    path = newest_ulog(a.ulog)
    R, D = analyse(path)
    if a.world:
        from fleet_drone import route
        D["mission"] = route.load_mission(a.world)
    if not a.no_plots:
        R["plots"] = plots(R, D, a.out, a.tag)
    txt = json.dumps(R, indent=1, default=lambda o: float(o) if isinstance(o, np.floating) else str(o))
    print(txt)
    if a.json:
        with open(a.json, "w") as fh:
            fh.write(txt)
    ok = all(math.isfinite(v) for v in R.values() if isinstance(v, float))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
