#!/usr/bin/env python3
import argparse
import os
import glob

import rosbag
import numpy as np
from scipy.io import savemat
import matplotlib.pyplot as plt

# -----------------------------
# Topic → Variable mapping
# -----------------------------

# Joint-space experiment (exp_JointSpace)
TOPIC_MAP_JS = {
    '/cepheus/q1': 'q1',
    '/cepheus/q2': 'q2',
    '/cepheus/q3': 'q3',
    '/cepheus/theta0': 'theta0',

    '/cepheus/q1d': 'q1d',
    '/cepheus/q2d': 'q2d',
    '/cepheus/q3d': 'q3d',
    '/cepheus/theta0d': 'theta0d',

    '/cepheus/q1dot': 'q1dot',
    '/cepheus/q2dot': 'q2dot',
    '/cepheus/q3dot': 'q3dot',
    '/cepheus/theta0dot': 'theta0dot',

    '/cepheus/q1ddot': 'q1ddot',
    '/cepheus/q2ddot': 'q2ddot',
    '/cepheus/q3ddot': 'q3ddot',
    '/cepheus/theta0ddot': 'theta0ddot',

    '/cepheus/torquerw': 'torqrw',
    '/cepheus/torqueq1': 'torq1',
    '/cepheus/torqueq2': 'torq2',
    '/cepheus/torqueq3': 'torq3',
}

# Cartesian-space experiment (exp_CartesianSpace)
# x, y are in millimetres
TOPIC_MAP_CS = {
    # End-effector actual
    '/cepheus/xee_x': 'xee',
    '/cepheus/xee_x_dot': 'xee_dot',
    '/cepheus/xee_y': 'yee',
    '/cepheus/xee_y_dot': 'yee_dot',

    # Desired Cartesian (reference)
    '/cepheus/xd_x': 'xd',
    '/cepheus/xd_x_dot': 'xd_dot',
    '/cepheus/xd_y': 'yd',
    '/cepheus/xd_y_dot': 'yd_dot',

    # Target Cartesian (trajectory) - read to MAT only
    '/cepheus/xt_x': 'xt_x',
    '/cepheus/xt_x_dot': 'xt_x_dot',
    '/cepheus/xt_y': 'xt_y',
    '/cepheus/xt_y_dot': 'xt_y_dot',

    # End-effector angle actual & desired
    '/cepheus/xee_theta': 'thetaee',
    '/cepheus/xee_theta_dot': 'thetaee_dot',   # actual θ̇_ee
    '/cepheus/xd_theta': 'thetad',             # desired θ_ee
    '/cepheus/xd_theta_dot': 'thetad_dot',     # desired θ̇_ee

    # Target EE angle - MAT only
    '/cepheus/xt_theta': 'theta_t',
    '/cepheus/xt_theta_dot': 'theta_t_dot',

    # Base angle actual & desired
    '/cepheus/xee_theta0': 'theta0',
    '/cepheus/xee_theta0_dot': 'theta0dot',    # actual θ̇_0
    '/cepheus/xd_theta0': 'theta0d',           # desired θ_0
    '/cepheus/xd_theta0_dot': 'theta0d_dot',   # desired θ̇_0

    # Target base angle - MAT only
    '/cepheus/xt_theta0': 'theta0_t',
    '/cepheus/xt_theta0_dot': 'theta0_t_dot',

    # Torques
    '/cepheus/torquerw': 'torquerw',
    '/cepheus/torqueq1': 'torq1',
    '/cepheus/torqueq2': 'torq2',
    '/cepheus/torqueq3': 'torq3',
}

TOPIC_MAPS = {
    'js': TOPIC_MAP_JS,
    'cs': TOPIC_MAP_CS,
    # rcs / sid intentionally unused for now
}

# -----------------------------
# Helpers
# -----------------------------

def pad_to_len(arr: np.ndarray, target_len: int) -> np.ndarray:
    """Pad 1D array with NaNs to target_len (or truncate if longer)."""
    n = arr.shape[0]
    if n == target_len:
        return arr
    if n > target_len:
        return arr[:target_len]
    out = np.full((target_len,), np.nan, dtype=float)
    out[:n] = arr
    return out

# -----------------------------
# Plotting: JointSpace
# -----------------------------

def plot_jointspace(time, data, bagname, save_base):
    """
    4×4 layout:

      columns: q1, q2, q3, theta0
      rows:
        0: angle (actual vs desired)      → [rad]
        1: angular velocity               → [rad/s]
        2: angle error                    → [rad]
        3: torque                         → [Nm]
    """
    joints = [
        {
            "angle": "q1",
            "angle_d": "q1d",
            "vel": "q1dot",
            "vel_d": "q1ddot",
            "torque": "torq1",
            "titles": {
                0: "Q1 Pos [rad]",
                1: "Q1 Vel [rad/s]",
                2: "Q1 Pos Error [rad]",
                3: "Q1 Torque [Nm]",
            },
        },
        {
            "angle": "q2",
            "angle_d": "q2d",
            "vel": "q2dot",
            "vel_d": "q2ddot",
            "torque": "torq2",
            "titles": {
                0: "Q2 Pos [rad]",
                1: "Q2 Vel [rad/s]",
                2: "Q2 Pos Error [rad]",
                3: "Q2 Torque [Nm]",
            },
        },
        {
            "angle": "q3",
            "angle_d": "q3d",
            "vel": "q3dot",
            "vel_d": "q3ddot",
            "torque": "torq3",
            "titles": {
                0: "Q3 Pos [rad]",
                1: "Q3 Vel [rad/s]",
                2: "Q3 Pos Error [rad]",
                3: "Q3 Torque [Nm]",
            },
        },
        {
            "angle": "theta0",
            "angle_d": "theta0d",
            "vel": "theta0dot",
            "vel_d": "theta0ddot",
            "torque": "torqrw",
            "titles": {
                0: "Base Angle [rad]",
                1: "Base Ang. Vel [rad/s]",
                2: "Base Angle Error [rad]",
                3: "RW Torque [Nm]",
            },
        },
    ]

    fig, axes = plt.subplots(
        nrows=4,
        ncols=4,
        sharex=True,
        figsize=(18, 10),
        constrained_layout=True,
    )

    fig.suptitle(f"Bag: {bagname} (JointSpace)", fontsize=16, fontweight='bold')

    for col, j in enumerate(joints):
        angle = j["angle"]
        angle_d = j["angle_d"]
        vel = j["vel"]
        vel_d = j["vel_d"]
        torque = j["torque"]

        has_angle = angle in data and np.any(~np.isnan(data[angle]))
        has_angle_d = angle_d in data and np.any(~np.isnan(data[angle_d]))
        has_vel = vel in data and np.any(~np.isnan(data[vel]))
        has_vel_d = vel_d in data and np.any(~np.isnan(data[vel_d]))
        has_torque = torque in data and np.any(~np.isnan(data[torque]))

        # Row 0: position (angle)
        ax = axes[0, col]
        if has_angle:
            ax.plot(time, data[angle], label="actual")
        if has_angle_d:
            ax.plot(time, data[angle_d], label="desired", linestyle='--')
        ax.set_title(j["titles"][0], fontsize=9)
        ax.tick_params(axis='y', which='both', labelleft=True)

        # Row 1: velocity
        ax = axes[1, col]
        if has_vel:
            ax.plot(time, data[vel], label="vel")
        if has_vel_d:
            ax.plot(time, data[vel_d], label="vel_ref", linestyle='--')
        ax.set_title(j["titles"][1], fontsize=9)
        ax.tick_params(axis='y', which='both', labelleft=True)

        # Row 2: position error
        ax = axes[2, col]
        if has_angle and has_angle_d:
            err = data[angle_d] - data[angle]
            ax.plot(time, err, label="error")
        ax.set_title(j["titles"][2], fontsize=9)
        ax.tick_params(axis='y', which='both', labelleft=True)

        # Row 3: torque
        ax = axes[3, col]
        if has_torque:
            ax.plot(time, data[torque], label="torque")
        ax.set_title(j["titles"][3], fontsize=9)
        ax.set_xlabel("time [s]")
        ax.tick_params(axis='y', which='both', labelleft=True)

    # Legend from first subplot with labels
    handles, labels = [], []
    for r in range(4):
        h, lab = axes[r, 0].get_legend_handles_labels()
        if h:
            handles, labels = h, lab
            break
    if handles:
        fig.legend(handles, labels, loc="upper right")

    png_path = f"{save_base}.png"
    fig.savefig(png_path, dpi=150)
    print(f"[js] Saved plot → {png_path}")

    plt.show()

# -----------------------------
# Plotting: CartesianSpace
# -----------------------------

def plot_cartesianspace(time, data, bagname, save_base):
    """
    4×4 layout for CartesianSpace:

      columns: X, Y, End-Eff. angle, Base angle
      rows:
        0: pos/angle (actual vs desired)
        1: velocity / angular velocity (actual vs desired)
        2: position / angle error (desired - actual)
        3: torque
    """
    cols = [
        {
            'pos': 'xee',
            'pos_d': 'xd',
            'vel': 'xee_dot',
            'vel_d': 'xd_dot',
            'torque': 'torq1',
            'titles': {
                0: "End-Eff. X [mm]",
                1: "End-Eff. X Vel [mm/s]",
                2: "End-Eff. X Error [mm]",
                3: "Q1 Torque [Nm]",
            },
        },
        {
            'pos': 'yee',
            'pos_d': 'yd',
            'vel': 'yee_dot',
            'vel_d': 'yd_dot',
            'torque': 'torq2',
            'titles': {
                0: "End-Eff. Y [mm]",
                1: "End-Eff. Y Vel [mm/s]",
                2: "End-Eff. Y Error [mm]",
                3: "Q2 Torque [Nm]",
            },
        },
        {
            'pos': 'thetaee',
            'pos_d': 'thetad',
            'vel': 'thetaee_dot',   # actual
            'vel_d': 'thetad_dot',  # desired
            'torque': 'torq3',
            'titles': {
                0: "End-Eff. Angle [rad]",
                1: "End-Eff. Ang. Vel [rad/s]",
                2: "End-Eff. Angle Error [rad]",
                3: "Q3 Torque [Nm]",
            },
        },
        {
            'pos': 'theta0',
            'pos_d': 'theta0d',
            'vel': 'theta0dot',     # actual
            'vel_d': 'theta0d_dot', # desired
            'torque': 'torquerw',
            'titles': {
                0: "Base Angle [rad]",
                1: "Base Ang. Vel [rad/s]",
                2: "Base Angle Error [rad]",
                3: "RW Torque [Nm]",
            },
        },
    ]

    fig, axes = plt.subplots(
        nrows=4,
        ncols=4,
        sharex=True,
        figsize=(18, 10),
        constrained_layout=True,
    )

    fig.suptitle(f"Bag: {bagname} (CartesianSpace)", fontsize=16, fontweight='bold')

    for col_idx, cfg in enumerate(cols):
        pos = cfg['pos']
        pos_d = cfg['pos_d']
        vel = cfg['vel']
        vel_d = cfg['vel_d']
        torque = cfg['torque']

        pos_exists = pos in data and np.any(~np.isnan(data[pos]))
        pos_d_exists = pos_d in data and np.any(~np.isnan(data[pos_d]))
        vel_exists = vel in data and np.any(~np.isnan(data[vel]))
        vel_d_exists = vel_d in data and np.any(~np.isnan(data[vel_d]))
        torque_exists = torque in data and np.any(~np.isnan(data[torque]))

        # Row 0 – position / angle (actual vs desired)
        ax = axes[0, col_idx]
        if pos_exists:
            ax.plot(time, data[pos], label='actual')
        if pos_d_exists:
            ax.plot(time, data[pos_d], label='desired', linestyle='--')
        ax.set_title(cfg['titles'][0], fontsize=9)
        ax.tick_params(axis='y', which='both', labelleft=True)

        # Row 1 – velocity / angular velocity (actual vs desired)
        ax = axes[1, col_idx]
        if vel_exists:
            ax.plot(time, data[vel], label='vel')
        if vel_d_exists:
            ax.plot(time, data[vel_d], label='vel_des', linestyle='--')
        ax.set_title(cfg['titles'][1], fontsize=9)
        ax.tick_params(axis='y', which='both', labelleft=True)

        # Row 2 – error (desired - actual)
        ax = axes[2, col_idx]
        if pos_exists and pos_d_exists:
            err = data[pos_d] - data[pos]
            ax.plot(time, err, label='error')
        ax.set_title(cfg['titles'][2], fontsize=9)
        ax.tick_params(axis='y', which='both', labelleft=True)

        # Row 3 – torque
        ax = axes[3, col_idx]
        if torque_exists:
            ax.plot(time, data[torque], label='torque')
        ax.set_title(cfg['titles'][3], fontsize=9)
        ax.set_xlabel('time [s]')
        ax.tick_params(axis='y', which='both', labelleft=True)

    # Common legend
    handles, labels = [], []
    for r in range(4):
        h, lab = axes[r, 0].get_legend_handles_labels()
        if h:
            handles, labels = h, lab
            break
    if handles:
        fig.legend(handles, labels, loc='upper right')

    png_path = f"{save_base}.png"
    fig.savefig(png_path, dpi=150)
    print(f"[cs] Saved plot → {png_path}")

    plt.show()

# -----------------------------
# Core conversion logic
# -----------------------------

def convert_bag(bag_file: str, exp: str, bagdir: str, bagname: str, no_plot: bool) -> None:
    # RCS and SID: do nothing for now
    if exp in ('rcs', 'sid'):
        print(f"[{exp}] Not implemented yet → doing nothing.")
        return

    topic_map = TOPIC_MAPS[exp]

    # Collect raw lists for all known signals
    signals = {v: [] for v in topic_map.values()}

    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=list(topic_map.keys())):
        key = topic_map[topic]
        val = getattr(msg, 'data', None)  # expect std_msgs/Float64
        if val is None:
            continue
        signals[key].append(val)
    bag.close()

    # Choose base signal for time indexing
    base_key = None
    if exp == 'cs' and len(signals.get('xee', [])) > 0:
        base_key = 'xee'
    elif len(signals.get('q1', [])) > 0:
        base_key = 'q1'
    else:
        for k, v in signals.items():
            if len(v) > 0:
                base_key = k
                break

    if base_key is None:
        raise RuntimeError("No data found in any of the expected topics.")

    base_len = len(signals[base_key])
    sample_rate = 100.0  # Hz
    time = np.arange(base_len) / sample_rate
    print(f"[{exp}] Using base signal '{base_key}' with {base_len} samples")
    print(f"[{exp}] Assuming {sample_rate:.1f} Hz → duration {time[-1]:.2f} s")

    # Build MAT data with unified length
    mat_data = {}
    for k, v in signals.items():
        arr = np.array(v, dtype=float) if len(v) > 0 else np.array([], dtype=float)
        mat_data[k] = pad_to_len(arr, base_len)
    mat_data['time'] = time

    # Save MAT next to bag – name is exactly the bag name
    mat_path = os.path.join(bagdir, f"{bagname}.mat")
    savemat(mat_path, mat_data)
    print(f"[{exp}] Saved MAT  → {mat_path}")

    if no_plot:
        return

    save_base = os.path.join(bagdir, bagname)

    if exp == 'js':
        plot_jointspace(time, mat_data, bagname, save_base)
    elif exp == 'cs':
        plot_cartesianspace(time, mat_data, bagname, save_base)

# -----------------------------
# Main
# -----------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Convert ROS bag to .mat and generate plots for "
            "exp_JointSpace / exp_CartesianSpace. "
            "RCS and SID are accepted but currently do nothing."
        )
    )
    # bagname is optional – if omitted we'll use the latest .bag in bagdir
    parser.add_argument(
        'bagname',
        nargs='?',
        help='Bag file name (without .bag). If omitted, the newest .bag in bagdir is used.',
    )
    parser.add_argument(
        '--bagdir',
        default='/home/theo/cepheus_ws_v2/bags/',
        help='Directory containing bag files (default: /home/theo/cepheus_ws_v2/bags/)',
    )
    parser.add_argument(
        '--no-plot',
        dest='no_plot',
        action='store_true',
        help='Do NOT show/produce plots (default: plots are generated)',
    )
    parser.add_argument(
        '--exp', '-e',
        choices=['js', 'cs', 'rcs', 'sid'],
        required=True,
        help='Experiment type (MANDATORY): '
             'js = JointSpace, cs = CartesianSpace, '
             'rcs = RelCartesianSpace (no-op), sid = SystemId (no-op).',
    )

    args = parser.parse_args()

    bagdir = args.bagdir.rstrip('/')

    if args.bagname is None:
        pattern = os.path.join(bagdir, '*.bag')
        bag_files = glob.glob(pattern)
        if not bag_files:
            raise RuntimeError(f"No .bag files found in {bagdir}, and no bagname was provided.")
        latest_bag = max(bag_files, key=os.path.getmtime)
        args.bagname = os.path.splitext(os.path.basename(latest_bag))[0]
        print(f"No bagname provided, using latest bag: {args.bagname}")

    bag_file = os.path.join(bagdir, f"{args.bagname}.bag")
    print(f"Opening bag: {bag_file}")

    if not os.path.exists(bag_file):
        raise FileNotFoundError(f"Bag file not found: {bag_file}")

    print(f"Experiment mode: {args.exp}")

    convert_bag(bag_file, args.exp, bagdir, args.bagname, args.no_plot)


if __name__ == '__main__':
    main()

