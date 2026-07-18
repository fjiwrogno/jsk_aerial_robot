#!/usr/bin/env python3
"""Plot the underwater demo results recorded by underwater_demo.py.

Usage:
  python3 plot_underwater_demo.py <demo_output_dir>   # containing demo.bag + segments.json

Produces in the same directory:
  attitude_tracking.png   desired vs actual roll / pitch / yaw
  depth_tracking.png      target vs measured depth (+ error)
  xy_motion.png           x / y position + body-frame horizontal velocity
  summary.json            per-segment numeric summary (displacement, tracking error, ...)
"""

import json
import math
import os
import sys

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rosbag
import tf.transformations as tft

NS = '/nami'

# palette (dataviz reference, light mode)
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
GRID = '#e7e6e2'
C_ACTUAL = '#2a78d6'   # series 1: measured / actual
C_TARGET = '#008300'   # series 2: desired / target
C_AUX = '#e87ba4'      # series 3: auxiliary (body-frame lateral)
BAND = (0, 0, 0, 0.05)  # command-active shading

plt.rcParams.update({
    'figure.facecolor': SURFACE,
    'axes.facecolor': SURFACE,
    'savefig.facecolor': SURFACE,
    'text.color': INK,
    'axes.edgecolor': INK2,
    'axes.labelcolor': INK,
    'xtick.color': INK2,
    'ytick.color': INK2,
    'axes.grid': True,
    'grid.color': GRID,
    'grid.linewidth': 0.8,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'font.size': 11,
    'legend.frameon': False,
})


def read_bag(bag_path):
    d = {
        'pid': {'t': [], 'roll_tgt': [], 'roll_err': [], 'pitch_tgt': [], 'pitch_err': [],
                'yaw_tgt': [], 'yaw_err': []},
        'odom': {'t': [], 'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': [],
                 'vx': [], 'vy': [], 'vz': []},
        'tgt_depth': {'t': [], 'v': []},
        'pressure': {'t': [], 'depth': []},
        'cmd': {'t': [], 'x': [], 'y': [], 'z': [], 'yaw': []},
        'thrust': {'t': [], 'f': []},
    }
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages():
            ts = t.to_sec()
            if topic == NS + '/debug/pose/pid':
                p = d['pid']
                p['t'].append(msg.header.stamp.to_sec())
                p['roll_tgt'].append(msg.roll.target_p)
                p['roll_err'].append(msg.roll.err_p)
                p['pitch_tgt'].append(msg.pitch.target_p)
                p['pitch_err'].append(msg.pitch.err_p)
                p['yaw_tgt'].append(msg.yaw.target_p)
                p['yaw_err'].append(msg.yaw.err_p)
            elif topic == NS + '/uav/cog/odom':
                o = d['odom']
                o['t'].append(msg.header.stamp.to_sec())
                pos = msg.pose.pose.position
                o['x'].append(pos.x)
                o['y'].append(pos.y)
                o['z'].append(pos.z)
                q = msg.pose.pose.orientation
                r, p_, y_ = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                o['roll'].append(r)
                o['pitch'].append(p_)
                o['yaw'].append(y_)
                tw = msg.twist.twist.linear
                o['vx'].append(tw.x)
                o['vy'].append(tw.y)
                o['vz'].append(tw.z)
            elif topic == NS + '/underwater/target_depth':
                d['tgt_depth']['t'].append(ts)
                d['tgt_depth']['v'].append(msg.data)
            elif topic == NS + '/pressure':
                d['pressure']['t'].append(msg.header.stamp.to_sec())
                d['pressure']['depth'].append(-(msg.fluid_pressure - 101.325) / 9.80638)
            elif topic == NS + '/underwater/cmd_vel':
                c = d['cmd']
                c['t'].append(ts)
                c['x'].append(msg.linear.x)
                c['y'].append(msg.linear.y)
                c['z'].append(msg.linear.z)
                c['yaw'].append(msg.angular.z)
            elif topic == NS + '/four_axes/command':
                d['thrust']['t'].append(ts)
                d['thrust']['f'].append(list(msg.base_thrust))
    for group in d.values():
        for k in group:
            group[k] = np.array(group[k])
    return d


def shade_segments(ax, segs, t0, label_y=None):
    for s in segs:
        a, b = s['t_cmd0'] - t0, s['t1'] - t0
        c = s['t_cmd1'] - t0
        if s['cmd']:
            ax.axvspan(a, c, color=BAND, zorder=0)
        if label_y is not None:
            ax.text((a + b) / 2, label_y, s['name'].replace('_', '\n'),
                    ha='center', va='top', fontsize=8.5, color=INK2,
                    transform=ax.get_xaxis_transform())


def unwrap_about(actual, target):
    """unwrap yaw so plotted actual stays near target across +-pi jumps"""
    out = np.array(actual, dtype=float)
    for i in range(len(out)):
        while out[i] - target[i] > math.pi:
            out[i] -= 2 * math.pi
        while out[i] - target[i] < -math.pi:
            out[i] += 2 * math.pi
    return out


def plot_attitude(d, segs, t0, out):
    p = d['pid']
    t = p['t'] - t0
    fig, axes = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True)
    fig.subplots_adjust(hspace=0.32)

    tgt_yaw = np.unwrap(p['yaw_tgt'])
    rows = [
        ('roll [deg]', np.degrees(p['roll_tgt']), np.degrees(p['roll_tgt'] - p['roll_err'])),
        ('pitch [deg]', np.degrees(p['pitch_tgt']), np.degrees(p['pitch_tgt'] - p['pitch_err'])),
        ('yaw [deg]', np.degrees(tgt_yaw), np.degrees(tgt_yaw - p['yaw_err'])),
    ]
    for ax, (name, tgt, act) in zip(axes, rows):
        ax.plot(t, act, color=C_ACTUAL, lw=1.8, label='actual')
        ax.plot(t, tgt, color=C_TARGET, lw=1.6, ls='--', label='desired')
        ax.set_ylabel(name)
        shade_segments(ax, segs, t0, label_y=1.13 if ax is axes[0] else None)
    axes[0].legend(loc='upper left', ncol=2)
    axes[0].set_title('Attitude tracking: desired vs actual', pad=34, color=INK, loc='left')
    axes[-1].set_xlabel('time [s]')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)


def plot_depth(d, segs, t0, out):
    fig, axes = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True,
                             gridspec_kw={'height_ratios': [2.2, 1]})
    fig.subplots_adjust(hspace=0.25)
    ax = axes[0]
    ax.plot(d['pressure']['t'] - t0, d['pressure']['depth'], color=C_ACTUAL, lw=1.8,
            label='measured (pressure)')
    ax.plot(d['odom']['t'] - t0, d['odom']['z'], color=INK2, lw=1.0, alpha=0.7,
            label='ground truth z')
    ax.plot(d['tgt_depth']['t'] - t0, d['tgt_depth']['v'], color=C_TARGET, lw=1.6, ls='--',
            label='target depth')
    ax.set_ylabel('depth z [m]')
    ax.legend(loc='upper right', ncol=3)
    ax.set_title('Depth control: target vs measured', pad=40, color=INK, loc='left')
    shade_segments(ax, segs, t0, label_y=1.09)

    # error on the common (pressure) timeline
    tgt_i = np.interp(d['pressure']['t'], d['tgt_depth']['t'], d['tgt_depth']['v'])
    err = d['pressure']['depth'] - tgt_i
    axes[1].plot(d['pressure']['t'] - t0, err, color=C_ACTUAL, lw=1.5)
    axes[1].axhline(0, color=INK2, lw=0.8)
    axes[1].set_ylabel('depth error [m]')
    axes[1].set_xlabel('time [s]')
    shade_segments(axes[1], segs, t0)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)


def plot_xy(d, segs, t0, out):
    o = d['odom']
    t = o['t'] - t0
    # body-frame horizontal velocity (surge u / sway v) from world velocity + yaw
    u = o['vx'] * np.cos(o['yaw']) + o['vy'] * np.sin(o['yaw'])
    v = -o['vx'] * np.sin(o['yaw']) + o['vy'] * np.cos(o['yaw'])

    fig, axes = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True)
    fig.subplots_adjust(hspace=0.32)
    axes[0].plot(t, o['x'], color=C_ACTUAL, lw=1.8)
    axes[0].set_ylabel('x [m]')
    axes[0].set_title('Horizontal motion under open-loop commands', pad=34, color=INK, loc='left')
    shade_segments(axes[0], segs, t0, label_y=1.13)
    axes[1].plot(t, o['y'], color=C_ACTUAL, lw=1.8)
    axes[1].set_ylabel('y [m]')
    shade_segments(axes[1], segs, t0)
    axes[2].plot(t, u, color=C_ACTUAL, lw=1.5, label='surge u (body x)')
    axes[2].plot(t, v, color=C_AUX, lw=1.5, label='sway v (body y)')
    axes[2].axhline(0, color=INK2, lw=0.8)
    axes[2].set_ylabel('body velocity [m/s]')
    axes[2].set_xlabel('time [s]')
    axes[2].legend(loc='upper left', ncol=2)
    shade_segments(axes[2], segs, t0)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)


def summarize(d, segs):
    o, p = d['odom'], d['pid']
    th = d['thrust']
    out = []
    for s in segs:
        m_o = (o['t'] >= s['t_cmd0']) & (o['t'] <= s['t1'])
        m_p = (p['t'] >= s['t_cmd0']) & (p['t'] <= s['t1'])
        if not m_o.any() or not m_p.any():
            continue
        item = {'name': s['name'], 'cmd': s['cmd'],
                'duration': round(s['t1'] - s['t_cmd0'], 1)}
        item['disp'] = [round(o[k][m_o][-1] - o[k][m_o][0], 3) for k in ('x', 'y', 'z')]
        dyaw = o['yaw'][m_o][-1] - o['yaw'][m_o][0]
        item['dyaw_deg'] = round(math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw))), 1)
        for ax_ in ('roll', 'pitch', 'yaw'):
            item[ax_ + '_rmse_deg'] = round(math.degrees(
                float(np.sqrt(np.mean(p[ax_ + '_err'][m_p] ** 2)))), 2)
            item[ax_ + '_max_tgt_deg'] = round(math.degrees(
                float(p[ax_ + '_tgt'][m_p][np.argmax(np.abs(p[ax_ + '_tgt'][m_p]))])), 1)
        item['mean_vx'] = round(float(np.mean(o['vx'][m_o])), 3)
        item['mean_vy'] = round(float(np.mean(o['vy'][m_o])), 3)
        if len(th['t']):
            m_t = (th['t'] >= s['t_cmd0']) & (th['t'] <= s['t1'])
            if m_t.any():
                f = th['f'][m_t]
                item['mean_aquatic_thrust'] = [round(float(np.mean(f[:, i])), 3)
                                               for i in range(4, min(8, f.shape[1]))]
        out.append(item)
    return out


def main():
    run_dir = sys.argv[1]
    bag_path = os.path.join(run_dir, 'demo.bag')
    with open(os.path.join(run_dir, 'segments.json')) as f:
        segs = json.load(f)
    d = read_bag(bag_path)
    t0 = segs[0]['t_cmd0'] - 3.0  # small pre-roll before the first segment

    plot_attitude(d, segs, t0, os.path.join(run_dir, 'attitude_tracking.png'))
    plot_depth(d, segs, t0, os.path.join(run_dir, 'depth_tracking.png'))
    plot_xy(d, segs, t0, os.path.join(run_dir, 'xy_motion.png'))

    summary = summarize(d, segs)
    with open(os.path.join(run_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print('plots written to', run_dir)


if __name__ == '__main__':
    main()
