#!/usr/bin/env python3
"""Convert recorded per-rotor LQI gains (thrust [N] per unit error, from
record_lqi_gains.py) into acceleration-level PID gains for nami_controller.

nami_controller (PoseLinearController) semantics:
  - Z PID output: linear acceleration [m/s^2]
        gain_acc = sum_i(k_i) / mass
  - ROLL/PITCH/YAW PID output: angular acceleration [rad/s^2]
        torque_gain = sum_i( ((r_i x u_i) + sigma_i * m_f_rate * u_i)[axis] * k_i )
        gain_angacc = torque_gain / I[axis]
Mass, inertia about CoG and rotor poses are computed from the xacro-expanded
URDF with all joints at zero (hover configuration for the fixed-rotor nami).

Usage:
  rosrun nami lqi_gains_to_pid.py [record_yaml] [urdf_xacro]
    record_yaml : defaults to the newest config/quad/LqiGainsRecord_*.yaml
    urdf_xacro  : defaults to robots/quad/nami.urdf.xacro
"""
import glob
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rospkg

M_F_RATE = -0.0068  # mz = m_f_rate * fz (common.xacro / MotorInfoDShot.yaml)
ROTOR_NUM = 4


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_to_tf(elem):
    T = np.eye(4)
    if elem is not None:
        xyz = [float(v) for v in elem.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in elem.get('rpy', '0 0 0').split()]
        T[:3, :3] = rpy_to_mat(*rpy)
        T[:3, 3] = xyz
    return T


def parse_record(path):
    """minimal parser for the lqi_raw_gains block of the record yaml"""
    gains, axis, inside = {}, None, False
    for line in open(path):
        s = line.split('#')[0].rstrip()
        if not s:
            continue
        if s == 'lqi_raw_gains:':
            inside = True
            continue
        if inside:
            if not s.startswith('  '):
                break
            if s.endswith(':') and not s.startswith('    '):
                axis = s.strip()[:-1]
                gains[axis] = {}
            elif ':' in s and axis:
                term, arr = s.split(':', 1)
                gains[axis][term.strip()] = [float(v) for v in arr.strip().strip('[]').split(',')]
    return gains


def main():
    pkg = rospkg.RosPack().get_path('nami')
    if len(sys.argv) > 1:
        record = sys.argv[1]
    else:
        records = sorted(glob.glob(os.path.join(pkg, 'config', 'quad', 'LqiGainsRecord_*.yaml')))
        if not records:
            sys.exit('no LqiGainsRecord_*.yaml found; run record_lqi_gains.py first')
        record = records[-1]
    urdf_xacro = sys.argv[2] if len(sys.argv) > 2 else os.path.join(pkg, 'robots', 'quad', 'nami.urdf.xacro')
    print('record: {}'.format(record))
    print('urdf  : {}'.format(urdf_xacro))

    root = ET.fromstring(subprocess.check_output(['xacro', urdf_xacro]).decode())

    # forward kinematics with all joints at zero
    joints = {}  # child link -> (parent link, static tf, joint axis)
    for j in root.findall('joint'):
        axis_elem = j.find('axis')
        axis = [float(v) for v in axis_elem.get('xyz').split()] if axis_elem is not None else [0, 0, 1]
        joints[j.find('child').get('link')] = (j.find('parent').get('link'),
                                               origin_to_tf(j.find('origin')), np.array(axis))
    link_tf = {}

    def fk(link):
        if link not in link_tf:
            if link not in joints:
                link_tf[link] = np.eye(4)
            else:
                parent, T_j, _ = joints[link]
                link_tf[link] = fk(parent) @ T_j
        return link_tf[link]

    # mass, CoG, inertia about CoG
    total_mass, weighted_pos, inertials = 0.0, np.zeros(3), []
    for l in root.findall('link'):
        inertial = l.find('inertial')
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        if m == 0:
            continue
        T = fk(l.get('name')) @ origin_to_tf(inertial.find('origin'))
        ie = inertial.find('inertia')
        I = np.array([[float(ie.get('ixx')), float(ie.get('ixy', 0)), float(ie.get('ixz', 0))],
                      [float(ie.get('ixy', 0)), float(ie.get('iyy')), float(ie.get('iyz', 0))],
                      [float(ie.get('ixz', 0)), float(ie.get('iyz', 0)), float(ie.get('izz'))]])
        R = T[:3, :3]
        inertials.append((m, T[:3, 3], R @ I @ R.T))
        total_mass += m
        weighted_pos += m * T[:3, 3]

    cog = weighted_pos / total_mass
    I_cog = np.zeros((3, 3))
    for m, p, I in inertials:
        d = p - cog
        I_cog += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    print('mass: {:.3f} kg, inertia diag: Ixx={:.4f} Iyy={:.4f} Izz={:.4f}'.format(
        total_mass, *np.diag(I_cog)))

    rotors = []
    for i in range(1, ROTOR_NUM + 1):
        T = fk('thrust{}'.format(i))
        _, _, axis = joints['thrust{}'.format(i)]
        sigma = 1.0 if axis[2] > 0 else -1.0
        rotors.append((T[:3, 3] - cog, T[:3, :3] @ np.array([0, 0, 1.0]), sigma))

    gains = parse_record(record)

    print()
    print('=== acceleration-level PID gains for nami_controller ===')
    z = {t: sum(gains['z'][t]) / total_mass for t in ('p', 'i', 'd')}
    print('z    : p_gain {:.3f}  i_gain {:.3f}  d_gain {:.3f}   [m/s^2 per (m, m*s, m/s)]'.format(
        z['p'], z['i'], z['d']))
    # attitude gains go to spinal as int16 with x1000 scaling (setAttitudeGains /
    # RollPitchYawTerm.msg), so they must stay below 32.767 or they overflow
    SPINAL_GAIN_CAP = 30.0  # keep some margin below 32.767
    for ax_name, ax_idx in (('roll', 0), ('pitch', 1), ('yaw', 2)):
        res = {}
        for term in ('p', 'i', 'd'):
            tau = sum((np.cross(r, u) + sigma * M_F_RATE * u)[ax_idx] * k
                      for (r, u, sigma), k in zip(rotors, gains[ax_name][term]))
            res[term] = tau / I_cog[ax_idx, ax_idx]
        print('{:5s}: p_gain {:.3f}  i_gain {:.3f}  d_gain {:.3f}   [rad/s^2 per (rad, rad*s, rad/s)]'.format(
            ax_name, res['p'], res['i'], res['d']))
        worst = max(abs(v) for v in res.values())
        if worst > 32.767:
            s = SPINAL_GAIN_CAP / worst
            print('       ^ exceeds the spinal int16 gain cap (32.767)! scaled by {:.4f}:'.format(s))
            print('         p_gain {:.3f}  i_gain {:.3f}  d_gain {:.3f}'.format(
                res['p'] * s, res['i'] * s, res['d'] * s))
    print()
    print('(xy gains are not provided by LQI; keep the existing values)')


if __name__ == '__main__':
    main()
