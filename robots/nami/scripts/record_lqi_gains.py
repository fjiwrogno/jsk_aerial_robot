#!/usr/bin/env python3
"""Record the LQI feedback gains published on debug/four_axes/gain into a
standalone yaml, as a reference for tuning the PID gains in NamiControl.yaml.

Usage:
  1. roslaunch nami bringup.launch real_machine:=false simulation:=true headless:=true lqi:=true
  2. rosrun nami record_lqi_gains.py [_robot_ns:=nami] [_output:=/path/to/file.yaml]

The recorded yaml contains:
  - the raw per-rotor LQI gains (rotor thrust [N] per unit state error)
  - aggregated per-axis values for comparison with the PID config
  - the PID gains currently on the rosparam server (if any) for side-by-side reference
"""

import datetime
import os

import rospy
import rospkg
from aerial_robot_msgs.msg import FourAxisGain


def fmt_list(values):
    return '[' + ', '.join('{:.4f}'.format(v) for v in values) + ']'


def axis_block(name, p, i, d, indent='  '):
    lines = ['{}{}:'.format(indent, name)]
    lines.append('{}  p: {}'.format(indent, fmt_list(p)))
    lines.append('{}  i: {}'.format(indent, fmt_list(i)))
    lines.append('{}  d: {}'.format(indent, fmt_list(d)))
    return lines


def mean_abs(values):
    return sum(abs(v) for v in values) / len(values) if values else 0.0


def main():
    rospy.init_node('record_lqi_gains')

    robot_ns = rospy.get_param('~robot_ns', 'nami')
    timeout = rospy.get_param('~timeout', 30.0)
    topic = '/{}/debug/four_axes/gain'.format(robot_ns)

    default_output = os.path.join(rospkg.RosPack().get_path('nami'), 'config', 'quad',
                                  'LqiGainsRecord_{}.yaml'.format(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
    output = rospy.get_param('~output', default_output)

    rospy.loginfo('waiting for LQI gains on %s (timeout: %.0fs) ...', topic, timeout)
    try:
        msg = rospy.wait_for_message(topic, FourAxisGain, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr('no LQI gain message received on %s. Is the LQI controller running? '
                     '(launch with lqi:=true, and controller/lqi/realtime_update: true)', topic)
        return

    rotor_num = len(msg.z_p_gain)
    rospy.loginfo('received LQI gains for %d rotors', rotor_num)

    lines = []
    lines.append('# LQI gains recorded from {}'.format(topic))
    lines.append('# date: {}'.format(datetime.datetime.now().isoformat()))
    lines.append('# rotor num: {}'.format(rotor_num))
    lines.append('')
    lines.append('# Raw per-rotor LQI feedback gains: rotor thrust [N] per unit state error.')
    lines.append('lqi_raw_gains:')
    lines += axis_block('roll', msg.roll_p_gain, msg.roll_i_gain, msg.roll_d_gain)
    lines += axis_block('pitch', msg.pitch_p_gain, msg.pitch_i_gain, msg.pitch_d_gain)
    lines += axis_block('yaw', msg.yaw_p_gain, msg.yaw_i_gain, msg.yaw_d_gain)
    lines += axis_block('z', msg.z_p_gain, msg.z_i_gain, msg.z_d_gain)
    lines.append('')
    lines.append('# Aggregated per-axis values:')
    lines.append('#   z          : sum over rotors -> total-thrust level, directly comparable to controller/z')
    lines.append('#   roll/pitch/yaw : mean of per-rotor absolute gains -> order-of-magnitude reference only')
    lines.append('#                    (torque-level gains additionally depend on the rotor moment arms)')
    lines.append('lqi_aggregated:')
    lines.append('  z:')
    lines.append('    p_gain: {:.4f}'.format(sum(msg.z_p_gain)))
    lines.append('    i_gain: {:.4f}'.format(sum(msg.z_i_gain)))
    lines.append('    d_gain: {:.4f}'.format(sum(msg.z_d_gain)))
    for axis, p, i, d in (('roll', msg.roll_p_gain, msg.roll_i_gain, msg.roll_d_gain),
                          ('pitch', msg.pitch_p_gain, msg.pitch_i_gain, msg.pitch_d_gain),
                          ('yaw', msg.yaw_p_gain, msg.yaw_i_gain, msg.yaw_d_gain)):
        lines.append('  {}:'.format(axis))
        lines.append('    p_gain: {:.4f}'.format(mean_abs(p)))
        lines.append('    i_gain: {:.4f}'.format(mean_abs(i)))
        lines.append('    d_gain: {:.4f}'.format(mean_abs(d)))
    lines.append('')

    # snapshot of the PID gains currently loaded on the rosparam server
    lines.append('# PID gains on the rosparam server at record time. NOTE: when recording from the')
    lines.append('# LQI session, z/yaw values are unused defaults; compare against NamiControl.yaml instead.')
    lines.append('current_pid:')
    found_pid = False
    for axis in ('xy', 'z', 'roll_pitch', 'yaw'):
        gains = {}
        for term in ('p_gain', 'i_gain', 'd_gain'):
            param = '/{}/controller/{}/{}'.format(robot_ns, axis, term)
            if rospy.has_param(param):
                gains[term] = rospy.get_param(param)
        if gains:
            found_pid = True
            lines.append('  {}:'.format(axis))
            for term in ('p_gain', 'i_gain', 'd_gain'):
                if term in gains:
                    lines.append('    {}: {}'.format(term, gains[term]))
    if not found_pid:
        lines.append('  {}  # none found (the LQI config does not define static z/yaw pid gains)')

    with open(output, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    rospy.loginfo('LQI gains recorded to: %s', output)


if __name__ == '__main__':
    main()
