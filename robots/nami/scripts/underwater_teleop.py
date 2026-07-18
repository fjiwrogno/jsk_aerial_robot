#!/usr/bin/env python3
"""Keyboard teleop for the nami underwater mode.

Publishes geometry_msgs/Twist on <robot_ns>/underwater/cmd_vel at a fixed rate:
  linear.x  : forward/backward stick [-1, 1] -> body-frame acc feedforward
  linear.y  : left/right stick      [-1, 1] -> body-frame acc feedforward
  linear.z  : surface/dive stick    [-1, 1] -> target depth increment rate
  angular.z : yaw stick             [-1, 1] -> target yaw increment rate

The command decays to zero shortly after the key is released (key-repeat keeps
it alive while held). State keys publish to the standard teleop topics.

Keys:
  w / s : forward / backward
  a / d : left / right
  r / f : surface (up) / dive (down)
  q / e : yaw left / yaw right
  1     : start  (arm motors)
  2     : takeoff (enter underwater hover: depth hold at current depth)
  l     : land
  0     : halt   (emergency stop)
  x     : zero all sticks
  Ctrl-C: quit
"""

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

PUB_RATE = 20.0        # [Hz]
HOLD_TIMEOUT = 0.4     # [s] stick decays to zero if the key is not repeated

AXIS_KEYS = {
    'w': ('linear.x', +5.0),
    's': ('linear.x', -5.0),
    'a': ('linear.y', +5.0),
    'd': ('linear.y', -5.0),
    'r': ('linear.z', +5.0),
    'f': ('linear.z', -5.0),
    'q': ('angular.z', +3.0),
    'e': ('angular.z', -3.0),
}


class UnderwaterTeleop(object):
    def __init__(self):
        robot_ns = rospy.get_param('~robot_ns', 'nami')
        self.cmd_pub = rospy.Publisher(robot_ns + '/underwater/cmd_vel', Twist, queue_size=1)
        self.start_pub = rospy.Publisher(robot_ns + '/teleop_command/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(robot_ns + '/teleop_command/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher(robot_ns + '/teleop_command/land', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher(robot_ns + '/teleop_command/halt', Empty, queue_size=1)

        # axis -> (value, last key stamp)
        self.axes = {axis: [0.0, 0.0] for axis, _ in AXIS_KEYS.values()}

    def handle_key(self, key, now):
        if key in AXIS_KEYS:
            axis, value = AXIS_KEYS[key]
            self.axes[axis][0] = value
            self.axes[axis][1] = now
        elif key == 'x':
            for state in self.axes.values():
                state[0] = 0.0
        elif key == '1':
            self.start_pub.publish(Empty())
            rospy.loginfo('start (arm)')
        elif key == '2':
            self.takeoff_pub.publish(Empty())
            rospy.loginfo('takeoff -> underwater hover (depth hold)')
        elif key == 'l':
            self.land_pub.publish(Empty())
            rospy.loginfo('land')
        elif key == '0':
            self.halt_pub.publish(Empty())
            rospy.logwarn('halt!')

    def spin(self):
        rate = rospy.Rate(PUB_RATE)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        print(__doc__)
        try:
            while not rospy.is_shutdown():
                now = rospy.get_time()
                while select.select([sys.stdin], [], [], 0)[0]:
                    self.handle_key(sys.stdin.read(1), now)

                cmd = Twist()
                for axis, (value, stamp) in self.axes.items():
                    if now - stamp > HOLD_TIMEOUT:
                        self.axes[axis][0] = 0.0
                        value = 0.0
                    obj, field = axis.split('.')
                    setattr(getattr(cmd, obj), field, value)
                self.cmd_pub.publish(cmd)

                status = ' | '.join('%s %+.1f' % (a, v[0]) for a, v in sorted(self.axes.items()))
                sys.stdout.write('\r' + status + '   ')
                sys.stdout.flush()
                rate.sleep()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print('')


if __name__ == '__main__':
    rospy.init_node('underwater_teleop')
    UnderwaterTeleop().spin()
