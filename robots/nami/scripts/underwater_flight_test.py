#!/usr/bin/env python3
"""Quick underwater flight test for nami (sim).

Assumes the underwater simulation is already running:
  roslaunch nami bringup.launch sim:=true rm:=false underwater:=true headless:=true

Sequence (each step has a timeout -> FAIL, never hangs):
  1. arm + takeoff  -> depth hold engages at the current depth
  2. depth : command a dive, target depth tracks within tolerance
  3. pitch : forward stick tilts the body (sign check)
  4. roll  : lateral stick tilts the body (sign check)
  5. yaw   : yaw stick rotates the heading (direction check)

Prints PASS/FAIL per item and exits non-zero if any failed.
"""

import math
import sys

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8

ROBOT_NS = 'nami'

TAKEOFF_STATE = 3
HOVER_STATE = 5


class UnderwaterFlightTest(object):
    def __init__(self):
        self.odom = None
        self.state = None
        rospy.Subscriber(ROBOT_NS + '/uav/cog/odom', Odometry, self._odom_cb)
        rospy.Subscriber(ROBOT_NS + '/flight_state', UInt8, self._state_cb)
        self.cmd_pub = rospy.Publisher(ROBOT_NS + '/underwater/cmd_vel', Twist, queue_size=1)
        self.start_pub = rospy.Publisher(ROBOT_NS + '/teleop_command/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(ROBOT_NS + '/teleop_command/takeoff', Empty, queue_size=1)
        self.results = []

    def _odom_cb(self, msg):
        self.odom = msg

    def _state_cb(self, msg):
        self.state = msg.data

    # --- helpers -----------------------------------------------------------
    def z(self):
        return self.odom.pose.pose.position.z

    def rpy(self):
        q = self.odom.pose.pose.orientation
        return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def wait(self, cond, timeout, what):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            if cond():
                return True
            rospy.sleep(0.2)
        rospy.logerr('timeout waiting for %s', what)
        return False

    def repeat_until(self, pub, cond, timeout, what):
        """publish an Empty about once a second until cond() or timeout"""
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            pub.publish(Empty())
            t1 = rospy.get_time()
            while not rospy.is_shutdown() and rospy.get_time() - t1 < 1.0:
                if cond():
                    return True
                rospy.sleep(0.2)
        rospy.logerr('timeout waiting for %s', what)
        return False

    def send_cmd(self, duration, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """publish a constant stick command at 20 Hz for `duration`, then zero"""
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = x, y, z, yaw
        t0 = rospy.get_time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() - t0 < duration:
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.cmd_pub.publish(Twist())  # release sticks

    def record(self, name, ok, detail):
        tag = 'PASS' if ok else 'FAIL'
        self.results.append((name, ok))
        (rospy.loginfo if ok else rospy.logerr)('[%s] %s: %s', tag, name, detail)

    # --- test items --------------------------------------------------------
    def run(self):
        if not self.wait(lambda: self.odom is not None, 20, 'first odom'):
            self.record('sim alive', False, 'no odom')
            return

        # 1. arm + engage depth hold
        # (publish repeatedly: a single publish right after advertise can be lost
        #  before the subscriber connection is established)
        armed = self.repeat_until(self.start_pub, lambda: self.state == 2, 15, 'ARM_ON')
        if armed:
            rospy.sleep(1.0)
            armed = self.repeat_until(self.takeoff_pub,
                                      lambda: self.state in (TAKEOFF_STATE, HOVER_STATE), 10, 'flight state')
        self.record('arm+engage', armed, 'flight state=%s' % self.state)
        if not armed:
            return
        rospy.sleep(3.0)

        # 2. depth: hold, then dive ~0.5 m and re-hold
        z_hold = self.z()
        rospy.sleep(4.0)
        hold_err = abs(self.z() - z_hold)
        self.record('depth hold', hold_err < 0.15, 'drift %.3f m in 4 s' % hold_err)

        z_before = self.z()
        self.send_cmd(2.0, z=-0.8)          # dive stick: target depth -= 0.8*max_dive_rate*2s
        rospy.sleep(6.0)
        dz = self.z() - z_before             # should be clearly negative
        self.record('depth dive', dz < -0.2, 'dz=%.3f m (expect < -0.2)' % dz)

        # 3. pitch: forward stick -> positive target pitch -> nose-down x motion
        pitch0 = self.rpy()[1]
        self.send_cmd(2.0, x=1.0)
        pitch_move = self.max_delta(lambda: self.rpy()[1], pitch0, 3.0)
        self.record('pitch response', pitch_move > 0.03,
                    'pitch moved %+.3f rad on forward cmd (expect > +0.03)' % pitch_move)
        rospy.sleep(2.0)

        # 4. roll: left stick -> negative target roll (y+ acc -> roll<0)
        roll0 = self.rpy()[0]
        self.send_cmd(2.0, y=1.0)
        roll_move = self.min_delta(lambda: self.rpy()[0], roll0, 3.0)
        self.record('roll response', roll_move < -0.03,
                    'roll moved %+.3f rad on left cmd (expect < -0.03)' % roll_move)
        rospy.sleep(2.0)

        # 5. yaw: positive yaw stick -> heading increases
        yaw0 = self.rpy()[2]
        self.send_cmd(3.0, yaw=1.0)
        rospy.sleep(2.0)
        dyaw = self.ang_diff(self.rpy()[2], yaw0)
        self.record('yaw response', dyaw > 0.05, 'dyaw=%+.3f rad (expect > +0.05)' % dyaw)

        # final sanity: still alive & holding depth
        alive = self.odom is not None and rospy.get_time() - self.odom.header.stamp.to_sec() < 2.0
        self.record('sim alive at end', alive, 'last odom age ok' if alive else 'odom stalled (crash?)')

    def max_delta(self, getter, ref, duration):
        best, t0 = 0.0, rospy.get_time()
        while rospy.get_time() - t0 < duration:
            best = max(best, getter() - ref)
            rospy.sleep(0.1)
        return best

    def min_delta(self, getter, ref, duration):
        best, t0 = 0.0, rospy.get_time()
        while rospy.get_time() - t0 < duration:
            best = min(best, getter() - ref)
            rospy.sleep(0.1)
        return best

    @staticmethod
    def ang_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def summary(self):
        print('\n========== underwater flight test summary ==========')
        for name, ok in self.results:
            print('  %-18s %s' % (name, 'PASS' if ok else 'FAIL'))
        failed = [n for n, ok in self.results if not ok]
        print('=====================================================')
        return 0 if not failed else 1


if __name__ == '__main__':
    rospy.init_node('underwater_flight_test')
    test = UnderwaterFlightTest()
    test.run()
    sys.exit(test.summary())
