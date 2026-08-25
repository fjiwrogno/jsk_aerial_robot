#!/usr/bin/env python3
"""Ramp nami's transformation joint to a target angle and hold it there.

The node only ever touches <robot_ns>/joints_ctrl.  It does not read or write the
flight state, the depth-hold target or the surge-allocation flag, so a shape
change can be commanded at any point of an underwater run without disturbing the
mode the vehicle is in - and equally, switching between depth-hold and surge does
not disturb the joint.

The robot model already reacts on its own: servo_bridge publishes the measured
angle on joint_states, RobotModel::updateRobotModel() re-runs the forward
kinematics every cycle, and the CoG, inertia and allocation matrix follow.  That
is why this is a script and not a controller.

Launching a second instance under the same node name replaces the running one,
so successive target angles hand over cleanly without a step in the command.
"""

import math

import rospy
from sensor_msgs.msg import JointState


def quintic(progress):
    """Smooth 0->1 blend with zero velocity and acceleration at both ends."""
    p = min(max(progress, 0.0), 1.0)
    return p * p * p * (10.0 - 15.0 * p + 6.0 * p * p)


class YawJointCommander(object):
    def __init__(self):
        self.joint_name = rospy.get_param('~joint_name', 'yaw_joint')
        self.target_deg = float(rospy.get_param('~target_angle_deg', 0.0))
        self.duration = float(rospy.get_param('~duration', 8.0))
        self.hold_duration = float(rospy.get_param('~hold_duration', -1.0))
        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        self.max_rate = float(rospy.get_param('~max_rate_deg_s', 15.0))
        self.min_deg = float(rospy.get_param('~min_angle_deg', -90.0))
        self.max_deg = float(rospy.get_param('~max_angle_deg', 90.0))
        self.settle_tolerance = math.radians(
            float(rospy.get_param('~tolerance_deg', 2.0)))
        wait_state = float(rospy.get_param('~wait_for_joint_state', 3.0))
        fallback_deg = float(rospy.get_param('~assume_start_deg', 0.0))

        if self.target_deg < self.min_deg or self.target_deg > self.max_deg:
            rospy.logerr('[yaw_joint] target %.1f deg is outside the allowed '
                         '[%.1f, %.1f] deg, refusing to move',
                         self.target_deg, self.min_deg, self.max_deg)
            raise rospy.ROSInitException('target angle out of range')

        self.measured = None
        rospy.Subscriber('joint_states', JointState, self._joint_state_cb)
        self.pub = rospy.Publisher('joints_ctrl', JointState, queue_size=1)

        # Start the ramp from where the joint actually is, so replacing a running
        # instance (or arriving after a manual move) never commands a step.
        deadline = rospy.Time.now() + rospy.Duration(max(wait_state, 0.0))
        while (self.measured is None and rospy.Time.now() < deadline
               and not rospy.is_shutdown()):
            rospy.sleep(0.05)
        if self.measured is None:
            self.start = math.radians(fallback_deg)
            rospy.logwarn('[yaw_joint] no "%s" on joint_states within %.1fs, '
                          'assuming it is at %.1f deg',
                          self.joint_name, wait_state, fallback_deg)
        else:
            self.start = self.measured

        self.target = math.radians(self.target_deg)
        travel_deg = abs(self.target_deg - math.degrees(self.start))
        if self.max_rate > 0.0:
            min_duration = travel_deg / self.max_rate
            if min_duration > self.duration:
                rospy.logwarn('[yaw_joint] %.1f deg in %.1fs exceeds the %.1f deg/s '
                              'slew limit, stretching to %.1fs',
                              travel_deg, self.duration, self.max_rate, min_duration)
                self.duration = min_duration
        self.duration = max(self.duration, 1.0 / self.rate_hz)
        self.command = self.start

    def _joint_state_cb(self, msg):
        try:
            self.measured = msg.position[msg.name.index(self.joint_name)]
        except (ValueError, IndexError):
            pass

    def _publish(self, angle):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [self.joint_name]
        msg.position = [angle]
        self.command = angle
        self.pub.publish(msg)

    def run(self):
        rospy.loginfo('[yaw_joint] %s: %.1f -> %.1f deg over %.1fs '
                      '(independent of depth-hold / surge mode)',
                      self.joint_name, math.degrees(self.start),
                      self.target_deg, self.duration)
        rate = rospy.Rate(self.rate_hz)
        begin = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - begin).to_sec()
            if elapsed >= self.duration:
                break
            self._publish(self.start + (self.target - self.start) * quintic(elapsed / self.duration))
            rate.sleep()

        if rospy.is_shutdown():
            return
        self._publish(self.target)
        err = ('n/a' if self.measured is None
               else '%.2f deg' % math.degrees(self.measured - self.target))
        rospy.loginfo('[yaw_joint] reached %.1f deg (tracking error %s), holding',
                      self.target_deg, err)

        hold_end = (None if self.hold_duration < 0.0
                    else rospy.Time.now() + rospy.Duration(self.hold_duration))
        warned = False
        while not rospy.is_shutdown():
            if hold_end is not None and rospy.Time.now() >= hold_end:
                break
            self._publish(self.target)
            if (not warned and self.measured is not None
                    and abs(self.measured - self.target) > self.settle_tolerance):
                rospy.logwarn('[yaw_joint] holding %.1f deg but measured %.1f deg - '
                              'servo may be stalling against water drag',
                              self.target_deg, math.degrees(self.measured))
                warned = True
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('yaw_joint_commander')
    try:
        YawJointCommander().run()
    except rospy.ROSInitException:
        pass
    except rospy.ROSInterruptException:
        pass
