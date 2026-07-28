#!/usr/bin/env python3

import csv
import math
import os
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from spinal.msg import Imu
from std_msgs.msg import Bool, Empty, UInt8


ARM_OFF_STATE = 0
ARM_ON_STATE = 2
HOVER_STATE = 5


def quintic_blend(progress):
    progress = min(max(progress, 0.0), 1.0)
    return 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5


def quaternion_to_rpy(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(min(max(sin_pitch, -1.0), 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class TransformationDemo:
    def __init__(self):
        self.robot_ns = rospy.get_param("~robot_ns", "/nami").rstrip("/")
        self.joint_name = rospy.get_param("~joint_name", "yaw_joint")
        self.target_angle = math.radians(rospy.get_param("~target_angle_deg", 30.0))
        self.certified_min_angle = math.radians(
            rospy.get_param("~certified_min_angle_deg", 0.0))
        self.certified_max_angle = math.radians(
            rospy.get_param("~certified_max_angle_deg", 30.0))
        self.transform_duration = rospy.get_param("~transform_duration", 8.0)
        self.preflight_duration = rospy.get_param("~preflight_duration", 2.0)
        self.settle_duration = rospy.get_param("~settle_duration", 5.0)
        self.hold_duration = rospy.get_param("~hold_duration", 10.0)
        self.return_to_zero = rospy.get_param("~return_to_zero", True)
        self.rate_hz = rospy.get_param("~command_rate", 40.0)
        self.joint_tolerance = rospy.get_param("~joint_tolerance", 0.05)
        self.controller_label = rospy.get_param("~controller_label", "unknown")
        csv_path = rospy.get_param(
            "~csv_path", "/tmp/nami_transformation_metrics.csv")

        if (self.transform_duration <= 0.0 or self.preflight_duration < 0.0 or
                self.rate_hz <= 0.0):
            raise ValueError(
                "transform_duration and command_rate must be positive, and "
                "preflight_duration must be non-negative")
        if self.certified_min_angle > self.certified_max_angle:
            raise ValueError("certified joint-angle range is invalid")
        if not (self.certified_min_angle <= self.target_angle <=
                self.certified_max_angle):
            raise ValueError(
                "target angle exceeds the offline-certified transformation range")

        csv_directory = os.path.dirname(csv_path)
        if csv_directory:
            os.makedirs(csv_directory, exist_ok=True)
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "ros_time", "phase", "controller", "joint_target", "joint_actual",
            "x", "y", "z", "roll", "pitch", "yaw"
        ])
        self.csv_sample_count = 0

        self.flight_state = None
        self.joint_position = None
        self.odom = None
        self.stability_valid = None
        self.imu_message_count = 0
        self.last_safe_position = None
        self.current_target = 0.0
        self.phase = "initializing"

        self.joint_pub = rospy.Publisher(
            self.robot_ns + "/joints_ctrl", JointState, queue_size=1)
        self.start_pub = rospy.Publisher(
            self.robot_ns + "/teleop_command/start", Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(
            self.robot_ns + "/teleop_command/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher(
            self.robot_ns + "/teleop_command/land", Empty, queue_size=1)

        rospy.Subscriber(
            self.robot_ns + "/flight_state", UInt8, self._flight_state_callback)
        rospy.Subscriber(
            self.robot_ns + "/joint_states", JointState, self._joint_state_callback)
        rospy.Subscriber(
            self.robot_ns + "/uav/cog/odom", Odometry, self._odom_callback)
        rospy.Subscriber(
            self.robot_ns + "/model/stability", Bool, self._stability_callback)
        rospy.Subscriber(
            self.robot_ns + "/imu", Imu, self._imu_callback)

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()

    def _flight_state_callback(self, msg):
        self.flight_state = msg.data

    def _joint_state_callback(self, msg):
        try:
            self.joint_position = msg.position[msg.name.index(self.joint_name)]
            if self.stability_valid:
                self.last_safe_position = self.joint_position
        except (ValueError, IndexError):
            return

    def _odom_callback(self, msg):
        self.odom = msg

    def _stability_callback(self, msg):
        self.stability_valid = msg.data
        if msg.data and self.joint_position is not None:
            self.last_safe_position = self.joint_position

    def _imu_callback(self, _msg):
        self.imu_message_count += 1

    def _check_flight_and_model(self):
        if self.flight_state != HOVER_STATE:
            raise RuntimeError("vehicle left hover state during transformation")
        if self.stability_valid is not True:
            raise RuntimeError("current configuration failed the stability check")

    def _wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rate.sleep()
        raise RuntimeError("timeout while waiting for " + description)

    def _command_until_state(self, publisher, target_state, timeout, description):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.flight_state == target_state:
                return
            publisher.publish(Empty())
            rate.sleep()
        raise RuntimeError("failed to " + description)

    def _publish_joint(self, position):
        self.current_target = position
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [self.joint_name]
        msg.position = [position]
        self.joint_pub.publish(msg)
        self._write_sample()

    def _write_sample(self):
        actual = float("nan") if self.joint_position is None else self.joint_position
        pose = [float("nan")] * 6
        if self.odom is not None:
            position = self.odom.pose.pose.position
            roll, pitch, yaw = quaternion_to_rpy(self.odom.pose.pose.orientation)
            pose = [position.x, position.y, position.z, roll, pitch, yaw]
        self.csv_writer.writerow([
            rospy.Time.now().to_sec(), self.phase, self.controller_label,
            self.current_target, actual
        ] + pose)
        self.csv_sample_count += 1
        if self.csv_sample_count % int(max(self.rate_hz, 1.0)) == 0:
            self.csv_file.flush()

    def _hold(self, position, duration):
        start = rospy.Time.now()
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
            self._check_flight_and_model()
            self._publish_joint(position)
            rate.sleep()

    def _preflight_hold(self):
        start = rospy.Time.now()
        rate = rospy.Rate(self.rate_hz)
        while (not rospy.is_shutdown() and
               (rospy.Time.now() - start).to_sec() < self.preflight_duration):
            if self.flight_state != ARM_OFF_STATE:
                raise RuntimeError("vehicle left disarmed state during preflight")
            if self.stability_valid is not True:
                raise RuntimeError("configuration became invalid during preflight")
            self._publish_joint(self.joint_position)
            rate.sleep()

    def _move_joint(self, start_position, target_position):
        start = rospy.Time.now()
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            progress = elapsed / self.transform_duration
            if progress >= 1.0:
                break
            self._check_flight_and_model()
            command = start_position + (
                target_position - start_position) * quintic_blend(progress)
            self._publish_joint(command)
            rate.sleep()

        self._publish_joint(target_position)
        self._wait_for(
            lambda: self.joint_position is not None and
            abs(self.joint_position - target_position) <= self.joint_tolerance,
            3.0, "joint convergence")

    def run(self):
        self._wait_for(
            lambda: self.flight_state is not None and
            self.joint_position is not None and
            self.stability_valid is True and
            self.imu_message_count > 0,
            30.0, "flight, joint, IMU, and valid model stability state")

        if not (self.certified_min_angle <= self.joint_position <=
                self.certified_max_angle):
            raise RuntimeError(
                "initial joint angle is outside the offline-certified range")

        if self.flight_state == ARM_OFF_STATE:
            self.phase = "preflight"
            self._preflight_hold()
            self.phase = "arming"
            self._command_until_state(
                self.start_pub, ARM_ON_STATE, 10.0, "arm")
        if self.flight_state == ARM_ON_STATE:
            self.phase = "takeoff"
            self._command_until_state(
                self.takeoff_pub, HOVER_STATE, 45.0, "take off")
        if self.flight_state != HOVER_STATE:
            raise RuntimeError(
                "demo requires ARM_OFF, ARM_ON, or HOVER flight state")

        rospy.loginfo("Nami transformation demo: settling in hover")
        self.phase = "settle"
        self._hold(self.joint_position, self.settle_duration)

        start_position = self.joint_position
        rospy.loginfo(
            "Nami transformation demo: moving %s from %.3f to %.3f rad",
            self.joint_name, start_position, self.target_angle)
        self.phase = "transform"
        self._move_joint(start_position, self.target_angle)

        self.phase = "hold"
        self._hold(self.target_angle, self.hold_duration)

        if self.return_to_zero:
            self.phase = "return"
            self._move_joint(self.target_angle, 0.0)
            self.phase = "return_settle"
            self._hold(0.0, 2.0)

        self.phase = "landing"
        self._command_until_state(
            self.land_pub, ARM_OFF_STATE, 30.0, "land")
        self.phase = "complete"
        self._write_sample()
        rospy.loginfo("Nami transformation demo completed")

    def restore_last_safe_configuration(self):
        if self.last_safe_position is None:
            return
        self.phase = "abort_restore"
        deadline = time.monotonic() + 2.0
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish_joint(self.last_safe_position)
            rate.sleep()


def main():
    rospy.init_node("nami_transformation_demo")
    demo = None
    try:
        demo = TransformationDemo()
        demo.run()
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        rospy.logerr("Nami transformation demo aborted: %s", error)
        if demo is not None and demo.flight_state == HOVER_STATE:
            demo.restore_last_safe_configuration()
            try:
                demo.phase = "abort_landing"
                demo._command_until_state(
                    demo.land_pub, ARM_OFF_STATE, 30.0, "land after abort")
            except RuntimeError as landing_error:
                rospy.logerr(
                    "Nami transformation demo could not confirm landing: %s",
                    landing_error)
        return 1
    finally:
        if demo is not None:
            demo.close()


if __name__ == "__main__":
    sys.exit(main())
