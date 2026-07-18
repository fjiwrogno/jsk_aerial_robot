#!/usr/bin/env python3
"""Underwater demo driver for nami (sim).

Runs a labeled command sequence (depth hold / dive / forward / backward /
left / right / yaw), records everything needed for the result plots into a
rosbag, and writes one mp4 per segment from the demo camera (if present).

Assumes the underwater simulation is already running:
  roslaunch nami bringup.launch sim:=true rm:=false underwater:=true headless:=true
Optionally with the demo camera spawned (see demo_cam.sdf).

Usage:
  rosrun nami underwater_demo.py [_out_dir:=/path/to/output]

Outputs in out_dir:
  demo.bag        rosbag with attitude / depth / command topics
  segments.json   per-segment [t0, t1] in ROS (sim) time + commands
  NN_<name>.mp4   per-segment video from /demo_cam/image_raw
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rospy
import tf.transformations as tft
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, UInt8

try:
    import cv2
except ImportError:
    cv2 = None

# chase-cam offset from the robot in world frame (looks back along the offset)
CAM_OFFSET = (4.2, -4.2, 0.3)

ROBOT_NS = 'nami'

ARM_ON_STATE = 2
TAKEOFF_STATE = 3
HOVER_STATE = 5

BAG_TOPICS = [
    '/{ns}/uav/cog/odom',
    '/{ns}/debug/pose/pid',
    '/{ns}/four_axes/command',
    '/{ns}/underwater/cmd_vel',
    '/{ns}/underwater/target_depth',
    '/{ns}/pressure',
    '/{ns}/flight_state',
    '/{ns}/ground_truth/pose',
    '/hydrodynamics/current_velocity',
]

# (name, {axis: value}, cmd_duration [s], settle_duration [s])
# stick magnitudes match underwater_teleop.py key presses
# dive first: near the free surface (z~0) the uuv hydrodynamics are inaccurate,
# so run every segment at the -1 m working depth
SEGMENTS = [
    ('dive',     {'z': -1.0},     3.0,  7.0),
    ('hold',     {},              0.0, 20.0),
    ('forward',  {'x': +5.0},     5.0,  5.0),
    ('backward', {'x': -5.0},     5.0,  5.0),
    ('left',     {'y': +5.0},     5.0,  5.0),
    ('right',    {'y': -5.0},     5.0,  5.0),
    ('yaw_left', {'yaw': +1.0},   5.0,  5.0),
    ('yaw_right', {'yaw': -1.0},  5.0,  5.0),
]


class VideoRecorder(object):
    """Writes /demo_cam/image_raw frames into one mp4 per segment."""

    def __init__(self, out_dir, info_fn=None):
        self.out_dir = out_dir
        self.info_fn = info_fn  # extra overlay line (e.g. position/velocity)
        self.lock = threading.Lock()
        self.writer = None
        self.label = ''
        self.frames = 0
        self.seen_any_image = False
        rospy.Subscriber('/demo_cam/image_raw', Image, self._image_cb,
                         queue_size=1, buff_size=1280 * 720 * 3 * 2)

    def _image_cb(self, msg):
        self.seen_any_image = True
        if cv2 is None:
            return
        with self.lock:
            if self.writer is None:
                return
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.putText(frame, self.label, (20, 45), cv2.FONT_HERSHEY_SIMPLEX,
                        1.2, (255, 255, 255), 2, cv2.LINE_AA)
            info = 't = %.1f s' % (rospy.get_time() - self.t0)
            if self.info_fn is not None:
                info += '   ' + self.info_fn()
            cv2.putText(frame, info, (20, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
            self.writer.write(frame)
            self.frames += 1

    def start(self, index, name, label):
        if cv2 is None:
            return
        path = os.path.join(self.out_dir, '%02d_%s.mp4' % (index, name))
        with self.lock:
            self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'),
                                          20.0, (1280, 720))
            self.label = label
            self.frames = 0
            self.t0 = rospy.get_time()

    def stop(self):
        if cv2 is None:
            return
        with self.lock:
            if self.writer is not None:
                self.writer.release()
                rospy.loginfo('video segment closed (%d frames)', self.frames)
            self.writer = None


class UnderwaterDemo(object):
    def __init__(self, out_dir):
        self.out_dir = out_dir
        self.odom = None
        self.state = None
        rospy.Subscriber(ROBOT_NS + '/uav/cog/odom', Odometry, self._odom_cb)
        rospy.Subscriber(ROBOT_NS + '/flight_state', UInt8, self._state_cb)
        self.cmd_pub = rospy.Publisher(ROBOT_NS + '/underwater/cmd_vel', Twist, queue_size=1)
        self.start_pub = rospy.Publisher(ROBOT_NS + '/teleop_command/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(ROBOT_NS + '/teleop_command/takeoff', Empty, queue_size=1)
        self.video = VideoRecorder(out_dir, info_fn=self._overlay_info)
        self.segments_log = []
        self.bag_proc = None

        # smooth chase cam: keep the (static) demo_cam model at a fixed offset
        # from the robot so it stays in frame over the whole drifting run
        self.cam_pos = None
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
            self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            rospy.Timer(rospy.Duration(0.1), self._cam_follow_cb)
        except rospy.ROSException:
            self.set_model_state = None
            rospy.logwarn('gazebo set_model_state unavailable: chase cam disabled')

    def _cam_follow_cb(self, _):
        if self.odom is None or self.set_model_state is None:
            return
        p = self.odom.pose.pose.position
        want = np.array([p.x + CAM_OFFSET[0], p.y + CAM_OFFSET[1], p.z + CAM_OFFSET[2]])
        if self.cam_pos is None:
            self.cam_pos = want
        self.cam_pos = self.cam_pos + 0.15 * (want - self.cam_pos)  # ~1.5 s lag, smooth
        dx, dy, dz = np.array([p.x, p.y, p.z]) - self.cam_pos
        yaw = math.atan2(dy, dx)
        pitch = -math.atan2(dz, math.hypot(dx, dy))
        q = tft.quaternion_from_euler(0, pitch, yaw)
        st = ModelState()
        st.model_name = 'demo_cam'
        st.pose.position.x, st.pose.position.y, st.pose.position.z = self.cam_pos
        st.pose.orientation.x, st.pose.orientation.y, st.pose.orientation.z, st.pose.orientation.w = q
        try:
            self.set_model_state(st)
        except rospy.ServiceException:
            pass

    def _odom_cb(self, msg):
        self.odom = msg

    def _state_cb(self, msg):
        self.state = msg.data

    def pos(self):
        p = self.odom.pose.pose.position
        return [p.x, p.y, p.z]

    def _overlay_info(self):
        if self.odom is None:
            return ''
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        return 'x=%+.1f y=%+.1f z=%+.2f m   vx=%+.2f vy=%+.2f m/s' % (p.x, p.y, p.z, v.x, v.y)

    # --- infrastructure ----------------------------------------------------
    def start_bag(self):
        topics = [t.format(ns=ROBOT_NS) for t in BAG_TOPICS]
        bag_path = os.path.join(self.out_dir, 'demo.bag')
        self.bag_proc = subprocess.Popen(
            ['rosbag', 'record', '--lz4', '-O', bag_path] + topics)
        rospy.loginfo('rosbag record started -> %s', bag_path)
        time.sleep(2.0)  # let subscriptions establish

    def stop_bag(self):
        if self.bag_proc is not None:
            self.bag_proc.send_signal(signal.SIGINT)
            self.bag_proc.wait()
            self.bag_proc = None
            rospy.loginfo('rosbag record stopped')

    def repeat_until(self, pub, cond, timeout, what):
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

    def publish_cmd(self, duration, x=0.0, y=0.0, z=0.0, yaw=0.0):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = x, y, z, yaw
        t0 = rospy.get_time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() - t0 < duration:
            self.cmd_pub.publish(cmd)
            rate.sleep()

    # --- sequence ----------------------------------------------------------
    def arm_and_engage(self):
        if not self.repeat_until(self.start_pub, lambda: self.state == ARM_ON_STATE, 15, 'ARM_ON'):
            return False
        rospy.sleep(1.0)
        if not self.repeat_until(self.takeoff_pub,
                                 lambda: self.state in (TAKEOFF_STATE, HOVER_STATE), 10, 'flight state'):
            return False
        rospy.sleep(3.0)  # depth hold settles
        return True

    def run(self):
        rospy.loginfo('waiting for first odom ...')
        t0 = rospy.get_time()
        while self.odom is None and not rospy.is_shutdown():
            if rospy.get_time() - t0 > 30:
                rospy.logerr('no odom, is the simulation running?')
                return 1
            rospy.sleep(0.5)

        self.start_bag()
        try:
            if not self.arm_and_engage():
                rospy.logerr('arm/engage failed (flight_state=%s)', self.state)
                return 1
            rospy.loginfo('depth hold engaged, starting segments')

            for i, (name, cmd, cmd_dur, settle_dur) in enumerate(SEGMENTS):
                label = 'nami underwater: %s  %s' % (
                    name, ' '.join('%s%+.1f' % (k, v) for k, v in cmd.items()))
                self.video.start(i, name, label)
                seg = {'name': name, 'cmd': cmd, 't_cmd0': rospy.get_time(),
                       'pos0': self.pos()}
                rospy.loginfo('--- segment %d: %s ---', i, name)
                if cmd_dur > 0:
                    self.publish_cmd(cmd_dur, **cmd)
                seg['t_cmd1'] = rospy.get_time()
                self.publish_cmd(settle_dur)  # released sticks (zero cmd)
                seg['t1'] = rospy.get_time()
                seg['pos1'] = self.pos()
                self.video.stop()
                self.segments_log.append(seg)
                d = np.array(seg['pos1']) - np.array(seg['pos0'])
                rospy.loginfo('    displacement dx=%+.2f dy=%+.2f dz=%+.2f m', *d)

            with open(os.path.join(self.out_dir, 'segments.json'), 'w') as f:
                json.dump(self.segments_log, f, indent=2)
            rospy.loginfo('segments.json written')
            if not self.video.seen_any_image:
                rospy.logwarn('no camera image received: videos are empty '
                              '(is demo_cam spawned?)')
            return 0
        finally:
            self.video.stop()
            self.stop_bag()


if __name__ == '__main__':
    rospy.init_node('underwater_demo')
    out_dir = rospy.get_param('~out_dir',
                              os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                           '..', 'demo_output',
                                           time.strftime('%Y%m%d_%H%M%S')))
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    rospy.loginfo('demo output dir: %s', out_dir)
    demo = UnderwaterDemo(out_dir)
    sys.exit(demo.run())
