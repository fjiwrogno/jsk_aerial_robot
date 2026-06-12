#!/usr/bin/env python
"""Terminal monitor for aerial_robot_msgs/RcRaw.

Shows live bar graphs of the 16 RC channels with observed min/max (useful for
calibration), the raw flags byte, failsafe/frame_lost states and the frame rate.

Usage:
  rosrun aerial_robot_rc rc_monitor.py [_topic:=/rc/raw]
Keys:
  r: reset observed min/max    q: quit
"""

import curses
import threading

import rospy
from aerial_robot_msgs.msg import RcRaw

CHANNEL_MAX = 2047


class RcMonitor(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.msg = None
        self.observed_min = [CHANNEL_MAX] * 16
        self.observed_max = [0] * 16
        self.frame_count = 0
        self.rate_window = []  # receive timestamps of the last second

        topic = rospy.get_param("~topic", "rc/raw")
        self.sub = rospy.Subscriber(topic, RcRaw, self.callback, queue_size=10)
        self.topic = self.sub.resolved_name

    def callback(self, msg):
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.msg = msg
            self.frame_count += 1
            for i, value in enumerate(msg.channels):
                if not msg.failsafe:
                    self.observed_min[i] = min(self.observed_min[i], value)
                    self.observed_max[i] = max(self.observed_max[i], value)
            self.rate_window.append(now)
            self.rate_window = [t for t in self.rate_window if now - t < 1.0]

    def reset_observed(self):
        with self.lock:
            self.observed_min = [CHANNEL_MAX] * 16
            self.observed_max = [0] * 16

    def draw(self, screen):
        screen.nodelay(True)
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_RED, -1)
        curses.init_pair(2, curses.COLOR_GREEN, -1)

        while not rospy.is_shutdown():
            key = screen.getch()
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("r"), ord("R")):
                self.reset_observed()

            with self.lock:
                msg = self.msg
                obs_min = list(self.observed_min)
                obs_max = list(self.observed_max)
                frame_count = self.frame_count
                # re-filter here: when the stream stops the callback no longer
                # prunes the window, and the displayed rate must decay to 0
                now = rospy.Time.now().to_sec()
                rate = len([t for t in self.rate_window if now - t < 1.0])

            screen.erase()
            height, width = screen.getmaxyx()
            bar_width = max(10, min(40, width - 40))

            screen.addnstr(
                0, 0, "RC monitor  topic=%s  frames=%d  rate=%dHz" % (self.topic, frame_count, rate), width - 1
            )

            if msg is None:
                screen.addnstr(2, 0, "waiting for %s ..." % self.topic, width - 1)
            else:
                status = "flags=0x%02X (0b%s)  frame_lost=%s  failsafe=%s" % (
                    msg.flags,
                    format(msg.flags, "08b"),
                    msg.frame_lost,
                    msg.failsafe,
                )
                color = curses.color_pair(1) if msg.failsafe or msg.frame_lost else curses.color_pair(2)
                screen.addnstr(1, 0, status, width - 1, color)

                hidden = 0
                for i, value in enumerate(msg.channels):
                    row = 3 + i
                    if row >= height - 1:
                        hidden = len(msg.channels) - i
                        break
                    filled = int(round(float(value) / CHANNEL_MAX * bar_width))
                    bar = "#" * filled + "-" * (bar_width - filled)
                    line = "ch%02d [%s] %4d  min=%4d max=%4d" % (
                        i + 1,
                        bar,
                        value,
                        obs_min[i] if obs_min[i] <= obs_max[i] else 0,
                        obs_max[i],
                    )
                    screen.addnstr(row, 0, line, width - 1)

                keys_hint = "keys: r=reset min/max  q=quit"
                if hidden:
                    keys_hint += "  (+%d channels hidden, enlarge terminal)" % hidden
                screen.addnstr(height - 1, 0, keys_hint, width - 1)

            screen.refresh()
            rospy.sleep(0.05)


def main():
    rospy.init_node("rc_monitor", anonymous=True)
    monitor = RcMonitor()
    curses.wrapper(monitor.draw)


if __name__ == "__main__":
    main()
