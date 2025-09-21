#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import time


def test_state_publisher():
    rospy.init_node("test_state_publisher", anonymous=True)

    pub = rospy.Publisher("state", Int32, queue_size=10)

    rate = rospy.Rate(0.1)  # 0.1 Hz (10 seconds)

    states = [0, 1]  # Air mode, Water mode
    state_index = 0

    rospy.loginfo("Test State Publisher started")

    while not rospy.is_shutdown():
        msg = Int32()
        msg.data = states[state_index]

        rospy.loginfo("Publishing state: %d (%s)", msg.data, "Air" if msg.data == 0 else "Water")
        pub.publish(msg)

        state_index = (state_index + 1) % len(states)
        rate.sleep()


if __name__ == "__main__":
    try:
        test_state_publisher()
    except rospy.ROSInterruptException:
        pass
