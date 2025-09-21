#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sys


def switch_state(state):
    rospy.init_node("state_switcher", anonymous=True)

    pub = rospy.Publisher("state", Int32, queue_size=10)

    # Wait for publisher to be ready
    rospy.sleep(0.5)

    msg = Int32()
    msg.data = state

    rospy.loginfo("Switching to state: %d (%s)", msg.data, "Air" if msg.data == 0 else "Water")
    pub.publish(msg)

    # Give time for message to be published
    rospy.sleep(0.5)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: switch_state.py <state_number>")
        print("  0: Air mode (unidirectional motors)")
        print("  1: Water mode (bidirectional motors)")
        sys.exit(1)

    try:
        state = int(sys.argv[1])
        if state not in [0, 1]:
            print("Invalid state. Use 0 for Air mode or 1 for Water mode")
            sys.exit(1)

        switch_state(state)
        print("State switch command sent successfully")

    except ValueError:
        print("State must be a number (0 or 1)")
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
