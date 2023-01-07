#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

def isData():
    """Returns True if there is data to be read from stdin."""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def talk_to_me() -> None:
    """test node to publish a message to a topic"""
    pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(60)
    rospy.loginfo("node started")
    speed = 0
    angular = 0
    while not rospy.is_shutdown():
        msg = Twist()
        if isData():
            c = sys.stdin.read(1) # read 2 characters
            # check for escape character
            if c == '\x1b':         # x1b is ESC
                break
            # check for w character
            if c == 'w':
                if speed <= 1.8:
                    speed += 0.2
            if c == 's':
                if speed >= -1.8:
                    speed -= 0.2
            if c == 'a':
                if angular <= 1.6:
                    angular += 0.2
            if c == 'd':
                if angular >= -1.6:
                    angular -= 0.2
        else:
            if speed >= 0.1 and speed != 0:
                speed -= 0.1
            if speed <= -0.1 and speed != 0:
                speed += 0.1
            if angular >= 0.2 and angular != 0:
                angular -= 0.2
            if angular <= -0.2 and angular != 0:
                angular += 0.2
        msg.linear.x = speed
        msg.angular.z = angular
        pub.publish(msg)
        rate.sleep()
    rospy.loginfo("excecution terminated")

if __name__ == '__main__':
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        talk_to_me()
    except rospy.ROSInternalException:
        pass
    finally:
        print("excecution terminated")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

