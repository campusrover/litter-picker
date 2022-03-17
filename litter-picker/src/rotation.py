#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import constants
from geometry_msgs.msg import Twist
import math

state_pub = rospy.Publisher(constants.STATE_TOPIC_NAME, Int32, queue_size=1)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)


def get_state(msg: Int32):
    global state
    state = msg.data


# Main program starts here
if __name__ == '__main__':
    state = constants.GO_TO_NEXT_WAYPOINT
    rospy.init_node('rotation')
    state_sub = rospy.Subscriber(constants.STATE_TOPIC_NAME, Int32, get_state)
    time_started_rotation = rospy.Time.now()
    twist = Twist()

    while not rospy.is_shutdown():
        if state == constants.REACHED_WAYPOINT:
            print("STATE in if", state)
            print("reached waypoint")
            time_started_rotation = rospy.Time.now()
            state_pub.publish(constants.ROTATING)
        elif state == constants.ROTATING:
            while (rospy.Time.now() - time_started_rotation).to_sec() <= 4 and not rospy.is_shutdown():
                print("STATE in while", state)
                print("inside rotation loop")
                twist.angular.z = math.pi/2
                cmd_vel_pub.publish(twist)
            state_pub.publish(constants.GO_TO_NEXT_WAYPOINT)
            print("STATE after while", state)
            twist.angular.z = 0
            cmd_vel_pub.publish(twist)
            print("TWIST: ", twist.angular.z)
            print("end rotating")
            
