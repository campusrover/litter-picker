#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import constants
from geometry_msgs.msg import Twist
import math

def get_state(msg):
    global state
    state = msg.data

def odom_cb(msg):
    global position 
    position = msg.data 

def rotation_cb(msg):
    global rotation_goal
    rotation_goal = msg.data

# Main program starts here
if __name__ == '__main__':
    rospy.init_node('rotation')
    state_pub = rospy.Subscriber(constants.STATE_TOPIC_NAME, Int32, get_state)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
    rotation_sub = rospy.Subscriber('rotation/goal', Float32, rotation_cb)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()

    while not rospy.is_shutdown():
        z = position.angular.z 
        
        if (rotation_goal - z) != 0:
            twist.angular.z = 0.2 
            state_pub.publish(0)
        else:  
            twist.angular.z = 0 
            state_pub.publish(1)
            
        cmd_vel_pub.publish(twist)
            
