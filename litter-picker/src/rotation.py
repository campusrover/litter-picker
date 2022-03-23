#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32
import constants
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry

def odom_cb(msg):
    global position 
    position = msg

def rotation_cb(msg):
    global rotation_goal
    rotation_goal = msg.data
    #print(rotation_goal)

# Main program starts here
if __name__ == '__main__':
    rospy.init_node('rotation')
    state_pub = rospy.Publisher('rotation/status', Int32, queue_size=1)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
    rotation_sub = rospy.Subscriber('rotation/goal', Float32, rotation_cb)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    position = None
    rotation_goal = None
    state = -1
    twist = Twist()

    while not rospy.is_shutdown():
        
        if (position != None and rotation_goal != None):
            z = position.pose.pose.orientation.z 

            if (abs(rotation_goal - z) > 0.2):
                twist.angular.z = 0.2 
                state = 2
                print("rotating!", rotation_goal, " ", z, " ", rotation_goal - z)
            else:  
                twist.angular.z = 0 
                state = 1
                position = None 
                rotation_goal = None
                #print("finished rotating")

            cmd_vel_pub.publish(twist)
            state_pub.publish(state)
            
