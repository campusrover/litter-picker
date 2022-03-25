#!/usr/bin/env python3
import math

import rospy
from std_msgs.msg import Int32, Float32
import states
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class RotationNode:
    def __init__(self):
        rospy.init_node('rotation')
        self.position = Odometry()
        self.rotation_goal = []
        self.state = states.AVAILABLE
        self.state_pub = rospy.Publisher('rotation/status', Int32, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self._create_odom_cb())
        self.rotation_sub = rospy.Subscriber('rotation/goal', Float32, self._create_rotation_cb())
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def _create_odom_cb(self):
        def cb(msg):
            self.position = msg
        return cb

    def _create_rotation_cb(self):
        def cb(msg):
            if len(self.rotation_goal) == 0:
                self.rotation_goal.append(msg.data)
        return cb

    def perform_action(self):
        if self.state == states.AVAILABLE and len(self.rotation_goal) != 0:
            self.state = states.IN_PROGRESS
        elif self.state == states.IN_PROGRESS:
            twist = Twist()

            # get the current orientation
            orientation = self.position.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            if yaw < 0:
                yaw += 2 * math.pi

            # get difference between the current orientation and the target
            diff = yaw - self.rotation_goal[0]
            if abs(diff) < 0.1:
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                self.state = states.DONE
            else:
                twist.angular.z = 0.2 if yaw > 0 else -0.2
                self.cmd_vel_pub.publish(twist)
                self.state_pub.publish(states.IN_PROGRESS)
        elif self.state == states.DONE:
            # remove it since we are done rotating
            self.rotation_goal.pop(0)
            self.state_pub.publish(states.DONE)
            self.state = states.AVAILABLE


# Main program starts here
if __name__ == '__main__':
    rotation_node = RotationNode()

    while not rospy.is_shutdown():
        rotation_node.perform_action()
