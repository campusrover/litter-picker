#!/usr/bin/env python3
import math

import actionlib
import actions
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from litter_picker.msg import RotationAction, RotationGoal


class RotationActionServer:
    def __init__(self, name):
        self.server = actionlib.SimpleActionServer(
            name,
            RotationAction,
            execute_cb=self.rotate,
            auto_start=False
        )
        self.position = Odometry()
        self.odom_sub = rospy.Subscriber('odom', Odometry, self._create_odom_cb())
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def _create_odom_cb(self):
        def cb(msg):
            self.position = msg

        return cb

    def rotate(self, goal: RotationGoal):
        rotation_goal = goal.a
        twist = Twist()

        # get the current orientation
        orientation = self.position.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        if yaw < 0:
            yaw += 2 * math.pi

        # get difference between the current orientation and the target
        diff = yaw - rotation_goal
        while abs(diff) >= 0.1:
            twist.angular.z = 0.2
            diff = yaw-rotation_goal
            self.cmd_vel_pub.publish(twist)

        # got to the desired state
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        self.server.set_succeeded(
            str.format("[Rotation Node]: Successfully reach angular position {}", str(goal.a))
        )


# Main program starts here
if __name__ == '__main__':
    rospy.init_node('rotation')
    rospy.loginfo('Rotation Node is running')
    rotation_node = RotationActionServer(actions.ROTATION_ACTION)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
