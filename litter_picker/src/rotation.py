#!/usr/bin/env python3
import math

import actionlib
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import CompressedImage

import actions
import rospy
from geometry_msgs.msg import Twist
from litter_picker.msg import RotationAction, RotationGoal, RotationResult

ROTATION_SPEED = 0.2


class RotationActionServer:
    def __init__(self, name):
        self.server = actionlib.SimpleActionServer(
            name,
            RotationAction,
            execute_cb=self.rotate,
            auto_start=False
        )
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('camera/depth/image_raw/compressed', CompressedImage,
                                          self.get_width_and_height_cb())
        self.image_wh = None

        self.bonding_box_err = False
        self.box_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.get_bonding_box_err_cb())
        self.result = RotationResult()
        self.timeout = 4 / ROTATION_SPEED

        self.wait_for_image_width_and_height()
        self.server.start()

    def get_width_and_height_cb(self):

        def cb(msg: CompressedImage):
            if self.image_wh is None:
                bridge = CvBridge()
                image = bridge.compressed_imgmsg_to_cv2(msg)
                self.image_wh = image.shape

        return cb

    def get_bonding_box_err_cb(self):

        def cb(msg: BoundingBoxes):
            if msg.bounding_boxes.count() == 0:
                self.bonding_box_err = None
            else:
                box: BoundingBox = msg.bounding_boxes[0]
                center = (box.xmin + box.xmax) / 2
                self.bonding_box_err = 100 - center

        return cb

    def wait_for_image_width_and_height(self):
        while self.image_wh is None:
            continue
        print(self.image_wh)

    def rotate(self, goal: RotationGoal):
        print("hello")
        starting_time = rospy.Time.now().to_sec()
        twist = Twist()

        while (rospy.Time.now() - starting_time).to_sec().real < self.timeout \
            and (self.bonding_box_err is None or self.bonding_box_err > 0.1):
            twist.angular.z = ROTATION_SPEED
            self.cmd_vel_pub.publish(twist)

        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

        if self.bonding_box_err is not None:
            self.result = "Now facing trash"
            self.server.set_succeeded(self.result)
        else:
            self.result = "No trash found"
            self.server.set_aborted(self.result)


# Main program starts here
if __name__ == '__main__':
    rospy.init_node('rotation')
    rospy.loginfo('Rotation Node is running')
    rotation_node = RotationActionServer(actions.ROTATION_ACTION)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
