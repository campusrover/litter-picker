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
from utils import get_first_bonding_box
import topics

ROTATION_SPEED = 0.2


class RotationActionServer:

    def __init__(self, name):
        self.server = actionlib.SimpleActionServer(name,
                                                   RotationAction,
                                                   execute_cb=self.rotate,
                                                   auto_start=False)
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)
        self.image_sub = rospy.Subscriber(topics.COMPRESSED_IMAGE_TOPIC, CompressedImage,
                                          self.get_width_and_height_cb())
        self.image_wh = None

        self.bonding_box_err = None
        self.box_sub = rospy.Subscriber(topics.BOUNDING_BOXES, BoundingBoxes,
                                        self.get_bonding_box_err_cb())
        self.box_id = None
        self.result = RotationResult()
        self.timeout = (math.pi * 2 + 1) / ROTATION_SPEED

        self.server.start()

    def get_width_and_height_cb(self):

        def cb(msg: CompressedImage):
            # infer once during the lifetime of the object
            if self.image_wh is None:
                bridge = CvBridge()
                image = bridge.compressed_imgmsg_to_cv2(msg)
                self.image_wh = image.shape

        return cb

    def get_bonding_box_err_cb(self):

        def cb(msg: BoundingBoxes):
            if self.image_wh is None:
                rospy.logwarn("unable to get the image's shape")
            else:
                _, w, _ = self.image_wh
                box = get_first_bonding_box(msg.bounding_boxes)
                if box is None:
                    self.bonding_box_err = None
                else:
                    center = (box.xmin + box.xmax) / 2
                    image_center = w / 2
                    self.bonding_box_err = abs(image_center - center)
                    self.box_id = box.id

        return cb

    def rotate(self, goal: RotationGoal):
        starting_time = rospy.Time.now().to_sec()
        twist = Twist()

        while (rospy.Time.now().to_sec() - starting_time) < self.timeout \
            and (self.bonding_box_err is None or self.bonding_box_err > 0.1):
            twist.angular.z = ROTATION_SPEED
            self.cmd_vel_pub.publish(twist)

        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

        if self.bonding_box_err is not None:
            self.result.msg = "Now facing trash"
            self.result.id = self.box_id
            self.server.set_succeeded(self.result)
        else:
            self.result.msg = "No trash found"
            self.server.set_aborted(self.result)


# Main program starts here
if __name__ == '__main__':
    rospy.init_node('rotation')
    rospy.loginfo('Rotation Node is running')
    rotation_node = RotationActionServer(actions.ROTATION_ACTION)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
