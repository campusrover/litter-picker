#!/usr/bin/env python3

import math

import rospy
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CompressedImage
from litter_picker.msg import Trash

import topics
from utils import get_first_bonding_box


class Vision:

    def __init__(self):
        self.image_sub = rospy.Subscriber(topics.IMAGE_TOPIC, CompressedImage, self.get_image_cb())
        self.box_sub = rospy.Subscriber(topics.BOUNDING_BOXES, BoundingBoxes,
                                        self.get_bounding_box_cb())
        self.trash_pub = rospy.Publisher(topics.TRASH_TOPIC, Trash)
        self.depth_sub = rospy.Subscriber(topics.DEPTH_CAMERA, CompressedImage, self.get_depth_cb())

        self.image = None
        self.box = None
        self.depth_image = None

    ########################
    ## Callback Functions ##
    ########################

    def get_bounding_box_cb(self):

        def cb(msg: BoundingBoxes):
            self.box = get_first_bonding_box(msg.bounding_boxes)

        return cb

    def get_image_cb(self):

        def cb(msg: CompressedImage):
            self.image = CvBridge().compressed_imgmsg_to_cv2(msg)

        return cb

    def get_depth_cb(self):

        def cb(msg):
            self.depth_image = CvBridge().compressed_imgmsg_to_cv2(msg)

        return cb

    ########################
    ## Vision Processors  ##
    ########################

    def calculate_obj_dist_to_center(self):
        if self.image is None:
            rospy.logwarn("[Vision Node:] image has not be loaded yet")
        else:
            if self.box is not None:
                _, w, _ = self.image.shape
                center = (self.box.xmin + self.box.xmax) / 2
                image_center = w / 2
                return center - image_center
            return None

    def calculate_dist_to_obj(self):
        if self.depth_image is None:
            rospy.logwarn("[Vision Node:] depth image has not been loaded yet")
        else:
            if self.box is not None:
                box_x, box_y = int((self.box.xmin + self.box.xmin) / 2), int(
                    (self.box.ymin + self.box.ymax) / 2)

                if box_x >= self.depth_image.shape[0] or box_y >= self.depth_image.shape[1]:
                    rospy.logwarn("[Vision Node:] bounding box coordinate out of bound")
                    return None
                dist_to_obj = self.depth_image[box_x][box_y]
                return dist_to_obj
            return None

    def publish_data(self):
        msg = Trash()
        msg.has_trash = self.box is not None
        if msg.has_trash:
            msg.err_to_center = self.calculate_obj_dist_to_center()
            msg.dist_to_trash = self.calculate_dist_to_obj()
        self.trash_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("vision_processor")
    vision_node = Vision()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vision_node.publish_data()
        rate.sleep()
