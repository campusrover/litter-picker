#!/usr/bin/env python3
"""
A ros node that is used for processing image and bounding box information in the background. It publishes
the information about the bounding boxes to the litter_picker/vision topic with the custom message type
of Trash.msg
"""

import rospy
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import CompressedImage
from litter_picker.msg import Trash

import topics
from utils import get_first_bonding_box

# the maximum age of the bounding box message allowed in seconds before the information may
# be discarded
MAXIMUM_MSG_AGE_ALLOWED = 0.5

# how close the bounding box is to the bottom of the image that we will claim that the
# litter picker is close enough to the object
PIXEL_TO_BOTTOM = 60


class Vision:
    """
    Handle the processing of the image reading from the camera as well as information about the bounding
    box published by the darknet_ros node, which runs YOLO on the camera image in the background
    """

    def __init__(self):
        """
        Constructor for the Vision class. The node subscribes to the topic that has CompressedImage message, and the
        topic that has the bounding box information produced when YOLO is run on the camera images. It processes the
        information it receives and publishes to the litter_picker/vision topic.
        """

        self.image_sub = rospy.Subscriber(topics.IMAGE_TOPIC, CompressedImage, self.get_image_cb())
        self.box_sub = rospy.Subscriber(topics.BOUNDING_BOXES, BoundingBoxes,
                                        self.get_bounding_box_cb())
        self.trash_pub = rospy.Publisher(topics.TRASH_TOPIC, Trash)

        # image from the camera
        self.image = None

        # the bounding box of object that is trash
        self.box = None

        # when was the information about the last bounding box received
        self.last_box_timestamp = 0

    ########################
    ## Callback Functions ##
    ########################

    def get_bounding_box_cb(self):

        def cb(msg: BoundingBoxes):
            self.last_box_timestamp = rospy.Time.now().to_sec()
            self.box = get_first_bonding_box(msg.bounding_boxes)

        return cb

    def get_image_cb(self):

        def cb(msg: CompressedImage):
            self.image = CvBridge().compressed_imgmsg_to_cv2(msg)

        return cb

    ########################
    ## Vision Processors  ##
    ########################

    def calculate_dist_to_obj(self):
        """
        Calculate the distance between the center of the bounding box to the center of the image. Also
        estimate if the litter picker is close enough to the object

        :return: the distance from the bounding box to the center of the image & whether the litter picker
                 is close enough to the object
        """
        if self.image is None:
            rospy.logwarn("[Vision Node:] image has not be loaded yet")
        else:
            if self.box is not None:
                h, w, _ = self.image.shape
                center = (self.box.xmin + self.box.xmax) / 2
                image_center = w / 2
                return center - image_center, self.is_close_enough_to_the_object(self.box, h)
            return -1

    def is_close_enough_to_the_object(self, box: BoundingBox, image_height: int):
        """
        Determine if the robot is close enough to the object by trying to calculate how close
        the bounding box is to the bottom of the image. When the object is further away from the
        camera, the floor will occupy the bottom part of the image.

        :param box: the bounding box
        :param image_height: the height of the image
        :return: a boolean value indicating if the litter picker is close enough to the object
        """
        if self.box is None:
            return False
        return image_height - box.ymax < PIXEL_TO_BOTTOM

    def publish_data(self):
        """
        Publish the information about the bounding box to the litter_picker/vision topic
        """
        time_now = rospy.Time.now().to_sec()
        msg = Trash()
        msg.has_trash = self.box is not None

        # discard the bounding box information if it is too old
        if time_now - self.last_box_timestamp > MAXIMUM_MSG_AGE_ALLOWED:
            msg.has_trash = False

        # only calculate the err_to_center of close_enough to the object when there is
        # a bounding box in the first place
        if msg.has_trash:
            msg.err_to_center, msg.close_enough = self.calculate_dist_to_obj()
        self.trash_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("vision_processor")
    vision_node = Vision()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vision_node.publish_data()
        rate.sleep()
