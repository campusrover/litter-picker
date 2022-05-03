#!/usr/bin/env python3

import math

import rospy
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CompressedImage
from litter_picker.msg import Trash

import topics
from utils import get_first_bonding_box


class Vision:
    def __init__(self):
        self.image_sub = rospy.Subscriber(topics.IMAGE_TOPIC, CompressedImage,
                                          self.get_width_and_height_cb())
        self.box_sub = rospy.Subscriber(topics.BOUNDING_BOXES, BoundingBoxes,
                                        self.get_bonding_box_err_cb())
        self.trash_pub = rospy.Publisher(topics.TRASH_TOPIC, Trash)
        self.depth_sub = rospy.Subscriber(topics.DEPTH_CAMERA, CompressedImage, self.get_depth_cb())

        self.image_wh = None
        self.bounding_box_err = None
        self.bounding_box_coord = None
        self.distance_to_box = None

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
                    self.bounding_box_coord = (
                        (box.xmin + box.xmax) / 2,
                        (box.ymin + box.ymax) / 2
                    )
                    center = (box.xmin + box.xmax) / 2
                    image_center = w / 2
                    self.bonding_box_err = abs(image_center - center)

        return cb

    def get_width_and_height_cb(self):

        def cb(msg: CompressedImage):
            # infer once during the lifetime of the object
            if self.image_wh is None:
                bridge = CvBridge()
                image = bridge.compressed_imgmsg_to_cv2(msg)
                self.image_wh = image.shape

        return cb

    def get_depth_cb(self):

        def cb(msg):
            if self.bounding_box_coord is None:
                self.distance_to_box = None
            else:
                bridge = CvBridge()
                image = bridge.compressed_imgmsg_to_cv2(msg)
                box_x, box_y = self.bounding_box_coord
                distance = image[box_x][box_y]
                self.distance_to_box = 0 if math.isnan(distance) else distance

        return cb

    def publish_data(self):
        msg = Trash()
        msg.has_trash = self.bounding_box_coord is None
        msg.dist_to_trash = self.distance_to_box
        msg.err_to_center = self.bounding_box_err

        self.trash_pub.publish()


if __name__ == '__main__':
    rospy.init_node("vision_processor")
    vision_node = Vision()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vision_node.publish_data()
        rate.sleep()
