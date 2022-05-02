#!/usr/bin/env python3
import rospy
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Pose, Point, Twist
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount, BoundingBox
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import math
from litter_picker.msg import TrashAction, TrashResult, TrashGoal
from actions import TRASH_ACTION
import topics
from utils import dist_between_two, get_first_bonding_box


class TrashLocalizationNode:

    def __init__(self):
        self.server = actionlib.SimpleActionServer(TRASH_ACTION,
                                                   TrashAction,
                                                   execute_cb=self.perform_action,
                                                   auto_start=False)
        self.server.start()

        self.current_pose = None
        self.box_coordinates = [None] * 2
        self.image = []
        self.img_height = 0
        self.img_width = 0
        self.result = TrashResult()
        self.box_id = None

        self.pose_sub = rospy.Subscriber(topics.AMCL_LOC, PoseWithCovarianceStamped,
                                         self._pose_cb())
        self.depth_sub = rospy.Subscriber(topics.DEPTH_CAMERA, CompressedImage, self._depth_cb())
        self.box_sub = rospy.Subscriber(topics.BOUNDING_BOXES, BoundingBoxes, self._box_cb())

        self.vel = Twist()
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)

    def _pose_cb(self):

        def pose_cb(msg):
            self.current_pose = msg.pose

        return pose_cb

    def _box_cb(self):

        def box_cb(msg: BoundingBoxes):
            box = get_first_bonding_box(msg.bounding_boxes)
            if box is not None:
                self.box_coordinates[0] = (box.xmax + box.xmin) / 2
                self.box_coordinates[1] = (box.ymax + box.xmin) / 2
            else:
                self.box_coordinates = [None] * 2

        return box_cb

    def _depth_cb(self):

        def depth_cb(msg):
            bridge = CvBridge()
            self.image = bridge.compressed_imgmsg_to_cv2(msg)
            self.img_height, self.img_width = self.image.shape
            # rospy.loginfo("[trash_localizer] image size -> h: %s, w: %s", self.img_height, self.img_width)
            # width: 1920
            # height: 1080

        return depth_cb

    def get_distance_vision(self, frame):
        print("trash_localizer is triggered")
        self.server.set_succeeded(self.result)

    def perform_action(self, goal: TrashGoal):
        self.box_id = goal.box_id
        self.get_distance_vision(self.image)

    def reset_node(self):
        self.current_pose = None
        self.box_coordinates = [None] * 2
        self.image = []
        self.box_id = None
        self.result = TrashResult()

    def get_bonding_box_coordinate(self):
        print("self.box_coordinates {}".format(self.box_coordinates))
        print("self.image.shape: {}".format(self.image.shape))
        if self.box_coordinates is None:
            return None
        elif len(self.box_coordinates) != 2:
            return None
        else:
            try:
                x, y = int(self.box_coordinates[0]), int(self.box_coordinates[1])
                h, w = self.image.shape
                if x >= h or y >= w:
                    return None
                else:
                    return x, y
            except:
                return None


if __name__ == '__main__':
    rospy.init_node("trash_localizer")
    rospy.loginfo('Localization Node is running')
    nav = TrashLocalizationNode()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()