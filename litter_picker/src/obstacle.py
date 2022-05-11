#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from litter_picker.msg import Obstacle
import topics


class ObstacleNode:

    def __init__(self):
        self.scan_sub = rospy.Subscriber(topics.SCAN, LaserScan, self._scan_cb())
        self.obstacle_pub = rospy.Publisher(topics.OBSTACLE, Obstacle, queue_size=2)
        self.front = []
        self.back = []
        self.front_distance = 0.7
        self.back_distance = 0.4

    def _scan_cb(self):

        def cb(msg):
            front_raw = msg.ranges[-30:-1] + msg.ranges[0:30]
            back_raw = msg.ranges[90:130]
            self.front = [data for data in front_raw if msg.range_min < data < msg.range_max]
            self.back = [data for data in back_raw if msg.range_min < data < msg.range_max]

        return cb

    def check_front_and_back(self):
        front_boolean = True
        back_boolean = True

        if self.front and min(self.front) < self.front_distance:
            front_boolean = False

        if self.back and min(self.back) < self.back_distance:
            back_boolean = False

        return front_boolean, back_boolean

    def publisher(self):
        front_boolean, back_boolean = self.check_front_and_back()
        msg = Obstacle()
        msg.safe_to_go_forward = front_boolean
        msg.safe_to_go_back = back_boolean

        self.obstacle_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("litter_picker_obstacle")
    obstacle_node = ObstacleNode()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        obstacle_node.publisher()
        rate.sleep()
