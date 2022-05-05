import math

import rospy
from geometry_msgs.msg import Twist

import topics
from task import Task
from litter_picker.msg import Trash

ROTATION_SPEED = 0.2


class RotationTask(Task):

    def __init__(self, state):
        super().__init__(state)
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)
        self.trash_sub = rospy.Subscriber(topics.TRASH_TOPIC, Trash, self.get_trash_cb())
        self.timeout = (math.pi * 2) / ROTATION_SPEED
        self.has_trash = False
        self.rate = rospy.Rate(10)

    def get_trash_cb(self):

        def cb(msg: Trash):
            self.has_trash = msg.has_trash

        return cb

    def start(self):
        rospy.loginfo("[Rotation Task:] rotating to detect trash")
        starting_time = rospy.Time.now().to_sec()
        twist = Twist()

        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - starting_time
                                           ) < self.timeout and self.has_trash is False:
            twist.linear.x = 0
            twist.angular.z = ROTATION_SPEED
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - starting_time < 3:
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def next(self):
        from navigation import NavigationTask
        from trash_localizer import TrashLocalizerTask

        if self.has_trash:
            rospy.loginfo("[Rotation Task:] Found trash ready to move toward it")
            return TrashLocalizerTask(self.state)
        else:
            rospy.loginfo(
                "[Rotation Task:] Did not find trash, will go to the next waypoint instead")
            return NavigationTask(self.state)
