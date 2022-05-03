import math

import rospy
from geometry_msgs.msg import Twist

import topics
from master import LitterPickerState
from task import Task
from litter_picker.msg import Trash
from navigation import NavigationTask
from trash_localizer import TrashLocalizerTask

ROTATION_SPEED = 0.2


class RotationTask(Task):

    def __init__(self, state: LitterPickerState):
        super().__init__(state)
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)
        self.trash_sub = rospy.Subscriber(topics.TRASH_TOPIC, Trash, self.get_trash_cb())
        self.timeout = (math.pi * 2) / ROTATION_SPEED
        self.has_trash = False

    def get_trash_cb(self):

        def cb(msg: Trash):
            self.has_trash = msg.has_trash

        return cb

    def start(self):
        starting_time = rospy.Time.now().to_sec()
        twist = Twist()

        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - starting_time
                                           ) < self.timeout and self.has_trash is False:
            twist.angular.z = ROTATION_SPEED
            self.cmd_vel_pub.publish(twist)

        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - starting_time < 3:
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)

    def next(self):
        if self.has_trash:
            rospy.loginfo("Found trash ready to move toward it")
            return TrashLocalizerTask(self.state)
        else:
            rospy.loginfo("Did not find trash, will go to the next waypoint instead")
            return NavigationTask(self.state)
