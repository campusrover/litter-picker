import math

import rospy
from geometry_msgs.msg import Twist

import topics
from task import Task
from litter_picker.msg import Trash

ROTATION_SPEED = 0.2


class RotationTask(Task):
    """
    Handle the rotation of the litter picker so that it can discover nearby trash around it. It subscribes
    to the topic that the vision node publishes to, which contains information about the bounding box. It publishes
    to the cmd_vel topic to create rotation movements.
    """

    def __init__(self, state):
        """
        Constructor of the rotation task

        :param state: the current litter picker state
        """
        super().__init__(state)
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)
        self.trash_sub = rospy.Subscriber(topics.TRASH_TOPIC, Trash, self.get_trash_cb())
        self.timeout = (math.pi * 2) / ROTATION_SPEED
        self.has_trash = False
        self.rate = rospy.Rate(10)

    def get_trash_cb(self):
        """
        Create the callback function for the litter_picker/vision topic. The callback function
        set the has_trash field to a boolean indicating whether there is trash nearby

        :return: the callback function for the litter_picker/vision topic
        """

        def cb(msg: Trash):
            self.has_trash = msg.has_trash

        return cb

    def start(self):
        """
        Start the task. Perform a 360 degrees rotation. If trash has been detected nearby, it will
        stop rotating, otherwise, it will perform a full 360 degrees rotation.
        """
        rospy.loginfo("[Rotation Task:] rotating to detect trash")
        starting_time = rospy.Time.now().to_sec()
        twist = Twist()

        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - starting_time
                                           ) < self.timeout and self.has_trash is False:
            twist.linear.x = 0
            twist.angular.z = ROTATION_SPEED
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        # stop the robot after trash has been discovered, or it has rotated for 360 degrees
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def next(self):
        """
        Determine the next task to be executed. If not trash has been detected after a 360 degrees
        rotation, then direct the robot to navigate to the next waypoint. Otherwise, direct the
        robot to move toward that trash

        :return: either an instance of the TrashLocalizationTask or an instance of the NavigationTask
        """
        from navigation import NavigationTask
        from trash_localizer import TrashLocalizerTask

        if self.has_trash:
            rospy.loginfo("[Rotation Task:] Found trash ready to move toward it")
            return TrashLocalizerTask(self.state)
        else:
            rospy.loginfo(
                "[Rotation Task:] Did not find trash, will go to the next waypoint instead")
            return NavigationTask(self.state)
