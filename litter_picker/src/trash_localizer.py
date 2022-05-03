import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

from rotation import RotationTask
from master import LitterPickerState
from task import Task
from litter_picker.msg import Trash
import topics


class TrashLocalizerTask(Task):

    def __init__(self, state: LitterPickerState):
        super().__init__(state)
        self.sub = rospy.Subscriber(topics.TRASH_TOPIC, Trash, self._cb())
        self.pose_sub = rospy.Subscriber(topics.AMCL_LOC, PoseWithCovarianceStamped,
                                         self._pose_cb())
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)

        self.has_box = False
        self.dist_to_trash = None
        self.err_to_center = None
        self.current_pose = None
        self.closest_distance = 0.3
        self.vel = Twist()
        self.vel.linear.x = 0.2
        self.stop = Twist()

    def _cb(self):

        def cb(msg):
            self.has_box = msg.has_trash
            self.dist_to_trash = msg.dist_to_trash
            self.err_to_center = msg.err_to_center

        return cb

    def _pose_cb(self):

        def pose_cb(msg):
            self.current_pose = msg.pose

        return pose_cb

    def start(self):
        rospy.loginfo("[Trash Localizer:] In the process of moving toward trash")
        if self.has_box:
            while not rospy.is_shutdown() and (self.dist_to_trash > self.closest_distance):
                self.vel.angular.z = self.err_to_center / 3000
                self.cmd_vel_pub.publish(self.vel)
                rospy.loginfo("[Trash localizer:] current velocity = {}, and angular speed = {}".format(
                    self.vel.angular.z, self.vel.linear.x))
        else:
            self.cmd_vel_pub.publish(self.stop)

    def next(self):
        if self.has_box:
            return TrashLocalizerTask(self.state)
        else:
            return RotationTask(self.state)
