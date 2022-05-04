import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

from utils import dist_between_two
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
        dist_to_trash_local = self.dist_to_trash
        original_x, original_y = self.current_pose.pose.position.x, self.current_pose.pose.position.y
        dist_covered = 0
        while not rospy.is_shutdown() and (dist_covered < dist_to_trash_local):
            self.vel.angular.z = self.err_to_center / 3000
            self.cmd_vel_pub.publish(self.vel)
            dist_covered = dist_between_two(self.current_pose.pose.position.x,
                                            self.current_pose.pose.position.y, original_x,
                                            original_y)
            rospy.loginfo(
                "[Trash localizer:] current velocity = {}, angular speed = {}, and dist_left = {}".
                format(self.vel.angular.z, self.vel.linear.x, dist_to_trash_local - dist_covered))

        self.cmd_vel_pub.publish(self.stop)

    def next(self):
        from rotation import RotationTask

        if self.has_box:
            return TrashLocalizerTask(self.state)
        else:
            return RotationTask(self.state)
