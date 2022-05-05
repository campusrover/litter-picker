import rospy
from geometry_msgs.msg import Twist
from master import LitterPickerState
from task import Task
from litter_picker.msg import Trash
import topics

MAXIMUM_TRASH_ALLOWED = 1


class TrashLocalizerTask(Task):

    def __init__(self, state: LitterPickerState):
        super().__init__(state)
        self.sub = rospy.Subscriber(topics.TRASH_TOPIC, Trash, self._trash_cb())
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)

        self.has_box = False
        self.is_close_enough = False
        self.err_to_center = None
        self.vel = Twist()
        self.vel.linear.x = 0.2
        self.stop = Twist()
        self.rate = rospy.Rate(10)

        self.has_succeeded = False

    def _trash_cb(self):

        def cb(msg: Trash):
            self.has_box = msg.has_trash
            self.is_close_enough = msg.close_enough
            self.err_to_center = msg.err_to_center

        return cb

    def start(self):
        rospy.loginfo("[Trash localizer: ] ready to move toward the trash")
        while not rospy.is_shutdown() and self.has_box and not self.is_close_enough:
            self.vel.angular.z = -self.err_to_center / 2500
            self.cmd_vel_pub.publish(self.vel)
            rospy.loginfo("[Trash localizer:] current velocity = {}, angular speed = {}".format(
                self.vel.linear.x, self.vel.angular.z
            ))
            self.rate.sleep()

        self.cmd_vel_pub.publish(self.stop)

        if self.has_box:
            rospy.loginfo("[Trash localizer:] move forward to trap the trash")
            self.trap_trash()
            self.has_succeeded = True
        else:
            rospy.logwarn("[Trash localizer: bounding box has lost")

    def trap_trash(self):
        starting_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - starting_time < 2:
            self.vel.angular.z = 0
            self.cmd_vel_pub.publish(self.vel)
            self.rate.sleep()

        self.cmd_vel_pub.publish(self.stop)

    def next(self):
        from rotation import RotationTask
        from collection_site import MoveToCollectionSiteTask

        if self.has_succeeded:
            self.state.number_of_trash_picked += 1
            rospy.loginfo("[Trash localizer:] currently picked {} pieces of trash".format(
                self.state.number_of_trash_picked))

            if self.state.number_of_trash_picked >= MAXIMUM_TRASH_ALLOWED:
                return MoveToCollectionSiteTask(self.state)
            return RotationTask(self.state)
        else:
            return RotationTask(self.state)
