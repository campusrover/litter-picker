import rospy
from geometry_msgs.msg import Twist
from master import LitterPickerState
from task import Task
from litter_picker.msg import Trash
import topics

# the number of seconds to move the litter picker forward to trap the trash
NUMBER_OF_SECONDS_TO_TRAP = 2

# the maximum amount of trash allowed before moving to the collection site
MAXIMUM_TRASH_ALLOWED = 1


class TrashLocalizerTask(Task):
    """
    Handle the trash localization task and move toward the trash to collect it. It subscribes to
    the topic that the vision node publishes to, which contains information on how much the
    litter picker should turn to face directly to the trash, and whether the litter picker is
    close enough to the trash
    """

    def __init__(self, state: LitterPickerState):
        """
        Constructor for the TrashLocalizationTask

        :param state: the current LitterPicker state
        """
        super().__init__(state)
        self.sub = rospy.Subscriber(topics.TRASH_TOPIC, Trash, self._trash_cb())
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)

        # whether there is a bounding box/trash nearby
        self.has_box = False

        # whether we are close enough to the trash to trap it
        self.is_close_enough = False

        # how far away is the trash to the center of the bounding box
        self.err_to_center = None

        # twist message for moving the litter picker toward the trash
        self.vel = Twist()
        self.vel.linear.x = 0.2

        # twist message for stopping the robot
        self.stop = Twist()

    def _trash_cb(self):
        """
        Create a callback function that update the has_box, is_close_enough, err_to_center
        field as it receives incoming messages from the litter_picker/vision topic

        :return: the callback function
        """

        def cb(msg: Trash):
            self.has_box = msg.has_trash
            self.is_close_enough = msg.close_enough
            self.err_to_center = msg.err_to_center

        return cb

    def start(self):
        """
        Start execute the task. It first tries to move toward the trash and adjusting itself
        so that it directly faces the trash as it moves. After it becomes close to the trash,
        it will try to move forward a bit to trap the trash
        """
        rospy.loginfo("[Trash localizer: ] ready to move toward the trash")
        while not rospy.is_shutdown() and self.has_box and not self.is_close_enough:
            self.vel.angular.z = -self.err_to_center / 1500
            self.cmd_vel_pub.publish(self.vel)
            rospy.loginfo("[Trash localizer:] current velocity = {}, angular speed = {}".format(
                self.vel.linear.x, self.vel.angular.z))
            self.rate.sleep()

        self.cmd_vel_pub.publish(self.stop)

        if self.has_box:
            rospy.loginfo("[Trash localizer:] move forward to trap the trash")
            self.trap_trash()
            self.has_succeeded = True
        else:
            rospy.logwarn("[Trash localizer: bounding box has lost")

    def trap_trash(self):
        """
        Move forward a bit to trap the trash
        """
        # perhaps also check if the bounding box is in the center before doing the trap
        starting_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown(
        ) and rospy.Time.now().to_sec() - starting_time < NUMBER_OF_SECONDS_TO_TRAP:
            self.vel.angular.z = 0
            self.cmd_vel_pub.publish(self.vel)
            self.rate.sleep()

        self.cmd_vel_pub.publish(self.stop)

    def next(self):
        """
        If the trash pickup has been successful, check if the current number of trash picked has
        exceeded the maximum number of trash allowed, if it does, it will direct the litter picker
        to go to the collection site. Otherwise, it will try to rotate to see if there is nearby trash
        to pick up. If the pickup fails, it will try to rotate, and try again.

        :return: either an instance of MoveToCollectionSiteTask or an instance of RotationTask
        """
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
