import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal

import topics
from task import Task
from utils import navigate_to_waypoint

NUMBER_OF_SECONDS_TO_BACKUP = 2

BACK_UP_SPEED = -0.2


class MoveToCollectionSiteTask(Task):
    """
    This class handles the robot's movement to a central collection site. It publishes movement
    to the cmd_vel and uses move_base actionlib client to navigate the litter picker to the
    collection site.
    """

    def __init__(self, state):
        """
        constructor for the MoveToCollectionSiteTask

        :param state: the current litter picker state
        """
        super().__init__(state)
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def start(self):
        """
        Start executing the task: first move the litter picker to the collection site, then
        drop off the trash by backing up.
        """
        collection_site: MoveBaseGoal = self.state.collection_site

        rospy.loginfo(
            "[Collection Site Task:] sending the litter picker to the collection site: {}, {}".
            format(collection_site.target_pose.pose.position.x,
                   collection_site.target_pose.pose.position.y))

        if navigate_to_waypoint(collection_site):
            rospy.loginfo(
                "[Collection Site Task:] has successfully reached the collection site: {}, {}".
                format(collection_site.target_pose.pose.position.x,
                       collection_site.target_pose.pose.position.y))
            self.has_succeeded = True
        else:
            rospy.logwarn(
                "[Collection Site Task:] has failed reached the collection site: {}, {}: try again".
                format(collection_site.target_pose.pose.position.x,
                       collection_site.target_pose.pose.position.y))

        if self.has_succeeded:
            # only back up if we successfully reach the trash collection site
            rospy.loginfo("[Collection Site Task:] now backing up")
            self.back_up()

    def back_up(self):
        """
        Backing up the litter picker for a set amount of time
        """
        starting_time = rospy.Time.now().to_sec()
        twist = Twist()

        while not rospy.is_shutdown(
        ) and rospy.Time.now().to_sec() - starting_time < NUMBER_OF_SECONDS_TO_BACKUP:
            twist.linear.x = BACK_UP_SPEED
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def next(self):
        """
        Get the next Task to be executed. If the backup was successful, return an instance of the
        Navigation Task to move the litter picker to the next waypoint. Otherwise, try again.

        :return: either an instance of NavigationTask or MoveToCollectionSiteTask
        """
        from navigation import NavigationTask

        if self.has_succeeded:
            # clear the trash
            self.state.number_of_trash_picked = 0
            return NavigationTask(self.state)
        else:
            return MoveToCollectionSiteTask(self.state)
