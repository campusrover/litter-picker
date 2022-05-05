import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal

import topics
from task import Task
from utils import navigate_to_waypoint


class MoveToCollectionSiteTask(Task):

    def __init__(self, state):
        super().__init__(state)
        self.cmd_vel_pub = rospy.Publisher(topics.CMD_VEL, Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.has_succeeded = False

    def start(self):
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
            rospy.loginfo("[Collection Site Task:] now backing up")
            self.back_up()

    def back_up(self):
        starting_time = rospy.Time.now().to_sec()
        twist = Twist()

        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - starting_time < 1:
            twist.linear.x = -0.2
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def next(self):
        from navigation import NavigationTask

        if self.has_succeeded:
            return NavigationTask(self.state)
        else:
            return MoveToCollectionSiteTask(self.state)
