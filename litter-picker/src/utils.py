from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy


def read_waypoints(file_path: str):
    waypoints_list = []

    with open(file_path) as f:
        waypoints = f.readlines()
        for waypoint in waypoints:
            goal_pose = MoveBaseGoal()
            x, y, z = _process_waypoint(waypoint)

            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.pose.position.x = x
            goal_pose.target_pose.pose.position.y = y
            goal_pose.target_pose.pose.position.z = z

            # doesn't matter since, it's going to rotate for 360 degrees anyway
            goal_pose.target_pose.pose.orientation.x = 0
            goal_pose.target_pose.pose.orientation.y = 0
            goal_pose.target_pose.pose.orientation.z = 0
            goal_pose.target_pose.pose.orientation.w = 1
            waypoints_list.append(goal_pose)

    return waypoints_list


def _process_waypoint(waypoint_str: str):
    x, y, z = waypoint_str.split(",")
    try:
        return float(x), float(y), float(z)
    except:
        rospy.ERROR("Fail to convert {}, {}, {} into float".format(x, y, z))
        raise ValueError(
            "Fail to convert {}, {}, {} into float".format(x, y, z))


def get_state_topic():
    return rospy.Publisher('state')
