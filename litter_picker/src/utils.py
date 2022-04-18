from litter_picker.msg import NavigationGoal
import rospy


def read_waypoints(file_path: str):
    waypoints_list = []

    with open(file_path) as f:
        waypoints = f.readlines()
        for waypoint in waypoints:
            goal_pose = NavigationGoal()
            x, y, z = _process_waypoint(waypoint)
            goal_pose.x, goal_pose.y, goal_pose.z = x, y, z
            goal_pose.a = 0

            waypoints_list.append(goal_pose)

    return waypoints_list


def _process_waypoint(waypoint_str: str):
    x, y, z = waypoint_str.split(",")
    try:
        return float(x), float(y), float(z)
    except:
        rospy.ERROR("Fail to convert {}, {}, {} into float".format(x, y, z))
        raise ValueError("Fail to convert {}, {}, {} into float".format(x, y, z))
