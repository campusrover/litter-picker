import math
from typing import Optional, List

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from litter_picker.msg import NavigationGoal
from trash_classes import trash_classes
import rospy


def read_waypoints(file_path: str) -> List[NavigationGoal]:
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


def get_first_bonding_box(boxes: BoundingBoxes, box_id=None) -> Optional[BoundingBox]:
    for box in boxes:
        rospy.loginfo("found {} as bounding box".format(box.Class))
        if box.Class in trash_classes:
            if box_id is None:
                return box
            else:
                if box.id == box_id:
                    return box
                else:
                    return None
    return None


def dist_between_two(x_now, y_now, x_origin, y_origin):
    return math.sqrt((x_now - x_origin)**2 + (y_now - y_origin)**2)
