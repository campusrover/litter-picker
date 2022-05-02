import math
from typing import Optional, List

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from litter_picker.msg import NavigationGoal
from trash_classes import trash_classes
import rospy


def read_waypoints(file_path: str) -> List[MoveBaseGoal]:
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
