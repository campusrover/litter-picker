import json
from typing import Optional, List

import actionlib
from actionlib_msgs.msg import GoalStatus
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from trash_class import trash_classes

import rospy


def read_config(file_path: str) -> (list, dict):
    with open(file_path) as f:
        data = json.load(f)

    if "waypoints" not in data:
        raise ValueError("expected waypoints field")
    if "collection_site" not in data:
        raise ValueError("expected collection site")
    if "pose" not in data["collection_site"]:
        raise ValueError("expected pose for collection_site")
    if "orientation" not in data["collection_site"]:
        raise ValueError("expected orientation for collection_site")

    return data["waypoints"], data["collection_site"]


def read_waypoints(waypoint_list: list) -> List[MoveBaseGoal]:
    move_base_goals = []

    for waypoint in waypoint_list:
        if len(waypoint) != 3:
            raise ValueError("waypoint must contain x, y, and z coordinate")

        goal_pose = MoveBaseGoal()
        x, y, z = waypoint

        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = x
        goal_pose.target_pose.pose.position.y = y
        goal_pose.target_pose.pose.position.z = z

        # doesn't matter since, it's going to rotate for 360 degrees anyway
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = 0
        goal_pose.target_pose.pose.orientation.w = 1

        move_base_goals.append(goal_pose)

    return move_base_goals


def read_collection_site(collection_site: dict) -> MoveBaseGoal:
    # hardcoded: need to change so that it reads from a file

    goal = MoveBaseGoal()
    x, y, z = collection_site["pose"]
    o_x, o_y, o_z, o_w = collection_site["orientation"]

    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z

    goal.target_pose.pose.orientation.x = o_x
    goal.target_pose.pose.orientation.y = o_y
    goal.target_pose.pose.orientation.z = o_z
    goal.target_pose.pose.orientation.w = o_w

    return goal


def _process_waypoint(waypoint_str: str):
    x, y, z = waypoint_str.split(",")
    try:
        return float(x), float(y), float(z)
    except:
        rospy.ERROR("Fail to convert {}, {}, {} into float".format(x, y, z))
        raise ValueError("Fail to convert {}, {}, {} into float".format(x, y, z))


def get_first_bonding_box(boxes: BoundingBoxes) -> Optional[BoundingBox]:
    for box in boxes:
        if box.Class in trash_classes:
            return box
    return None


def navigate_to_waypoint(waypoint) -> bool:
    move_base_client = actionlib.SimpleActionClient(
        "move_base",
        MoveBaseAction,
    )
    move_base_client.wait_for_server()

    move_base_client.send_goal(waypoint)
    move_base_client.wait_for_result()
    if move_base_client.get_state() == GoalStatus.SUCCEEDED:
        return True
    else:
        return False
