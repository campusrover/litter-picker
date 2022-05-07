"""
Contains utility functions shared by all the other nodes/modules
"""

import json
from typing import Optional, List

import actionlib
from actionlib_msgs.msg import GoalStatus
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from trash_class import trash_classes

# required fields name for the json config file
COLLECTION_SITE_ORIENTATION_FIELD = "orientation"
COLLECTION_SITE_POSE_FIELD = "pose"
COLLECTION_SITE_FIELD = "collection_site"
WAYPOINTS_FIELD = "waypoints"


def read_config(file_path: str) -> (list, dict):
    """
    Read the json file that contains waypoints and collection site coordinate

    :param file_path: the filepath to the json config file

    :return: the set of waypoints coordinates as list, and the list of coordinate of the collection site
    """
    with open(file_path) as f:
        data = json.load(f)

    if WAYPOINTS_FIELD not in data:
        raise ValueError("expected waypoints field")
    if COLLECTION_SITE_FIELD not in data:
        raise ValueError("expected collection site")
    if COLLECTION_SITE_POSE_FIELD not in data[COLLECTION_SITE_FIELD]:
        raise ValueError("expected pose for collection_site")
    if COLLECTION_SITE_ORIENTATION_FIELD not in data[COLLECTION_SITE_FIELD]:
        raise ValueError("expected orientation for collection_site")

    return data[WAYPOINTS_FIELD], data[COLLECTION_SITE_FIELD]


def read_waypoints(waypoint_list: list) -> List[MoveBaseGoal]:
    """
    Convert the list of waypoint coordinates into a list of MoveBaseGoal

    :param waypoint_list: the list waypoints' coordinates

    :return: a list of MoveBaseGoal
    """
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
    """
    Convert the list of coordinate of the collection site to an instance of the MoveBaseGoal

    :param collection_site: the collection site coordinate

    :return: a MoveBaseGoal
    """

    goal = MoveBaseGoal()
    x, y, z = collection_site[COLLECTION_SITE_POSE_FIELD]
    o_x, o_y, o_z, o_w = collection_site[COLLECTION_SITE_ORIENTATION_FIELD]

    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z

    goal.target_pose.pose.orientation.x = o_x
    goal.target_pose.pose.orientation.y = o_y
    goal.target_pose.pose.orientation.z = o_z
    goal.target_pose.pose.orientation.w = o_w

    return goal


def get_first_bonding_box(boxes: BoundingBoxes) -> Optional[BoundingBox]:
    """
    Get the first bounding box that belongs to the trash class.

    :param boxes: the list of detected bounding boxes

    :return: the first box that belongs to the trash class. If no valid box can be found, return None
    """
    for box in boxes:
        if box.Class in trash_classes:
            return box
    return None


def navigate_to_waypoint(waypoint: MoveBaseGoal) -> bool:
    """
    Instantiate a move_base_client and the goal to the move_base actionlib server. If the task
    has been completed successfully, return True. Otherwise, return False

    :param waypoint: an instance of MoveBaseGoal

    :return: whether the task has been completed successfully or not
    """
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
