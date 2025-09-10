#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from fetch_api import FetchAPI

if __name__ == "__main__":
    robot = FetchAPI()

    # Move base
    robot.move_to(1.0, 0.0)

    # Define a "cup" pose
    cup_pose = geometry_msgs.msg.Pose()
    cup_pose.position.x = 0.6
    cup_pose.position.y = 0.0
    cup_pose.position.z = 0.9
    cup_pose.orientation.w = 1.0

    # Pick + place
    robot.pick_object(cup_pose)

    shelf_pose = geometry_msgs.msg.Pose()
    shelf_pose.position.x = 0.8
    shelf_pose.position.y = -0.2
    shelf_pose.position.z = 1.0
    shelf_pose.orientation.w = 1.0

    robot.place_object(shelf_pose)
