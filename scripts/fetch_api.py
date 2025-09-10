#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class FetchAPI:
    def __init__(self):
        # Initialize ROS and MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('fetch_api', anonymous=True)

        # Arm + Gripper MoveIt groups
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        # Navigation client
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()

    # ---- Navigation ----
    def move_to(self, x, y, yaw=0.0):
        """Navigate base to (x, y, yaw) in map frame."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # simple, ignoring yaw for now

        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()
        return self.nav_client.get_result()

    # ---- Arm ----
    def move_arm_to_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.w = qw
        pose_target.orientation.x = qx
        pose_target.orientation.y = qy
        pose_target.orientation.z = qz

        self.arm.set_pose_target(pose_target)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def move_arm_joints(self, joint_values):
        self.arm.go(joint_values, wait=True)
        self.arm.stop()

    # ---- Gripper ----
    def open_gripper(self):
        self.gripper.set_joint_value_target([0.04, 0.04])
        self.gripper.go(wait=True)

    def close_gripper(self):
        self.gripper.set_joint_value_target([0.0, 0.0])
        self.gripper.go(wait=True)

    # ---- High-level ----
    def pick_object(self, pose):
        # Move above
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)
        # Lower
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z)
        # Grab
        self.close_gripper()
        # Lift
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)

    def place_object(self, pose):
        # Move above
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)
        # Lower
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z)
        # Release
        self.open_gripper()
        # Lift
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)
