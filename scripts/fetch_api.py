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

        # Arm + Gripper groups
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        # Navigation client
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()

    # -------- Navigation --------
    def move_to(self, x, y, yaw=0.0):
        """Navigate base to (x, y, yaw) in map frame."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # simple, could convert yaw→quat

        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()
        return self.nav_client.get_result()

    # -------- Arm control --------
    def move_arm_to_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """Move arm end-effector to a Cartesian pose."""
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
        """Directly set arm joint values (list of 7)."""
        self.arm.go(joint_values, wait=True)
        self.arm.stop()

    # -------- Gripper control --------
    def open_gripper(self):
        self.gripper.set_joint_value_target([0.04, 0.04])  # open fingers
        self.gripper.go(wait=True)

    def close_gripper(self):
        self.gripper.set_joint_value_target([0.0, 0.0])  # close fingers
        self.gripper.go(wait=True)

    # -------- High-level manipulation --------
    def pick_object(self, pose):
        """Example: move above object, lower, close gripper."""
        # move above
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)
        # lower
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z)
        # grab
        self.close_gripper()
        # lift
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)

    def place_object(self, pose):
        """Move to target, lower, open gripper."""
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z)
        self.open_gripper()
        self.move_arm_to_pose(pose.position.x, pose.position.y, pose.position.z + 0.1)
