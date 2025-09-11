#!/usr/bin/env python

import rospy
import moveit_commander
from std_msgs.msg import String

class FetchBehavior:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        rospy.init_node('fetch_behavior_node', anonymous=True)

        # Arm + Gripper groups
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        # Subscribe to string commands (from API/LLM layer later)
        rospy.Subscriber("/fetch_behavior/command", String, self.callback)

        rospy.loginfo("FetchBehavior node ready.")

    def callback(self, msg):
        rospy.loginfo("Got command: %s", msg.data)

        if msg.data == "wave":
            self.wave()
        elif msg.data == "open_gripper":
            self.open_gripper()
        elif msg.data == "close_gripper":
            self.close_gripper()
        else:
            rospy.logwarn("Unknown command.")

    def wave(self):
        pose = self.arm.get_current_pose().pose
        pose.position.y += 0.2
        self.arm.set_pose_target(pose)
        self.arm.go(wait=True)
        rospy.sleep(1)
        pose.position.y -= 0.2
        self.arm.set_pose_target(pose)
        self.arm.go(wait=True)
        rospy.loginfo("Waved arm.")

    def open_gripper(self):
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)

    def close_gripper(self):
        self.gripper.set_named_target("close")
        self.gripper.go(wait=True)

if __name__ == "__main__":
    try:
        FetchBehavior()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
