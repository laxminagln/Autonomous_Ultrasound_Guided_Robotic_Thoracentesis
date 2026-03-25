#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def get_current_pose():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize([])
    rospy.init_node("get_panda_current_pose", anonymous=True)

    # Create MoveGroupCommander for panda_arm
    move_group = moveit_commander.MoveGroupCommander("panda_arm")

    # Get current pose of the end-effector
    current_pose = move_group.get_current_pose().pose

    # Print position
    rospy.loginfo("Current End-Effector Position:")
    rospy.loginfo(f"  x: {current_pose.position.x:.4f}")
    rospy.loginfo(f"  y: {current_pose.position.y:.4f}")
    rospy.loginfo(f"  z: {current_pose.position.z:.4f}")

    # Convert quaternion to RPY
    quat = current_pose.orientation
    quaternion = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    # Print orientation in RPY (radians)
    rospy.loginfo("Current End-Effector Orientation (RPY in radians):")
    rospy.loginfo(f"  Roll:  {roll:.4f}")
    rospy.loginfo(f"  Pitch: {pitch:.4f}")
    rospy.loginfo(f"  Yaw:   {yaw:.4f}")
if __name__ == "__main__":
    get_current_pose()
