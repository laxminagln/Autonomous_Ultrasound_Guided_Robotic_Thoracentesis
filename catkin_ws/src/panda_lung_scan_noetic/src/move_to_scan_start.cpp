#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <XmlRpcValue.h>

#include <vector>
#include <string>
#include <iostream>

static std::vector<double> getVectorParam(ros::NodeHandle& nh, const std::string& name) {
  std::vector<double> out;
  XmlRpc::XmlRpcValue v;
  if (nh.getParam(name, v) && v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < v.size(); ++i) {
      if (v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
          v[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        out.push_back(static_cast<double>(v[i]));
      }
    }
  }
  return out;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_scan_start");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
  move_group.setPoseReferenceFrame("panda_link0");
  move_group.setEndEffectorLink("panda_link8");
  move_group.setPlannerId("PTP");
  move_group.setStartStateToCurrentState();

  move_group.setGoalJointTolerance(0.05);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.05);

  double approach_velocity_scaling = 0.03;
  double approach_acceleration_scaling = 0.03;
  nh.param("approach_velocity_scaling", approach_velocity_scaling, approach_velocity_scaling);
  nh.param("approach_acceleration_scaling", approach_acceleration_scaling, approach_acceleration_scaling);

  move_group.setMaxVelocityScalingFactor(approach_velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(approach_acceleration_scaling);

  std::vector<double> start_pose_vec = getVectorParam(nh, "start_pose");
  if (start_pose_vec.size() != 7) {
    ROS_ERROR("start_pose must contain 7 values: x y z qx qy qz qw");
    return 1;
  }

  geometry_msgs::Pose start_pose;
  start_pose.position.x = start_pose_vec[0];
  start_pose.position.y = start_pose_vec[1];
  start_pose.position.z = start_pose_vec[2];
  start_pose.orientation.x = start_pose_vec[3];
  start_pose.orientation.y = start_pose_vec[4];
  start_pose.orientation.z = start_pose_vec[5];
  start_pose.orientation.w = start_pose_vec[6];

  ROS_INFO("Planning move to scan start pose...");
  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(start_pose);

  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  if (move_group.plan(start_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to plan start pose.");
    return 1;
  }

  ROS_INFO("Press ENTER to move to scan start pose...");
  std::cin.get();

  if (move_group.execute(start_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to execute start pose motion.");
    return 1;
  }

  ROS_INFO("Reached scan start pose successfully.");
  return 0;
}