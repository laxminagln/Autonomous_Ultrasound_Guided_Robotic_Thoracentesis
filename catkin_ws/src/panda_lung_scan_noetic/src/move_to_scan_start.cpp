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

static geometry_msgs::Pose vectorToPose(const std::vector<double>& pose_vec) {
  geometry_msgs::Pose pose;
  pose.position.x = pose_vec[0];
  pose.position.y = pose_vec[1];
  pose.position.z = pose_vec[2];
  pose.orientation.x = pose_vec[3];
  pose.orientation.y = pose_vec[4];
  pose.orientation.z = pose_vec[5];
  pose.orientation.w = pose_vec[6];
  return pose;
}

static bool planAndExecutePose(moveit::planning_interface::MoveGroupInterface& move_group,
                               const geometry_msgs::Pose& target_pose,
                               const std::string& label) {
  ROS_INFO_STREAM("Planning move to " << label << "...");

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveItErrorCode rc = move_group.plan(plan);

  if (rc != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR_STREAM("Failed to plan " << label << ".");
    return false;
  }

  ROS_INFO_STREAM("Press ENTER to move to " << label << "...");
  std::cin.get();

  rc = move_group.execute(plan);
  if (rc != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR_STREAM("Failed to execute motion to " << label << ".");
    return false;
  }

  move_group.stop();
  move_group.clearPoseTargets();

  ROS_INFO_STREAM("Reached " << label << " successfully.");
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_scan_start");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

  // panda_link0 = robot base frame
  // panda_link8 = end-effector frame
  move_group.setPoseReferenceFrame("panda_link0");
  move_group.setEndEffectorLink("panda_link8");

  // Match the Python logic as closely as possible
  move_group.setPlannerId("LIN");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setStartStateToCurrentState();

  double approach_velocity_scaling = 0.28;
  double approach_acceleration_scaling = 0.03;
  nh.param("approach_velocity_scaling", approach_velocity_scaling, approach_velocity_scaling);
  nh.param("approach_acceleration_scaling", approach_acceleration_scaling, approach_acceleration_scaling);

  move_group.setMaxVelocityScalingFactor(approach_velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(approach_acceleration_scaling);

  std::vector<double> home_pose_vec = getVectorParam(nh, "home_pose");
  std::vector<double> start_pose_vec = getVectorParam(nh, "start_pose");

  if (home_pose_vec.size() != 7) {
    ROS_ERROR("home_pose must contain 7 values: x y z qx qy qz qw");
    return 1;
  }

  if (start_pose_vec.size() != 7) {
    ROS_ERROR("start_pose must contain 7 values: x y z qx qy qz qw");
    return 1;
  }

  geometry_msgs::Pose home_pose = vectorToPose(home_pose_vec);
  geometry_msgs::Pose start_pose = vectorToPose(start_pose_vec);

  ROS_WARN("===== MOVE TO SCAN START SEQUENCE =====");
  ROS_WARN("This sequence will move the robot to HOME first, then to SCAN START.");
  ROS_WARN("Robot base frame is panda_link0. End-effector target frame is panda_link8.");
  ROS_WARN("Press ENTER to begin home-position planning...");
  std::cin.get();

  if (!planAndExecutePose(move_group, home_pose, "home pose")) {
    return 1;
  }

  if (!planAndExecutePose(move_group, start_pose, "scan start pose")) {
    return 1;
  }

  ROS_INFO("Robot has reached home pose first and then scan start pose.");
  ROS_INFO("It is now ready for the scan step.");
  return 0;
}