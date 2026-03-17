#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <XmlRpcValue.h>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

// ---------------- Utility ----------------
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

// ---------------- MAIN ----------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_moveit_real");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_WARN("===== SCAN NODE STARTED =====");
  ROS_WARN("If you are in RViz-only mode, this will only animate in MoveIt.");
  ROS_WARN("If you are connected to the real robot, ensure workspace is clear and E-stop is ready.");
  ROS_WARN("Press ENTER to continue...");
  std::cin.get();

  // ---------------- MoveIt Setup ----------------
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

  move_group.setPoseReferenceFrame("panda_link0");
  move_group.setEndEffectorLink("panda_link8");

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  // These affect the move-to-start phase more than the Cartesian playback.
  move_group.setMaxVelocityScalingFactor(0.01);
  move_group.setMaxAccelerationScalingFactor(0.01);

  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector: %s", move_group.getEndEffectorLink().c_str());

  // ---------------- Load Parameters ----------------
  int scan_axis = 1;
  int lateral_axis = 0;
  int contact_axis = 2;

  double scan_length = 0.04;     // meters
  double scan_speed = 0.002;     // kept for parameter compatibility / optional waypoint density logic
  double scan_cycles = 1.0;
  double contact_offset = -0.002;
  double lateral_offset = 0.0;

  // Retiming scalars: these are what actually slow down the Cartesian trajectory.
  double cartesian_velocity_scaling = 0.02;
  double cartesian_acceleration_scaling = 0.02;

  nh.param("scan_axis", scan_axis, scan_axis);
  nh.param("lateral_axis", lateral_axis, lateral_axis);
  nh.param("contact_axis", contact_axis, contact_axis);
  nh.param("scan_length", scan_length, scan_length);
  nh.param("scan_speed", scan_speed, scan_speed);
  nh.param("scan_cycles", scan_cycles, scan_cycles);
  nh.param("contact_offset", contact_offset, contact_offset);
  nh.param("lateral_offset", lateral_offset, lateral_offset);
  nh.param("cartesian_velocity_scaling", cartesian_velocity_scaling, cartesian_velocity_scaling);
  nh.param("cartesian_acceleration_scaling", cartesian_acceleration_scaling, cartesian_acceleration_scaling);

  // Clamp retiming values to safe valid range
  cartesian_velocity_scaling = std::max(0.001, std::min(1.0, cartesian_velocity_scaling));
  cartesian_acceleration_scaling = std::max(0.001, std::min(1.0, cartesian_acceleration_scaling));

  ROS_INFO("scan_axis: %d | lateral_axis: %d | contact_axis: %d", scan_axis, lateral_axis, contact_axis);
  ROS_INFO("scan_length: %.4f m | scan_speed(param): %.6f | scan_cycles: %.1f", scan_length, scan_speed, scan_cycles);
  ROS_INFO("contact_offset: %.4f m | lateral_offset: %.4f m", contact_offset, lateral_offset);
  ROS_INFO("cartesian_velocity_scaling: %.4f | cartesian_acceleration_scaling: %.4f",
           cartesian_velocity_scaling, cartesian_acceleration_scaling);

  // ---------------- Start Pose ----------------
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

  // ---------------- Move to Start ----------------
  ROS_INFO("Planning move to scan start pose...");
  move_group.setPoseTarget(start_pose);

  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  if (move_group.plan(start_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to plan start pose");
    return 1;
  }

  ROS_INFO("Press ENTER to move to start pose...");
  std::cin.get();

  if (move_group.execute(start_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to execute move to start pose");
    return 1;
  }

  ros::Duration(1.0).sleep();

  // ---------------- Generate Scan Waypoints ----------------
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose p = start_pose;

  double* pos[3] = {&p.position.x, &p.position.y, &p.position.z};

  // Apply fixed offsets before scanning
  *pos[lateral_axis] += lateral_offset;
  *pos[contact_axis] += contact_offset;

  // First waypoint: settled contact pose
  waypoints.push_back(p);

  // Keep waypoint count moderate and stable
  int half_points = 30;
  int total_cycles = std::max(1, static_cast<int>(scan_cycles));

  for (int c = 0; c < total_cycles; ++c) {
    // Forward pass
    for (int i = 0; i <= half_points; ++i) {
      geometry_msgs::Pose wp = p;
      double alpha = static_cast<double>(i) / static_cast<double>(half_points);
      double offset = -0.5 * scan_length + alpha * scan_length;

      double* wppos[3] = {&wp.position.x, &wp.position.y, &wp.position.z};
      *wppos[scan_axis] += offset;

      waypoints.push_back(wp);
    }

    // Backward pass
    for (int i = 0; i <= half_points; ++i) {
      geometry_msgs::Pose wp = p;
      double alpha = static_cast<double>(i) / static_cast<double>(half_points);
      double offset = 0.5 * scan_length - alpha * scan_length;

      double* wppos[3] = {&wp.position.x, &wp.position.y, &wp.position.z};
      *wppos[scan_axis] += offset;

      waypoints.push_back(wp);
    }
  }

  ROS_INFO("Generated %zu scan waypoints.", waypoints.size());

  // ---------------- Cartesian Path ----------------
  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.005;
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory);

  ROS_INFO("Cartesian path success: %.2f%%", fraction * 100.0);

  if (fraction < 0.8) {
    ROS_WARN("Path incomplete. Adjust start pose, orientation, scan axis, or reduce scan length.");
  }

  // ---------------- Retime Cartesian Trajectory ----------------
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  if (!current_state) {
    ROS_ERROR("Failed to get current robot state for trajectory retiming.");
    return 1;
  }

  robot_trajectory::RobotTrajectory rt(current_state->getRobotModel(), "panda_arm");
  rt.setRobotTrajectoryMsg(*current_state, trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool retime_success = iptp.computeTimeStamps(
      rt,
      cartesian_velocity_scaling,
      cartesian_acceleration_scaling);

  if (!retime_success) {
    ROS_WARN("Failed to retime Cartesian trajectory. Executing unretimed path.");
  } else {
    rt.getRobotTrajectoryMsg(trajectory);
    ROS_INFO("Cartesian trajectory successfully retimed.");
  }

  moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory_ = trajectory;

  // ---------------- Final Safety ----------------
  ROS_WARN("READY TO EXECUTE SCAN");
  ROS_WARN("Press ENTER to start scan...");
  std::cin.get();

  if (move_group.execute(cart_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to execute scan trajectory.");
    return 1;
  }

  ROS_INFO("Scan complete.");
  ros::waitForShutdown();
  return 0;
}