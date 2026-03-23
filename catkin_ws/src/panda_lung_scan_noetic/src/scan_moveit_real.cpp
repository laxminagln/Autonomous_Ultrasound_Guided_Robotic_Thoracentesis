#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <XmlRpcValue.h>

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>

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

static double clampDouble(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

static void setAxisValue(geometry_msgs::Pose& pose, int axis, double value) {
  if (axis == 0) {
    pose.position.x = value;
  } else if (axis == 1) {
    pose.position.y = value;
  } else {
    pose.position.z = value;
  }
}

static double getAxisValue(const geometry_msgs::Pose& pose, int axis) {
  if (axis == 0) {
    return pose.position.x;
  } else if (axis == 1) {
    return pose.position.y;
  } else {
    return pose.position.z;
  }
}

static void appendInterpolatedSegment(
    const geometry_msgs::Pose& from,
    const geometry_msgs::Pose& to,
    int num_points,
    std::vector<geometry_msgs::Pose>& out,
    bool skip_first = true) {
  int points = std::max(1, num_points);

  for (int i = 0; i <= points; ++i) {
    if (skip_first && i == 0) {
      continue;
    }

    double alpha = static_cast<double>(i) / static_cast<double>(points);
    geometry_msgs::Pose p = from;

    p.position.x = from.position.x + alpha * (to.position.x - from.position.x);
    p.position.y = from.position.y + alpha * (to.position.y - from.position.y);
    p.position.z = from.position.z + alpha * (to.position.z - from.position.z);

    // Keep fixed orientation for the whole scan
    p.orientation = from.orientation;

    out.push_back(p);
  }
}

static double curveZOffset(double x_local, double curve_half_width, double curve_drop) {
  if (curve_half_width <= 1e-6) {
    return 0.0;
  }
  double ratio = x_local / curve_half_width;
  return -curve_drop * ratio * ratio;
}

static std::vector<double> buildPassCenters(int num_passes, double curve_half_width, double coverage_ratio) {
  std::vector<double> centers;
  int passes = std::max(1, num_passes);
  double ratio = clampDouble(coverage_ratio, 0.0, 1.0);
  double usable_half_width = ratio * curve_half_width;

  if (passes == 1) {
    centers.push_back(0.0);
    return centers;
  }

  double start = -usable_half_width;
  double end = usable_half_width;
  double step = (end - start) / static_cast<double>(passes - 1);

  for (int i = 0; i < passes; ++i) {
    centers.push_back(start + static_cast<double>(i) * step);
  }
  return centers;
}

// ---------------- MAIN ----------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_moveit_real");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_WARN("===== CURVED ULTRASOUND SCAN NODE STARTED =====");
  ROS_WARN("In RViz simulation, this animates the full curved 4-pass scan.");
  ROS_WARN("On the real robot, ensure workspace is clear and E-stop is ready.");
  ROS_WARN("Press ENTER to continue...");
  std::cin.get();

  // ---------------- MoveIt Setup ----------------
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

  move_group.setPoseReferenceFrame("panda_link0");
  move_group.setEndEffectorLink("panda_link8");

  move_group.setPlannerId("PTP");
  move_group.setStartStateToCurrentState();

  // Keep the execution robustness fixes
  move_group.setGoalJointTolerance(0.05);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.05);

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector: %s", move_group.getEndEffectorLink().c_str());

  // ---------------- Load Parameters ----------------
  int scan_axis = 1;
  int lateral_axis = 0;
  int contact_axis = 2;

  double settle_time = 2.0;

  // Updated measurements
  double scan_length = 0.145;      // 14.5 cm straight direction
  double scan_width = 0.105;       // 10.5 cm curved direction
  int num_passes = 4;
  double scan_cycles = 1.0;

  double curve_half_width = 0.0525; // half of 10.5 cm
  double curve_drop = 0.010;        // 7.5 cm center vs 6.5 cm edges
  double pass_coverage_ratio = 0.75;

  double scan_speed = 0.003;      // informational / compatibility only
  double contact_offset = 0.0;    // safe default for RViz
  double lateral_offset = 0.0;

  int connector_points = 12;
  double eef_step = 0.003;

  // Scan timing (kept as before)
  double cartesian_velocity_scaling = 0.01;
  double cartesian_acceleration_scaling = 0.01;

  // Faster motion for approach and return
  double approach_velocity_scaling = 0.05;
  double approach_acceleration_scaling = 0.05;
  double return_velocity_scaling = 0.05;
  double return_acceleration_scaling = 0.05;

  nh.param("scan_axis", scan_axis, scan_axis);
  nh.param("lateral_axis", lateral_axis, lateral_axis);
  nh.param("contact_axis", contact_axis, contact_axis);

  nh.param("settle_time", settle_time, settle_time);
  nh.param("scan_length", scan_length, scan_length);
  nh.param("scan_width", scan_width, scan_width);
  nh.param("num_passes", num_passes, num_passes);
  nh.param("scan_cycles", scan_cycles, scan_cycles);

  nh.param("curve_half_width", curve_half_width, curve_half_width);
  nh.param("curve_drop", curve_drop, curve_drop);
  nh.param("pass_coverage_ratio", pass_coverage_ratio, pass_coverage_ratio);

  nh.param("scan_speed", scan_speed, scan_speed);
  nh.param("contact_offset", contact_offset, contact_offset);
  nh.param("lateral_offset", lateral_offset, lateral_offset);

  nh.param("connector_points", connector_points, connector_points);
  nh.param("eef_step", eef_step, eef_step);

  nh.param("cartesian_velocity_scaling", cartesian_velocity_scaling, cartesian_velocity_scaling);
  nh.param("cartesian_acceleration_scaling", cartesian_acceleration_scaling, cartesian_acceleration_scaling);

  nh.param("approach_velocity_scaling", approach_velocity_scaling, approach_velocity_scaling);
  nh.param("approach_acceleration_scaling", approach_acceleration_scaling, approach_acceleration_scaling);
  nh.param("return_velocity_scaling", return_velocity_scaling, return_velocity_scaling);
  nh.param("return_acceleration_scaling", return_acceleration_scaling, return_acceleration_scaling);

  cartesian_velocity_scaling = clampDouble(cartesian_velocity_scaling, 0.001, 1.0);
  cartesian_acceleration_scaling = clampDouble(cartesian_acceleration_scaling, 0.001, 1.0);
  approach_velocity_scaling = clampDouble(approach_velocity_scaling, 0.001, 1.0);
  approach_acceleration_scaling = clampDouble(approach_acceleration_scaling, 0.001, 1.0);
  return_velocity_scaling = clampDouble(return_velocity_scaling, 0.001, 1.0);
  return_acceleration_scaling = clampDouble(return_acceleration_scaling, 0.001, 1.0);

  pass_coverage_ratio = clampDouble(pass_coverage_ratio, 0.0, 1.0);
  num_passes = std::max(1, num_passes);
  connector_points = std::max(2, connector_points);
  eef_step = std::max(0.001, eef_step);

  ROS_INFO("scan_axis=%d lateral_axis=%d contact_axis=%d", scan_axis, lateral_axis, contact_axis);
  ROS_INFO("scan_length=%.4f m scan_width=%.4f m num_passes=%d",
           scan_length, scan_width, num_passes);
  ROS_INFO("curve_half_width=%.4f m curve_drop=%.4f m coverage=%.2f",
           curve_half_width, curve_drop, pass_coverage_ratio);
  ROS_INFO("contact_offset=%.4f m lateral_offset=%.4f m",
           contact_offset, lateral_offset);
  ROS_INFO("cartesian_velocity_scaling=%.4f cartesian_acceleration_scaling=%.4f",
           cartesian_velocity_scaling, cartesian_acceleration_scaling);
  ROS_INFO("approach_velocity_scaling=%.4f approach_acceleration_scaling=%.4f",
           approach_velocity_scaling, approach_acceleration_scaling);
  ROS_INFO("return_velocity_scaling=%.4f return_acceleration_scaling=%.4f",
           return_velocity_scaling, return_acceleration_scaling);

  // Debug: print current joint values
  std::vector<double> joints = move_group.getCurrentJointValues();
  for (size_t i = 0; i < joints.size(); ++i) {
    ROS_INFO("Current joint %zu: %f", i + 1, joints[i]);
  }

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
  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(start_pose);

  // Faster approach move
  move_group.setMaxVelocityScalingFactor(approach_velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(approach_acceleration_scaling);

  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  if (move_group.plan(start_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to plan start pose.");
    return 1;
  }

  ROS_INFO("Press ENTER to move to start pose...");
  std::cin.get();

  ros::Duration(1.0).sleep();

  if (move_group.execute(start_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to execute start pose motion.");
    return 1;
  }

  ROS_INFO("Settling for %.2f seconds...", settle_time);
  ros::Duration(settle_time).sleep();

  // ---------------- Build Curved 4-Pass Serpentine ----------------
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<double> pass_centers = buildPassCenters(num_passes, curve_half_width, pass_coverage_ratio);

  ROS_INFO("Pass centers across curved width:");
  for (size_t i = 0; i < pass_centers.size(); ++i) {
    double z_rel = curveZOffset(pass_centers[i], curve_half_width, curve_drop);
    ROS_INFO("  pass %zu: x_local = %.4f m, z_curve_offset = %.4f m", i + 1, pass_centers[i], z_rel);
  }

  geometry_msgs::Pose current_pose = start_pose;
  waypoints.push_back(current_pose);

  const double half_scan_length = 0.5 * scan_length;
  const int line_points = std::max(20, static_cast<int>(std::ceil(scan_length / eef_step)));
  const int total_cycles = std::max(1, static_cast<int>(scan_cycles));

  for (int cycle = 0; cycle < total_cycles; ++cycle) {
    for (int pass = 0; pass < num_passes; ++pass) {
      bool forward = (pass % 2 == 0);

      double x_local = pass_centers[pass];
      double x_value = lateral_offset + x_local;
      double z_value = start_pose.position.z + contact_offset + curveZOffset(x_local, curve_half_width, curve_drop);

      double y_start_local = forward ? -half_scan_length : half_scan_length;
      double y_end_local   = forward ?  half_scan_length : -half_scan_length;

      geometry_msgs::Pose pass_start = current_pose;
      geometry_msgs::Pose pass_end = current_pose;

      setAxisValue(pass_start, lateral_axis, getAxisValue(start_pose, lateral_axis) + x_value);
      setAxisValue(pass_start, scan_axis, getAxisValue(start_pose, scan_axis) + y_start_local);
      setAxisValue(pass_start, contact_axis, z_value);

      setAxisValue(pass_end, lateral_axis, getAxisValue(start_pose, lateral_axis) + x_value);
      setAxisValue(pass_end, scan_axis, getAxisValue(start_pose, scan_axis) + y_end_local);
      setAxisValue(pass_end, contact_axis, z_value);

      // Smooth connector from current pose to start of this pass
      appendInterpolatedSegment(current_pose, pass_start, connector_points, waypoints, true);

      // Longitudinal scan for this pass
      appendInterpolatedSegment(pass_start, pass_end, line_points, waypoints, true);

      current_pose = pass_end;
    }
  }

  ROS_INFO("Generated %zu waypoints for curved serpentine scan.", waypoints.size());

  // ---------------- Cartesian Path ----------------
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory);

  ROS_INFO("Cartesian path success: %.2f%%", fraction * 100.0);

  if (fraction < 0.8) {
    ROS_WARN("Low Cartesian path fraction. Check start_pose, orientation, dimensions, or reduce scan_length/scan_width.");
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
    ROS_WARN("Failed to retime trajectory. Executing unretimed path.");
  } else {
    rt.getRobotTrajectoryMsg(trajectory);
    ROS_INFO("Cartesian trajectory retimed successfully.");
  }

  // ---------------- Ensure strictly increasing timestamps ----------------
  double last_time = 0.0;
  for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    double t = trajectory.joint_trajectory.points[i].time_from_start.toSec();

    if (t <= last_time) {
      t = last_time + 1e-4;  // enforce strictly increasing times
      trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(t);
    }

    last_time = t;
  }

  moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory_ = trajectory;

  // ---------------- Final Safety ----------------
  ROS_WARN("READY TO EXECUTE CURVED %d-PASS SCAN", num_passes);
  ROS_WARN("Press ENTER to start scan...");
  std::cin.get();

  if (move_group.execute(cart_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to execute scan trajectory.");
    return 1;
  }

  ROS_INFO("Curved scan complete. Returning to start pose...");

  // ---------------- Return to Start Pose ----------------
  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(start_pose);

  move_group.setMaxVelocityScalingFactor(return_velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(return_acceleration_scaling);

  moveit::planning_interface::MoveGroupInterface::Plan return_plan;
  if (move_group.plan(return_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to plan return-to-start motion.");
    return 1;
  }

  ros::Duration(0.5).sleep();

  if (move_group.execute(return_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Failed to execute return-to-start motion.");
    return 1;
  }

  ROS_INFO("Returned to start pose successfully.");
  ros::waitForShutdown();
  return 0;
}