#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <XmlRpcValue.h>
#include <vector>
#include <string>

static std::vector<double> getVectorParam(ros::NodeHandle& nh, const std::string& name) {
  std::vector<double> out;
  XmlRpc::XmlRpcValue v;
  if (nh.getParam(name, v) && v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < v.size(); ++i) {
      if (v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || v[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        out.push_back(static_cast<double>(v[i]));
      }
    }
  }
  return out;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_moveit_preview");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("Press ENTER in terminal to start scan...");
  std::cin.get();

  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
  move_group.setPoseReferenceFrame("panda_link0");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  int scan_axis = 1;
  int lateral_axis = 0;
  int contact_axis = 2;
  double settle_time = 2.0;
  double scan_length = 0.06;
  double scan_speed = 0.01;
  double scan_cycles = 2.0;
  double contact_offset = -0.004;
  double lateral_offset = 0.0;

  nh.param("scan_axis", scan_axis, scan_axis);
  nh.param("lateral_axis", lateral_axis, lateral_axis);
  nh.param("contact_axis", contact_axis, contact_axis);
  nh.param("settle_time", settle_time, settle_time);
  nh.param("scan_length", scan_length, scan_length);
  nh.param("scan_speed", scan_speed, scan_speed);
  nh.param("scan_cycles", scan_cycles, scan_cycles);
  nh.param("contact_offset", contact_offset, contact_offset);
  nh.param("lateral_offset", lateral_offset, lateral_offset);

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

  ROS_INFO("Moving to scan start pose...");
  move_group.setPoseTarget(start_pose);
  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  bool ok = (move_group.plan(start_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!ok) {
    ROS_ERROR("Failed to plan to start pose.");
    return 1;
  }
  move_group.execute(start_plan);
  ros::Duration(1.0).sleep();

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose p = start_pose;

  // Apply lateral and contact offsets
  double* pos[3] = {&p.position.x, &p.position.y, &p.position.z};
  *pos[lateral_axis] += lateral_offset;
  *pos[contact_axis] += contact_offset;

  // Settle waypoint
  waypoints.push_back(p);

  int half_points = std::max(10, static_cast<int>((scan_length / std::max(0.001, scan_speed)) * 20.0));
  int total_cycles = std::max(1, static_cast<int>(scan_cycles));

  for (int c = 0; c < total_cycles; ++c) {
    // Forward
    for (int i = 0; i <= half_points; ++i) {
      geometry_msgs::Pose wp = p;
      double alpha = static_cast<double>(i) / static_cast<double>(half_points);
      double line_offset = -0.5 * scan_length + alpha * scan_length;
      double* wppos[3] = {&wp.position.x, &wp.position.y, &wp.position.z};
      *wppos[scan_axis] += line_offset;
      waypoints.push_back(wp);
    }
    // Backward
    for (int i = 0; i <= half_points; ++i) {
      geometry_msgs::Pose wp = p;
      double alpha = static_cast<double>(i) / static_cast<double>(half_points);
      double line_offset = 0.5 * scan_length - alpha * scan_length;
      double* wppos[3] = {&wp.position.x, &wp.position.y, &wp.position.z};
      *wppos[scan_axis] += line_offset;
      waypoints.push_back(wp);
    }
  }

  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.005;
  const double jump_threshold = 0.0;

  ROS_INFO("Computing Cartesian scan path...");
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  ROS_INFO_STREAM("Cartesian path fraction: " << fraction);

  if (fraction < 0.8) {
    ROS_WARN("Low Cartesian path fraction. Check start pose, orientation, or scan axis.");
  }

  moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory_ = trajectory;

  ROS_INFO("Executing scan path in RViz/fake controller...");
  move_group.execute(cart_plan);

  ROS_INFO("Scan preview complete.");
  ros::waitForShutdown();
  return 0;
}