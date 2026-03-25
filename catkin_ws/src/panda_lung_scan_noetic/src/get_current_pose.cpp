#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "get_current_pose");
  ros::NodeHandle nh("~");

  std::string base_frame = "panda_link0";
  std::string ee_frame = "panda_link8";
  double rate_hz = 2.0;

  nh.param("base_frame", base_frame, base_frame);
  nh.param("ee_frame", ee_frame, ee_frame);
  nh.param("rate", rate_hz, rate_hz);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Duration(1.0).sleep();

  ros::Rate rate(rate_hz);

  while (ros::ok()) {
    try {
      geometry_msgs::TransformStamped tf_stamped =
          tf_buffer.lookupTransform(base_frame, ee_frame, ros::Time(0), ros::Duration(1.0));

      const auto& t = tf_stamped.transform.translation;
      const auto& q = tf_stamped.transform.rotation;

      ROS_INFO_STREAM("\nCurrent pose of " << ee_frame << " in " << base_frame << " frame:"
                      << "\n  position:    x = " << t.x
                      << ", y = " << t.y
                      << ", z = " << t.z
                      << "\n  orientation: qx = " << q.x
                      << ", qy = " << q.y
                      << ", qz = " << q.z
                      << ", qw = " << q.w);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(2.0, "Could not get transform from "
                                        << base_frame << " to " << ee_frame
                                        << ": " << ex.what());
    }

    rate.sleep();
  }

  return 0;
}