#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_preview");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("scan_path",1,true);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("scan_target",1);

    double scan_length = 0.06;
    double scan_speed = 0.01;

    nav_msgs::Path path;
    path.header.frame_id="panda_link0";

    int points=100;

    for(int i=0;i<points;i++)
    {
        geometry_msgs::PoseStamped p;

        p.header.frame_id="panda_link0";

        double t=(double)i/points;
        p.pose.position.x = 0.5;
        p.pose.position.y = (t-0.5)*scan_length;
        p.pose.position.z = 0.25;

        p.pose.orientation.w = 1.0;

        path.poses.push_back(p);
    }

    path_pub.publish(path);

    ros::Rate r(50);

    double t=0;

    while(ros::ok())
    {
        geometry_msgs::PoseStamped p;

        p.header.frame_id="panda_link0";

        double s = sin(t*scan_speed);

        p.pose.position.x = 0.5;
        p.pose.position.y = s*scan_length/2;
        p.pose.position.z = 0.25;

        p.pose.orientation.w=1.0;

        pose_pub.publish(p);

        t+=0.02;

        r.sleep();
    }

    return 0;
}
