#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"
#include "../include/iana_driver/Vector3.h"

using namespace Iana;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    ROS_INFO_STREAM("x: " << x << " y" << y << " z" << z << " w" << w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_rotate_test_node");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
        n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );
    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, odomCallback);
    ros::Rate rat(10);

    while(ros::ok())
    {
        ros::spinOnce();
        velocityChanger->PublishVelocity(Vector3::Zero, Vector3(0.5, 0.5, 0.5));
        rat.sleep();
    }
    return 0;

}
