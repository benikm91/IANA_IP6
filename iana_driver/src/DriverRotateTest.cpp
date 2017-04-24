#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h";
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"

// using namespace Iana;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_rotate_test_node");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
        n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );

    // ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, OdometryCallback(velocityChanger));
    ros::Rate rat(10);
    std::shared_ptr<const Action> action = std::shared_ptr<const Action>(new DriveForward(velocityChanger));

    while(ros::ok())
    {
        ros::spinOnce();
        // velocityPublisher->publish(twist);
        rat.sleep();
    }
    return 0;

}
