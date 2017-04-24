#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"
#include "../include/iana_driver/Vector3.h"
#include "../include/iana_driver/Quaternion.h"

using namespace Iana;

Vector3 direction = Vector3(0, 0, 0.5);

void ChangeDirection() { direction = direction.Negate(); }
bool TurningLeft() { return direction.Z() > 0; }
bool TurningRight() { return direction.Z() < 0; }

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    Quaternion q = Quaternion::FromMsg(msg->pose.pose.orientation);
    double roll, pitch, yawn;
    std::tie(roll, pitch, yawn) = q.ToEulerianAngle();
    ROS_DEBUG_STREAM("roll: " << roll << " pitch: " << pitch << " yawn: " << yawn);

    if (yawn > 180 && TurningLeft())
    {
        ChangeDirection();
    }
    if (yawn < 0 && TurningRight())
    {
        ChangeDirection();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_rotate_test_node");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
        n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );
    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, OdomCallback);
    ros::Rate rat(10);

    while(ros::ok())
    {
        ros::spinOnce();
        velocityChanger->PublishVelocity(Vector3::Zero, direction);
        rat.sleep();
    }
    return 0;

}
