#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"
#include "../include/iana_driver/Vector3.h"
#include "../include/iana_driver/Quaternion.h"
#include <math.h>

using namespace Iana;

Vector3 direction = Vector3(0, 0, 0.5);

void ChangeDirection() { direction = direction.Negate(); }
bool TurningLeft() { return direction.Z() > 0; }
bool TurningRight() { return direction.Z() < 0; }

double currentYawn;

double oldYawn = -10;

int numOfRuns = 5 * 2;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (numOfRuns <= 0) 
    {
        direction = Vector3::Zero;
        return;
    }
    Quaternion q = Quaternion::FromMsg(msg->pose.pose.orientation);
    double roll, pitch, yawn;
    std::tie(roll, pitch, yawn) = q.ToEulerianAngle();
    ROS_WARN_STREAM("roll: " << roll << " pitch: " << pitch << " yawn: " << yawn);
    yawn += M_PI;
    ROS_WARN_STREAM("left: " << TurningLeft() << " RIGHT: " << TurningRight());
    if (oldYawn == -10) 
    {
        oldYawn = yawn;
        return;
    }
    double deltaYawn = std::abs(yawn - oldYawn);
    if (deltaYawn > M_PI) deltaYawn = std::fmin(yawn, oldYawn) + 2*M_PI - std::fmax(yawn, oldYawn);
    currentYawn += deltaYawn;
    if (currentYawn >= numOfRuns * M_PI) 
    {
        direction = Vector3::Zero;
        numOfRuns = -1;
    }
    /*if (currentYawn >=  M_PI) 
    {
        currentYawn = - (currentYawn - M_PI);
        ChangeDirection();
        numOfRuns--;
        if (numOfRuns == 0) direction = Vector3::Zero;
    }*/
    oldYawn = yawn;
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
