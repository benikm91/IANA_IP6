#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/iana_driver/DriverPlan.h"
#include "../include/iana_driver/VelocityChanger.h"
#include <math.h>

using namespace Iana;

int meters = 1.00;

bool startPositionSet = false;
Vector3 startPosition = Vector3::Zero;

bool driverPlanDone = false;

void FinalVerdict(const nav_msgs::Odometry::ConstPtr &msg) {
    Vector3 position = Vector3::FromMsg(msg->pose.pose.position);
    if (!startPositionSet)
    {
        startPosition = position;
        startPositionSet = true;
        return;
    }
    if (driverPlanDone)
    {
        driverPlanDone = false;
        ROS_INFO_STREAM("Final distance: " << (startPosition - position).Length());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_x_meters");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
            n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );

    std::queue<std::shared_ptr<PlanAction>> actions;
    actions.push(std::make_shared<DrivingForwardAction>(0));
    actions.push(std::make_shared<DrivingForwardAction>(meters));

    DriverPlan driverPlan(
        velocityChanger,
        actions
    );

    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &DriverPlan::Update, &driverPlan);
    ros::Subscriber sub2 = n.subscribe<nav_msgs::Odometry>("/odom", 1000, FinalVerdict);

    ros::Rate rat(1);
    while(ros::ok()) {
        ros::spinOnce();
        driverPlanDone = driverPlan.Done();
        rat.sleep();
    }

}
