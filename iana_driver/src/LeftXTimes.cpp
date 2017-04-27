#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/iana_driver/DriverPlan.h"
#include "../include/iana_driver/VelocityChanger.h"
#include <math.h>

using namespace Iana;

int numOfRuns = 5;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "left_x_times");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
            n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );

    std::queue<std::shared_ptr<PlanAction>> actions;
    actions.push(std::make_shared<DrivingForwardAction>(0));
    actions.push(std::make_shared<TurningAction>(2*M_PI * numOfRuns, TurningAction::Left));

    DriverPlan driverPlan(
            velocityChanger,
            actions
    );

    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &DriverPlan::Update, &driverPlan);

    ros::spin();

}
