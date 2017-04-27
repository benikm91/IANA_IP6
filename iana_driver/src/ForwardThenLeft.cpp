#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/iana_driver/DriverPlan.h"
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"
#include "../include/iana_driver/Vector3.h"
#include "../include/iana_driver/Quaternion.h"
#include <memory.h>
#include <math.h>
#include <queue>

using namespace Iana;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_plan");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
            n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );

    std::queue<std::shared_ptr<PlanAction>> actions;
    actions.push(std::make_shared<DrivingForwardAction>(0)); // needed for initializing next action correctly. TODO get ride of this :)
    actions.push(std::make_shared<DrivingForwardAction>(0.4));
    actions.push(std::make_shared<TurningAction>(M_PI, TurningAction::Left));

    DriverPlan driverPlan(
        velocityChanger,
        actions
    );

    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &DriverPlan::Update, &driverPlan);

    ros::spin();

    return 0;

}
