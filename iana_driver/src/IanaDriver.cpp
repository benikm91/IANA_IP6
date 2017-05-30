#include <memory>
#include "ros/ros.h"
#include "iana_driver/NoDriverArgs.h"
#include "iana_driver/RandomDriverArgs.h"
#include "../include/driver/Driver.h"
#include "../include/driver/NullDriver.h"
#include "../include/driver/RandomDriver.h"


std::shared_ptr<Iana::Driver> driver;

void SetNoDriverCallback(const iana_driver::NoDriverArgs::ConstPtr& msg)
{
    driver = std::make_shared<Iana::NullDriver>();
}

void SetRandomDriverCallback(const iana_driver::RandomDriverArgs::ConstPtr& msg)
{
    driver = std::make_shared<Iana::RandomDriver>();
}

int main(int argc, char **argv)
{
    driver = std::make_shared<Iana::NullDriver>();

    ros::init(argc, argv, "iana_driver");

    ros::NodeHandle n;

    ros::Subscriber stop_subscriber = n.subscribe<iana_driver::NoDriverArgs>("/iana/driver/stop", 1000, SetNoDriverCallback);
    ros::Subscriber random_subscriber = n.subscribe<iana_driver::RandomDriverArgs>("/iana/driver/random", 1000, SetRandomDriverCallback);

    ros::Rate rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        driver->Run();
        rate.sleep();
    }

    return 0;
}
