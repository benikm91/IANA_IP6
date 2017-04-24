#include <memory>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "kobuki_msgs/BumperEvent.h"
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"

using namespace Iana;

bool bumper[3] = { false };

struct BumperCallback
{
private:

    std::shared_ptr<VelocityChanger> velocityChanger;

public:

    BumperCallback(std::shared_ptr<VelocityChanger> velocityChanger) : velocityChanger(velocityChanger) { }

public:
    void operator()(const kobuki_msgs::BumperEvent::ConstPtr& msg) const
    {
        bumper[msg->bumper] = msg->state;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_random_node");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
        n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );

    ros::Subscriber sub = n.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 1000, BumperCallback(velocityChanger));
    ros::Rate rat(2);
    std::shared_ptr<Action> action = std::make_shared<DriveForward>(velocityChanger);
    while(ros::ok()) {
        ros::spinOnce();
        if (bumper[0] || bumper[1] || bumper[2]) action = action->Collision();
        action = action->Execute();
        rat.sleep();
    }

    return 0;

}
