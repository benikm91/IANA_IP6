#include <memory>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "kobuki_msgs/BumperEvent.h"
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"
#include "std_msgs/Float32.h"

using namespace Iana;

bool collisionAhead = false;

struct BumperCallback
{
private:

    std::shared_ptr<VelocityChanger> velocityChanger;

public:

    BumperCallback(std::shared_ptr<VelocityChanger> velocityChanger) : velocityChanger(velocityChanger) { }

public:
    void operator()(const std_msgs::Float32::ConstPtr& msg) const
    {
        collisionAhead = msg->data < 0.8;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_random_node");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
        n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );

    ros::Subscriber sub = n.subscribe<std_msgs::Float32>("/collision_ahead", 1000, BumperCallback(velocityChanger));
    ros::Rate rat(2);
    std::shared_ptr<Action> action = std::make_shared<DriveForward>(velocityChanger);
    while(ros::ok()) {
        ros::spinOnce();
        if (collisionAhead) action = action->Collision();
        action = action->Execute();
        rat.sleep();
    }

    return 0;

}
