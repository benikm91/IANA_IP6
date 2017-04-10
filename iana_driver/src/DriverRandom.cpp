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

    const VelocityChanger* velocityChanger;

public:

    BumperCallback(const VelocityChanger* velocityChanger) : velocityChanger(velocityChanger) { }

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
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    const VelocityChanger* velocityChanger = new VelocityChanger(&velocityPublisher);

    ros::Subscriber sub = n.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 1000, BumperCallback(velocityChanger));
    ros::Rate rat(2);
    std::shared_ptr<const Action> action = std::shared_ptr<const Action>(new DriveForward(velocityChanger));
    while(ros::ok()) {
        ros::spinOnce();
        if (bumper[0] || bumper[1] || bumper[2]) action = action->Collision();
        action = action->Execute();
        rat.sleep();
    }
    return 0;

}
