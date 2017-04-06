#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "kobuki_msgs/BumperEvent.h"
#include "../include/iana_driver/VelocityChanger.h"

using namespace Iana;

Vector3 turnLeft = Vector3(0, 0, 0.2);

bool bumper[3] = { false };

struct BumperCallback
{
private:

    VelocityChanger* velocityChanger;

public:

    BumperCallback(VelocityChanger* velocityChanger) : velocityChanger(velocityChanger) { }

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

    VelocityChanger* velocityChanger = new VelocityChanger(
        &velocityPublisher,
        *Vector3::Left,
        *Vector3::Zero
    );

    ros::Subscriber sub = n.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 1000, BumperCallback(velocityChanger));
    ros::Rate rat(20);
    while(ros::ok()) {
        ros::spinOnce();
        ROS_INFO("%d %d %d", bumper[0], bumper[1], bumper[2]);
        if (bumper[0] || bumper[1] || bumper[2])
        {
            velocityChanger->ChangeVelocity(*Vector3::Zero, turnLeft);
        }
        else
        {
            velocityChanger->ChangeVelocity(Vector3(0.2, 0.0, 0.0), *Vector3::Zero);
        }
    }
    return 0;

}
