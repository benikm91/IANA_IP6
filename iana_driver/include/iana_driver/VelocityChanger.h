#ifndef IANA_DRIVER_DRIVER_H
#define IANA_DRIVER_DRIVER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "Vector3.h"

namespace Iana
{

    typedef geometry_msgs::Vector3 Vector3Message;

    Vector3 RandomVector3() { return Vector3(1.0, 0.0, 1.0); }

    class VelocityChanger
    {

    private:
        const ros::Publisher velocityPublisher;

    public:
        VelocityChanger(ros::Publisher&& velocityPublisher) : velocityPublisher(velocityPublisher)
        { }

    public:

        void PublishVelocity(const Vector3 linear, const Vector3 angular) const
        {
            geometry_msgs::Twist twist;
            twist.linear = linear;
            twist.angular = angular;
            this->velocityPublisher.publish(twist);
        }

    };

}

#endif