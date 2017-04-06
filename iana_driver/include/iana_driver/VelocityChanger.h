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
        const ros::Publisher* velocityPublisher;
        Vector3 currentLinear;
        Vector3 currentAngular;

    public:
        VelocityChanger(ros::Publisher* velocityPublisher) : VelocityChanger(velocityPublisher, RandomVector3(), RandomVector3()) { }
        VelocityChanger(ros::Publisher* velocityPublisher, Vector3 startLinear, Vector3 startAngular)
                : velocityPublisher(velocityPublisher),
                  currentLinear(startLinear),
                  currentAngular(startAngular)
        { }

    private:

        void publishVelocity(const Vector3 linear, const Vector3 angular) const
        {
            geometry_msgs::Twist twist;
            twist.linear = linear;
            twist.angular = angular;
            ROS_INFO("[%f]", angular.y);
            this->velocityPublisher->publish(twist);
        }

    public:
        void ChangeVelocity(const Vector3 linear, const Vector3 angular)
        {
            this->currentLinear = linear;
            this->currentAngular = angular;
            this->PublishCurrentVelocity();
        }

        void PublishCurrentVelocity() const
        {
            this->publishVelocity(this->currentLinear, this->currentAngular);
        }

    };

}

#endif