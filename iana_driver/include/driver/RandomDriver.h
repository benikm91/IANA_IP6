//
// Created by metataro on 10.05.17.
//

#ifndef IANA_DRIVER_RANDOMDRIVER_H
#define IANA_DRIVER_RANDOMDRIVER_H

#include <memory>
#include "Driver.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "../iana_driver/VelocityChanger.h"
#include "../iana_driver/Actions.h"

namespace Iana
{
    class RandomDriver: public Driver
    {
        ros::Rate m_rat;
        bool m_collisionAhead;
        float m_collision_ahead_threshold;

        ros::NodeHandle m_nodeHandle;
        ros::Subscriber m_collisionAheadSubscriber;
        std::shared_ptr<VelocityChanger> m_velocityChanger;
        std::shared_ptr<Action> m_action;

    public:
        RandomDriver(float collision_ahead_threshold = 0.8)
            : m_rat(2)
            , m_collisionAhead(false)
            , m_collision_ahead_threshold(collision_ahead_threshold)
        {
            m_collisionAheadSubscriber = m_nodeHandle.subscribe<std_msgs::Float32>("/collision_ahead", 1, &RandomDriver::CollisionAheadCallback, this);
            m_velocityChanger = std::make_shared<VelocityChanger>(m_nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1));
            m_action = std::make_shared<DriveForward>(m_velocityChanger);
        }

        void Run() override
        {
            if (m_collisionAhead) m_action = m_action->Collision();
            m_action = m_action->Execute();
            m_rat.sleep();
        }

        void CollisionAheadCallback(const std_msgs::Float32::ConstPtr& msg)
        {
            m_collisionAhead = msg->data < m_collision_ahead_threshold;
        }
    };
};

#endif //IANA_DRIVER_RANDOMDRIVER_H
