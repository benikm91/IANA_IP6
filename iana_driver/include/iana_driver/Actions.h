
#ifndef IANA_DRIVER_ACTIONS_H
#define IANA_DRIVER_ACTIONS_H

#include <memory>
#include "VelocityChanger.h"
#include "Vector3.h"

namespace Iana {

    struct Action
    {
        virtual std::shared_ptr<Action> Execute() const = 0;
        virtual std::shared_ptr<Action> Collision() const = 0;
    };

    struct VelocityChangeAction : public Action
    {
    protected:
        const std::shared_ptr<Iana::VelocityChanger> m_velocityChanger;
    public:
        VelocityChangeAction(std::shared_ptr<VelocityChanger> velocityChanger) : m_velocityChanger(velocityChanger) { }
    };

    struct TurnLeft : public VelocityChangeAction
    {
        const int m_times;

        TurnLeft(std::shared_ptr<VelocityChanger> velocityChanger, const int times);

        std::shared_ptr<Action> Execute() const;
        std::shared_ptr<Action> Collision() const;
    };

    struct DriveForward : public VelocityChangeAction
    {

    public:
        DriveForward(std::shared_ptr<VelocityChanger> velocityChanger);

        std::shared_ptr<Action> Execute() const;
        std::shared_ptr<Action> Collision() const;

    };

    TurnLeft::TurnLeft(std::shared_ptr<VelocityChanger> velocityChanger, const int times) : VelocityChangeAction(velocityChanger), m_times(times) { }

    std::shared_ptr<Action> TurnLeft::Execute() const
    {
        ROS_INFO("TURNING LEFT");
        if (m_times == 0) return std::make_shared<DriveForward>(m_velocityChanger);
        m_velocityChanger->PublishVelocity(Vector3::Zero, Vector3(0.5, 0.5, 0.5));
        return std::make_shared<TurnLeft>(m_velocityChanger, m_times - 1);
    }

    std::shared_ptr<Action> TurnLeft::Collision() const
    {
        return std::make_shared<TurnLeft>(m_velocityChanger, m_times);
    }


    DriveForward::DriveForward(std::shared_ptr<VelocityChanger> velocityChanger) : VelocityChangeAction(velocityChanger) { }

    std::shared_ptr<Action> DriveForward::Execute() const
    {
        ROS_INFO("DRIVING FORWARD");
        m_velocityChanger->PublishVelocity(Vector3(0.2, 0.0, 0.0), Vector3::Zero);
        return std::make_shared<DriveForward>(m_velocityChanger);
    }

    std::shared_ptr<Action> DriveForward::Collision() const
    {
        return std::make_shared<TurnLeft>(m_velocityChanger, 5);
    }

}

#endif //IANA_DRIVER_ACTIONS_H
