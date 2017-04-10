
#ifndef IANA_DRIVER_ACTIONS_H
#define IANA_DRIVER_ACTIONS_H

#include <memory>
#include "VelocityChanger.h"
#include "Vector3.h"

namespace Iana {

    struct Action
    {
        virtual std::shared_ptr<const Action> Execute() const = 0;
        virtual std::shared_ptr<const Action> Collision() const = 0;
    };

    struct TurnLeft : public Action
    {

    private:
        const int times;
        const VelocityChanger* velocityChanger;

    public:
        TurnLeft(const VelocityChanger* velocityChanger, const int times);

        std::shared_ptr<const Action> Execute() const;
        std::shared_ptr<const Action> Collision() const;
    };

    struct DriveForward : public Action
    {

    private:
        const VelocityChanger* velocityChanger;

    public:
        DriveForward(const VelocityChanger* velocityChanger);

        std::shared_ptr<const Action> Execute() const;
        std::shared_ptr<const Action> Collision() const;

    };

    TurnLeft::TurnLeft(const VelocityChanger* velocityChanger, const int times) : velocityChanger(velocityChanger), times(times) { }

    std::shared_ptr<const Action> TurnLeft::Execute() const
    {
        ROS_INFO("TURNING LEFT");
        if (this->times == 0) return std::shared_ptr<const Action>(new DriveForward(this->velocityChanger));
        this->velocityChanger->PublishVelocity(*Vector3::Zero, Vector3(0.5, 0.5, 0.5));
        return std::shared_ptr<const Action>(new TurnLeft(this->velocityChanger, this->times - 1));
    }

    std::shared_ptr<const Action> TurnLeft::Collision() const
    {
        return std::shared_ptr<const Action>(new TurnLeft(this->velocityChanger, this->times));
    }


    DriveForward::DriveForward(const VelocityChanger* velocityChanger) : velocityChanger(velocityChanger) { }

    std::shared_ptr<const Action> DriveForward::Execute() const
    {
        ROS_INFO("DRIVING FORWARD");
        this->velocityChanger->PublishVelocity(Vector3(0.2, 0.0, 0.0), *Vector3::Zero);
        return std::shared_ptr<const Action>(new DriveForward(this->velocityChanger));
    }

    std::shared_ptr<const Action> DriveForward::Collision() const
    {
        return std::shared_ptr<const Action>(new TurnLeft(velocityChanger, 5));
    }

}

#endif //IANA_DRIVER_ACTIONS_H
