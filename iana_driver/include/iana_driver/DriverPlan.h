#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "VelocityChanger.h"
#include "Vector3.h"
#include "Quaternion.h"
#include <memory.h>
#include <math.h>
#include <queue>

#define DRIVING_SPEED 0.5
#define TURNING_SPEED 0.5

namespace Iana {

    class PlanAction {
    public:
        virtual void Init(Quaternion orientation, Vector3 position) = 0;

        virtual void
        Update(std::shared_ptr <VelocityChanger> velocityChanger, Quaternion orientation, Vector3 position) = 0;

        virtual bool GoalReached(Quaternion orientation, Vector3 position) const = 0;
    };

    class DrivingForwardAction : public PlanAction {

    private:
        const double m_distance;
        Vector3 m_startPosition = Vector3::Zero;
        const double m_drivingSpeed;

    public:

        DrivingForwardAction(double distance, double speed = DRIVING_SPEED) : m_distance(distance),
                                                                              m_drivingSpeed(speed) {}

        void Init(Quaternion orientation, Vector3 position) { m_startPosition = position; }

        void Update(std::shared_ptr <VelocityChanger> velocityChanger, Quaternion orientation, Vector3 position) {
            if (GoalReached(orientation, position)) return;

            ROS_INFO("DrivingForwardAction - Current: %f | Goal: %f", DistanceSoFar(position), m_distance);
            velocityChanger->PublishVelocity(m_drivingSpeed * Vector3::Left, Vector3::Zero);
        }

        bool GoalReached(Quaternion orientation, Vector3 position) const { return m_distance <= DistanceSoFar(position); }

        inline double DistanceSoFar(Vector3 position) const { return (m_startPosition - position).Length(); }

    };

    class TurningAction : public PlanAction {

    public:
        const static int Left;
        const static int Right;

    private:
        const double m_angle;
        const int m_direction;
        const double m_turningSpeed;
        double m_rotatedSoFar = 0;
        double m_oldYawn;

    public:
        TurningAction(double angle, int direction, double speed = TURNING_SPEED)
                : m_angle(angle),
                  m_direction(direction),
                  m_turningSpeed(speed) {}

    public:
        void Init(Quaternion orientation, Vector3 position) {
            double roll, pitch, startYawn;
            std::tie(roll, pitch, startYawn) = orientation.ToEulerianAngle();

            m_oldYawn = startYawn + M_PI;
        }

        void Update(std::shared_ptr <VelocityChanger> velocityChanger, Quaternion orientation, Vector3 position) {
            if (GoalReached(orientation, position)) return;

            double roll, pitch, yawn;
            std::tie(roll, pitch, yawn) = orientation.ToEulerianAngle();
            yawn += M_PI;

            double deltaYawn = std::abs(yawn - m_oldYawn);
            if (deltaYawn > M_PI) deltaYawn = std::fmin(yawn, m_oldYawn) + 2*M_PI - std::fmax(yawn, m_oldYawn);
            m_rotatedSoFar += deltaYawn;
            ROS_INFO("TurningAction - Current: %f | Goal: %f", m_rotatedSoFar, m_angle);
            m_oldYawn = yawn;

            velocityChanger->PublishVelocity(Vector3::Zero, (m_direction * m_turningSpeed) * Vector3::Forward);
        }

        bool GoalReached(Quaternion orientation, Vector3 position) const { return m_angle <= m_rotatedSoFar; }

    };

    const int TurningAction::Left = 1;
    const int TurningAction::Right = -1;

    class DriverPlan {

    private:

        std::shared_ptr <VelocityChanger> m_velocityChanger;
        std::queue <std::shared_ptr<PlanAction>> m_actions;

    public:

        DriverPlan(std::shared_ptr <VelocityChanger> vc, std::queue <std::shared_ptr<PlanAction>> actions)
                : m_velocityChanger(vc), m_actions(actions) {}

    public:

        void Update(const nav_msgs::Odometry::ConstPtr &msg) {
            Quaternion orientation = Quaternion::FromMsg(msg->pose.pose.orientation);
            Vector3 position = Vector3::FromMsg(msg->pose.pose.position);

            ROS_INFO("Number of remaining actions %zu", m_actions.size());

            if (m_actions.size() > 0) {
                m_actions.front()->Update(m_velocityChanger, orientation, position);
                if (m_actions.front()->GoalReached(orientation, position)) {
                    ROS_INFO("Goal reached for current action => Select next action");
                    m_actions.pop();
                    if (m_actions.size() > 0) m_actions.front()->Init(orientation, position);
                }
            }
            else
            {
                m_velocityChanger->PublishVelocity(Vector3::Zero, Vector3::Zero);
            }
        }

    };

}
