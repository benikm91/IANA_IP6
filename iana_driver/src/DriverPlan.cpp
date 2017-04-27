#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "../include/iana_driver/VelocityChanger.h"
#include "../include/iana_driver/Actions.h"
#include "../include/iana_driver/Vector3.h"
#include "../include/iana_driver/Quaternion.h"
#include <memory.h>
#include <math.h>
#include <queue>

using namespace Iana;

class PlanAction
{
public:
    virtual void Init(Quaternion orientation, Vector3 position) = 0;
    virtual void Update(std::shared_ptr<VelocityChanger> velocityChanger, Quaternion orientation, Vector3 position) = 0;
    virtual bool GoalReached(Quaternion orientation, Vector3 position) = 0;
};

class DriveForwardAction : public PlanAction
{

private:
    const double m_distance;
    Vector3 m_startPosition = Vector3::Zero;

public:

    DriveForwardAction(double distance) : m_distance(distance) { }

    void Init(Quaternion orientation, Vector3 position) { m_startPosition = position; }

    void Update(std::shared_ptr<VelocityChanger> velocityChanger, Quaternion orientation, Vector3 position)
    {
	ROS_WARN("DRIVING FORWARD");
        if (GoalReached(orientation, position)) return;

	ROS_WARN_STREAM("STILL DRIVING " << (m_startPosition - position).Length());
        velocityChanger->PublishVelocity(Vector3(0.22,0,0), Vector3::Zero);
    }

    bool GoalReached(Quaternion orientation, Vector3 position) { return m_distance <= (m_startPosition - position).Length(); }

};

class TurningAction : public PlanAction
{

public:
    const static int Left;
    const static int Right;

private:
    const int m_angle;
    const int m_direction;
    double m_rotatedSoFar = 0;
    double m_oldYawn;

public:
    TurningAction(double angle, int direction) : m_angle(angle), m_direction(direction) { }

public:
    void Init(Quaternion orientation, Vector3 position)
    {
        double roll, pitch, startYawn;
        std::tie(roll, pitch, startYawn) = orientation.ToEulerianAngle();

        m_oldYawn = startYawn;
    }

    void Update(std::shared_ptr<VelocityChanger> velocityChanger, Quaternion orientation, Vector3 position)
    {
        if (GoalReached(orientation, position)) return;

        double roll, pitch, yawn;
        std::tie(roll, pitch, yawn) = orientation.ToEulerianAngle();

        m_rotatedSoFar += yawn - m_oldYawn;
	ROS_WARN_STREAM("STILL TURNING" << m_rotatedSoFar); 
        m_oldYawn = yawn;

        velocityChanger->PublishVelocity(Vector3::Zero, m_direction * Vector3(0,0,0.2));
    }

    bool GoalReached(Quaternion orientation, Vector3 position) { return m_angle <= m_rotatedSoFar; }

};
const int TurningAction::Left = 1;
const int TurningAction::Right = -1;

class DriverPlan
{

private:

    std::shared_ptr<VelocityChanger> m_velocityChanger;
    std::queue<std::shared_ptr<PlanAction>> m_actions;

public:

    DriverPlan(std::shared_ptr<VelocityChanger> vc, std::queue<std::shared_ptr<PlanAction>> actions) : m_velocityChanger(vc), m_actions(actions)
    { }

public:

    void Update(const nav_msgs::Odometry::ConstPtr& msg)
    {
	ROS_WARN("UPDATE");
        Quaternion orientation = Quaternion::FromMsg(msg->pose.pose.orientation);
        Vector3 position = Vector3::FromMsg(msg->pose.pose.position);

	ROS_WARN_STREAM("Remaining actions: " << m_actions.size());

        if (m_actions.size() > 0)
        {
            m_actions.front()->Update(m_velocityChanger, orientation, position);
            if (m_actions.front()->GoalReached(orientation, position))
            {
		ROS_WARN("Next Action");
                m_actions.pop();
		if (m_actions.size() > 0) m_actions.front()->Init(orientation, position);
            }
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_plan");
    ros::NodeHandle n;

    std::shared_ptr<VelocityChanger> velocityChanger = std::make_shared<VelocityChanger>(
            n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)
    );

    std::queue<std::shared_ptr<PlanAction>> actions;

    actions.push(std::make_shared<DriveForwardAction>(0));
    actions.push(std::make_shared<DriveForwardAction>(0.4));
    actions.push(std::make_shared<TurningAction>(M_PI, TurningAction::Left));

    DriverPlan driverPlan(
        velocityChanger,
        actions
    );

    ROS_WARN("Subscribing to /odom topic");

    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &DriverPlan::Update, &driverPlan);

    ros::spin();

    return 0;

}
