#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

namespace Iana
{
    class LookAhead
    {
        ros::NodeHandle m_nodeHandle;
        ros::Publisher m_collisionAheadPublisher;
        ros::Subscriber m_depthImageSubscriber;
    public:
        LookAhead()
        {
            m_collisionAheadPublisher = m_nodeHandle.advertise<std_msgs::Float32>("/collision_ahead", 1000);
            m_depthImageSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &LookAhead::DepthImageCallback, this);
	    }

        void DepthImageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
	    std::vector<float> ranges = msg->ranges;
            auto begin = ranges.begin();
            auto end = ranges.end();
	    end = std::remove_if(begin, end, [&](float range) { return range <= msg->range_min || range >= msg->range_max; });
	    std::sort(begin, end);
	    std_msgs::Float32 minRangeMsg;
            minRangeMsg.data = begin[2];
            m_collisionAheadPublisher.publish(minRangeMsg);

	/*
            float range = msg->range_max;
            for (it; it != end; ++it)
            {
                float range = *it;
                if (range < min_range && range >= msg->range_min)
                {
                    min_range = range;
                }
            }
            if (min_range != msg->range_max)
            {
                std_msgs::Float32 minRangeMsg;
                minRangeMsg.data = min_range;
                m_collisionAheadPublisher.publish(minRangeMsg);
            }*/
        }
    };
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "look_ahead");

    Iana::LookAhead lookAhead;

    ros::spin();

    return 0;
}
