#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

namespace Iana
{
    class LookAhead
    {
      private:
        ros::NodeHandle m_nodeHandle;
        ros::Publisher m_collisionAheadPublisher;
        ros::Subscriber m_depthImageSubscriber;
        float threshold_factor;

      public:
        LookAhead()
        {
            m_collisionAheadPublisher = m_nodeHandle.advertise<std_msgs::Float32>("/collision_ahead", 1000);
            m_depthImageSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &LookAhead::DepthImageCallback, this);
            this->threshold_factor = 0.5f;
        }

        void DepthImageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            std::vector<float> ranges = msg->ranges;
            auto begin = ranges.begin();
            auto end = ranges.end();
            auto threshold = ranges.size() * this->threshold_factor;
            auto offset = ranges.size() / 4;
            begin += offset;
            end -= offset;
            auto count_min_range = std::count_if(begin, end, [msg](float range) { return range <= msg->range_min; });
            end = std::remove_if(begin, end, [&](float range) { return range <= msg->range_min || range >= msg->range_max; });
            std::sort(begin, end);
            std_msgs::Float32 minRangeMsg;
            if (count_min_range <= threshold)
                minRangeMsg.data = begin[2];
            else
                minRangeMsg.data = msg->range_min;
            m_collisionAheadPublisher.publish(minRangeMsg);
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
