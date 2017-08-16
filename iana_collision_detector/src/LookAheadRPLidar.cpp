#include <vector>
#include <math.h>
#include <algorithm>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

namespace Iana
{
    class LookAheadRPLidar
    {
      private:
        ros::NodeHandle m_nodeHandle;
        ros::Publisher m_collisionAheadPublisher;
        ros::Publisher m_collisionAheadScanPublisher;
        ros::Subscriber m_depthImageSubscriber;
        float threshold_factor;

      public:
        LookAheadRPLidar()
        {
            m_collisionAheadPublisher = m_nodeHandle.advertise<std_msgs::Float32>("/iana/collision_detector/collision_ahead", 1000);
            m_collisionAheadScanPublisher = m_nodeHandle.advertise<sensor_msgs::LaserScan>("/lookAhead/considered_scan", 1000);
            m_depthImageSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &LookAheadRPLidar::LaserScanCallback, this);
            this->threshold_factor = 0.5f;
        }


        // This only works with RPLidar that is 180 degrees yawn relative to the base.
        void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            std::vector<float> ranges = msg->ranges;
            auto consideration_angle = M_PI / 4;
            auto n = consideration_angle / msg->angle_increment;
            auto begin = ranges.begin();
            auto end = ranges.end();

            if (m_collisionAheadScanPublisher.getNumSubscribers() > 0) {
                sensor_msgs::LaserScan scanMsg;
                scanMsg.header = msg->header;
                scanMsg.angle_min = msg->angle_min;
                scanMsg.angle_max = msg->angle_max;
                scanMsg.angle_increment = msg->angle_increment;
                scanMsg.time_increment = msg->time_increment;
                scanMsg.range_min = msg->range_min;
                scanMsg.range_max = msg->range_max;
                std::vector<float> considered_range_all(ranges.size());
                std::copy(ranges.begin(), ranges.begin() + n / 2, considered_range_all.begin());
                std::copy(ranges.end() - n / 2, ranges.end(), considered_range_all.end() - n / 2);
                scanMsg.ranges = considered_range_all;
                m_collisionAheadScanPublisher.publish(scanMsg);
            }

            std::vector<float> considered_range(n);
            n++; // because of later integer division.
            std::copy(ranges.begin(), ranges.begin() + n / 2, considered_range.begin());
            std::copy(ranges.end() - n / 2, ranges.end(), considered_range.end() - n / 2);

            std_msgs::Float32 minRangeMsg;
            if (!considered_range.empty()) {
                std::sort(considered_range.begin(), considered_range.end());
                minRangeMsg.data = considered_range[0];
            } else {
                // throw Exception ?
                minRangeMsg.data = msg->range_max;
            }
            m_collisionAheadPublisher.publish(minRangeMsg);
        }
    };
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "look_ahead_360");

    Iana::LookAheadRPLidar lookAheadRPLidar;

    ros::spin();

    return 0;
}
