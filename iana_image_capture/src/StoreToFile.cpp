#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <sstream>
#include <iostream>
#include <sys/stat.h>

namespace Iana
{
    class StoreToFile
    {
        ros::NodeHandle m_nodeHandle;
        ros::Subscriber m_rawImageSubscriber;
        int m_nextImageIndex;
        std::string m_folderName;
    public:
        StoreToFile()
            : m_nextImageIndex(0)
        {
            m_rawImageSubscriber = m_nodeHandle.subscribe<sensor_msgs::Image>("/rgb/image_raw", 1, &StoreToFile::RGBImageRawCallback, this);

            std::string destination_folder;
            ros::NodeHandle private_node_handle("~");
            if (private_node_handle.getParam("destination_folder", destination_folder))
            {
                ROS_INFO("Got param: %s", destination_folder.c_str());
            }
            else
            {
                ROS_ERROR("Failed to get param 'destination_folder'");
                exit(1);
            }

            std::ostringstream folderName;
            folderName << destination_folder << "iana_image_capture/";
            if (mkdir(folderName.str().c_str(), 0777) != 0)
            {
                ROS_ERROR("Failed to make folder: %s", folderName.str().c_str());
                exit(1);
            }

            folderName << "capture_" << std::time(nullptr) << "/";
            if (mkdir(folderName.str().c_str(), 0777) != 0)
            {
                ROS_ERROR("Failed to make folder: %s", folderName.str().c_str());
                exit(1);
            }

            m_folderName = folderName.str();
        }

        void RGBImageRawCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cvImagePtr;
            try
            {
                cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // store to jpg file
            std::ostringstream jpgFileName;
            jpgFileName << m_folderName << m_nextImageIndex << "rgb_image_raw_" << ".jpg";
            imwrite(jpgFileName.str(), cvImagePtr->image);


            // store to yaml file
            std::ostringstream yamlFileName;
            yamlFileName << m_folderName << m_nextImageIndex << "rgb_image_raw_" << ".yml";
            cv::FileStorage fs(yamlFileName.str(), cv::FileStorage::WRITE);
            fs << "rgb_image_raw" << cvImagePtr->image;
            fs.release();

            m_nextImageIndex++;
        }
    };
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "store_to_file");

    Iana::StoreToFile storeToFile;

    ros::spin();

    return 0;
}
