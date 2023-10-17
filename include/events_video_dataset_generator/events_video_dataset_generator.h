#pragma once
#include <dvs_msgs/EventArray.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#define ROS_TOPIC_BUFFER_SIZE 100

namespace event_camera_algorithms {

class EventsVideoDatasetGenerator
{
  public:
    EventsVideoDatasetGenerator(ros::NodeHandle& nh);
    virtual ~EventsVideoDatasetGenerator();

  private:

  private:
    // ROS communication
    ros::NodeHandle& _nh;
    std::string _dataset_name;
    std::string _folder_name;
    std::string _bag_path;

    int _general_index{0};
    bool _is_recording{false};

    // Data
    std::vector<dvs_msgs::Event> _events_buffer;
};

} // namespace event_camera_algorithms