#include <events_video_dataset_generator/events_video_dataset_generator.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "events_video_dataset_generator_node");
    ros::NodeHandle nh("~");

    event_camera_algorithms::EventsVideoDatasetGenerator events_video_dataset_generator(nh);
    ROS_INFO("\033[1;32m--> Events Video Dataset Generator Node Started.\033[0m");
    ros::spin();
    return 0;
}