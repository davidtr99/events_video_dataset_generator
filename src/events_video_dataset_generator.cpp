#include <events_video_dataset_generator/events_video_dataset_generator.h>

namespace event_camera_algorithms {

EventsVideoDatasetGenerator::EventsVideoDatasetGenerator(ros::NodeHandle& nh) : _nh(nh)
{
    ROS_DEBUG("[EventsVideoDatasetGenerator::EventsVideoDatasetGenerator]");

    std::string events_topic;
    _nh.param<std::string>("events_topic", events_topic, "/dvs/events");

    std::string video_topic;
    _nh.param<std::string>("video_topic", video_topic, "/dvs/images");

    _nh.param<std::string>("bag_path", _bag_path, "/home/dtejero/test_simulation.bag");
    _nh.param<std::string>("folder_name", _folder_name, "dataset");
    _nh.param<std::string>("dataset_name", _dataset_name, "testbed_shelves");

    std::string path    = ros::package::getPath("events_video_dataset_generator");
    std::string command = "mkdir " + path + "/data/" + _folder_name;
    const auto  res     = system(command.c_str());

    std::vector<std::string> topics_to_play = {events_topic, video_topic};

    ROS_INFO("Starting playback...");
    rosbag::Bag bag;
    bag.open(_bag_path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics_to_play));

    for (rosbag::MessageInstance const& msg : view) {
        if (msg.getTopic() == events_topic) {
            ROS_DEBUG("[EventsVideoDatasetGenerator::events_subscriber])");
            const auto& events_msg = msg.instantiate<dvs_msgs::EventArray>();
            _events_buffer.insert(_events_buffer.end(), events_msg->events.begin(), events_msg->events.end());
        }

        else if (msg.getTopic() == video_topic) {
            ROS_DEBUG("[EventsVideoDatasetGenerator::images_subscriber])");
            const auto& image_msg = msg.instantiate<sensor_msgs::Image>();

            // Save image in a file image.jpg
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            std::string timestamp_str = std::to_string(image_msg->header.stamp.toNSec());

            std::stringstream filename;
            filename << _dataset_name << "_" << std::setfill('0') << std::setw(4) << _general_index << "_"
                     << std::setfill('0') << std::setw(12) << timestamp_str << ".png";
            std::string filepath = path + "/data/" + _folder_name + "/" + filename.str();
            cv::imwrite(filepath, cv_ptr->image);
            ROS_INFO("Image saved in %s", filepath.c_str());

            if (!_is_recording) {
                _is_recording = true;
                _general_index++;
                continue;
            }

            const int         events_index = _general_index - 1;
            std::stringstream filename_csv;
            filename_csv << _dataset_name << "_" << std::setfill('0') << std::setw(4) << events_index << ".csv";

            std::string   filepath_csv = path + "/data/" + _folder_name + "/" + filename_csv.str();
            std::ofstream file(filepath_csv);
            file << ",p,t,x,y" << std::endl;
            for (int i = 0; i < _events_buffer.size(); i++) {
                int polarity = int(_events_buffer[i].polarity);
                if (polarity == 0) {
                    polarity = -1;
                }
                file << i << "," << polarity << "," << _events_buffer[i].ts.toNSec() << "," << _events_buffer[i].x
                     << "," << _events_buffer[i].y << std::endl;
            }

            _events_buffer.clear();

            ROS_INFO("Events saved in %s", filepath_csv.c_str());
            _general_index++;
        }

        // Simulate a delay (adjust as needed)
        ros::Duration(0.01).sleep();

        if (!ros::ok()) {
            break; // Exit playback if ROS is not okay (e.g., Ctrl+C)
        }
    }

    ROS_INFO("Playback finished.");
    bag.close();
}

EventsVideoDatasetGenerator::~EventsVideoDatasetGenerator()
{
    ROS_DEBUG("[EventsVideoDatasetGenerator::~EventsVideoDatasetGenerator]");
}

} // namespace event_camera_algorithms
