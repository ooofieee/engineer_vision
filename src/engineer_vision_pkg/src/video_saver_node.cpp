#include"rclcpp/rclcpp.hpp"
#include"opencv4/opencv2/opencv.hpp"
#include"sensor_msgs/msg/image.hpp"
#include"cv_bridge/cv_bridge.h"
#include<filesystem>

class video_saver_node:public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    cv::VideoWriter video_writer;
    std::string video_path;
    bool is_writer_initialized_ = false;

public:
    video_saver_node():Node("video_saver_node")
    {
        video_path = "/home/mac/RECORD";

        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "processed_image",
            10,
            std::bind(&video_saver_node::image_callback, this, std::placeholders::_1)
        );
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received empty frame");
            return;
        }

        if (!is_writer_initialized_)
        {
            video_writer.open(video_path + "/output.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, frame.size(), true);

            if (!video_writer.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Could not open the video writer");
                return;
            }

            is_writer_initialized_ = true;
        }

        video_writer.write(frame);
    }       
};

int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<video_saver_node>());
    rclcpp::shutdown();
    return 0;
}