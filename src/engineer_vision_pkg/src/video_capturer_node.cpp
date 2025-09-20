#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/header.hpp"

class video_capturer_node : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    cv::VideoCapture cap;
    int count = 0;

    void imgPublisher()
    {
        cap >> frame;
        if (frame.empty())
        {
            RCLCPP_WARN(get_logger(), "frame is empty");
            timer_->cancel();
            return;
        }
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        this->publisher_->publish(*msg);
        count++;
        //RCLCPP_INFO(get_logger(), "frame %d published", count);
    }

public:
    explicit video_capturer_node(const std::string &node_name, const std::string path) : Node(node_name)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("redeem_box_image", 10);
        cap.open(path);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", path.c_str());
            return;
        }
        auto interval = std::chrono::milliseconds(static_cast<int>(1000 / cap.get(cv::CAP_PROP_FPS)));
        timer_ = this->create_wall_timer(interval, std::bind(&video_capturer_node::imgPublisher, this));
    }
    ~video_capturer_node()
    {
        cap.release();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string path = "/home/mac/Code/ws_0/src/engineer_vision_pkg/resources/exchange_station.mp4";
    auto node = std::make_shared<video_capturer_node>("video_capturer_node", path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
