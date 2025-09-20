#include "rclcpp/rclcpp.hpp"
#include "state_assessor.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"

class state_assessor_node : public rclcpp::Node
{
private:
    state_assessor assessor_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready2act_publisher_;


public:
    state_assessor_node(const std::string node_name) : 
        rclcpp::Node(node_name), 
        assessor_(this->shared_from_this()), 
        tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())) {

        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, this, false);
        RCLCPP_INFO(this->get_logger(), "State Assessor Node started");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&state_assessor_node::timer_callback, this));
        

    }

private:
    void timer_callback() {
        const auto transformation = tf_buffer->lookupTransform("camera", "center", tf2::TimePointZero);
        cv::Point3f center(transformation.transform.translation.x,
                           transformation.transform.translation.y,
                           transformation.transform.translation.z);
        ready2act_publisher_->publish(std_msgs::msg::Bool().set__data(assessor_.ready_to_act(center)));
    }

};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<state_assessor_node>("state_assessor_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}