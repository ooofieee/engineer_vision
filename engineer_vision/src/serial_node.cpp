#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include <chrono>

using namespace std::chrono_literals;

class serial_node : public rclcpp::Node
{
private:
    serial::Serial serial_port;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    unsigned char Txbuffer[8] = {0};
    double roll, pitch, yaw;
    double data[3] = {0};
public:
    serial_node(const std::string &node_name) : Node(node_name)
    {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, false);
        listen_tf();

        serial_set();
        std::vector<uint8_t> Txbuffer(sizeof(data));
        auto timer_ = this->create_wall_timer(1000ms, [&]() -> void {
        data[0] = roll;
        data[1] = pitch;
        data[2] = yaw;
        std::memcpy(Txbuffer.data(), data, sizeof(data));
        serial_port.write(Txbuffer);
        });
    }

    void listen_tf()
    {
        const auto transformation = buffer_->lookupTransform("camera", "target", rclcpp::Time(0));
        auto rotation = transformation.transform.rotation;
        try
        {
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TransformException: %s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "rotation: %f, %f, %f", roll, pitch, yaw);
    }

    void serial_set()
    {
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial::Timeout time_ = serial::Timeout::simpleTimeout(2000);
        serial_port.setTimeout(time_);
        serial_port.open();
        if (serial_port.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "serial port opened");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "serial port error");
        }
    }

    void serial_write()
    {
        Txbuffer[0] = '1';
        serial_port.write(Txbuffer, 1);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_node>("serial_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}