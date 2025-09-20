#include "rclcpp/rclcpp.hpp"

#include "mtc.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mtc_node");
    MTC mtc(node);
    mtc.doTask();

    rclcpp::spin(node);
    rclcpp::shutdown();
}