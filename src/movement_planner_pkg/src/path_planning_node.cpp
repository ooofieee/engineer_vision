#include "rclcpp/rclcpp.hpp"
#include "move_group.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_node");
    
    move_group mg_panda_arm(node, "panda_arm");
    move_group mg_hand(node, "hand");
    
    
    mg_panda_arm.interfaces_init();
    mg_hand.interfaces_init();
    
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.5;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.5;
    target_pose.orientation.w = 1.0;

    mg_panda_arm.move_to_pose(target_pose);
    RCLCPP_INFO(node->get_logger(), "get current pose: (%f, %f, %f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}