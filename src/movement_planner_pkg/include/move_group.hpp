#include <memory>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class move_group
{
private:
    rclcpp::Node::SharedPtr node;
    std::string group_name;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

public:
    explicit move_group(const rclcpp::Node::SharedPtr &node,
                        const std::string &group_name)
                        : node(node), group_name(group_name){
        RCLCPP_INFO(this->node->get_logger(), "MoveGroupInterface created");
    }
    
    void interfaces_init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, group_name);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(node->get_namespace(), true);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);
    }

    void move_to_pose(const geometry_msgs::msg::Pose target_pose)
    {
        move_group_interface_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            move_group_interface_->execute(plan);
            RCLCPP_INFO(node->get_logger(), "Move to pose successful");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan or execute move to pose");
        }
    }

    void back_to_extended_pose()
    {
        move_group_interface_->setNamedTarget("extended");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            move_group_interface_->execute(plan);
            RCLCPP_INFO(node->get_logger(), "Returned to home pose successfully");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan or execute return to home pose");
        }
    }

    geometry_msgs::msg::Pose get_current_pose()
    {
        return move_group_interface_->getCurrentPose().pose;
    }

    void spin_your_wrist()
    {
        return;
    }

    void createCollision()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "cube_slot";
        collision_object.header.frame_id = "camera"; // Use the camera frame for collision object
        collision_object.operation = collision_object.ADD;

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0; // No rotation

        collision_object.pose = pose;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1; // Length
        primitive.dimensions[1] = 0.1; // Width
        primitive.dimensions[2] = 0.1; // Height

        collision_object.primitives.push_back(primitive);
        
        planning_scene_interface_->applyCollisionObject(collision_object);
    }

};