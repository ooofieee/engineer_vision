#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <chrono>
#include "foxglove_msgs/msg/scene_entity.hpp"
#include "foxglove_msgs/msg/scene_update.hpp"
#include "foxglove_msgs/msg/cube_primitive.hpp"
#include "foxglove_msgs/msg/scene_entity_deletion.hpp"

class RB_builder_node : public rclcpp::Node
{
public:
    RB_builder_node() : Node("RB_builder_node")
    {
        RCLCPP_INFO(this->get_logger(), "RB_builder_node launched");

        center.resize(3);
        corners.resize(4);
        for(int i = 0; i < 4; i++) {
            corners[i].resize(3);
        }
        
        last_center_.resize(3);
        last_corners_.resize(4);
        for(int i = 0; i < 4; i++) {
            last_corners_[i].resize(3);
        }
        last_roll_ = last_pitch_ = last_yaw_ = 0.0;
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        buffer_->setUsingDedicatedThread(true);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, false);
        
        tf_received_ = false;
        object_created_ = false;
        last_update_time_ = this->now();

        // fmsg.id = "cube_288mm";
        // fmsg.frame_id = "camera";

        foxglove_publisher_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("/scene_entities", 10);
        scene_updater_ = std::make_shared<foxglove_msgs::msg::SceneUpdate>();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), 
            std::bind(&RB_builder_node::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (get_tf()) {
            if (has_significant_change()) {
                RCLCPP_INFO(logger, "Detected significant TF change, updating model...");
                create_object();
                last_update_time_ = this->now();
                tf_received_ = true;
                update_last_tf_data();
            }
        }
    }

    bool get_tf()
    {
        try
        {
            if (!buffer_->canTransform("camera", "target", tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                RCLCPP_WARN_THROTTLE(logger, *this->get_clock(), 2000, "Transform from 'camera' to 'target' unavailable yet");
                return false;
            }

            transformation = buffer_->lookupTransform("camera", "target", tf2::TimePointZero);
            auto rotation = transformation.transform.rotation;
            auto translation = transformation.transform.translation;
            
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            center[0] = translation.x;
            center[1] = translation.y;
            center[2] = translation.z;

            RCLCPP_INFO(logger, "Got center transform: [%.3f, %.3f, %.3f], rotation: [%.3f, %.3f, %.3f]", 
                       center[0], center[1], center[2], roll, pitch, yaw);

            bool all_corners_available = true;
            for (int i = 0; i < 4; i++)
            {
                std::string corner_frame = "corner" + std::to_string(i);
                
                if (!buffer_->canTransform("camera", corner_frame, tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                    RCLCPP_WARN_THROTTLE(logger, *this->get_clock(), 2000, "Transform from 'camera' to '%s' not available", corner_frame.c_str());
                    all_corners_available = false;
                    continue;
                }

                auto corner_transform = buffer_->lookupTransform("camera", corner_frame, tf2::TimePointZero);
                auto corner_translation = corner_transform.transform.translation;
                corners[i][0] = corner_translation.x;
                corners[i][1] = corner_translation.y;
                corners[i][2] = corner_translation.z;
                
                RCLCPP_DEBUG(logger, "Got corner %d: [%.3f, %.3f, %.3f]", i, corners[i][0], corners[i][1], corners[i][2]);
            }

            if (!all_corners_available) {
                RCLCPP_WARN_THROTTLE(logger, *this->get_clock(), 2000, "Not all corner transforms available yet");
                return false;
            }

            RCLCPP_INFO(logger, "Successfully got all transforms");
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(logger, *this->get_clock(), 2000, "TransformException: %s", ex.what());
            return false;
        }
    }

    void create_object()
    {
        if(center.size() != 3 || corners.size() != 4) {
            RCLCPP_ERROR(logger, "Invalid tf data: center or corners not properly initialized");
            RCLCPP_ERROR(logger, "center.size()=%zu, corners.size()=%zu", center.size(), corners.size());
            return;
        }

        for(int i = 0; i < 4; i++) {
            if(corners[i].size() != 3) {
                RCLCPP_ERROR(logger, "Invalid corners data: corner[%d].size()=%zu", i, corners[i].size());
                return;
            }
        }

        RCLCPP_INFO(logger, "TF data validation passed: center[3], corners[4][3]");

        if (object_created_) {
            remove_existing_object("cube_288mm");
        }

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "cube_288mm";
        collision_object.header.frame_id = "camera";
        collision_object.header.stamp = this->get_clock()->now();
        collision_object.operation = collision_object.ADD;

        collision_object.primitives.resize(1);
        collision_object.primitive_poses.resize(1);

        collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_object.primitives[0].dimensions.resize(3);

        calculateCubeGeometry(cube_pose);

        const double cube_size = 0.288;
        collision_object.primitives[0].dimensions[0] = cube_size;
        collision_object.primitives[0].dimensions[1] = cube_size;
        collision_object.primitives[0].dimensions[2] = cube_size;
        collision_object.primitive_poses[0] = cube_pose;

        RCLCPP_INFO(logger, "CollisionObject created: 288mm cube");
        RCLCPP_INFO(logger, "Frame ID: %s, Operation: ADD", collision_object.header.frame_id.c_str());
        RCLCPP_INFO(logger, "Cube dimensions: [%.3fm, %.3fm, %.3fm]", cube_size, cube_size, cube_size);

        planning_scene_interface.applyCollisionObject(collision_object);
        foxglove_visualization();
        foxglove_publisher_->publish(*scene_updater_);

        rclcpp::sleep_for(std::chrono::milliseconds(50));

        std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();
        bool found = false;
        for(const auto& id : object_ids) {
            if(id == "cube_288mm") {
                found = true;
                break;
            }
        }

        object_created_ = true;
        RCLCPP_INFO(logger, "288mm cube collision object updated in planning scene");
        RCLCPP_INFO(logger, "Object verification: %s", found ? "SUCCESS" : "WARNING - Not found");
        RCLCPP_INFO(logger, "Total objects in planning scene: %zu", object_ids.size());
    }

    void remove_existing_object(const std::string &object_id)
    {
        moveit_msgs::msg::CollisionObject remove_object;
        remove_object.id = object_id;
        remove_object.header.frame_id = "camera";
        remove_object.operation = remove_object.REMOVE;
        
        planning_scene_interface.applyCollisionObject(remove_object);

        scene_updater_->deletions.clear();
        scene_updater_->entities.clear();
        deletion_msg.id = object_id;
        scene_updater_->deletions.push_back(deletion_msg);
        foxglove_publisher_->publish(*scene_updater_);

        RCLCPP_INFO(logger, "Removed existing %s collision object", object_id.c_str());
    }

    bool has_significant_change()
    {
        const double threshold = 0.005;
        const double angle_threshold = 0.05;
        
        if (!tf_received_) {
            return true;
        }
        
        for (int i = 0; i < 3; i++) {
            if (std::abs(center[i] - last_center_[i]) > threshold) {
                return true;
            }
        }
        
        if (std::abs(roll - last_roll_) > angle_threshold ||
            std::abs(pitch - last_pitch_) > angle_threshold ||
            std::abs(yaw - last_yaw_) > angle_threshold) {
            return true;
        }
        
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                if (std::abs(corners[i][j] - last_corners_[i][j]) > threshold) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    void update_last_tf_data()
    {
        last_center_ = center;
        last_corners_ = corners;
        last_roll_ = roll;
        last_pitch_ = pitch;
        last_yaw_ = yaw;
    }

    void calculateCubeGeometry(geometry_msgs::msg::Pose& pose)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        pose.position.x = center[0] / 100.0;
        pose.position.y = center[1] / 100.0;
        pose.position.z = center[2] / 100.0 + 0.144;

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        RCLCPP_INFO(logger, "Calculated 288mm cube geometry at position: [%.3f, %.3f, %.3f]",
                   pose.position.x, pose.position.y, pose.position.z);
    }

    void foxglove_visualization()
    {
        foxglove_msgs::msg::SceneEntity fmsg_local;
        fmsg_local.id = "cube_288mm";
        fmsg_local.frame_id = "camera";

        scene_updater_->deletions.clear();
        scene_updater_->entities.clear();

        foxglove_msgs::msg::CubePrimitive cube;
        cube.pose = this->cube_pose;
        cube.pose.position.x /= 100.0;
        cube.pose.position.y /= 100.0;
        cube.pose.position.z /= 100.0;

        cube.size.x = 0.288;
        cube.size.y = 0.288;
        cube.size.z = 0.288;

        cube.color.r = 1.0;
        cube.color.g = 0.0;
        cube.color.b = 0.0;
        cube.color.a = 0.7;

        fmsg_local.cubes.push_back(cube);
        scene_updater_->entities.push_back(fmsg_local);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger logger = this->get_logger();
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr foxglove_publisher_;
    geometry_msgs::msg::TransformStamped transformation;
    geometry_msgs::msg::Pose cube_pose;
    double roll, pitch, yaw;
    std::vector<double> center;
    std::vector<std::vector<double>> corners;
    bool tf_received_;
    bool object_created_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    
    rclcpp::Time last_update_time_;
    std::vector<double> last_center_;
    std::vector<std::vector<double>> last_corners_;
    double last_roll_, last_pitch_, last_yaw_;

    foxglove_msgs::msg::SceneUpdate::SharedPtr scene_updater_;
    foxglove_msgs::msg::SceneEntityDeletion deletion_msg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RB_builder_node>();
    rclcpp::spin(node);    
    rclcpp::shutdown();
    return 0;
}