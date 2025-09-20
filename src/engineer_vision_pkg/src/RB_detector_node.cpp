#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_processer.hpp"
#include "redeem_box.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <numeric>

class RB_detector_node : public rclcpp::Node
{
private:
    // Node related variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    
    cv::Mat frame;
    cv_bridge::CvImagePtr framePtr;
    
    // Camera related variables
    cv::Mat camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix;
    cv::Mat rvec, tvec, obj2Cam;
    std::vector<cv::Point3f> objPoints = {cv::Point3f(-120.0f, -120.0f, 0.0f),
                                          cv::Point3f(120.0f, -120.0f, 0.0f),
                                          cv::Point3f(120.0f, 120.0f, 0.0f),
                                          cv::Point3f(-120.0f, 120.0f, 0.0f)};
    std::vector<cv::Point3f> objCorners2cam;
    cv::Point3f objCenter2cam;
    
    // Point tracking variables
    std::vector<cv::Point2f> circle_pt_sorted, triangle_pt_sorted;
    
    // Transform related variables
    geometry_msgs::msg::TransformStamped transformation;
    std::vector<geometry_msgs::msg::TransformStamped> transformation_corners;
    tf2::Quaternion q;
    double roll, pitch, yaw;

public:
    RB_detector_node(const std::string node_name) : rclcpp::Node(node_name)
    {
        RCLCPP_INFO(get_logger(), "RB_detector_node launched");
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_frame", 10);
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        publish_static_camera_transform();

        
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("redeem_box_image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg)->void{
            framePtr = cv_bridge::toCvCopy(msg, msg->encoding);
            frame = framePtr->image;
            image_processer processer(frame);
            processer.preprocess();
            processer.filter();
            processer.fitting();
            frame = processer.image;
            redeem_box target(processer.triangle, processer.circle);

            loadCameraParams();
            pnpSolver(target.result.corners);
            publish_tf(tvec, roll, pitch, yaw, objCorners2cam);
            visualize(target);
            publish_frame();

        });
    }

    void publish_frame()
    {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        this->publisher_->publish(*msg);
    }

    void visualize(const redeem_box &target)
    {
        for (const auto corner :target.result.corners)
        {
            cv::circle(frame, corner, 10, cv::Scalar(0, 255, 255), cv::FILLED);
        }
        for (size_t i = 0; i < target.result.corners.size(); i++)
        {
            cv::putText(frame, std::to_string(i), target.result.corners[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(frame, target.result.center_2d, 10, cv::Scalar(255, 0, 0), cv::FILLED);
        cv::putText(frame, "Center", target.result.center_2d, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        cv::putText(frame, "Roll: " + std::to_string(roll * 360 / (2 * CV_PI)), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Pitch: " + std::to_string(pitch * 360 / (2 * CV_PI)), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Yaw: " + std::to_string(yaw * 360 / (2 * CV_PI)), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2); 
    }

    void loadCameraParams()
    {
        cv::FileStorage fs("/home/mac/Code/ws_0/src/engineer_vision_pkg/config/camera_info.yaml", cv::FileStorage::READ);
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distortion_coefficients;
        fs["rectification_matrix"] >> rectification_matrix;
        fs["projection_matrix"] >> projection_matrix;
        fs.release();
    }

    void pnpSolver(const std::vector<cv::Point2f> &points)
    {
        if(points.size() != 4)
        {
            RCLCPP_WARN(get_logger(), "Invalid number of points for PnP: expected 4, got %zu", points.size());
            return;
        }
        
        objCorners2cam.clear();

        bool success = cv::solvePnPRansac(objPoints, points, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        if (success)
        {
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            obj2Cam = rotation_matrix.t();
            
            roll = atan2(obj2Cam.at<double>(2, 1), obj2Cam.at<double>(2, 2));
            pitch = atan2(-obj2Cam.at<double>(2, 0), sqrt(obj2Cam.at<double>(2, 1) * obj2Cam.at<double>(2, 1) + obj2Cam.at<double>(2, 2) * obj2Cam.at<double>(2, 2)));
            yaw = atan2(obj2Cam.at<double>(1, 0), obj2Cam.at<double>(0, 0));
            
            cv::Matx33f R_matx = obj2Cam;
            cv::Vec3f t_vec(tvec);
            
            for(int i = 0; i < 4; i++)
            {
                cv::Vec3f cam_vec = R_matx * cv::Vec3f(objPoints[i].x, objPoints[i].y, objPoints[i].z) + t_vec;
                cv::Point3f corners2cam(cam_vec[0], cam_vec[1], cam_vec[2]);
                objCorners2cam.push_back(corners2cam);
            }

            cv::Vec3f center_cam_vec = R_matx * cv::Vec3f(0.0f, 0.0f, 0.0f) + t_vec;
            objCenter2cam = cv::Point3f(center_cam_vec[0], center_cam_vec[1], center_cam_vec[2]);
            
            RCLCPP_INFO(get_logger(), "PnP solved successfully, computed %zu corner positions", objCorners2cam.size());
            RCLCPP_INFO(get_logger(), "Target center 3D position: (%.3f, %.3f, %.3f)", 
                       objCenter2cam.x, objCenter2cam.y, objCenter2cam.z);
            for(int i = 0; i < 4; i++) {
                RCLCPP_DEBUG(get_logger(), "Corner %d: (%.3f, %.3f, %.3f)", 
                           i, objCorners2cam[i].x, objCorners2cam[i].y, objCorners2cam[i].z);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "solvePnP failed");
            return;
        }
    }

    void publish_tf(const cv::Mat &translation, double roll, double pitch, double yaw, const std::vector<cv::Point3f> &objCorners2cam)
    {
        if (!(translation.rows == 3 && translation.cols == 1) &&
            !(translation.rows == 1 && translation.cols == 3))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid translation shape: expected 3x1 or 1x3, got %dx%d", translation.rows, translation.cols);
            return;
        }
    
        if (translation.type() != CV_64F)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid translation type: expected CV_64F");
            return;
        }
    
        if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid rotation angles: roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
            return;
        }

        if (objCorners2cam.size() != 4)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid number of corners: expected 4, got %zu", objCorners2cam.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing transform with translation: (%.3f, %.3f, %.3f) and rotation: (roll=%.1f°, pitch=%.1f°, yaw=%.1f°)", 
            translation.at<double>(0), translation.at<double>(1), translation.at<double>(2), 
            roll * 180.0 / CV_PI, pitch * 180.0 / CV_PI, yaw * 180.0 / CV_PI);

        transformation.header.stamp = this->get_clock()->now();
        transformation.header.frame_id = "camera";
        transformation.child_frame_id = "target";
    
        transformation.transform.translation.x = translation.at<double>(0);
        transformation.transform.translation.y = translation.at<double>(1);
        transformation.transform.translation.z = translation.at<double>(2);

        q.setRPY(roll, pitch, yaw);
        transformation.transform.rotation = tf2::toMsg(q);
        broadcaster_->sendTransform(transformation);

        geometry_msgs::msg::TransformStamped center_transform;
        center_transform.header.stamp = this->get_clock()->now();
        center_transform.header.frame_id = "camera";
        center_transform.child_frame_id = "center";
        
        center_transform.transform.translation.x = objCenter2cam.x;
        center_transform.transform.translation.y = objCenter2cam.y;
        center_transform.transform.translation.z = objCenter2cam.z;
        center_transform.transform.rotation = tf2::toMsg(q);
        broadcaster_->sendTransform(center_transform);

        transformation_corners.resize(4);

        for (int i = 0; i < 4; i++)
        {
            transformation_corners[i].header.stamp = this->get_clock()->now();
            transformation_corners[i].header.frame_id = "camera";
            transformation_corners[i].child_frame_id = "corner" + std::to_string(i);

            transformation_corners[i].transform.translation.x = objCorners2cam[i].x;
            transformation_corners[i].transform.translation.y = objCorners2cam[i].y;
            transformation_corners[i].transform.translation.z = objCorners2cam[i].z;
            transformation_corners[i].transform.rotation = tf2::toMsg(q);
            broadcaster_->sendTransform(transformation_corners[i]);
            
            RCLCPP_DEBUG(this->get_logger(), "Published corner%d: (%.3f, %.3f, %.3f)", 
                        i, objCorners2cam[i].x, objCorners2cam[i].y, objCorners2cam[i].z);
        }

        RCLCPP_INFO(this->get_logger(), "Successfully published target and %zu corner transforms", objCorners2cam.size());
    }

    void publish_static_camera_transform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "panda_hand";
        static_transform.child_frame_id = "camera";
        
        static_transform.transform.translation.x = 0.1;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = -0.05;
        
        tf2::Quaternion camera_orientation;
        camera_orientation.setRPY(0.0, 0.1745, 0.0);
        
        static_transform.transform.rotation = tf2::toMsg(camera_orientation);
        
        static_broadcaster_->sendTransform(static_transform);
        
        RCLCPP_INFO(get_logger(), "Published static transform from panda_hand to camera");
        RCLCPP_INFO(get_logger(), "Camera position: [%.3f, %.3f, %.3f]", 
                   static_transform.transform.translation.x,
                   static_transform.transform.translation.y, 
                   static_transform.transform.translation.z);
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RB_detector_node>("RB_detector_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}