#pragma once
#include <vector>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include <cmath>

class state_assessor
{
public:
    state_assessor(rclcpp::Node::SharedPtr node) : node_(node) {}
    ~state_assessor() = default;

    bool ready_to_act(cv::Point3f &center_point) {
        if(center_history_.size() < 15) {
            center_history_.push_back(center_point);
            return false;
        } else {
            center_history_.pop_front();
            center_history_.push_back(center_point);
        }

        cv::Point3f sum = {0, 0, 0};
        cv::Point3f mean = {0, 0, 0};
        
        for (const auto& point : center_history_) {
            mean.x += point.x;
            mean.y += point.y;
            mean.z += point.z;
        }
        mean.x /= center_history_.size();
        mean.y /= center_history_.size();
        mean.z /= center_history_.size();
        
        for (const auto& point : center_history_) {
            sum.x += pow(point.x - mean.x, 2);
            sum.y += pow(point.y - mean.y, 2);
            sum.z += pow(point.z - mean.z, 2);
        }
        
        float variance_x = sum.x / center_history_.size();
        float variance_y = sum.y / center_history_.size();
        float variance_z = sum.z / center_history_.size();
        float total_variance = sqrt(variance_x + variance_y + variance_z);

        float threshold = 5.0;
        if (total_variance > threshold) {
            return false;
        }
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::deque<cv::Point3f> center_history_;
};