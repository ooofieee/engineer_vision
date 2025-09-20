#pragma once

#include "opencv4/opencv2/highgui.hpp"
#include <numeric>
#include <queue>
#include <unordered_map>

class redeem_box
{
public:
    struct _features_
    {
        //minEncloingTriangle
        std::vector<std::vector<cv::Point2f>> triangle;
        std::vector<cv::Point2f> max_corner;

        //minEnclosingCircle
        std::vector<cv::Point2f> circle;
        std::vector<float> radius;
    };

    struct _result_
    {
        cv::Point2f center_2d;
        std::vector<cv::Point2f> corners;
        enum direction {LEFT, RIGHT};
        bool consequence;
    };

    _features_ features;
    _result_ result;


    redeem_box ( std::vector<std::vector<cv::Point>> &triangle,
                 std::vector<std::pair<cv::Point2f, float>> &circle)
    {
        this->features.triangle.clear();
        this->features.circle.clear();
        this->features.radius.clear();
        this->features.max_corner.clear();
        this->features.triangle.resize(triangle.size());
        this->features.circle.resize(circle.size());
        this->features.radius.resize(circle.size());
        this->features.max_corner.resize(triangle.size());

        int count = 0;
        for (const auto &tri : triangle)
        {
            for (const auto &point : tri)
            {
                this->features.triangle[count].push_back(cv::Point2f(point));
            }
            count++;
        }

        for (const auto &circle_ : circle)
        {
            this->features.circle.push_back(circle_.first);
            this->features.radius.push_back(circle_.second);
        }

        max_corner_calculator();
        middle_point_screen(this->features.max_corner, 50.0f);
        this->result.corners = sortPointsClockwise(this->result.corners, this->result.center_2d);
        time_smoother(this->result.corners);
    }



private:

    void max_corner_calculator ()
    {
        for (size_t i = 0; i < features.triangle.size(); i++)
        {
            if (features.triangle[i].size() == 3)
            {
                double length0 = cv::norm(features.triangle[i][1] - features.triangle[i][2]);
                double length1 = cv::norm(features.triangle[i][0] - features.triangle[i][2]);
                double length2 = cv::norm(features.triangle[i][1] - features.triangle[i][0]);

                if (length0 > length1 && length0 > length2)
                    features.max_corner[i] = features.triangle[i][0];
                else if (length1 > length0 && length1 > length2)
                    features.max_corner[i] = features.triangle[i][1];
                else
                    features.max_corner[i] = features.triangle[i][2];
            }
        }
    }

    void middle_point_screen(const std::vector<cv::Point2f>& points, float thres) 
    {
        // 使用空间网格加速查找
        std::unordered_map<int, std::vector<int>> gridMap;
        const float gridSize = thres * 2.0f;  // 网格大小设为阈值的2倍
        
        // 计算网格键值的lambda
        auto getGridKey = [gridSize](const cv::Point2f& pt) {
            int gridX = static_cast<int>(pt.x / gridSize);
            int gridY = static_cast<int>(pt.y / gridSize);
            return gridX * 10000 + gridY;  // 简单哈希键
        };
        
        // 存储中点及其对应点对
        std::vector<std::pair<cv::Point2f, std::pair<int, int>>> midPoints;
        
        for (size_t i = 0; i < points.size(); i++) {
            for (size_t j = i + 1; j < points.size(); j++) {
                // 计算中点
                cv::Point2f mid = 0.5f * (points[i] + points[j]);
                
                // 获取当前中点的网格键
                int key = getGridKey(mid);
                
                // 检查邻近网格
                bool found = false;
                for (int idx : gridMap[key]) {
                    float dist = cv::norm(midPoints[idx].first - mid);
                    if (dist < thres) {
                        // 获取点对索引
                        int idx1 = midPoints[idx].second.first;
                        int idx2 = midPoints[idx].second.second;
                        
                        // 确保四个不同的点
                        if (idx1 != static_cast<int>(i) && idx1 != static_cast<int>(j) &&
                            idx2 != static_cast<int>(i) && idx2 != static_cast<int>(j)) {
                            
                            // 找到匹配点对
                            this->result.corners = {
                                points[idx1], points[idx2],
                                points[i], points[j]
                            };
                            this->result.center_2d = 0.5f * (mid + midPoints[idx].first);
                            return;
                        }
                    }
                }
                
                // 未找到匹配，添加新中点
                if (!found) {
                    midPoints.emplace_back(mid, std::make_pair(i, j));
                    gridMap[key].push_back(midPoints.size() - 1);
                }
            }
        }
        
        // 未找到符合条件的点
        this->result.consequence = 0;
    }

    bool DistVarianceValidation(const std::vector<cv::Point2f> &circle_pts, const std::vector<cv::Point2f> &triangle_pts, float max_variance = 100.0f)
    {
        if (circle_pts.size() != 4 || triangle_pts.size() != 4)
        {
            this->result.consequence = 0;
            return false;
        }

        std::vector<float> dists;
        for (int i = 0; i < 4; ++i)
        {
            float d = cv::norm(circle_pts[i] - triangle_pts[i]);
            dists.push_back(d);
        }

        float mean = std::accumulate(dists.begin(), dists.end(), 0.0f) / dists.size();

        float variance = 0.0f;
        for (float d : dists)
        {
            variance += (d - mean) * (d - mean);
        }
        variance /= dists.size();
        if (variance < max_variance)
        {
            this->result.consequence = 1;
            return true;
        }
        else
        {
            this->result.consequence = 0;
            return false;
        }
    }

    std::vector<cv::Point2f> sortPointsClockwise(const std::vector<cv::Point2f> &points, const cv::Point2f &center)
    {
        std::vector<std::pair<cv::Point2f, double>> angle_point_pairs;

        for (const auto &pt : points)
        {
            double angle = atan2(pt.y - center.y, pt.x - center.x);
            angle_point_pairs.emplace_back(pt, angle);
        }

        std::sort(angle_point_pairs.begin(), angle_point_pairs.end(),
                  [](const std::pair<cv::Point2f, double> &a, const std::pair<cv::Point2f, double> &b)
                  {
                      return a.second > b.second;
                  });

        std::vector<cv::Point2f> sorted;
        for (const auto &pair : angle_point_pairs)
        {
            sorted.push_back(pair.first);
        }

        return sorted;
    }

    std::vector<cv::Point2f> time_smoother(std::vector<cv::Point2f> &points)
    {
        std::vector<cv::Point2f> point_temp;
        std::queue<std::vector<cv::Point2f>> point_queue;
        point_temp.resize(4, cv::Point2f(0.0, 0.0));

        auto isZeroPoint = [](const cv::Point2f &pt)
        {
            return pt.x == 0.0f && pt.y == 0.0f;
        };

        if (points.size() != 4 || std::any_of(points.begin(), points.end(), isZeroPoint))
        {
            std::cout<<"Invalid or zero points in input, skipping filter"<<std::endl;
            return point_temp;
        }

        if (point_queue.size() < 2)
        {
            point_queue.push(points);
        }
        else
        {
            point_queue.pop();
            point_queue.push(points);
        }

        if (point_queue.size() == 2)
        {
            const auto &newest = point_queue.back();
            const auto &oldest = point_queue.front();

            if (newest.size() == 4 && oldest.size() == 4 &&
                !std::any_of(newest.begin(), newest.end(), isZeroPoint) &&
                !std::any_of(oldest.begin(), oldest.end(), isZeroPoint))
            {
                for (size_t i = 0; i < 4; i++)
                {
                    point_temp[i] = newest[i] * 0.2f + oldest[i] * 0.8f;
                }
            }
            else
            {
                std::cout<< "Queue contains zero points, skipping filter"<<std::endl;;
            }
            return point_temp;
        }

        return point_temp;
    }

};