#pragma once

#include "opencv4/opencv2/highgui.hpp"
#include <cmath>


class image_processer
{
public:
    cv::Mat image;
    cv::Mat image_diff, image_diff_inv;
    cv::Mat mask;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::pair<cv::Point2f, float>> circle;
    std::vector<std::vector<cv::Point>> triangle;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Mat> bgr_channels;
    bool consequence;

    image_processer(cv::Mat &image) : image(image), consequence(false) {}

    void preprocess()
    {
        if (image.empty())
        {
            consequence = 0;
            return;
        }

        cv::split(image, bgr_channels);
        for(size_t i = 0; i < bgr_channels.size(); i++)
        {
            cv::threshold(bgr_channels[i], bgr_channels[i], 210, 255, CV_THRESH_BINARY);
        }
        cv::merge(bgr_channels, image);
        cv::subtract(bgr_channels[0], bgr_channels[2], image_diff);
        cv::threshold(image_diff, mask, 70, 255, CV_THRESH_BINARY);
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }

    void filter()
    {        
        contours = contourArea_filter(contours, 300);
        contours = shape_filter(contours, 6);
        cv::drawContours(image, contours, -1, cv::Scalar(0,0,255), cv::FILLED);
    }

    void fitting()
    {
        circle.clear();
        triangle.clear();

        circle.resize(contours.size());
        triangle.resize(contours.size());
        
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::minEnclosingCircle(contours[i], circle[i].first, circle[i].second);
            cv::minEnclosingTriangle(contours[i], triangle[i]);
        }
    }

private:
    cv::Mat gamma_adjust(const cv::Mat &input, double gamma) 
    {
        cv::Mat lookupTable(1, 256, CV_8U);
        uchar* lut = lookupTable.ptr<uchar>();
        
        for (int i = 0; i < 256; ++i) {
            lut[i] = cv::saturate_cast<uchar>(std::pow(i / 255.0, 1.0 / gamma) * 255.0);
        }
        
        cv::Mat output;
        cv::LUT(input, lookupTable, output);
        
        return output;
    }

    cv::Mat hsv_adjust(const cv::Mat &input, double ratio)
    {
        cv::Mat hsv_image, output;
        std::vector<cv::Mat> hsv_channels;
        cv::cvtColor(input, hsv_image, cv::COLOR_BGR2HSV);
        cv::split(hsv_image, hsv_channels);
        hsv_channels[2] *= ratio;
        cv::merge(hsv_channels, output);
        cv::cvtColor(output, output, cv::COLOR_HSV2BGR);
        return output;
    }

    std::vector<std::vector<cv::Point>> contourArea_filter(const std::vector<std::vector<cv::Point>> contours, const int thres)
    {
        std::vector<std::vector<cv::Point>> contours_filtered;

        for (const auto &contour : contours)
        {
            if (cv::contourArea(contour) > thres)
            {
                contours_filtered.push_back(contour);
            }
        }
        
        return contours_filtered;
    }

    std::vector<std::vector<cv::Point>> shape_filter(const std::vector<std::vector<cv::Point>> contours, const int thres)
    {
        float peri;
        std::vector<cv::Point> conPoly;
        std::vector<std::vector<cv::Point>> contours_filtered;
        
        for (const auto &contour : contours)
        {
            peri = cv::arcLength(contour, true);
            cv::approxPolyDP(contour, conPoly, 0.02 * peri, true);
            if (conPoly.size() > static_cast<size_t>(thres - 1) && conPoly.size() < static_cast<size_t>(thres + 3))
            {
                contours_filtered.push_back(contour);
            }
        }
        return contours_filtered;
    }



};