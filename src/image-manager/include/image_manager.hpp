#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "base64.h" 
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <filesystem>
namespace fs = std::filesystem;

class ImageManager
{
public:
    ImageManager() = default;
    ~ImageManager() = default;
    void init(ros::NodeHandle& nh)
    {
        capture_image_sub_ = nh.subscribe("/capture_image", 10, &ImageManager::captureImageCallback, this);
        receive_image_sub_ = nh.subscribe("/camera/color/image_raw", 10, &ImageManager::receiveImageCallback, this);
        send_image_sub_ = nh.subscribe("/send_image", 10, &ImageManager::sendImageCallback, this);
        send_image_pub_ = nh.advertise<std_msgs::String>("/image_topic", 1000);
        nh.getParam("folder_path", image_save_folder_path_);
        nh.param("capture_image_delay_time", capture_image_delay_time_, 0.0);
    }
    void captureImageCallback(const std_msgs::UInt8Ptr& msg);
    void receiveImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void sendImageCallback(const std_msgs::Empty& msg);
    void createFolder(const std::string& folder_path);
private:
    void sendImageToOther();
    std::string imageToBase64(const cv::Mat& image);
    bool has_image_send_ = true;
    bool has_image_ = false;
    std::string image_save_folder_path_;
    sensor_msgs::Image current_image_msg_;
    ros::Publisher send_image_pub_;
    ros::Subscriber receive_image_sub_;  // receive image
    ros::Subscriber capture_image_sub_;  // trigger capture image
    ros::Subscriber send_image_sub_;   // send current photos
    double capture_image_delay_time_;  // second
};