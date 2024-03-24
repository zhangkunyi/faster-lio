#include "image_manager.hpp"

void ImageManager::captureImageCallback(const std_msgs::UInt8Ptr& msg)
{
    ROS_INFO("capture image is received");
    if(has_image_send_)
    {
        createFolder(image_save_folder_path_);
        has_image_send_ = false;
    }

    try {
        if(has_image_)
        {
            ros::Duration(capture_image_delay_time_).sleep();
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(current_image_msg_, "rgb8");
            //保存图像到指定文件夹
            std::string filename = "image_" + std::to_string(current_image_msg_.header.stamp.toSec()) + ".png";  // 使用时间戳创建唯一的文件名

            std::string filePath = image_save_folder_path_ + "/" + filename;
            std::cout << "filePath " << filePath << std::endl;
            cv::imwrite(filePath, cv_ptr->image);
        }

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void ImageManager::receiveImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_ONCE("Image is received");
    has_image_ = true;
    current_image_msg_ = *msg;
}

void ImageManager::createFolder(const std::string& folder_path)
{
    ROS_INFO("Create Folder");
    std::string folder_name = std::to_string(ros::Time::now().toSec());
    image_save_folder_path_ = folder_path + "/" + folder_name;
    std::stringstream ss;
    ss << image_save_folder_path_;
    fs::create_directory(image_save_folder_path_);
}

void ImageManager::sendImageCallback(const std_msgs::Empty& msg)
{

    // Check if there are images in the folder
    bool hasImages = false;
    for (const auto& entry : fs::directory_iterator(image_save_folder_path_)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
            hasImages = true;
            break;
        }
    }
    if (!hasImages) {
        ROS_ERROR("No images found in the folder: %s", image_save_folder_path_.c_str());
        return; // Abort if no images are found
    }

    ROS_INFO_ONCE("send image is received");
    for (const auto& entry : fs::directory_iterator(image_save_folder_path_)) {
        std::cout <<"I am here!" << std::endl; 
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
            cv::Mat image = cv::imread(entry.path().string(), cv::IMREAD_COLOR);
            std::string base64Data = imageToBase64(image);
            std_msgs::String base64_string;
            base64_string.data = base64Data;
            std::cout <<"I am here1!" << std::endl; 
            send_image_pub_.publish(base64_string);
        }
    }
    has_image_send_ = true;
}

std::string ImageManager::imageToBase64(const cv::Mat& image) {
    std::vector<uchar> buffer;
    cv::imencode(".png", image, buffer);

    // 使用 const unsigned char* 转换
    std::string encoded = base64_encode(reinterpret_cast<const unsigned char*>(buffer.data()), buffer.size());
    return encoded;
}