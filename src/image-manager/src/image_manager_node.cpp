#include "image_manager.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_manager");
    ros::NodeHandle nh("~");
    ros::Rate rate(10); //10hz
    ROS_INFO("I am ready");
    ImageManager image_manager;
    image_manager.init(nh);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}