#include "../include/publishpkg/publish.h"


void pub::pub_image()
{
    if(_cap_open)
    {
        cv::Mat frame;
        _cap >> frame;
        if (frame.empty()) {
            ROS_WARN("frame are empty");
            return;
        }
        sensor_msgs::ImagePtr raw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        raw_msg->header.stamp = ros::Time::now();
        _raw_pub.publish(raw_msg);
    }
    else
    {
        ROS_WARN("Can not open video");
    }
}
void pub::pub_compress_img()
{
    if(_cap_open)
    {
        cv::Mat frame;
        _cap >> frame;
        if (frame.empty()) {
            ROS_WARN("frame are empty");
            return;
        }
        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.header.stamp = ros::Time::now();
        compressed_msg.format = "jpeg";
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(80);  // 质量参数 (0-100)
        if (cv::imencode(".jpg", frame, compressed_msg.data, compression_params)) {
            _compressed_pub.publish(compressed_msg);
        } else {
            ROS_ERROR("图像压缩失败");
        }
    }
    else
    {
        ROS_WARN("Can not open video");
    }
}

void pub::get_param()
{
    _nh.param("number",p._number,2);
    _nh.param("image_name", p._name_raw,std::string("camera/iamge_raw"));
    _nh.param("compress_image_name", p._name_compress,std::string( "camera/compress_image"));
}