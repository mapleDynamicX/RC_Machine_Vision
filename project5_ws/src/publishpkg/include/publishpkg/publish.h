#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cstdbool>
#include <iostream>

struct param
{
    int _number;
    std::string _name_raw;
    std::string _name_compress;
};

class pub
{
public:
    pub(int number, std::string name_raw, std::string name_compress):_nh("/"),_it(_nh), _cap(number,cv::CAP_V4L2), 
    _raw_pub(_it.advertise(name_raw, 10)),
    _compressed_pub(_nh.advertise<sensor_msgs::CompressedImage>(name_compress, 10)),
    _cap_open(false)
    {
        if (_cap.isOpened())
        {
            _cap_open = true;
        }
    } 
    pub():_nh("/"),_it(_nh)
    {
        get_param();
        _raw_pub = _it.advertise(p._name_raw, 10);
        _compressed_pub = _nh.advertise<sensor_msgs::CompressedImage>(p._name_compress, 10);
        _cap.open(p._number,cv::CAP_V4L2);
        if (_cap.isOpened())
        {
            std::cout<<"_cap.isOpened()"<<std::endl;
            _cap_open = true;
        }
        else
        {
            while(!_cap.isOpened())
            {
                _cap.open(p._number,cv::CAP_V4L2);
            }
        }
    }
    void pub_image();//发布原图
    void pub_compress_img();//发布压缩图片
    void get_param();//获得参数
    ~pub()
    {
        if(_cap_open)
        {
            _cap.release();
        }
        _raw_pub.shutdown();
        _compressed_pub.shutdown();
    }
private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    cv::VideoCapture _cap;
    image_transport::Publisher _raw_pub;
    ros::Publisher _compressed_pub;
    bool _cap_open = false;
    param p;
};

