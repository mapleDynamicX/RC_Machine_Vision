#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/subscriber_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <cstdbool>



class camerainfo
{
public:
    camerainfo(std::string name):_nh("/")
    {
        _sub = _nh.subscribe(name, 1, &camerainfo::cameraInfoCallback, this);

    }
    camerainfo():_nh("/")
    {
        get_param();
        _sub = _nh.subscribe(_camerainfo_name, 1, &camerainfo::cameraInfoCallback, this);
    }
    void get_param();
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    std::string _camerainfo_name;
    std::string _frame_id;
    int _height;
    int _width;
    std::vector<double> _D;//畸变系数
    boost::array<double, 9> _K;//内参矩阵
    boost::array<double, 9> _R;//旋转矩阵
    boost::array<double, 12> _P;//投影矩阵
    bool _cam_model_initialized = false;
};

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Sync;//用于rgb和深度图时间对齐时促发
class bridge
{
public:
    bridge():_camerainfo(), _nh("/"), _it(_nh)
    {
        _pub_cloud = _nh.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 10);
        get_param();
        // 订阅同步的RGB和深度图像
        _rgb_sub.subscribe(_it, _rgb_sub_name, 10);
        _depth_sub.subscribe(_it, _depth_sub_name, 10);
        sync_.reset(new Sync(SyncPolicy(10), _rgb_sub, _depth_sub));
        sync_->registerCallback(boost::bind(&bridge::imageCallback, this, _1, _2));
    }
    void get_param();
    //同步订阅RGB和深度图像
    void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg);
private:
    camerainfo _camerainfo;
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    std::string _rgb_sub_name;
    std::string _depth_sub_name;
    ros::Publisher _pub_cloud;
    image_transport::SubscriberFilter _rgb_sub;
    image_transport::SubscriberFilter _depth_sub;
    boost::shared_ptr<Sync> sync_;

};

