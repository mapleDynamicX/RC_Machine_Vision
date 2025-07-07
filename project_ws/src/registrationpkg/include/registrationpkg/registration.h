#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// 引入点云容器类定义头文件
#include <pcl/point_cloud.h>
// 引入体素网格滤波器头文件，用于点云下采样
#include <pcl/filters/voxel_grid.h>
// 引入法线估计类头文件，用于计算点云法线
#include <pcl/features/normal_3d.h>
// 引入ICP(迭代最近点)配准算法头文件
#include <pcl/registration/icp.h>
// 引入非线性ICP配准算法头文件
#include <pcl/registration/icp_nl.h>
// 引入点对面变换估计头文件，使用最小二乘法
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cstdbool>


// 定义点类型为带颜色(RGB)的XYZ点
typedef pcl::PointXYZRGB PoinTT;
// 定义点云类型为XYZRGB点组成的点云
typedef pcl::PointCloud<PoinTT> PointCloudT;

class registration
{
public:
    registration(std::string transform_topic, std::string full_map_path, std::string cloud_sub_name):
    _nh("/"),
    _transform_topic(transform_topic),
    _full_map_path(full_map_path),
    _cloud_sub_name(cloud_sub_name),
    _cloud_full(new PointCloudT),
    _cloud_current(new PointCloudT),
    _transformation(false)
    {
        // 从文件加载参考点云
        pcl::io::loadPCDFile(_full_map_path, *_cloud_full);
        //创建发布者
        //_odom_pub = _nh.advertise<nav_msgs::Odometry>(_odom_pub_name, 50);
        _transform_pub = _nh.advertise<geometry_msgs::TransformStamped>(_transform_topic, 10);
        //订阅点云
        _cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &registration::cloudCallback, this);
        // 创建TF广播器对象
        _tf_broadcaster = std::make_unique<tf::TransformBroadcaster>();

    }
    registration():_nh("/"), _transformation(false),
    _cloud_full(new PointCloudT),
    _cloud_current(new PointCloudT)
    {

        get_param();
        // 从文件加载参考点云"/home/maple/study/project_ws/src/registrationpkg/pcd/full_map.pcd"
        // pcl::PCDReader reader;
        // std::cout<<_full_map_path<<std::endl;
        // if(reader.read<PoinTT> (_full_map_path, *_cloud_full) == -1)
        // {
        //     std::cout<<"read fail"<<std::endl;
        // }

        if(pcl::io::loadPCDFile(_full_map_path, *_cloud_full) == -1)
        {
           std::cout<<"read fail"<<std::endl;
        }
        std::cout<<_cloud_full->is_dense<<std::endl;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_cloud_full, *_cloud_full, indices);  // 原地覆盖
        //创建发布者
        //_odom_pub = _nh.advertise<nav_msgs::Odometry>(_odom_pub_name, 50);
        _transform_pub = _nh.advertise<geometry_msgs::TransformStamped>(_transform_topic, 10);
        //订阅点云
        _cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &registration::cloudCallback, this);
        // 创建TF广播器对象
        _tf_broadcaster = std::make_unique<tf::TransformBroadcaster>();
        std::cout<<"init successed"<<std::endl;
    }

    void get_param();
    //过滤无效点云
    size_t removeInvalidPointsManual(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    //点云配准
    Eigen::Matrix4f registerPointClouds();
    //订阅点云
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    ros::NodeHandle _nh;
    ros::Subscriber _cloud_sub;
    //ros::Publisher _odom_pub;
    //std::string _odom_pub_name;
    ros::Publisher _transform_pub;
    std::string _transform_topic;
    std::string _cloud_sub_name;
    std::string _full_map_path;
    PointCloudT::Ptr _cloud_full;
    PointCloudT::Ptr _cloud_current;
    bool _transformation;
    // TF广播器智能指针
    std::unique_ptr<tf::TransformBroadcaster> _tf_broadcaster;
};
