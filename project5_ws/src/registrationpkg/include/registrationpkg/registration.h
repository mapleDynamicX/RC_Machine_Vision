#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>// 定义点云数据类型
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
// 包含所需的PCL点云库头文件  
#include <pcl/registration/icp.h>  // 迭代最近点配准算法
#include <pcl/registration/sample_consensus_prerejective.h>  // 基于采样的预估计配准方法
#include <pcl/keypoints/uniform_sampling.h>  // 均匀关键点采样
#include <pcl/features/normal_3d_omp.h>  // 基于OpenMP并行的法线估计
#include <pcl/features/fpfh_omp.h>  // 基于OpenMP并行的FPFH特征计算
#include <Eigen/Dense>
#include <iostream>
#include <cstdbool>


// 定义点类型为带颜色(RGB)的XYZ点
typedef pcl::PointXYZRGB PoinTT;
// 定义点云类型为XYZRGB点组成的点云
typedef pcl::PointCloud<PoinTT> PointCloudTT;

class registration
{
public:
    registration(std::string transform_topic, std::string full_map_path, std::string cloud_sub_name):
    _nh("/"),
    _transform_topic(transform_topic),
    _full_map_path(full_map_path),
    _cloud_sub_name(cloud_sub_name),
    _cloud_full(new PointCloudTT),
    _cloud_current(new PointCloudTT),
    _transformation(false),
    _cloud_full_downsampled(new PointCloudTT),
    _normals_full(new pcl::PointCloud<pcl::Normal>),
    _cloud_current_downsampled(new PointCloudTT),
    _max_iteration(0)
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

        // 创建体素网格滤波器对象
        pcl::VoxelGrid<PoinTT> voxel_grid;
        // 设置体素大小为0.1cm立方体
        voxel_grid.setLeafSize(0.001f, 0.001f, 0.001f);
        // 设置输入参考点云
        voxel_grid.setInputCloud(_cloud_full);
        // 执行下采样，结果存入cloud_full_downsampled
        voxel_grid.filter(*_cloud_full_downsampled);

        // 创建法线估计对象
        pcl::NormalEstimation<PoinTT, pcl::Normal> ne2;
        // 创建法线点云的智能指针
        //pcl::PointCloud<pcl::Normal>::Ptr normals_full(new pcl::PointCloud<pcl::Normal>);
        // 创建KD树搜索对象
        pcl::search::KdTree<PoinTT>::Ptr tree2(new pcl::search::KdTree<PoinTT>());
        // 设置法线估计的输入点云（下采样后的当前点云）
        ne2.setInputCloud(_cloud_full_downsampled);
        // 设置搜索方法为KD树
        ne2.setSearchMethod(tree2);
        // 设置法线估计的搜索半径为3cm
        //ne2.setRadiusSearch(0.03);
        ne2.setKSearch(30);
        // 计算法线并存入normals_current
        ne2.compute(*_normals_full);

        std::cout<<"init successed"<<std::endl;

    }
    registration():_nh("/"), _transformation(false),
    _cloud_full(new PointCloudTT),
    _cloud_current(new PointCloudTT),
    _cloud_full_downsampled(new PointCloudTT),
    _normals_full(new pcl::PointCloud<pcl::Normal>),
    _cloud_current_downsampled(new PointCloudTT),
    _max_iteration(0)
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

        // 创建体素网格滤波器对象
        pcl::VoxelGrid<PoinTT> voxel_grid;
        // 设置体素大小为0.1cm立方体
        voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
        // 设置输入参考点云
        voxel_grid.setInputCloud(_cloud_full);
        // 执行下采样，结果存入cloud_full_downsampled
        voxel_grid.filter(*_cloud_full_downsampled);

        // 创建法线估计对象
        pcl::NormalEstimation<PoinTT, pcl::Normal> ne2;
        // 创建法线点云的智能指针
        //pcl::PointCloud<pcl::Normal>::Ptr normals_full(new pcl::PointCloud<pcl::Normal>);
        // 创建KD树搜索对象
        pcl::search::KdTree<PoinTT>::Ptr tree2(new pcl::search::KdTree<PoinTT>());
        // 设置法线估计的输入点云（下采样后的当前点云）
        ne2.setInputCloud(_cloud_full_downsampled);
        // 设置搜索方法为KD树
        ne2.setSearchMethod(tree2);
        // 设置法线估计的搜索半径为3cm
        //ne2.setRadiusSearch(0.03);
        ne2.setKSearch(30);
        // 计算法线并存入normals_current
        ne2.compute(*_normals_full);
        std::cout<<"init successed"<<std::endl;
    }

    void get_param();
    //过滤无效点云
    size_t removeInvalidPointsManual(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    //点云配准
    Eigen::Matrix4f registerPointClouds();
    
    Eigen::Matrix4f registerPointClouds2RANSAC();
    Eigen::Matrix4f registerPointClouds2ICP();
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
    PointCloudTT::Ptr _cloud_full;
    PointCloudTT::Ptr _cloud_current;
    PointCloudTT::Ptr _cloud_full_downsampled;
    pcl::PointCloud<pcl::Normal>::Ptr _normals_full;
    PointCloudTT::Ptr _cloud_current_downsampled;
    Eigen::Matrix4f _transform;
    bool _transformation;
    int _max_iteration;
    // TF广播器智能指针
    std::unique_ptr<tf::TransformBroadcaster> _tf_broadcaster;
};
