// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>// 定义点云数据类型
// #include <pcl_conversions/pcl_conversions.h>
// // 引入点云容器类定义头文件
// #include <pcl/point_cloud.h>
// // 引入体素网格滤波器头文件，用于点云下采样
// #include <pcl/filters/voxel_grid.h>
// // 引入法线估计类头文件，用于计算点云法线
// #include <pcl/features/normal_3d.h>
// // 引入ICP(迭代最近点)配准算法头文件
// #include <pcl/registration/icp.h>
// // 引入非线性ICP配准算法头文件
// #include <pcl/registration/icp_nl.h>
// // 引入点对面变换估计头文件，使用最小二乘法
// #include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
// // 包含所需的PCL点云库头文件  
// #include <pcl/registration/sample_consensus_prerejective.h>  // 基于采样的预估计配准方法
// #include <pcl/keypoints/uniform_sampling.h>  // 均匀关键点采样
// #include <pcl/features/normal_3d_omp.h>  // 基于OpenMP并行的法线估计
// #include <pcl/features/fpfh_omp.h>  // 基于OpenMP并行的FPFH特征计算
// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <csignal>
// #include <fstream>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <mutex>
// #include <nav_msgs/Odometry.h>
// #include <pcl/common/angles.h>
// #include <pcl/common/transforms.h>
// #include <pcl/features/fpfh.h>
// #include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/ia_ransac.h>
// #include <pcl/registration/ndt.h>
// #include <sensor_msgs/LaserScan.h>
// #include <tf/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf_conversions/tf_eigen.h>
// #include <vector>
// #include <tf2_ros/buffer.h>
// #include <functional>
// #include <std_msgs/String.h>
// #include <tf2_msgs/TFMessage.h>
// #include <iostream>
// #include <cstdbool>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include <memory>
#include <iostream>
#include <cstdbool>

// 定义点类型为带颜色(RGB)的XYZ点
typedef pcl::PointXYZRGB PointTT;
// 定义点云类型为XYZRGB点组成的点云
typedef pcl::PointCloud<PointTT> PointCloudTT;

class registration2
{
public:
    registration2(std::string full_map_path, std::string cloud_sub_name,std::string tf_sub_name):
    _nh("/"),
    _full_map_path(full_map_path),
    _cloud_sub_name(cloud_sub_name),
    _tf_sub_name(tf_sub_name),
    _cloud_full(new PointCloudTT),
    _cloud_current(new PointCloudTT),
    _transformation(false),
    _cloud_full_downsampled(new PointCloudTT)
    {
        // 从文件加载参考点云
        pcl::io::loadPCDFile(_full_map_path, *_cloud_full);
        //订阅点云
        _cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &registration2::cloudCallback, this);
        _tf_sub = _nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &registration2::tfCallback, this);
        // 创建TF广播器对象
        _tf_broadcaster = std::make_unique<tf::TransformBroadcaster>();

        // 创建体素网格滤波器对象
        pcl::VoxelGrid<PointTT> voxel_grid;
        // 设置体素大小为0.1cm立方体
        voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);
        // 设置输入参考点云
        voxel_grid.setInputCloud(_cloud_full);
        // 执行下采样，结果存入cloud_full_downsampled
        voxel_grid.filter(*_cloud_full_downsampled);
        std::cout<<"init successed"<<std::endl;
    }
    registration2():_nh("/"), _transformation(false),
    _cloud_full(new PointCloudTT),
    _cloud_current(new PointCloudTT),
    _cloud_full_downsampled(new PointCloudTT)
    {
        get_param();
        if(pcl::io::loadPCDFile(_full_map_path, *_cloud_full) == -1)
        {
           std::cout<<"read fail"<<std::endl;
        }
        std::cout<<_cloud_full->is_dense<<std::endl;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_cloud_full, *_cloud_full, indices);  // 原地覆盖
        //订阅点云
        _cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &registration2::cloudCallback, this);
        _tf_sub = _nh.subscribe<tf2_msgs::TFMessage>(_tf_sub_name, 10, &registration2::tfCallback, this);
        // 创建TF广播器对象
        _tf_broadcaster = std::make_unique<tf::TransformBroadcaster>();
        // 创建体素网格滤波器对象
        pcl::VoxelGrid<PointTT> voxel_grid;
        // 设置体素大小为0.1cm立方体
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        // 设置输入参考点云
        voxel_grid.setInputCloud(_cloud_full);
        // 执行下采样，结果存入cloud_full_downsampled
        voxel_grid.filter(*_cloud_full_downsampled);
        float z_min = 0.0, z_max = 1.0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud(_cloud_full_downsampled);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min, z_max);
        pass_z.setNegative(true);
        pass_z.filter(*filtered_cloud);
        *_cloud_full_downsampled = *filtered_cloud;
        std::cout<<"init successed"<<std::endl;
    }

    void get_param();
    //订阅点云
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
    Eigen::Matrix4f icp_nlRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan, Eigen::Matrix4f transformation);
    Eigen::Matrix4f ndtRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan);
private:
    ros::NodeHandle _nh;
    ros::Subscriber _cloud_sub;
    std::string _cloud_sub_name;
    ros::Subscriber _tf_sub;
    std::string _tf_sub_name;
    std::string _full_map_path;
    PointCloudTT::Ptr _cloud_full;
    PointCloudTT::Ptr _cloud_current;
    PointCloudTT::Ptr _cloud_full_downsampled;
    Eigen::Matrix4f _transform;
    bool _transformation;
    // TF广播器智能指针
    std::unique_ptr<tf::TransformBroadcaster> _tf_broadcaster;
};
