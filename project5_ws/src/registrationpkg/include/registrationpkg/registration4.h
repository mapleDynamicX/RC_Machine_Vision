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
#include <thread>
#include <mutex>

// 定义点类型为带颜色(RGB)的XYZ点
typedef pcl::PointXYZRGB PointTT;
// 定义点云类型为XYZRGB点组成的点云
typedef pcl::PointCloud<PointTT> PointCloudTT;
Eigen::Matrix4f _transform;
//ros::Subscriber _cloud_sub;
PointCloudTT::Ptr _cloud_current(new PointCloudTT);
//ros::Subscriber _tf_sub;
//void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
//void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
//void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
PointCloudTT::Ptr _icp_cloud_change(new PointCloudTT);
PointCloudTT::Ptr _cloud_full_downsampled(new PointCloudTT);

std::mutex mutex_transformCallback;
std::mutex mutex_cloudCallback;
std::mutex mutex_icp_cloud_change;
std::mutex mutex_cloud_full_downsampled;

class registration2
{
public:
    // registration2(std::string full_map_path, std::string cloud_sub_name,std::string tf_sub_name):
    // _nh("/"),
    // _full_map_path(full_map_path),
    // //_cloud_sub_name(cloud_sub_name),
    // _tf_sub_name(tf_sub_name),
    // _cloud_full(new PointCloudTT),
    // //_cloud_current(new PointCloudTT),
    // _transformation(false),
    // _cloud_full_downsampled(new PointCloudTT)
    // {
    //     // 从文件加载参考点云
    //     pcl::io::loadPCDFile(_full_map_path, *_cloud_full);
    //     //订阅点云
    //     //_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &registration2::cloudCallback, this);
    //     //_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &cloudCallback);
    //     //_tf_sub = _nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &registration2::tfCallback, this);
    //     _transform_pub = _nh.advertise<geometry_msgs::TransformStamped>("camera/tf", 10);
    //     // 创建TF广播器对象
    //     _tf_broadcaster = std::make_unique<tf::TransformBroadcaster>();
    //     // 创建体素网格滤波器对象
    //     pcl::VoxelGrid<PointTT> voxel_grid;
    //     // 设置体素大小为0.1cm立方体
    //     voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);
    //     // 设置输入参考点云
    //     voxel_grid.setInputCloud(_cloud_full);
    //     // 执行下采样，结果存入cloud_full_downsampled
    //     voxel_grid.filter(*_cloud_full_downsampled);
    //     std::cout<<"init successed"<<std::endl;
    // }

    registration2():
    _nh("/"), 
    _transformation(false),
    _cloud_change(new PointCloudTT),
    //_icp_cloud_change(new PointCloudTT),
    _cloud_full(new PointCloudTT),
    //_cloud_current(new PointCloudTT),
    //_cloud_full_downsampled(new PointCloudTT),
    _flag_update(false)
    {
        //get_param();
        // if(pcl::io::loadPCDFile(_full_map_path, *_cloud_full) == -1)
        // {
        //    std::cout<<"read fail"<<std::endl;
        // }
        // std::cout<<_cloud_full->is_dense<<std::endl;
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*_cloud_full, *_cloud_full, indices);  // 原地覆盖
        //订阅点云
        //_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &registration2::cloudCallback, this);
        //_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_cloud_sub_name, 10, &cloudCallback);
        //_tf_sub = _nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &registration2::tfCallback, this);
        //_tf_sub = _nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &tfCallback);
        //发布tf消息
        _transform_pub = _nh.advertise<geometry_msgs::TransformStamped>("camera/relocation", 10);
        // 创建TF广播器对象
        _tf_broadcaster = std::make_unique<tf::TransformBroadcaster>();
        // // 创建体素网格滤波器对象
        // pcl::VoxelGrid<PointTT> voxel_grid;
        // // 设置体素大小为0.1cm立方体
        // voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        // // 设置输入参考点云
        // voxel_grid.setInputCloud(_cloud_full);
        // // 执行下采样，结果存入cloud_full_downsampled
        // voxel_grid.filter(*_cloud_full_downsampled);
        // float z_min = 0.0, z_max = 1.0;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        // pass_z.setInputCloud(_cloud_full_downsampled);
        // pass_z.setFilterFieldName("z");
        // pass_z.setFilterLimits(z_min, z_max);
        // pass_z.setNegative(true);
        // pass_z.filter(*filtered_cloud);
        // *_cloud_full_downsampled = *filtered_cloud;
        std::cout<<"init successed"<<std::endl;
    }

    //void get_param();
    //订阅点云
    //void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    //void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
    //Eigen::Matrix4f icp_nlRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan, Eigen::Matrix4f transformation);
    Eigen::Matrix4f icp_nlRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan);
    Eigen::Matrix4f ndtRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan);
    //计算
    void calculate();
    void map();
private:
    ros::NodeHandle _nh;
    //ros::Subscriber _cloud_sub;
    //std::string _cloud_sub_name;
    //ros::Subscriber _tf_sub;
    ros::Publisher _transform_pub;
    //std::string _tf_sub_name;
    //std::string _full_map_path;
    PointCloudTT::Ptr _cloud_change;
    //PointCloudTT::Ptr _icp_cloud_change;
    PointCloudTT::Ptr _cloud_full;
    //PointCloudTT::Ptr _cloud_current;
    //PointCloudTT::Ptr _cloud_full_downsampled;

    Eigen::Matrix4f _T;
    bool _transformation;
    bool _flag_update;
    // TF广播器智能指针
    std::unique_ptr<tf::TransformBroadcaster> _tf_broadcaster;
};
