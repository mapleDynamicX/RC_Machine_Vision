#include"../include/registrationpkg/registration4.h"

// void registration2::get_param()
// {
//     //_nh.param("full_map_path",_full_map_path, std::string("/home/maple/study/project_ws/src/registrationpkg/pcd/full_map.pcd"));
//     //_nh.param("rgb_cloud_name",_cloud_sub_name, std::string("rgb_cloud"));
//     //_nh.param("tf_sub_name",_tf_sub_name, std::string("tf"));
// }

// // 将TransformStamped转换为4x4齐次变换矩阵
// Eigen::Matrix4d transformToMatrix(const geometry_msgs::TransformStamped& transform)
// {
//     // 提取平移分量
//     Eigen::Vector3d translation;
//     translation.x() = transform.transform.translation.x;
//     translation.y() = transform.transform.translation.y;
//     translation.z() = transform.transform.translation.z;
    
//     // 提取旋转四元数
//     Eigen::Quaterniond rotation;
//     rotation.x() = transform.transform.rotation.x;
//     rotation.y() = transform.transform.rotation.y;
//     rotation.z() = transform.transform.rotation.z;
//     rotation.w() = transform.transform.rotation.w;
    
//     // 创建4x4齐次变换矩阵
//     Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
//     // 设置左上角3x3旋转部分
//     matrix.block<3,3>(0,0) = rotation.toRotationMatrix();
//     // 设置右上角3x1平移部分
//     matrix.block<3,1>(0,3) = translation;
    
//     return matrix;
// }

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // 1. 解析时间戳和坐标系ID
    ROS_INFO("Received transform from [%s] to [%s]", 
             msg->header.frame_id.c_str(), 
             msg->child_frame_id.c_str());
    
    // 2. 提取平移和旋转数据
    Eigen::Vector3d translation(
        msg->transform.translation.x,
        msg->transform.translation.y,
        msg->transform.translation.z
    );
    
    Eigen::Quaterniond rotation(
        msg->transform.rotation.w,
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z
    );
    
    // 3. 创建4x4齐次变换矩阵
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = translation;
    transform.rotate(rotation);
    
    // 获取变换矩阵的4x4矩阵形式
    Eigen::Matrix4d transformation_matrix = transform.matrix();
    mutex_transformCallback.lock();
    _transform = transformation_matrix.cast<float>();
    mutex_transformCallback.unlock();
}


//void registration2::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // 转换为PCL点云格式 
    std::cout<<"cloudCallback"<<std::endl;
    if(_cloud_current == nullptr)
    {
        std::cout<<"_cloud_current == nullptr"<<std::endl;
        return;
    }
    PointCloudTT::Ptr cloud(new PointCloudTT);
    pcl::fromROSMsg(*msg, *cloud);
    mutex_transformCallback.lock();
    mutex_cloudCallback.lock();
    pcl::transformPointCloud(*cloud, *_cloud_current, _transform);
    mutex_cloudCallback.unlock();
    mutex_transformCallback.unlock();
}



// //void registration2::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) 
// void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) 
// {
// //   ROS_INFO("Received %ld transforms:", msg->transforms.size());
// //   // 遍历所有变换
// //   for (const auto& transform : msg->transforms) {
// //     ROS_INFO("Frame: %s -> %s", 
// //              transform.header.frame_id.c_str(),
// //              transform.child_frame_id.c_str());
    
// //     // 提取位置和姿态
// //     const auto& trans = transform.transform.translation;
// //     const auto& rot = transform.transform.rotation;
// //     ROS_INFO("  Translation: [%.2f, %.2f, %.2f]", trans.x, trans.y, trans.z);
// //     ROS_INFO("  Rotation: [%.2f, %.2f, %.2f, %.2f]", rot.x, rot.y, rot.z, rot.w);
// //   }
//     ROS_INFO("Received %ld transforms:", msg->transforms.size());
  
//     // 遍历所有变换 - 使用显式类型声明
//     std::string s2 = "camera_link";
//     for (size_t i = 0; i < msg->transforms.size(); ++i) {
//     const geometry_msgs::TransformStamped& transform = msg->transforms[i];
//     std::cout<<"init _transform"<<std::endl;
//     //if(transform.header.frame_id )
//     std::string s1 = transform.child_frame_id;
//     std::cout<<s1<<std::endl;
//     if(s1 == s2)
//     {
//         shared_data_mutex.lock();
//         _transform = transformToMatrix(transform).cast<float>();
//         shared_data_mutex.unlock();
//     }
//     else
//     {
//         std::cout<<"s1 != s2"<<std::endl;
//     }
//     // ROS_INFO("Frame: %s -> %s", 
//     //          transform.header.frame_id.c_str(),
//     //          transform.child_frame_id.c_str());
    
//     // // 显式声明位置和姿态类型
//     // const geometry_msgs::Vector3& trans = transform.transform.translation;
//     // const geometry_msgs::Quaternion& rot = transform.transform.rotation;
    
//     // ROS_INFO("  Translation: [%.2f, %.2f, %.2f]", trans.x, trans.y, trans.z);
//     // ROS_INFO("  Rotation: [%.2f, %.2f, %.2f, %.2f]", rot.x, rot.y, rot.z, rot.w);
//   }
// }

Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& rot_mat) 
{
    // 直接通过矩阵构造四元数
    Eigen::Quaterniond q(rot_mat);
    
    // 可选：确保w分量为非负（避免双重覆盖问题）
    if (q.w() < 0) {
        q.coeffs() = -q.coeffs();
    }
    return q;
}

Eigen::Matrix4f registration2::ndtRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan)
{
    if(_cloud_full_downsampled == nullptr)
    {
        *_cloud_full = *pc_scan;
        mutex_cloud_full_downsampled.lock();
        *_cloud_full_downsampled = *pc_scan;
            mutex_cloud_full_downsampled.unlock();
        std::cout<<"_cloud_full_downsampled == nullptr"<<std::endl;
        _transformation = false;
        return Eigen::Matrix4f::Identity();
    }
    else if(_cloud_full_downsampled->points.size() <= 50)
    {
        *_cloud_full = *pc_scan;
        mutex_cloud_full_downsampled.lock();
        *_cloud_full_downsampled = *pc_scan;
        mutex_cloud_full_downsampled.unlock();
        std::cout<<"_cloud_full_downsampled->points.size() <= 50"<<std::endl;
        _transformation = false;
        return Eigen::Matrix4f::Identity();
    }
    // 创建NDT配准对象
    pcl::NormalDistributionsTransform<PointTT, PointTT> ndt;
    // 设置最大迭代次数（限制算法执行时间）
    ndt.setMaximumIterations(10);
    // 设置变换收敛阈值（当变换增量小于此值时停止迭代）
    ndt.setTransformationEpsilon(1e-3);
    // 设置牛顿法优化步长（影响收敛速度和稳定性）
    ndt.setStepSize(0.01);
    // 设置NDT网格分辨率（决定空间划分粒度，影响精度和计算量）
    ndt.setResolution(0.5);
    // 设置待配准的源点云（扫描点云）
    ndt.setInputSource(pc_scan);

    mutex_cloud_full_downsampled.lock();
    // 设置目标参考点云（地图点云）
    ndt.setInputTarget(_cloud_full_downsampled);
    // 准备配准后的输出点云容器
    //pcl::PointCloud<PointTT> final;
    // 执行配准计算：使用初始位姿估计开始迭代
    // final将存储配准后的点云
    //ndt.align(final, _transform);
    mutex_icp_cloud_change.lock();
    ndt.align(*_icp_cloud_change);
    mutex_icp_cloud_change.unlock();
    mutex_cloud_full_downsampled.unlock();

    // 检查配准是否收敛（达到收敛条件）
    // 获取最终配准变换矩阵（4x4刚体变换矩阵）
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    if (!ndt.hasConverged())
    {
        ROS_ERROR("ndt not Converged");
        _transformation = false;
    }
    else
    {
        transformation = ndt.getFinalTransformation();
        // 计算配准质量得分（越小表示匹配越好）
        double score = ndt.getFitnessScore();
        std::cout<<"ndt score"<<score<<std::endl;
        _transformation = true;
        _flag_update = true;
    }
    //  print4x4Matrix(transformation);
    return transformation;
}

//Eigen::Matrix4f registration2::icp_nlRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan, Eigen::Matrix4f transformation)
Eigen::Matrix4f registration2::icp_nlRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan)
{
    if(_cloud_full_downsampled == nullptr)
    {
        *_cloud_full = *pc_scan;
        mutex_cloud_full_downsampled.lock();
        *_cloud_full_downsampled = *pc_scan;
        mutex_cloud_full_downsampled.unlock();
        std::cout<<"_cloud_full_downsampled == nullptr"<<std::endl;
        _transformation = false;
        return Eigen::Matrix4f::Identity();
    }
    else if(_cloud_full_downsampled->points.size() <= 50)
    {
        *_cloud_full = *pc_scan;
        mutex_cloud_full_downsampled.lock();
        *_cloud_full_downsampled = *pc_scan;
        mutex_cloud_full_downsampled.unlock();
        std::cout<<"_cloud_full_downsampled->points.size() <= 50"<<std::endl;
        _transformation = false;
        return Eigen::Matrix4f::Identity();
    }
    std::cout<<"_cloud_full_downsampled->point.size(): "<<_cloud_full_downsampled->points.size()<<std::endl;
    // 创建非线性ICP配准对象（使用Levenberg-Marquardt优化）
    pcl::IterativeClosestPointNonLinear<PointTT, PointTT> icp_nl;
    // 设置最大对应点距离阈值（超出此距离的点对不参与配准计算）
    icp_nl.setMaxCorrespondenceDistance(1);
    // 设置最大迭代次数（限制优化过程执行次数）
    icp_nl.setMaximumIterations(5);
    // 设置变换收敛阈值（当连续两次变换矩阵的变化小于此值时停止迭代）
    icp_nl.setTransformationEpsilon(1e-5);
    // 设置源点云（待配准的扫描点云）
    icp_nl.setInputSource(pc_scan);
    // 设置目标点云（参考地图点云）

    mutex_cloud_full_downsampled.lock();
    icp_nl.setInputTarget(_cloud_full_downsampled);
    // 准备配准后的输出点云容器
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 执行配准计算：使用初始变换估计开始优化过程
    /*
    pcl::IterativeClosestPointNonLinear::align 是点云库（PCL）中用于执行非线性迭代最近点（ICP）配准算法的函数。
    它的目标是将源点云（source cloud）配准到目标点云（target cloud），从而找到两个点云之间的最优变换矩阵（通常是刚性变换：旋转+平移）。
    */
   // 使用传入的transformation矩阵作为初始变换估计
    //icp_nl.align(*final, transformation);
    mutex_icp_cloud_change.lock();
    icp_nl.align(*_icp_cloud_change);
    mutex_icp_cloud_change.unlock();
    mutex_cloud_full_downsampled.unlock();
    Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
    // 检查配准是否收敛（达到收敛条件）
    if (!icp_nl.hasConverged())
    {
       ROS_ERROR("ndt_nl not Converged");
       _transformation = false;
    }
    else
    {
        // 获取最终配准变换矩阵（4x4刚体变换矩阵）
        transformation2 = icp_nl.getFinalTransformation();
        // 计算配准质量得分（点对距离平方和均值，越小匹配越好）
        double score = icp_nl.getFitnessScore();
        std::cout<<"icp score"<< score <<std::endl;
        //  print4x4Matrix(transformation);
        _transformation = true;
        _flag_update = true;
    }
    return transformation2;
}



void registration2::calculate()
{
    //std::cout<<"_cloud_full_downsampled size: "<<_cloud_full_downsampled->size()<<std::endl;
    //std::cout<<"_cloud_current size: "<<_cloud_current->size()<<std::endl;
    //ros::Time tic = ros::Time::now();
    // Eigen::Matrix4f transform2 = ndtRegistration(_cloud_current);
    // if(_transformation == false)
    // {
    //     std::cout<<"fail to icp"<<std::endl;
    //     return;
    // }
    //Eigen::Matrix4f transform = icp_nlRegistration(_cloud_current, transform2);
    //PointCloudTT::Ptr cloud_current_(new PointCloudTT);
    ros::Time tic = ros::Time::now();
    if(_cloud_current == nullptr)
    {
        std::cout<<"calculate() _cloud_current == nullptr"<<std::endl;
        return;
    }

    std::cout<<"mutex_cloudCallback.lock();"<<std::endl;
    //mutex_transformCallback.lock();
    mutex_cloudCallback.lock();
    //Eigen::Matrix4f _T = _transform;
    _cloud_change->points.resize(_cloud_current->points.size());
    *_cloud_change = *_cloud_current;
    //mutex_transformCallback.unlock();
    mutex_cloudCallback.unlock();
    std::cout<<"mutex_cloudCallback.unlock();"<<std::endl;
    if(_cloud_change == nullptr)
    {
        std::cout<<"cloud_current_ == nullptr"<<std::endl;
        return;
    }
    else if(_cloud_change->points.size() <= 50)
    {
        std::cout<<"cloud_current_->points.size():" <<_cloud_change->points.size()<<std::endl;
        return;
    }
    std::cout<<"cloud_current_->points.size():" <<_cloud_change->points.size()<<std::endl;
    ROS_INFO("copy Time: %f", (ros::Time::now() - tic).toSec());
    //Eigen::Matrix4f transform = icp_nlRegistration(_cloud_change, _T);
    Eigen::Matrix4f transform = icp_nlRegistration(_cloud_change);
    if(_transformation == false)
    {
        std::cout<<"fail to icp"<<std::endl;
        return;
    }
    //ROS_INFO("ndt Time: %f", (ros::Time::now() - tic).toSec());
    //Eigen::Matrix4f inverse_transform = transform.inverse();
    // 将矩阵转换为仿射变换表示
    //Eigen::Affine3f pose(inverse_transform);
    Eigen::Affine3f pose(transform);
    // 提取平移分量
    Eigen::Vector3f position = pose.translation();
    // 提取旋转矩阵
    Eigen::Matrix3f rotation = pose.rotation();

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "camera_base";
    // 填充变换数据
    transformStamped.transform.translation.x = position.x();
    transformStamped.transform.translation.y = position.y();
    transformStamped.transform.translation.z = position.z();
    //转换为ROS消息类型
    //四元数
    Eigen::Quaterniond q = matrixToQuaternion(rotation.cast<double>());
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    // 广播TF变换
    //_tf_broadcaster->sendTransform(transformStamped);
    std::cout<<"_tf_broadcaster->sendTransform(transformStamped)"<<std::endl;
    //发布tf                                                              
    _transform_pub.publish(transformStamped);
}


void registration2::map()
{
    if(_icp_cloud_change == nullptr)
    {
        std::cout<<"_icp_cloud_change == nullptr"<<std::endl;
        return;
    }
    else if(_icp_cloud_change->points.size() <= 50)
    {
        std::cout<<"_icp_cloud_change->points.size() <= 50"<<std::endl;
        return;
    }
    else if(_flag_update == false)
    {
        std::cout<<"_flag_update == false"<<std::endl;
        return;
    }
    mutex_icp_cloud_change.lock();
    *_cloud_full += *_icp_cloud_change;
    _flag_update = false;
    mutex_icp_cloud_change.unlock();
    if(_cloud_full->points.size() >= 15000)
    {
        _cloud_full->points.erase(_cloud_full->points.begin(), _cloud_full->points.begin() + 5000);
        _cloud_full->width = _cloud_full->size();  // 更新宽度
    }


    pcl::VoxelGrid<PointTT> voxel_grid;
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
    // 设置输入当前点云
    voxel_grid.setInputCloud(_cloud_full);
    // 执行下采样，结果存入cloud_current_downsampled
    mutex_cloud_full_downsampled.lock();
    voxel_grid.filter(*_cloud_full_downsampled);
    mutex_cloud_full_downsampled.unlock();
}









