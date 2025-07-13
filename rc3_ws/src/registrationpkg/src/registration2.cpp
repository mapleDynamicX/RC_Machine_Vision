#include"../include/registrationpkg/registration2.h"

void registration2::get_param()
{
    _nh.param("transform_topic",_transform_topic, std::string("camera/odometry"));
    _nh.param("full_map_path",_full_map_path, std::string("/home/maple/study/project_ws/src/registrationpkg/pcd/full_map.pcd"));
    _nh.param("rgb_cloud_name",_cloud_sub_name, std::string("rgb_cloud"));
    _nh.param("tf_sub_name",_tf_sub_name, std::string("tf"));
}

// 将TransformStamped转换为4x4齐次变换矩阵
Eigen::Matrix4d transformToMatrix(const geometry_msgs::TransformStamped& transform)
{
    // 提取平移分量
    Eigen::Vector3d translation;
    translation.x() = transform.transform.translation.x;
    translation.y() = transform.transform.translation.y;
    translation.z() = transform.transform.translation.z;
    
    // 提取旋转四元数
    Eigen::Quaterniond rotation;
    rotation.x() = transform.transform.rotation.x;
    rotation.y() = transform.transform.rotation.y;
    rotation.z() = transform.transform.rotation.z;
    rotation.w() = transform.transform.rotation.w;
    
    // 创建4x4齐次变换矩阵
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    // 设置左上角3x3旋转部分
    matrix.block<3,3>(0,0) = rotation.toRotationMatrix();
    // 设置右上角3x1平移部分
    matrix.block<3,1>(0,3) = translation;
    
    return matrix;
}

void registration2::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
//   ROS_INFO("Received %ld transforms:", msg->transforms.size());
//   // 遍历所有变换
//   for (const auto& transform : msg->transforms) {
//     ROS_INFO("Frame: %s -> %s", 
//              transform.header.frame_id.c_str(),
//              transform.child_frame_id.c_str());
    
//     // 提取位置和姿态
//     const auto& trans = transform.transform.translation;
//     const auto& rot = transform.transform.rotation;
//     ROS_INFO("  Translation: [%.2f, %.2f, %.2f]", trans.x, trans.y, trans.z);
//     ROS_INFO("  Rotation: [%.2f, %.2f, %.2f, %.2f]", rot.x, rot.y, rot.z, rot.w);
//   }
  ROS_INFO("Received %ld transforms:", msg->transforms.size());
  
  // 遍历所有变换 - 使用显式类型声明
  for (size_t i = 0; i < msg->transforms.size(); ++i) {
    const geometry_msgs::TransformStamped& transform = msg->transforms[i];
    std::cout<<"init _transform"<<std::endl;
    _transform = transformToMatrix(transform).cast<float>();
    // ROS_INFO("Frame: %s -> %s", 
    //          transform.header.frame_id.c_str(),
    //          transform.child_frame_id.c_str());
    
    // // 显式声明位置和姿态类型
    // const geometry_msgs::Vector3& trans = transform.transform.translation;
    // const geometry_msgs::Quaternion& rot = transform.transform.rotation;
    
    // ROS_INFO("  Translation: [%.2f, %.2f, %.2f]", trans.x, trans.y, trans.z);
    // ROS_INFO("  Rotation: [%.2f, %.2f, %.2f, %.2f]", rot.x, rot.y, rot.z, rot.w);
  }
}

Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& rot_mat) {
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
    // 设置目标参考点云（地图点云）
    ndt.setInputTarget(_cloud_full_downsampled);
    // 准备配准后的输出点云容器
    pcl::PointCloud<PointTT> final;
    // 执行配准计算：使用初始位姿估计开始迭代
    // final将存储配准后的点云
    ndt.align(final, _transform);
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
    }
    //  print4x4Matrix(transformation);
    return transformation;
  }

Eigen::Matrix4f registration2::icp_nlRegistration(const pcl::PointCloud<PointTT>::Ptr& pc_scan, Eigen::Matrix4f transformation)
{
    // 创建非线性ICP配准对象（使用Levenberg-Marquardt优化）
    pcl::IterativeClosestPointNonLinear<PointTT, PointTT> icp_nl;
    // 设置最大对应点距离阈值（超出此距离的点对不参与配准计算）
    icp_nl.setMaxCorrespondenceDistance(0.5);
    // 设置最大迭代次数（限制优化过程执行次数）
    icp_nl.setMaximumIterations(10);
    // 设置变换收敛阈值（当连续两次变换矩阵的变化小于此值时停止迭代）
    icp_nl.setTransformationEpsilon(1e-5);
    // 设置源点云（待配准的扫描点云）
    icp_nl.setInputSource(pc_scan);
    // 设置目标点云（参考地图点云）
    icp_nl.setInputTarget(_cloud_full_downsampled);
    // 准备配准后的输出点云容器
    pcl::PointCloud<PointTT> final;
    // 执行配准计算：使用初始变换估计开始优化过程
    /*
    pcl::IterativeClosestPointNonLinear::align 是点云库（PCL）中用于执行非线性迭代最近点（ICP）配准算法的函数。
    它的目标是将源点云（source cloud）配准到目标点云（target cloud），从而找到两个点云之间的最优变换矩阵（通常是刚性变换：旋转+平移）。
    */
   // 使用传入的initial矩阵作为初始变换估计
    icp_nl.align(final, transformation);
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
    }
    return transformation2;
}

void registration2::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // 转换为PCL点云格式
    std::cout<<"cloudCallback"<<std::endl;
    if(_cloud_current == nullptr)
    {
        std::cout<<"_cloud_current == nullptr"<<std::endl;
        return;
    }
    pcl::fromROSMsg(*msg, *_cloud_current);
    std::cout<<"_cloud_full_downsampled size: "<<_cloud_full_downsampled->size()<<std::endl;
    std::cout<<"_cloud_current size: "<<_cloud_current->size()<<std::endl;
    ros::Time tic = ros::Time::now();
    Eigen::Matrix4f transform2 = ndtRegistration(_cloud_current);
    if(_transformation == false)
    {
        std::cout<<"fail to icp"<<std::endl;
        return;
    }
    Eigen::Matrix4f transform = icp_nlRegistration(_cloud_current, transform2);
    if(_transformation == false)
    {
        std::cout<<"fail to icp"<<std::endl;
        return;
    }
    ROS_INFO("ndt Time: %f", (ros::Time::now() - tic).toSec());
    Eigen::Matrix4f inverse_transform = transform.inverse();
    // 将矩阵转换为仿射变换表示
    Eigen::Affine3f pose(inverse_transform);
    // 提取平移分量
    Eigen::Vector3f position = pose.translation();
    // 提取旋转矩阵
    Eigen::Matrix3f rotation = pose.rotation();

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "camera_base";
    // 填充变换数据
    transformStamped.transform.translation.x = position.x();
    transformStamped.transform.translation.y = position.y();
    transformStamped.transform.translation.z = position.z();
    // 转换为ROS消息类型
    //四元数
    Eigen::Quaterniond q = matrixToQuaternion(rotation.cast<double>());
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    // 广播TF变换
    _tf_broadcaster->sendTransform(transformStamped);
    //发布tf                                                              
    _transform_pub.publish(transformStamped);
}