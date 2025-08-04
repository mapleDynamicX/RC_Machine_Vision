#include"../include/registrationpkg/registration.h"

void registration::get_param()
{
    _nh.param("transform_topic",_transform_topic, std::string("camera/odometry"));
    _nh.param("full_map_path",_full_map_path, std::string("/home/maple/study/project_ws/src/registrationpkg/pcd/full_map.pcd"));
    _nh.param("rgb_cloud_name",_cloud_sub_name, std::string("rgb_cloud"));
}


// 定义点云配准函数，输入参考点云和当前点云的智能指针，返回4x4变换矩阵
Eigen::Matrix4f registration::registerPointClouds2ICP()
{
    Eigen::Matrix4f rot_mat;
    rot_mat << 0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    if(_cloud_full == nullptr)
    {
        std::cout<<"no cloud full"<<std::endl;

    }
    else if(_cloud_current == nullptr)
    {
        std::cout<<"no cloud current"<<std::endl;
    }
    else
    {
        //检查有无无效点云
        std::cout<<"nan number:"<< removeInvalidPointsManual(_cloud_current) <<std::endl;
        std::cout<<"nan number:"<< removeInvalidPointsManual(_cloud_full) <<std::endl;
        // // 创建体素网格滤波器对象
        // pcl::VoxelGrid<PoinTT> voxel_grid;
        // // 设置体素大小为0.1cm立方体
        // voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
        // // 创建参考点云下采样结果的智能指针
        // PointCloudTT::Ptr cloud_full_downsampled(new PointCloudTT);
        // // 设置输入参考点云
        // voxel_grid.setInputCloud(_cloud_full);
        // // 执行下采样，结果存入cloud_full_downsampled
        // voxel_grid.filter(*cloud_full_downsampled);

        // pcl::VoxelGrid<PoinTT> voxel_grid2;
        // voxel_grid2.setLeafSize(0.05f, 0.05f, 0.05f);
        // // 创建当前点云下采样结果的智能指针
        // PointCloudTT::Ptr _cloud_current_downsampled(new PointCloudTT);
        // // 设置输入当前点云
        // voxel_grid2.setInputCloud(_cloud_current);
        // // 执行下采样，结果存入_cloud_current_downsampled
        // voxel_grid2.filter(*_cloud_current_downsampled);

        std::cout<<"cloud_full_downsampled size:"<<_cloud_full_downsampled->size()<<std::endl;
        std::cout<<"_cloud_current_downsampled size:"<<_cloud_current_downsampled->size()<<std::endl;

        if(_cloud_current_downsampled->size() < 50)
        {
            std::cout<<"not enough"<<std::endl;
            _transformation = false;
            return rot_mat;
        }

        //PointCloudTT::Ptr _cloud_current_downsampled = _cloud_current;
        //过滤无效点云
        //std::vector<int> indices;
        //pcl::removeNaNFromPointCloud(*cloud_full_downsampled, *cloud_full_downsampled, indices);  // 原地覆盖
        //std::vector<int> indices2;
        //pcl::removeNaNFromPointCloud(*_cloud_current_downsampled, *_cloud_current_downsampled, indices);  // 原地覆盖


        //问题排查，保存点云
        //pcl::io::savePCDFileASCII("_cloud_current.pcd", *_cloud_current);
        //pcl::io::savePCDFileASCII("_cloud_full.pcd", *_cloud_full);
        
        // // 创建法线估计对象
        // pcl::NormalEstimation<PoinTT, pcl::Normal> ne2;
        // // 创建法线点云的智能指针
        // pcl::PointCloud<pcl::Normal>::Ptr normals_full(new pcl::PointCloud<pcl::Normal>);
        // // 创建KD树搜索对象
        // pcl::search::KdTree<PoinTT>::Ptr tree2(new pcl::search::KdTree<PoinTT>());
        // // 设置法线估计的输入点云（下采样后的当前点云）
        // ne2.setInputCloud(_cloud_full_downsampled);
        // // 设置搜索方法为KD树
        // ne2.setSearchMethod(tree2);
        // // 设置法线估计的搜索半径为3cm
        // //ne2.setRadiusSearch(0.03);
        // ne2.setKSearch(30);
        // // 计算法线并存入normals_current
        // ne2.compute(*normals_full);
        // 创建带法线的点云容器（包含XYZ+RGB+法线信息）
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_full_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // 合并点云和法线数据
        pcl::concatenateFields(*_cloud_full_downsampled, *_normals_full, *cloud_full_with_normals);

        
        // 创建法线估计对象
        pcl::NormalEstimation<PoinTT, pcl::Normal> ne;
        // 创建法线点云的智能指针
        pcl::PointCloud<pcl::Normal>::Ptr normals_current(new pcl::PointCloud<pcl::Normal>);
        // 创建KD树搜索对象
        pcl::search::KdTree<PoinTT>::Ptr tree(new pcl::search::KdTree<PoinTT>());
        // 设置法线估计的输入点云（下采样后的当前点云）
        ne.setInputCloud(_cloud_current_downsampled);
        // 设置搜索方法为KD树
        ne.setSearchMethod(tree);
        // 设置法线估计的搜索半径为3cm
        //ne.setRadiusSearch(0.03);
        ne.setKSearch(30);
        // 计算法线并存入normals_current
        ne.compute(*normals_current);
        // 创建带法线的点云容器（包含XYZ+RGB+法线信息）
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_current_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // 合并点云和法线数据
        pcl::concatenateFields(*_cloud_current_downsampled, *normals_current, *cloud_current_with_normals);
        // 创建非线性ICP对象智能指针（支持法线信息）
        pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp(
            new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>());


        // 设置源点云（当前帧）
        icp->setInputSource(cloud_current_with_normals);
        // 设置目标点云（参考点云）
        icp->setInputTarget(cloud_full_with_normals);
        // 设置ICP最大迭代次数50次
        icp->setMaximumIterations(1000);
        // 设置变换收敛阈值（两次变换的差异）
        icp->setTransformationEpsilon(1e-8);
        // 设置欧几里得适应度收敛阈值
        icp->setEuclideanFitnessEpsilon(0.001);
        // 设置匹配最大距离阈值5cm
        icp->setMaxCorrespondenceDistance(0.05);
        // 创建配准结果点云容器
        pcl::PointCloud<pcl::PointXYZRGBNormal> final_cloud;
        // 执行配准算法
        icp->align(final_cloud);
        // 获取最终变换矩阵
        Eigen::Matrix4f transformation = icp->getFinalTransformation();
        // 输出变换矩阵
        //std::cout << "Transformation matrix:\n" << transformation << std::endl;
        // 输出配准误差分数
        //std::cout << "Fitness score: " << icp->getFitnessScore() << std::endl;
        // 返回4x4变换矩阵
        _transformation = true;
        return transformation;
    }
    _transformation = false;
    return rot_mat;
}

Eigen::Matrix4f registration::registerPointClouds2RANSAC()
{
    Eigen::Matrix4f rot_mat;
    rot_mat << 0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    if(_cloud_full == nullptr)
    {
        std::cout<<"no cloud full"<<std::endl;

    }
    else if(_cloud_current == nullptr)
    {
        std::cout<<"no cloud current"<<std::endl;
    }
    else
    {
        //检查有无无效点云
        //std::cout<<"nan number:"<< removeInvalidPointsManual(_cloud_current) <<std::endl;
        //std::cout<<"nan number:"<< removeInvalidPointsManual(_cloud_full) <<std::endl;
        // pcl::VoxelGrid<PoinTT> voxel_grid2;
        // voxel_grid2.setLeafSize(0.05f, 0.05f, 0.05f);
        // // 创建当前点云下采样结果的智能指针
        //PointCloudTT::Ptr _cloud_current_downsampled(new PointCloudTT);
        // // 设置输入当前点云
        // voxel_grid2.setInputCloud(_cloud_current);
        // // 执行下采样，结果存入_cloud_current_downsampled
        // voxel_grid2.filter(*_cloud_current_downsampled);
        *_cloud_current_downsampled = *_cloud_current;
        //*_cloud_current_downsampled = *_cloud_current;
        std::cout<<"cloud_full_downsampled size:"<<_cloud_full_downsampled->size()<<std::endl;
        std::cout<<"_cloud_current_downsampled size:"<<_cloud_current_downsampled->size()<<std::endl;
        if(_cloud_current_downsampled->size() < 50)
        {
            std::cout<<"not enough"<<std::endl;
            _transformation = false;
            return rot_mat;
        }

        // 创建源点云和目标点云的共享指针对象
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        source_cloud = _cloud_current_downsampled;
        target_cloud = _cloud_full_downsampled;
        // 实例化法线估计对象
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
        // 创建存储法线的点云指针
        pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
        // 创建KD树搜索对象
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        
        // 设置法线估计的搜索方法
        normal_estimator.setSearchMethod(tree);
        // 设置法线估计的搜索半径（单位：米）
        //normal_estimator.setRadiusSearch(0.1);
        normal_estimator.setKSearch(30);
        // 计算源点云法线
        normal_estimator.setInputCloud(source_cloud);
        normal_estimator.compute(*source_normals);
        
        //normal_estimator.setRadiusSearch(0.05);
        // 计算目标点云法线
        normal_estimator.setInputCloud(target_cloud);
        normal_estimator.compute(*target_normals);

        // 实例化FPFH特征估计对象
        pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimator;
        // 创建存储FPFH特征的指针
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
        
        // 设置FPFH的搜索方法和搜索半径
        fpfh_estimator.setSearchMethod(tree);
        //fpfh_estimator.setRadiusSearch(0.1);
        fpfh_estimator.setKSearch(30);
        // 计算源点云FPFH特征
        fpfh_estimator.setInputCloud(source_cloud);
        fpfh_estimator.setInputNormals(source_normals);
        fpfh_estimator.compute(*source_features);
        
        // 计算目标点云FPFH特征
        fpfh_estimator.setInputCloud(target_cloud);
        fpfh_estimator.setInputNormals(target_normals);
        fpfh_estimator.compute(*target_features);

        // 创建SAC-IA配准对象
        pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sacpr;
        // 设置源点云
        sacpr.setInputSource(source_cloud);
        // 设置源点云特征
        sacpr.setSourceFeatures(source_features);
        // 设置目标点云
        sacpr.setInputTarget(target_cloud);
        // 设置目标点云特征
        sacpr.setTargetFeatures(target_features);

        sacpr.setMaximumIterations(1000);          // RANSAC迭代次数
        sacpr.setNumberOfSamples(3);               // 每次采样点数
        sacpr.setCorrespondenceRandomness(5);      // 特征随机选取数量
        sacpr.setSimilarityThreshold(0.8f);        // 相似性阈值
        sacpr.setMaxCorrespondenceDistance(0.1f);  // 内点距离阈值
        sacpr.setInlierFraction(0.3f);             // 内点比例要求

        // 执行配准并存储结果
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        sacpr.align(*output_cloud);

        // 检查配准是否收敛
        if (!sacpr.hasConverged()) 
        {
            std::cout<<"fail to RANSAC"<<std::endl;
            _transformation = false;
            return rot_mat;
        }
        Eigen::Matrix4f transform = sacpr.getFinalTransformation();
        //应用粗配准结果
        // PointCloudTT::Ptr transformed(new PointCloudTT);
        // pcl::transformPointCloud(*__cloud_current_downsampled, *transformed, transform);
        // *__cloud_current_downsampled = *transformed;
        *_cloud_current_downsampled = *output_cloud;
        _transformation = true;
        return transform;
    }
    _transformation = false;
    return rot_mat;
}

Eigen::Matrix4f registration::registerPointClouds()
{
    Eigen::Matrix4f rot_mat;
    rot_mat << 0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;
    if(_cloud_full == nullptr)
    {
        std::cout<<"no cloud full"<<std::endl;

    }
    else if(_cloud_current == nullptr)
    {
        std::cout<<"no cloud current"<<std::endl;
    }
    else
    {
        //检查有无无效点云
        //std::cout<<"nan number:"<< removeInvalidPointsManual(_cloud_current) <<std::endl;
        //std::cout<<"nan number:"<< removeInvalidPointsManual(_cloud_full) <<std::endl;
        // // 创建体素网格滤波器对象
        // pcl::VoxelGrid<PoinTT> voxel_grid;
        // // 设置体素大小为0.1cm立方体
        // voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
        // // 创建参考点云下采样结果的智能指针
        // PointCloudTT::Ptr cloud_full_downsampled(new PointCloudTT);
        // // 设置输入参考点云
        // voxel_grid.setInputCloud(_cloud_full);
        // // 执行下采样，结果存入cloud_full_downsampled
        // voxel_grid.filter(*cloud_full_downsampled);

        // pcl::VoxelGrid<PoinTT> voxel_grid2;
        // voxel_grid2.setLeafSize(0.05f, 0.05f, 0.05f);
        // // 创建当前点云下采样结果的智能指针
        // PointCloudTT::Ptr _cloud_current_downsampled(new PointCloudTT);
        // // 设置输入当前点云
        // voxel_grid2.setInputCloud(_cloud_current);
        // // 执行下采样，结果存入_cloud_current_downsampled
        // voxel_grid2.filter(*_cloud_current_downsampled);

        std::cout<<"cloud_full_downsampled size:"<<_cloud_full_downsampled->size()<<std::endl;
        std::cout<<"_cloud_current_downsampled size:"<<_cloud_current_downsampled->size()<<std::endl;

        if(_cloud_current_downsampled->size() < 500)
        {
            std::cout<<"not enough"<<std::endl;
            _transformation = false;
            return rot_mat;
        }

        //PointCloudTT::Ptr _cloud_current_downsampled = _cloud_current;
        //过滤无效点云
        //std::vector<int> indices;
        //pcl::removeNaNFromPointCloud(*cloud_full_downsampled, *cloud_full_downsampled, indices);  // 原地覆盖
        //std::vector<int> indices2;
        //pcl::removeNaNFromPointCloud(*_cloud_current_downsampled, *_cloud_current_downsampled, indices);  // 原地覆盖


        //问题排查，保存点云
        //pcl::io::savePCDFileASCII("_cloud_current.pcd", *_cloud_current);
        //pcl::io::savePCDFileASCII("_cloud_full.pcd", *_cloud_full);
        
        // // 创建法线估计对象
        // pcl::NormalEstimation<PoinTT, pcl::Normal> ne2;
        // // 创建法线点云的智能指针
        // pcl::PointCloud<pcl::Normal>::Ptr normals_full(new pcl::PointCloud<pcl::Normal>);
        // // 创建KD树搜索对象
        // pcl::search::KdTree<PoinTT>::Ptr tree2(new pcl::search::KdTree<PoinTT>());
        // // 设置法线估计的输入点云（下采样后的当前点云）
        // ne2.setInputCloud(_cloud_full_downsampled);
        // // 设置搜索方法为KD树
        // ne2.setSearchMethod(tree2);
        // // 设置法线估计的搜索半径为3cm
        // //ne2.setRadiusSearch(0.03);
        // ne2.setKSearch(30);
        // // 计算法线并存入normals_current
        // ne2.compute(*normals_full);
        // 创建带法线的点云容器（包含XYZ+RGB+法线信息）
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_full_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // 合并点云和法线数据
        pcl::concatenateFields(*_cloud_full_downsampled, *_normals_full, *cloud_full_with_normals);

        
        // 创建法线估计对象
        pcl::NormalEstimation<PoinTT, pcl::Normal> ne;
        // 创建法线点云的智能指针
        pcl::PointCloud<pcl::Normal>::Ptr normals_current(new pcl::PointCloud<pcl::Normal>);
        // 创建KD树搜索对象
        pcl::search::KdTree<PoinTT>::Ptr tree(new pcl::search::KdTree<PoinTT>());
        // 设置法线估计的输入点云（下采样后的当前点云）
        ne.setInputCloud(_cloud_current_downsampled);
        // 设置搜索方法为KD树
        ne.setSearchMethod(tree);
        // 设置法线估计的搜索半径为3cm
        //ne.setRadiusSearch(0.03);
        ne.setKSearch(30);
        // 计算法线并存入normals_current
        ne.compute(*normals_current);
        // 创建带法线的点云容器（包含XYZ+RGB+法线信息）
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_current_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // 合并点云和法线数据
        pcl::concatenateFields(*_cloud_current_downsampled, *normals_current, *cloud_current_with_normals);
        // 创建非线性ICP对象智能指针（支持法线信息）
        pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp(
            new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>());


        // 设置源点云（当前帧）
        icp->setInputSource(cloud_current_with_normals);
        // 设置目标点云（参考点云）
        icp->setInputTarget(cloud_full_with_normals);
        // 设置ICP最大迭代次数50次
        icp->setMaximumIterations(50);
        // 设置变换收敛阈值（两次变换的差异）
        icp->setTransformationEpsilon(1e-8);
        // 设置欧几里得适应度收敛阈值
        icp->setEuclideanFitnessEpsilon(0.001);
        // 设置匹配最大距离阈值5cm
        icp->setMaxCorrespondenceDistance(0.05);
        // 创建配准结果点云容器
        pcl::PointCloud<pcl::PointXYZRGBNormal> final_cloud;
        // 执行配准算法
        icp->align(final_cloud);
        // 获取最终变换矩阵
        Eigen::Matrix4f transformation = icp->getFinalTransformation();
        // 输出变换矩阵
        //std::cout << "Transformation matrix:\n" << transformation << std::endl;
        // 输出配准误差分数
        //std::cout << "Fitness score: " << icp->getFitnessScore() << std::endl;
        // 返回4x4变换矩阵
        _transformation = true;
        return transformation;
    }
    _transformation = false;
    return rot_mat;
}



// 假设输入是3x3的旋转矩阵（Row-major顺序）
Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& rot_mat) {
    // 直接通过矩阵构造四元数
    Eigen::Quaterniond q(rot_mat);
    
    // 可选：确保w分量为非负（避免双重覆盖问题）
    if (q.w() < 0) {
        q.coeffs() = -q.coeffs();
    }
    return q;
}

void registration::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // 转换为PCL点云格式
    std::cout<<"cloudCallback"<<std::endl;
    if(_cloud_current == nullptr)
    {
        std::cout<<"_cloud_current == nullptr"<<std::endl;
        return;
    }
    pcl::fromROSMsg(*msg, *_cloud_current);
    std::cout<<"change success"<<_cloud_current->size()<<std::endl;
    // 执行配准获取变换矩阵
    //Eigen::Matrix4f transform = registerPointClouds();
    // Eigen::Matrix4f ransac_transform;
    // if(_max_iteration == 0)
    // {
    //     ransac_transform = registerPointClouds2RANSAC();
    //     _max_iteration = 2;
    // }
    // else
    // {
    //     pcl::VoxelGrid<PoinTT> voxel_grid2;
    //     voxel_grid2.setLeafSize(0.05f, 0.05f, 0.05f);
    //     // 创建当前点云下采样结果的智能指针
    //     PointCloudTT::Ptr _cloud_current_downsampled(new PointCloudTT);
    //     // 设置输入当前点云
    //     voxel_grid2.setInputCloud(_cloud_current);
    //     // 执行下采样，结果存入_cloud_current_downsampled
    //     voxel_grid2.filter(*_cloud_current_downsampled);
    //     __cloud_current_downsampled = _cloud_current_downsampled;
    //     PointCloudTT::Ptr transformed(new PointCloudTT);
    //     pcl::transformPointCloud(*__cloud_current_downsampled, *transformed, _transform);
    //     *__cloud_current_downsampled = *transformed;
    //     _max_iteration--;
    //     ransac_transform = _transform;
    // }
    // if(_transformation == false)
    // {
    //     return;
    // }
    // Eigen::Matrix4f ransac_transform = registerPointClouds2RANSAC();
    // Eigen::Matrix4f icp_transform = registerPointClouds2ICP();
    // if(_transformation == false)
    // {
    //     return;
    // }
    // Eigen::Matrix4f transform = icp_transform * ransac_transform;
    // _transform = transform;
    // 执行配准获取变换矩阵
    Eigen::Matrix4f transform = registerPointClouds();
    if(_transformation == false)
    {
        return;
    }
    Eigen::Matrix4f inverse_transform = transform.inverse();
    // 将矩阵转换为仿射变换表示
    Eigen::Affine3f pose(inverse_transform);
    // 提取平移分量
    Eigen::Vector3f position = pose.translation();
    // 提取旋转矩阵
    Eigen::Matrix3f rotation = pose.rotation();
    //std::cout << "Current position: " << position.transpose() << std::endl;
    //std::cout << "Rotation matrix:\n" << rotation << std::endl;
    // //世界坐标
    // float x = position.x();
    // float y = position.y();
    // float z = position.z();
    // // 旋转矩阵转欧拉角（ZYX顺序）
    // Eigen::Matrix3f rotation = pose.rotation();
    // float yaw   = atan2(rotation(1,0), rotation(0,0));  
    // float pitch = atan2(-rotation(2,0), sqrt(rotation(0,0)*rotation(0,0) + rotation(1,0)*rotation(1,0))); 
    // float roll  = atan2(rotation(2,1), rotation(2,2)); 

    //发布消息
    // 创建TransformStamped消息
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
    
    // 发布变换话题
    _transform_pub.publish(transformStamped);

}


















//过滤无效点云
size_t registration::removeInvalidPointsManual(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if(!cloud || cloud->empty()) 
    return 0;
    
    // 创建新的临时点云用于存储过滤后的有效点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 预分配内存空间以提高效率
    filtered->reserve(cloud->size());
    
    // 初始化无效点计数器
    size_t invalid_count = 0;
    // 遍历原始点云中的每个点
    for(const auto& p : cloud->points) {
        // 综合检查：点的坐标是否有限 + RGB值是否有效（非NaN且在0-255范围内）
        const bool is_valid = 
            pcl::isFinite(p) &&                      // 检查x,y,z坐标有限
            !std::isnan(p.r) &&                      // 红色分量非NaN
            !std::isnan(p.g) &&                      // 绿色分量非NaN
            !std::isnan(p.b) &&                      // 蓝色分量非NaN
            p.r >= 0 && p.g >= 0 && p.b >= 0 &&      // RGB值不小于0
            p.r <= 255 && p.g <= 255 && p.b <= 255;  // RGB值不大于255
        
        // 根据有效性决定点的去向
        if(is_valid) {
            filtered->push_back(p);  // 有效点存入新点云
        } else {
            invalid_count++;          // 无效点计数增加
        }
    }
    
    // 更新新点云的元数据属性
    filtered->width = filtered->size();  // 设置点云宽度为有效点数
    filtered->height = 1;                // 设置高度为1（无组织结构）
    filtered->is_dense = false;           // 标记点云无无效点
    
    // 用过滤后的点云交换原云内容（高效替换）
    cloud->swap(*filtered);
    
    // 返回检测到的无效点数量
    return invalid_count;
}






