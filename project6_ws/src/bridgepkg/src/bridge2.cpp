#include"../include/bridgepkg/bridge.h"

//相机话题名称
void camerainfo::get_param()
{
    _nh.param("camera_info_name", _camerainfo_name, std::string("camera/camera_info"));
}
//获得rgb和深度图话题
void bridge::get_param()
{
    _nh.param("camera_color_image_name",_rgb_sub_name, std::string("camera/color/image_raw"));
    _nh.param("camera_depth_image_name",_depth_sub_name, std::string("camera/depth/image"));
}

//获得相机参数
void camerainfo::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    _frame_id = msg->header.frame_id;
    _height = msg->height;
    _width = msg->width;
    _D = msg->D;
    _K = msg->K;
    _R = msg->R;
    _P = msg->P;
    _cam_model_initialized = true;
    std::cout<<"_cam_model_initialized = true;"<<std::endl;
    _sub.shutdown();//关闭订阅
}
//过滤无效点云
size_t removeInvalidPointsManual(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
    filtered->is_dense = true;           // 标记点云无无效点
    
    // 用过滤后的点云交换原云内容（高效替换）
    cloud->swap(*filtered);
    
    // 返回检测到的无效点数量
    return invalid_count;
}
// ROS回调函数，处理同步的RGB和深度图像消息
void bridge::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, 
                   const sensor_msgs::ImageConstPtr& depth_msg) {
    // 检查相机模型是否已初始化
    if (_camerainfo._cam_model_initialized == false) {
        // 每秒打印一次警告，防止日志刷屏
        ROS_WARN_THROTTLE(1.0, "Camera model not initialized!");
        return;  // 模型未初始化直接返回
    }

    try {
        // 将ROS RGB图像转换为OpenCV格式 (BGR三通道)
        cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb_image = cv_rgb->image;  // 提取OpenCV矩阵

        // 准备存储深度图的矩阵
        cv::Mat depth_image;
        // 处理16位无符号整数深度格式 (单位：毫米)
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg);
            // 转换为32位浮点数并缩放到米 (乘以0.001)
            cv_depth->image.convertTo(depth_image, CV_32F, 0.001);
        }
        // 处理32位浮点深度格式 (单位：米)
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            // 直接共享数据避免拷贝
            depth_image = cv_bridge::toCvShare(depth_msg)->image;
        }
        // 不支持的深度图像格式处理
        else {
            ROS_ERROR("Unsupported depth encoding: %s", depth_msg->encoding.c_str());
            return;  // 格式错误立即返回
        }

        // 创建XYZRGB点云指针
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->is_dense = false;         // 可能包含NaN点（无效点）
        //cloud->header.frame_id = _camerainfo._frame_id;  // 设置点云坐标系
        cloud->width = _camerainfo._width;   // 点云宽度 = 图像列数
        cloud->height = _camerainfo._height;  // 点云高度 = 图像行数
        cloud->points.resize(_camerainfo._width *_camerainfo._height);  // 预分配内存
        
        // 遍历深度图每个像素
        for (int v = 0; v <  _camerainfo._height; v+=5) {
            for (int u = 0; u < _camerainfo._width; u+=5) {
                float d = depth_image.at<float>(v, u);  // 获取当前像素深度值
                pcl::PointXYZRGB& pt = cloud->points[v*_camerainfo._width + u];  // 获取对应点云点

                // 无效深度处理 (深度≤0或NaN)
                if (d <= 0.0 || std::isnan(d)) {
                    // 设为NaN点 (PCL会自动忽略)
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                    continue;  // 跳过后续处理
                }

                // 将像素坐标转换为3D射线
                cv::Point2d uv(u, v);
                double fx = _camerainfo._K[0];
                double fy = _camerainfo._K[4];
                double cx = _camerainfo._K[2];
                double cy = _camerainfo._K[5];
                // 填充点云坐标
                // pt.x = (u - cx) * d / fx;
                // pt.y = (v - cy) * d / fy;
                // pt.z = d;

                pt.x = (u - cx) * d / fx;
                pt.y = d;
                pt.z = -((v - cy) * d / fy);

                // 从RGB图像获取对应像素颜色 (OpenCV是BGR顺序)
                cv::Vec3b color = rgb_image.at<cv::Vec3b>(v, u);
                pt.r = color[2];  // OpenCV的B通道 -> PCL的R通道
                pt.g = color[1];  // G通道保持不变
                pt.b = color[0];  // OpenCV的R通道 -> PCL的B通道
            }
        }

        // 将PCL点云转换为ROS消息格式
        sensor_msgs::PointCloud2 cloud_msg;
        //std::cout<<_camerainfo._frame_id<<std::endl;
        //std::cout<<"nan number:" << removeInvalidPointsManual(cloud) << std::endl;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        float z_min = 1.0, z_max = 10.0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("y");
        pass_z.setFilterLimits(z_min, z_max);
        pass_z.setNegative(true);
        pass_z.filter(*filtered_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(50);             // 邻域点数 (建议值: 50)
        sor.setStddevMulThresh(1); // 阈值倍数 (建议值: 1.0-3.0)
        sor.filter(*filtered_cloud2);


        // pcl::VoxelGrid<PoinTT> voxel_grid2;
        // voxel_grid2.setLeafSize(0.05f, 0.05f, 0.05f);
        // // 创建当前点云下采样结果的智能指针
        // PointCloudTT::Ptr cloud_current_downsampled(new PointCloudTT);
        // // 设置输入当前点云
        // voxel_grid2.setInputCloud(filtered_cloud);
        // // 执行下采样，结果存入cloud_current_downsampled
        // voxel_grid2.filter(*cloud_current_downsampled);

        //std::cout<<"cloud->points.size:"<<cloud_current_downsampled->points.size()<<std::endl;
        pcl::toROSMsg(*filtered_cloud2, cloud_msg);
        std::cout<<"cloud->points.size:"<<filtered_cloud2->points.size()<<std::endl;
        cloud_msg.header.frame_id = _camerainfo._frame_id;
        cloud_msg.header.stamp = ros::Time::now();
        // 发布生成的点云
        _pub_cloud.publish(cloud_msg);
        //test
        // static ros::Publisher pub_cloud2 = _nh.advertise<sensor_msgs::PointCloud2>("rgb_full_cloud", 10);
        // PointCloudTT::Ptr _cloud_full(new PointCloudTT);
        // pcl::io::loadPCDFile("/home/maple/study/project_ws/src/registrationpkg/pcd/full_map.pcd", *_cloud_full);
        // sensor_msgs::PointCloud2 cloud_msg2;
        // pcl::toROSMsg(*_cloud_full, cloud_msg2);
        // cloud_msg2.header.frame_id = _camerainfo._frame_id;
        // cloud_msg2.header.stamp = ros::Time::now();
        // pub_cloud2.publish(cloud_msg2);
        //std::cout<<"publish"<<std::endl;

    } catch (cv_bridge::Exception& e) {
        // 捕获图像格式转换异常
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

