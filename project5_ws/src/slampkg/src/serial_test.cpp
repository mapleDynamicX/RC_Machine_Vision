#include <ros/ros.h>
#include "../include/slampkg/ros_mcu0.h"
#include "../include/slampkg/serial_test.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <nav_msgs/Odometry.h>  // 核心头文件
#include <geometry_msgs/PoseWithCovariance.h> 
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
//M_PI;


namespace mcu0_serial
{
    serial_mcu::serial_mcu(const std::string& port)
    {
        serial_port_ = port;
        ros::NodeHandle private_nh("~");
        private_nh.param("baud", serial_baud_, 9600);
        // 持续尝试打开串口直到成功
        while (!serial_.isOpen())
        {
            ROS_WARN("try to open port %s", serial_port_.c_str());
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baud_);
            serial_.setFlowcontrol(serial::flowcontrol_none);
            serial_.setParity(serial::parity_none); // default is parity_none
            serial_.setStopbits(serial::stopbits_one);
            serial_.setBytesize(serial::eightbits);
            serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
            serial_.setTimeout(time_out);
            serial_.open();
        }
        ROS_INFO("port open success");
    }
    // 析构函数：关闭串口
    serial_mcu::~serial_mcu()
    {
        if (serial_.isOpen())
            serial_.close();
    }
    // 检查串口是否打开
    bool serial_mcu::isOpen() const{
        return serial_.isOpen();
    }
    // 串口数据发送函数
    size_t serial_mcu::serial_send(uint8_t frame_id, float msgs[], uint8_t length)
    {
        // 构建数据帧：帧头+ID+数据长度+数据+CRC+帧尾
        uint8_t buff_msg[length * 4 + 8] = {0};
        //uint8_t buff_msg[length + 8] = {0};
        frameout_.frame_head[0] = FRAME_HEAD_0;
        frameout_.frame_head[1] = FRAME_HEAD_1;
        frameout_.frame_id = frame_id;
        frameout_.data_length = length * 4;
        // 填充浮点数据（转换为字节）
        for (int i = 0; i < length; i++)
        {
            frameout_.data.msg_get[i] = msgs[i];
        }

        buff_msg[0] = FRAME_HEAD_0;
        buff_msg[1] = FRAME_HEAD_1;
        //buff_msg[2] = frame_id;
        //buff_msg[3] = length * 4;
        buff_msg[2] = length * 4;


        for (int q = 0; q < length * 4; q++)
        {
            buff_msg[3 + q] = frameout_.data.buff_msg[q];
        }

        // frameout_.check_code.crc_code = CRC16_Table(frameout_.data.buff_msg, length * 4);
        // buff_msg[4 + length * 4] = frameout_.check_code.crc_buff[0];
        // buff_msg[5 + length * 4] = frameout_.check_code.crc_buff[1];
        // buff_msg[6 + length * 4] = FRAME_END_0;
        // buff_msg[7 + length * 4] = FRAME_Erc3_wsND_1;
        buff_msg[3 + length * 4] = FRAME_END_0;
        buff_msg[4 + length * 4] = FRAME_END_1;

        //size_t write_num = serial_.write(buff_msg, length * 4 + 8);
        size_t write_num = serial_.write(buff_msg, length * 4 + 5);
        ROS_INFO("write %d", write_num);
        return write_num;
    }
    // 串口数据接收函数
    bool serial_mcu::serial_read(uint8_t* received_frame_id, float msgs[], uint8_t* received_length) {
        if (!serial_.isOpen()) {
            ROS_ERROR("Serial port is not open.");
            return false;
        }

        uint8_t byte;//声明临时变量存储当前读取的字节
        //循环处理串口缓冲区中的所有可用数据
        while (serial_.available() > 0) {
            /*
            从串口读取1字节数据
            读取失败则跳过后续处理继续循环
            */
            ros::Rate rate(300);

            if (serial_.read(&byte, 1) != 1) 
            {
                
                continue;
            }

            if (byte == FRAME_HEAD_0) {
                //std::cout<<0<<std::endl;
                // Check FRAME_HEAD_1
                if (serial_.read(&byte, 1) != 1 || byte != FRAME_HEAD_1) continue;
                //std::cout<<1<<std::endl;
                // Read frame ID
                // if (serial_.read(&framein_.frame_id, 1) != 1) continue;
                // *received_frame_id = framein_.frame_id;
                // std::cout<<2<<std::endl;
                // Read data length
                uint8_t data_length;
                if (serial_.read(&data_length, 1) != 1) 
                {
                    //std::cout<<(int)data_length<<std::endl;
                    continue;
                }
                
                size_t expected_data_length = data_length;
                //std::cout<<expected_data_length<<std::endl;
                // Validate data length检查数据长度是否超出缓冲区容量
                if (expected_data_length > sizeof(framein_.data.buff_msg)) {
                    ROS_WARN("Data too long: %d", expected_data_length);
                    continue;
                }

                // Read data payload读取数据域内容到缓冲区
                //serial_.read(framein_.data.buff_msg, expected_data_length);
                if (serial_.read(framein_.data.buff_msg, expected_data_length) != expected_data_length) continue;

                // Skip CRC and check frame end
                uint8_t end_bytes[2];
                //if (serial_.read(framein_.check_code.crc_buff, 2) != 2) continue;
                //if (serial_.read(framein_.check_code.crc_buff, 1) != 1) continue;
                if (serial_.read(end_bytes, 2) != 2 || end_bytes[0] != FRAME_END_0 || end_bytes[1] != FRAME_END_1) continue;

                // Extract float data
                *received_length = expected_data_length / 4;
                for (size_t i = 0; i < *received_length; ++i) {
                    msgs[i] = framein_.data.msg_get[i];
                }
                std::cout<<3<<std::endl;
                rate.sleep(); // 休眠至满足100Hz频率
                return true;
            }
        }
        
        return false;
    }
    uint16_t CRC16_Table(uint8_t *p, uint8_t counter)
    {
        static const uint16_t CRC16Table[256] =
        {
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
            0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
            0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
            0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
            0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
            0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
            0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
            0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
            0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
            0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
            0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
            0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
            0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
            0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
            0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
            0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
            0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
            0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
            0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
            0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
            0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
            0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
            0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
            0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
            0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
            0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
            0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
            0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
            0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
            0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
            0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
            0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
        };

        uint16_t crc16 = 0;
        for (int i = 0; i < counter; i++)
        {
            uint8_t value = p[i];
            crc16 = CRC16Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8);
        }
        return crc16;
    }
}




using namespace mcu0_serial;

serial_mcu* serialComm;

int image_width = 640;
int image_height = 480;

float initial_x = 0.0f;
float initial_y = 0.0f;
float initial_yaw = 0.0f;
bool has_initial = false;

// void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//     float x_centered = msg->pose.position.x;
//     float y_centered = msg->pose.position.y;

//     tf2::Quaternion q(
//         msg->pose.orientation.x,
//         msg->pose.orientation.y,
//         msg->pose.orientation.z,
//         msg->pose.orientation.w
//     );
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw_rad;
//     m.getRPY(roll, pitch, yaw_rad);
//     float yaw_deg = static_cast<float>(yaw_rad * 180.0 / M_PI);

//     if (has_initial) {
//         x_centered += initial_x;
//         y_centered += initial_y;
//         yaw_deg += initial_yaw;
//     }

//     float send_data[5] = {x_centered, y_centered, yaw_deg , 1 , 1};
//     serialComm->serial_send(2, send_data, 5);
// }

// void bboxCallback(const vision_msgs::BoundingBox2DArray::ConstPtr& msg)
// {
//     float image_center_x = image_width / 2.0f;
//     float image_center_y = image_height / 2.0f;

//     for (const auto& bbox : msg->boxes) {
//         float x_centered = bbox.center.x - image_center_x;
//         float y_centered = image_center_y - bbox.center.y;

//         float send[2] = {
//             x_centered,     
//             y_centered,     
//         };
//         serialComm->serial_send(1, send, 2);
//     }
// }

// ros::Publisher Odom_pub;
// ros::Subscriber tf_sub;
// ros::Publisher _transform_pub;
// //ros::Publisher tf_pub;
// /*
// float initial_x = 0.0f;
// float initial_y = 0.0f;
// float initial_yaw = 0.0f;
// bool has_initial = false;
// */
// // 收到消息时的回调函数
// void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
//     // 1. 打印基本信息
//     // ROS_INFO("Received transform from '%s' to '%s'", 
//     //          msg->header.frame_id.c_str(), 
//     //          msg->child_frame_id.c_str());
    
//     // 2. 获取时间戳
//     // ros::Time stamp = msg->header.stamp;
//     // ROS_INFO("Timestamp: %f sec", stamp.toSec());
    
//     // 3. 提取平移量 (x, y, z)
//     double x = msg->transform.translation.x;
//     double y = msg->transform.translation.y;
//     double z = msg->transform.translation.z;
//     //ROS_INFO("Translation: [%.2f, %.2f, %.2f]", x, y, z);



//     // 4. 提取旋转四元数 (x, y, z, w)
//     double qx = msg->transform.rotation.x;
//     double qy = msg->transform.rotation.y;
//     double qz = msg->transform.rotation.z;
//     double qw = msg->transform.rotation.w;
//     // ROS_INFO("Rotation Quaternion: [%.2f, %.2f, %.2f, %.2f]", qx, qy, qz, qw);
//     tf2::Quaternion quat_tf(qx, qy, qz, qw);
//     tf2::Matrix3x3 m(quat_tf);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);
//     // 弧度转角度
//     roll *= 180.0 / M_PI;
//     pitch *= 180.0 / M_PI;
//     yaw *= 180.0 / M_PI;
//     double _yaw = 0;
//     if(yaw  + 180 >= 60)
//     {
//         _yaw = yaw - 60 + 360;
//     }
//     else
//     {
//         _yaw = yaw - 60;
//     }
//     double radians = _yaw * (M_PI / 180.0); 
//     double _x = qx + 0.222*cos(radians);
//     double _y = qy +  0.222*sin(radians);
//     float msgs[3] = {0};
//     msgs[0] = _x;
//     msgs[1] = _y;
//     msgs[2] = _yaw;
//     std::cout<<"------------------------------------------"<<std::endl;
//     std::cout<<"_x: "<< _x << "_y: " << _y << "_yaw: "<< _yaw << std::endl;
//     std::cout<<"------------------------------------------"<<std::endl;
//     serialComm->serial_send(0, msgs, 3);
// }

// void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        
//     //nav_msgs::Odometry odom_msg;
//     double _yaw = 0;
//     if(initial_yaw + 60 >= 180)
//     {
//         _yaw = initial_yaw + 60 - 360;
//     }
//     else
//     {
//         _yaw = initial_yaw + 60;
//     }
//     double radians = initial_yaw * (M_PI / 180.0); 
//     double _x = initial_x - 0.222*cos(radians);
//     double _y = initial_y -  0.222*sin(radians);
    
//     // // 1. 设置消息头
//     // odom_msg.header.stamp = msg->header.stamp;
//     // odom_msg.header.frame_id = "odom";         // 世界坐标系
//     // odom_msg.child_frame_id = msg->header.frame_id;

//     // // 2. 设置位姿信息 (position + orientation)
//     // odom_msg.pose.pose.position.x = _x;      // X坐标（米）
//     // odom_msg.pose.pose.position.y = _y;      // Y坐标（米）
//     // odom_msg.pose.pose.position.z = 1.4;       // Z坐标（米）
    
//     // 使用四元数表示方向
//     double roll = 0.0;   
//     double pitch = 0.0;       
//     double yaw = (_yaw / 180) * M_PI;    
//     tf2::Quaternion q;
//     q.setRPY(roll, pitch, yaw); 
//     // odom_msg.pose.pose.orientation.x = q.x();
//     // odom_msg.pose.pose.orientation.y = q.y();
//     // odom_msg.pose.pose.orientation.z = q.z();
//     // odom_msg.pose.pose.orientation.w = q.w();
    
    
//     // 3. 设置协方差矩阵 (6x6, 行主序)
//     // double position_variance = 0.02;           // 位置方差
//     // double orientation_variance = 0.01;        // 方向方差
//     // boost::array<double, 36> cov = {{
//     //     position_variance, 0, 0, 0, 0, 0,
//     //     0, position_variance, 0, 0, 0, 0,
//     //     0, 0, position_variance, 0, 0, 0,
//     //     0, 0, 0, orientation_variance, 0, 0,
//     //     0, 0, 0, 0, orientation_variance, 0,
//     //     0, 0, 0, 0, 0, orientation_variance
//     // }};
//     // odom_msg.pose.covariance = cov;

//     static tf2_ros::TransformBroadcaster tf_broadcaster;
    
//     geometry_msgs::TransformStamped transform;
//     transform.header.stamp = msg->header.stamp;  
//     transform.header.frame_id = "world";         // 父坐标系
//     transform.child_frame_id = msg->header.frame_id;     // 子坐标系
//     transform.transform.translation.x = _x;
//     transform.transform.translation.y = _y;
//     transform.transform.translation.z = 1.4;
//     transform.transform.rotation.x = q.x();
//     transform.transform.rotation.y = q.y();
//     transform.transform.rotation.z = q.z();
//     transform.transform.rotation.w = q.w();
    
    
//     // 5. 发布消息
//     //Odom_pub.publish(odom_msg);
//     //tf_pub.publish(transform);
//     tf_broadcaster.sendTransform(transform);
    

//     //test
//     // initial_x += 0.001;
//     // initial_y += 0.001;
//     // float msgs[3] = {0};
//     // msgs[0] = 11.1;
//     // msgs[1] = 11.1;
//     // msgs[2] = 11.30141;
//     // serialComm->serial_send(0, msgs, 3);

//     _transform_pub.publish(transform);
//     std::cout<<"ok"<<std::endl;
// }

// struct TransformData {
//     double x = 0.0;
//     double y = 0.0;
//     double z = 0.0;
//     double roll = 0.0;
//     double pitch = 0.0;
//     double yaw = 0.0;
//     ros::Time stamp;
// } current_transform;

// void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
// {

//     for (const auto& transform : msg->transforms) {
//         // 获取源坐标系和目标坐标系
//         std::string source_frame = transform.header.frame_id;
//         std::string target_frame = transform.child_frame_id;
        
//         // 获取平移数据
//         float x = transform.transform.translation.x;
//         float y = transform.transform.translation.y;
//         float z = transform.transform.translation.z;
        
//         // 获取旋转数据（四元数）
//         float qx = transform.transform.rotation.x;
//         float qy = transform.transform.rotation.y;
//         float qz = transform.transform.rotation.z;
//         float qw = transform.transform.rotation.w;
        
//         //转化欧拉角
//         tf2::Quaternion tf_quat(qx, qy, qz, qw);
//         tf2::Matrix3x3 matrix(tf_quat);
//         double roll = 0.0;
//         double pitch = 0.0;
//         double yaw = 0.0;
//         matrix.getRPY(roll, pitch, yaw);
        
//         // 打印示例
//         ROS_INFO_STREAM("Processed transform: " << source_frame << " -> " << target_frame);
//         ROS_INFO("Position: (%.3f, %.3f, %.3f)", x, y, z);
//     }

// }
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "serial_node");
//     ros::NodeHandle nh;
    
//     nh.param<int>("image_width", image_width, 640);
//     nh.param<int>("image_height", image_height, 480);

//     std::string port;
//     nh.param<std::string>("serial_port", port, "/dev/ttyUSB0");
//     try {
//         serialComm = new serial_mcu(port);
//         ROS_INFO("Serial port initialized successfully");
//     } catch (const std::exception& e) {
//         ROS_ERROR("Failed to initialize serial port: %s", e.what());
//         return -1;
//     }

//     //ros::Subscriber pose_sub = nh.subscribe("/pose_stamped", 20, poseCallback);
//     //ros::Subscriber bbox_sub = nh.subscribe("/yolo/detections", 20, bboxCallback);
//     //ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 100, tfCallback);    
//     ros::Subscriber sub = nh.subscribe("camera/camera_info", 1, cameraInfoCallback);
//     //Odom_pub = nh.advertise<nav_msgs::Odometry>("/camera/odom", 10);
//     ros::Publisher restart_pub = nh.advertise<std_msgs::Bool>("/restart_slam", 1);
//     ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("/send_status", 10);
//     _transform_pub = nh.advertise<geometry_msgs::TransformStamped>("camera/tf", 10);
//     tf_sub = nh.subscribe("camera/relocation", 1, transformCallback);
//     //tf_pub = nh.advertise<tf2_msgs::TFMessage>("/camera/tf", 10);
//     ros::AsyncSpinner spinner(2);
//     spinner.start();
//     // ros::Rate loop_rate(10);
//     while (ros::ok()) {
//         //ros::spinOnce();

//         std_msgs::Bool status_msg;
//         status_msg.data = serialComm->isOpen();
//         status_pub.publish(status_msg);

//         uint8_t frame_id;
//         float received_data[32];
//         uint8_t data_length;
        
//         if (serialComm->serial_read(&frame_id, received_data, &data_length)) {
//             //if (frame_id == 1 && data_length >= 3) {
//                 initial_x = received_data[0];
//                 initial_y = received_data[1];
//                 initial_yaw = received_data[2];
//                 has_initial = true;                         
                
//                 std_msgs::Bool restart_msg;
//                 restart_msg.data = true;
//                 restart_pub.publish(restart_msg);
                
//                 ROS_INFO("Initial values set: X=%.4f, Y=%.4f, Yaw=%.4f", 
//                         initial_x, initial_y, initial_yaw);
//             //}
//         }

//     //loop_rate.sleep();
//        //ros::spinOnce();
//     }

//     //elete serialComm;
//     //ros::spin();
//     ros::waitForShutdown();
//     return 0;
// }



double func(double x)
{
    //return 0.00000000002581*x*x*x*x*x - 0.0000000336*x*x*x*x + 0.00000173996*x*x*x - 0.0044338*x*x + 0.5634*x - 27.3707 - 0.065;
    return (0.0634)*std::pow(M_E, x * 9.9520)+0.7001;
}

double func1(double x)
{
    //f(x) = -0.0000x³ + 0.0001x² + 0.0041x + 1.2000
    return -12443.9549*x*x*x + 8829.4844*x*x + 4103.5704*x + 12000.0000 - 760.00;
}
double func2(double x)
{
    //f(x) = -0.0000x³ + 0.0000x² + 0.0061x + 1.3000
    return -12443.9549*x*x*x + 1816.3203*x*x + 6103.4912*x + 13000.0000 - 760.00;
}
double func3(double x)
{
    //f(x) = 0.0000x³ + -0.0000x² + 0.0057x + 1.4000
    return 19921.5535*x*x*x + (-4339.3309)*x*x + 5687.4720*x + 14000.0000 - 760.00;
}
double func4(double x)
{
    //f(x) = -0.0000x³ + 0.0001x² + 0.0061x + 1.5000
    return -4442.8628*x*x*x + 6424.8821*x*x + 6063.1006*x + 15000.0000 - 760.00;
}
double func5(double x)
{
    //f(x) = 0.0000x³ + 0.0000x² + 0.0076x + 1.6000
    return 130826.3682*x*x*x + 4493.4363*x*x + 7645.2741*x + 16000.0000 - 760.00;
}
double func6(double x)
{
    //f(x) = -0.0000x³ + 0.0005x² + 0.0129x + 1.7000
    return -418512.2396*x*x*x + 45589.9234*x*x + 12889.5027*x + 17000.0000 - 760.00;
}
double func7(double x)
{
    //f(x) = 0.0000x³ + -0.0004x² + 0.0130x + 1.8000
    return 114330.0759*x*x*x - 44017.7322*x*x + 13001.7100*x + 18000.0000 - 760.00;
}
double func8(double x)
{
    //f(x) = 0.0000x³ + -0.0001x² + 0.0076x + 1.9000
    return 154948.8835*x*x*x + (-8559.4025)*x*x + 7566.2858*x + 19000.0000 - 760.00;
}
double func9(double x)
{
    //f(x) = -0.0000x³ + 0.0005x² + 0.0118x + 2.0000
    return -443449.9994*x*x*x + 45190.8157*x*x + 11801.9761*x + 20000.0000 - 760.00;
}
double func10(double x)
{
    //f(x) = 0.0000x³ + -0.0006x² + 0.0106x + 2.1000
    return 395602.4428*x*x*x + (-60438.9742)*x*x + 10591.2723*x + 21000.0000 - 760.00;
}
double func11(double x)
{
    //f(x) = -0.0000x³ + 0.0007x² + 0.0121x + 2.2000
    return -390440.0922*x*x*x + 74073.7684*x*x + 12136.6399*x + 22000.0000 - 760.00;
}
double func12(double x)
{
    //f(x) = 0.0000x³ + -0.0000x² + 0.0168x + 2.3000
    return 72830.2770*x*x*x + (-2436.8720)*x*x + 16815.9620*x + 23000.0000 - 760.00;
}
double func13(double x)
{
    //f(x) = -0.0000x³ + 0.0001x² + 0.0173x + 2.4000
    return -3663.2413*x*x*x + 10471.5663*x*x + 17290.6517*x + 24000.0000 - 760.00;
}
double func14(double x)
{
    //f(x) = 0.0001x³ + 0.0001x² + 0.0237x + 3.2000
    return 636720.5745*x*x*x + 6240.5225*x*x + 23724.8059*x + 32000.0000 - 760.00;
}
double func15(double x)
{
    //0.0001x³ + 0.0008x² + 0.0273x + 3.3000
    return 636720.5745*x*x*x + 82666.0931*x*x + 27281.9596*x + 33000.0000 - 760.00;
}


void boundingBoxCallback(const vision_msgs::BoundingBox2DArray::ConstPtr& msg)
{
    // 打印消息头信息
    ROS_INFO("Received bounding boxes in frame: %s", msg->header.frame_id.c_str());
    double fx = 387.2624;
    // 遍历所有检测到的边界框
    for (const auto& box : msg->boxes) {
        // 提取中心点坐标
        double center_x = box.center.x;
        double center_y = box.center.y;
        
        // 提取边界框尺寸
        double size_x = box.size_x;
        double size_y = box.size_y;
        

        // 打印边界框信息
        ROS_INFO("Box: Center(%.2f, %.2f), Size(%.2fx%.2f)", 
                center_x, center_y, size_x, size_y);
        
        //double depth = func(box.center.y - box.size_y / 2);
        
        double _x = box.center.y - box.size_y / 2;
        std::cout << "_x: " << _x << std::endl;
        //double depth = func(_x / 100.0) / 10000.0;
        double depth = func(_x / 1000.0);
        double offset = (center_x - 320) * depth / fx;
        double yaw = atan(offset / depth) * 180 / M_PI;
        std::cout << "depth :  " << depth << std::endl;
        std::cout << "yaw :  " <<  yaw << std::endl;
        if(depth <= 0)
        {
            return;
        }
        
        
        float msgs[3] = {0};
        msgs[0] = 0;
        msgs[1] = depth;
        msgs[2] = yaw;
        serialComm->serial_send(0, msgs, 3);
    }
               

}

void boundingBoxCallback2(const vision_msgs::BoundingBox2DArray::ConstPtr& msg)
{
    // 打印消息头信息
    ROS_INFO("Received bounding boxes in frame: %s", msg->header.frame_id.c_str());
    double fx = 387.2624;
    // 遍历所有检测到的边界框
    for (const auto& box : msg->boxes) {
        // 提取中心点坐标
        double center_x = box.center.x;
        double center_y = box.center.y;
        
        // 提取边界框尺寸
        double size_x = box.size_x;
        double size_y = box.size_y;
        

        // 打印边界框信息
        ROS_INFO("Box: Center(%.2f, %.2f), Size(%.2fx%.2f)", 
                center_x, center_y, size_x, size_y);
        
        //double depth = func(box.center.y - box.size_y / 2);
        
        double _x = box.center.y - box.size_y / 2;
        std::cout << "_x: " << _x << std::endl;
        //double depth = func(_x / 100.0) / 10000.0;
        double depth = -1;
        
        // if(_x >= 189.87 && _x <= 208.66)
        // {
        //     depth = func1(_x);
        // }
        // else if(_x >= 208.66 && _x <= 225.15)
        // {
        //     depth = func2(_x);
        // }
        // else if(_x >= 225.15 && _x <= 243.16)
        // {
        //     depth = func3(_x);
        // }
        // else if(_x >= 243.16 && _x <= 257.65)
        // {
        //     depth = func4(_x);
        // }
        // else if(_x >= 257.65 && _x <= 268.12)
        // {
        //     depth = func5(_x);
        // }
        // else if(_x >= 268.12 && _x <= 275.26)
        // {
        //     depth = func6(_x);
        // }
        // else if(_x >= 275.26 && _x <= 285.60)
        // {
        //     depth = func7(_x);
        // }
        // else if(_x >= 285.60 && _x <= 297.16)
        // {
        //     depth = func8(_x);
        // }
        // else if(_x >= 297.16 && _x <= 305.10)
        // {
        //     depth = func9(_x);
        // }
        // else if(_x >= 305.10 && _x <= 316.43)
        // {
        //     depth = func10(_x);
        // }
        // else if(_x >= 316.43 && _x <= 322.96)
        // {
        //     depth = func11(_x);
        // }
        // else if(_x >= 322.96 && _x <= 328.87)
        // {
        //     depth = func12(_x);
        // }
        // else if(_x >= 328.87 && _x <= 367.37)
        // {
        //     depth = func13(_x);
        // }
        // else if(_x >= 367.37 && _x <= 371.37)
        // {
        //     depth = func14(_x);
        // }
        // else if(_x >= 371.37 && _x <= 374.63)
        // {
        //     depth = func15(_x);
        // }
        // else
        // {
        //     return;
        // }
        if(_x >= 189.87 && _x <= 208.66)
        {
            depth = func1(_x / 100.0) / 10000.0;
        }
        else if(_x >= 208.66 && _x <= 225.15)
        {
            depth = func2(_x / 100.0) / 10000.0;
        }
        else if(_x >= 225.15 && _x <= 243.16)
        {
            depth = func3(_x / 100.0) / 10000.0;
        }
        else if(_x >= 243.16 && _x <= 257.65)
        {
            depth = func4(_x / 100.0) / 10000.0;
        }
        else if(_x >= 257.65 && _x <= 268.12)
        {
            depth = func5(_x / 100.0) / 10000.0;
        }
        else if(_x >= 268.12 && _x <= 275.26)
        {
            depth = func6(_x / 100.0) / 10000.0;
        }
        else if(_x >= 275.26 && _x <= 285.60)
        {
            depth = func7(_x / 100.0) / 10000.0;
        }
        else if(_x >= 285.60 && _x <= 297.16)
        {
            depth = func8(_x / 100.0) / 10000.0;
        }
        else if(_x >= 297.16 && _x <= 305.10)
        {
            depth = func9(_x / 100.0) / 10000.0;
        }
        else if(_x >= 305.10 && _x <= 316.43)
        {
            depth = func10(_x / 100.0) / 10000.0;
        }
        else if(_x >= 316.43 && _x <= 322.96)
        {
            depth = func11(_x / 100.0) / 10000.0;
        }
        else if(_x >= 322.96 && _x <= 328.87)
        {
            depth = func12(_x / 100.0) / 10000.0;
        }
        else if(_x >= 328.87 && _x <= 367.37)
        {
            depth = func13(_x / 100.0) / 10000.0;
        }
        else if(_x >= 367.37 && _x <= 371.37)
        {
            depth = func14(_x / 100.0) / 10000.0;
        }
        else if(_x >= 371.37 && _x <= 374.63)
        {
            depth = func15(_x / 100.0) / 10000.0;
        }
        else
        {
            return;
        }


        double offset = (center_x - 320) * depth / fx;
        double yaw = atan(offset / depth) * 180 / M_PI;
        std::cout << "depth :  " << depth << std::endl;
        std::cout << "yaw :  " <<  yaw << std::endl;
        if(depth <= 0)
        {
            return;
        }
        
        
        float msgs[3] = {0};
        msgs[0] = 0;
        msgs[1] = depth;
        msgs[2] = yaw;
        serialComm->serial_send(0, msgs, 3);
    }
               

}






void boundingBoxCallback3(const vision_msgs::BoundingBox2DArray::ConstPtr& msg)
{
    // 打印消息头信息
    ROS_INFO("Received bounding boxes in frame: %s", msg->header.frame_id.c_str());
    double fx = 387.2624;
    // 遍历所有检测到的边界框
    for (const auto& box : msg->boxes) {
        // 提取中心点坐标
        double center_x = box.center.x;
        double center_y = box.center.y;
        
        // 提取边界框尺寸
        double size_x = box.size_x;
        double size_y = box.size_y;
        

        // 打印边界框信息
        ROS_INFO("Box: Center(%.2f, %.2f), Size(%.2fx%.2f)", 
                center_x, center_y, size_x, size_y);
        
        //double depth = func(box.center.y - box.size_y / 2);
        
        double _x = box.center.y - box.size_y / 2;
        std::cout << "_x: " << _x << std::endl;
        //double depth = func(_x / 100.0) / 10000.0;
        double depth = -1;
        
        // if(_x >= 189.87 && _x <= 208.66)
        // {
        //     depth = func1(_x);
        // }
        // else if(_x >= 208.66 && _x <= 225.15)
        // {
        //     depth = func2(_x);
        // }
        // else if(_x >= 225.15 && _x <= 243.16)
        // {
        //     depth = func3(_x);
        // }
        // else if(_x >= 243.16 && _x <= 257.65)
        // {
        //     depth = func4(_x);
        // }
        // else if(_x >= 257.65 && _x <= 268.12)
        // {
        //     depth = func5(_x);
        // }
        // else if(_x >= 268.12 && _x <= 275.26)
        // {
        //     depth = func6(_x);
        // }
        // else if(_x >= 275.26 && _x <= 285.60)
        // {
        //     depth = func7(_x);
        // }
        // else if(_x >= 285.60 && _x <= 297.16)
        // {
        //     depth = func8(_x);
        // }
        // else if(_x >= 297.16 && _x <= 305.10)
        // {
        //     depth = func9(_x);
        // }
        // else if(_x >= 305.10 && _x <= 316.43)
        // {
        //     depth = func10(_x);
        // }
        // else if(_x >= 316.43 && _x <= 322.96)
        // {
        //     depth = func11(_x);
        // }
        // else if(_x >= 322.96 && _x <= 328.87)
        // {
        //     depth = func12(_x);
        // }
        // else if(_x >= 328.87 && _x <= 367.37)
        // {
        //     depth = func13(_x);
        // }
        // else if(_x >= 367.37 && _x <= 371.37)
        // {
        //     depth = func14(_x);
        // }
        // else if(_x >= 371.37 && _x <= 374.63)
        // {
        //     depth = func15(_x);
        // }
        // else
        // {
        //     return;
        // }
        if(_x >= 189.87 && _x <= 208.66)
        {
            depth = func1(_x / 100.0) / 10000.0;
        }
        else if(_x >= 208.66 && _x <= 225.15)
        {
            depth = func2(_x / 100.0) / 10000.0;
        }
        else if(_x >= 225.15 && _x <= 243.16)
        {
            depth = func3(_x / 100.0) / 10000.0;
        }
        else if(_x >= 243.16 && _x <= 257.65)
        {
            depth = func4(_x / 100.0) / 10000.0;
        }
        else if(_x >= 257.65 && _x <= 268.12)
        {
            depth = func5(_x / 100.0) / 10000.0;
        }
        else if(_x >= 268.12 && _x <= 275.26)
        {
            depth = func6(_x / 100.0) / 10000.0;
        }
        else if(_x >= 275.26 && _x <= 285.60)
        {
            depth = func7(_x / 100.0) / 10000.0;
        }
        else if(_x >= 285.60 && _x <= 297.16)
        {
            depth = func8(_x / 100.0) / 10000.0;
        }
        else if(_x >= 297.16 && _x <= 305.10)
        {
            depth = func9(_x / 100.0) / 10000.0;
        }
        else if(_x >= 305.10 && _x <= 316.43)
        {
            depth = func10(_x / 100.0) / 10000.0;
        }
        else if(_x >= 316.43 && _x <= 322.96)
        {
            depth = func11(_x / 100.0) / 10000.0;
        }
        else if(_x >= 322.96 && _x <= 328.87)
        {
            depth = func12(_x / 100.0) / 10000.0;
        }
        else if(_x >= 328.87 && _x <= 367.37)
        {
            depth = func13(_x / 100.0) / 10000.0;
        }
        else if(_x >= 367.37 && _x <= 371.37)
        {
            depth = func14(_x / 100.0) / 10000.0;
        }
        else if(_x >= 371.37 && _x <= 374.63)
        {
            depth = func15(_x / 100.0) / 10000.0;
        }
        else
        {
            return;
        }


        double offset = (center_x - 320) * depth / fx;
        double yaw = atan(offset / depth) * 180 / M_PI;
        std::cout << "depth :  " << depth << std::endl;
        std::cout << "yaw :  " <<  yaw << std::endl;
        if(depth <= 0)
        {
            return;
        }
        
        
        float msgs[3] = {0};
        msgs[0] = 0;
        msgs[1] = depth;
        msgs[2] = yaw;
        serialComm->serial_send(0, msgs, 3);
    }
               

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    
    nh.param<int>("image_width", image_width, 640);
    nh.param<int>("image_height", image_height, 480);

    std::string port;
    nh.param<std::string>("serial_port", port, "/dev/ttyUSB0");
    try {
        serialComm = new serial_mcu(port);
        ROS_INFO("Serial port initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize serial port: %s", e.what());
        return -1;
    }

    //ros::Subscriber pose_sub = nh.subscribe("/pose_stamped", 20, poseCallback);
    //ros::Subscriber bbox_sub = nh.subscribe("/yolo/detections", 20, bboxCallback);
    //ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 100, tfCallback);    
    //ros::Subscriber sub = nh.subscribe("camera/camera_info", 1, cameraInfoCallback);
    //Odom_pub = nh.advertise<nav_msgs::Odometry>("/camera/odom", 10);
    //ros::Publisher restart_pub = nh.advertise<std_msgs::Bool>("/restart_slam", 1);
    //ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("/send_status", 10);
    //_transform_pub = nh.advertise<geometry_msgs::TransformStamped>("camera/tf", 10);
    //ros::Subscriber tf_sub = nh.subscribe("camera/relocation", 1, transformCallback);
    ros::Subscriber bbox_sub = nh.subscribe("/yolo/detections", 1, boundingBoxCallback);
    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 10, imageCallback);
    //tf_pub = nh.advertise<tf2_msgs::TFMessage>("/camera/tf", 10);
    //ros::AsyncSpinner spinner(2);
    //spinner.start();
    // ros::Rate loop_rate(10); 

    // while (ros::ok()) {
    //     //ros::spinOnce();

    //     std_msgs::Bool status_msg;
    //     status_msg.data = serialComm->isOpen();
    //     status_pub.publish(status_msg);

    //     uint8_t frame_id;
    //     float received_data[32];
    //     uint8_t data_length;
        
    //     if (serialComm->serial_read(&frame_id, received_data, &data_length)) {
    //         //if (frame_id == 1 && data_length >= 3) {
    //             initial_x = received_data[0];
    //             initial_y = received_data[1];
    //             initial_yaw = received_data[2];
    //             has_initial = true;                         
                
    //             std_msgs::Bool restart_msg;
    //             restart_msg.data = true;
    //             restart_pub.publish(restart_msg);
                
    //             ROS_INFO("Initial values set: X=%.4f, Y=%.4f, Yaw=%.4f", 
    //                     initial_x, initial_y, initial_yaw);
    //         //}
    //     }

    // //loop_rate.sleep();
    // //ros::spinOnce();
    // }
    // ros::Rate a(30);
    // while(true)
    // {
    //     float msgs[3] = {0};
    //     msgs[0] = 0;
    //     msgs[1] = 1;
    //     msgs[2] = 2;
    //     serialComm->serial_send(0, msgs, 3);
    //     a.sleep();
    //     std::cout<<"input"<<std::endl;
    // }

    //elete serialComm;
    ros::spin();
    //ros::waitForShutdown();
    return 0;
}