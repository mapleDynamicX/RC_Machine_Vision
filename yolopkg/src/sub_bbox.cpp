#include <ros/ros.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <iostream>
#include <libserial/SerialPort.h>
using namespace LibSerial;
SerialPort serial_port;

// 回调函数，处理接收到的BoundingBox2DArray消息
void boundingBoxCallback(const vision_msgs::BoundingBox2DArray::ConstPtr& msg)
{
    char c = '0';
    // 打印消息头信息
    ROS_INFO("Received bounding boxes in frame: %s", msg->header.frame_id.c_str());
    
    // 遍历所有检测到的边界框
    for (const auto& box : msg->boxes) {
        // 提取中心点坐标
        double center_x = box.center.x;
        double center_y = box.center.y;
        
        // 提取边界框尺寸
        double size_x = box.size_x;
        double size_y = box.size_y;
        
        // 提取旋转角度（弧度）
        double theta = box.center.theta;
        std::cout << center_x << std::endl;
        // 打印边界框信息
        ROS_INFO("Box: Center(%.2f, %.2f), Size(%.2fx%.2f), Rotation: %.2f rad", 
                center_x, center_y, size_x, size_y, theta);
               
        if(center_x < 320)
        {
            c = 'r';
        }
        else
        {
            c = 'l';
        }
        serial_port.Write(&c);
        return;
    }
    serial_port.Write(&c);
    std::cout << 0 << std::endl;
    // 打印边界框总数
    ROS_INFO("Total boxes detected: %lu", msg->boxes.size());
    return;
}

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "sub_bbox_node");
    ros::NodeHandle nh;
    serial_port.Open("/dev/ttyUSB0");
    // 创建订阅者，订阅"bounding_boxes"话题
    ros::Subscriber sub = nh.subscribe("/yolo/detections", 10, boundingBoxCallback);
    
    // 进入循环等待回调
    ros::spin();
    
    return 0;
}