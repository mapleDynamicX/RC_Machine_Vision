#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <libserial/SerialPort.h>
//#include "onnx.cpp"



// namespace my_onnx
// {
//     cv::dnn::Net net;
// }

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat image = cv_ptr->image;
        // 处理图像（例如显示）
        cv::namedWindow("img", cv::WINDOW_NORMAL);
        //Bbox bbox;
        //neural_network(image, bbox, my_onnx::net);
        cv::imshow("img", image);
        cv::waitKey(30);
        std::cout << "recieve" << std::endl;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("转换失败: %s", e.what());
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "sub_img_node");
    //my_onnx::net = cv::dnn::readNetFromONNX("src/rcpkg/onnx/best.onnx");
    // if (my_onnx::net.empty()) {
    //     std::cerr << "Error: Failed to load model. Possible reasons:" << std::endl;
    //     return -1;
    // }
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 10, imageCallback);
    // 或者普通订阅者：ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);
    ros::spin();
    return 0;
}