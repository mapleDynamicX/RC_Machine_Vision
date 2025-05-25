#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>




int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pub_img_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 10);
    // 或者使用普通发布者：ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);

    cv::VideoCapture cap(2,cv::CAP_V4L2); // 打开摄像头
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened()) {
        ROS_ERROR("无法打开摄像头！");
        return -1;
    }

    ros::Rate rate(30); // 发布频率10Hz
    while (nh.ok()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        // 转换为ROS消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}