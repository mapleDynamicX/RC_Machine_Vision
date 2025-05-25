import rospy
import sys
import os
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pyrealsense2 as rs


class RealsensePublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('realsense_publisher', anonymous=True)

        # 创建图像发布者
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        #self.depth_colormap_pub = rospy.Publisher('/camera/depth/image_colormap', Image, queue_size=10)
        self.bridge = CvBridge()

        # 配置相机管道
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # 对齐深度到彩色帧
        self.align = rs.align(rs.stream.color)

        # 启动相机
        self.pipeline.start(config)

    def publish_images(self):
        try:
            while not rospy.is_shutdown():
                # 等待并获取帧数据
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)

                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # 转换OpenCV格式
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # 创建深度伪彩色图
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET
                )

                # 发布ROS消息
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                #depth_colormap_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding="bgr8")

                # 添加时间戳和帧ID
                color_msg.header.stamp = rospy.Time.now()
                color_msg.header.frame_id = "camera_link"

                #depth_colormap_msg.header.stamp = color_msg.header.stamp
                #depth_colormap_msg.header.frame_id = "camera_link"

                # 发布消息
                self.color_pub.publish(color_msg)
                #self.depth_colormap_pub.publish(depth_colormap_msg)

                # 控制循环频率
                rospy.sleep(0.03)  # ~30Hz

        finally:
            self.pipeline.stop()


if __name__ == '__main__':
    try:
        publisher = RealsensePublisher()
        publisher.publish_images()
    except rospy.ROSInterruptException:
        pass