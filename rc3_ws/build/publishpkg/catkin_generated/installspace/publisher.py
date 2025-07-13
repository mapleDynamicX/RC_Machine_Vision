import rospy
import sys
import os
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo

class RealsensePublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('realsense_publisher', anonymous=True)

        # 创建图像发布者
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/image', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
        self.bridge = CvBridge()

        # 配置相机管道
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # 初始化相机参数
        self.init_camera_parameters()
        # 对齐深度到彩色帧
        self.align = rs.align(rs.stream.color)

        # 启动相机
        self.pipeline.start(config)
    
    def init_camera_parameters(self):
        #"""在程序中直接设置相机参数"""
        # 基本参数
        self.image_width = 640
        self.image_height = 480
        self.distortion_model = "plumb_bob"  # ROS标准畸变模型
        
        # 内参矩阵K (3x3)
        self.fx = 387.2624  # x轴焦距
        self.fy = 387.2624  # y轴焦距
        self.cx = 318.6802  # 主点x坐标
        self.cy = 247.4131  # 主点y坐标
        
        # 畸变参数D (5个元素)
        self.distortion_k1 = 0.0
        self.distortion_k2 = 0.0
        self.distortion_k3 = 0.0
        self.distortion_p1 = 0.0
        self.distortion_p2 = 0.0
        
        # 投影矩阵P (3x4)
        # 通常是K矩阵在最后一列添加0
        self.p_matrix = [
            self.fx, 0.0,   self.cx, 0.0,
            0.0,   self.fy, self.cy, 0.0,
            0.0,   0.0,    1.0,     0.0
        ]
        
        # 矩形化矩阵R (3x3) - 对于非双目相机通常是单位矩阵
        self.rectification = [1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0]
    
    def create_camera_info_msg(self):
        #"""创建并填充CameraInfo消息"""
        msg = CameraInfo()
        
        # 设置消息头
        msg.header.stamp = rospy.Time.now()

        # 设置基本参数
        msg.width = self.image_width
        msg.height = self.image_height
        msg.distortion_model = self.distortion_model
        
        # 设置畸变参数D
        msg.D = [
            self.distortion_k1,
            self.distortion_k2,
            self.distortion_p1,
            self.distortion_p2,
            self.distortion_k3
        ]
        
        # 设置内参矩阵K
        msg.K = [
            self.fx, 0.0,   self.cx,
            0.0,   self.fy, self.cy,
            0.0,   0.0,    1.0
        ]
        
        # 设置投影矩阵P
        msg.P = self.p_matrix
        
        # 设置矩形化矩阵R
        msg.R = self.rectification
        
        # 设置binning参数 (无压缩)
        msg.binning_x = 1  # 无水平合并像素
        msg.binning_y = 1  # 无垂直合并像素
        
        # 设置ROI (整个图像)
        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = self.image_height
        msg.roi.width = self.image_width
        msg.roi.do_rectify = False
        
        return msg
    

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
                # depth_colormap = cv2.applyColorMap(
                #     cv2.convertScaleAbs(depth_image, alpha=0.03),
                #     cv2.COLORMAP_JET
                # )

                # 发布ROS消息
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")

                # 添加时间戳和帧ID
                color_msg.header.stamp = rospy.Time.now()
                color_msg.header.frame_id = "camera_link"
                

                depth_msg.header.stamp = color_msg.header.stamp
                depth_msg.header.frame_id = "camera_link"

                # 发布消息
                self.color_pub.publish(color_msg)
                self.depth_pub.publish(depth_msg)

                cam_info = self.create_camera_info_msg()
                cam_info.header.stamp = color_msg.header.stamp
                cam_info.header.frame_id = "camera_link"
                self.info_pub.publish(cam_info)

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








# import rospy
# import sys
# import os
# import time
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo
# from camera_info_manager import CameraInfoManager
# import pyrealsense2 as rs

# class RealsensePublisher:
#     def __init__(self):
#         # 初始化ROS节点
#         rospy.init_node('realsense_publisher', anonymous=True)
        
#         # 获取相机名称参数 (可选，但推荐)
#         camera_name = rospy.get_param("~camera_name", "camera")
#         rospy.loginfo(f"Starting RealSense publisher for camera: {camera_name}")

#         # 创建图像发布者
#         self.color_pub = rospy.Publisher(f'/{camera_name}/color/image_raw', Image, queue_size=10)
#         self.depth_pub = rospy.Publisher(f'/{camera_name}/depth/image_raw', Image, queue_size=10)
#         self.camera_info_pub = rospy.Publisher(f'/{camera_name}/color/camera_info', CameraInfo, queue_size=10)
#         self.bridge = CvBridge()

#         # 配置相机管道
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#         # 对齐深度到彩色帧
#         self.align = rs.align(rs.stream.color)

#         # 启动相机
#         pipeline_profile = self.pipeline.start(config)
        
#         # 获取内参信息
#         color_profile = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile()
#         self.intrinsics = color_profile.get_intrinsics()
#         rospy.loginfo(f"Camera intrinsics: {self.intrinsics}")
        
#         # 初始化相机信息管理器
#         self.cinfo_manager = CameraInfoManager(cname=camera_name)
#         self.cinfo_manager.setCameraName(camera_name)
#         self.cinfo_manager.loadCameraInfo()
        
#         # 设置相机信息
#         cam_info = CameraInfo()
#         cam_info.header.frame_id = f"{camera_name}_link"
#         cam_info.height = self.intrinsics.height
#         cam_info.width = self.intrinsics.width
#         cam_info.distortion_model = "plumb_bob"
#         cam_info.D = list(self.intrinsics.coeffs)
#         cam_info.K = [
#             self.intrinsics.fx, 0, self.intrinsics.ppx,
#             0, self.intrinsics.fy, self.intrinsics.ppy,
#             0, 0, 1
#         ]
#         cam_info.P = [
#             self.intrinsics.fx, 0, self.intrinsics.ppx, 0,
#             0, self.intrinsics.fy, self.intrinsics.ppy, 0,
#             0, 0, 1, 0
#         ]
#         self.cinfo_manager.setCameraInfo(cam_info)

#     def publish_images(self):
#         try:
#             rospy.loginfo("Starting RealSense image publishing...")
            
#             # 创建OpenCV窗口用于调试（可选）
#             cv2.namedWindow('Depth Preview', cv2.WINDOW_NORMAL)
            
#             while not rospy.is_shutdown():
#                 start_time = time.time()
                
#                 # 等待并获取帧数据
#                 frames = self.pipeline.wait_for_frames()
#                 aligned_frames = self.align.process(frames)

#                 depth_frame = aligned_frames.get_depth_frame()
#                 color_frame = aligned_frames.get_color_frame()

#                 if not depth_frame or not color_frame:
#                     rospy.logwarn("Skipping frame due to missing data")
#                     continue

#                 # 转换OpenCV格式
#                 depth_image = np.asanyarray(depth_frame.get_data())
#                 color_image = np.asanyarray(color_frame.get_data())

#                 # 发布彩色图像
#                 color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
#                 color_msg.header.stamp = rospy.Time.now()
#                 color_msg.header.frame_id = "camera_link"
#                 self.color_pub.publish(color_msg)
                
#                 # 发布深度图像（16UC1格式，单位：毫米）
#                 depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
#                 depth_msg.header.stamp = color_msg.header.stamp
#                 depth_msg.header.frame_id = "camera_link"
#                 self.depth_pub.publish(depth_msg)
                
#                 # 发布相机信息
#                 cam_info = self.cinfo_manager.getCameraInfo()
#                 cam_info.header.stamp = color_msg.header.stamp
#                 self.camera_info_pub.publish(cam_info)
                
#                 # 调试：显示深度预览（缩放以提高可视性）
#                 depth_preview = cv2.convertScaleAbs(depth_image, alpha=0.03)
#                 depth_preview = cv2.applyColorMap(depth_preview, cv2.COLORMAP_JET)
#                 cv2.imshow('Depth Preview', depth_preview)
                
#                 # 处理按键事件
#                 key = cv2.waitKey(1)
#                 if key == 27:  # ESC键
#                     rospy.signal_shutdown("User requested shutdown")
                
#                 # 控制循环频率
#                 loop_time = time.time() - start_time
#                 rospy.sleep(max(0, 0.033 - loop_time))  # 保持约30Hz

#         except Exception as e:
#             rospy.logerr(f"Error in publishing loop: {str(e)}")
            
#         finally:
#             rospy.loginfo("Shutting down RealSense pipeline")
#             self.pipeline.stop()
#             cv2.destroyAllWindows()


# if __name__ == '__main__':
#     try:
#         publisher = RealsensePublisher()
#         publisher.publish_images()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("ROS interrupted")
#     except Exception as e:
#         rospy.logerr(f"Unexpected error: {str(e)}")