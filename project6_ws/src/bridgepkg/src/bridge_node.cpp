#include"bridge2.cpp"

//将深度相机的数据转为彩色点云
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bridge_node");
    bridge cam_to_cloud;
    ros::spin();
    return 0;
}












