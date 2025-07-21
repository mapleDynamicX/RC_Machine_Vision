#include "publish.cpp"





int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_node");
    pub publish;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::Time tic = ros::Time::now();
        publish.pub_image();
        //publish.pub_compress_img();
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("publish.pub_image(): %f", (ros::Time::now() - tic).toSec());
    }
    return 0;
    
}