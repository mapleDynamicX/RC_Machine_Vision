#include "publish.cpp"





int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_node");
    pub publish;
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        publish.pub_image();
        //publish.pub_compress_img();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
    
}