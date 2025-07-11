#include"registration2.cpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "registration_node");
    //std::cout<<"xxx"<<std::endl;
    registration2 position;
    ros::spin();
    return 0;
}