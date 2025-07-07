#include"registration.cpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "registration_node");
    //std::cout<<"xxx"<<std::endl;
    registration position;
    ros::spin();
    return 0;
}