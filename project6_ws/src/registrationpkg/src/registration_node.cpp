// #include"registration2.cpp"


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "registration_node");
//     //std::cout<<"xxx"<<std::endl;
//     registration2 position;
//     ros::spin();
//     return 0;
// }





//////////////////////////////////////////////////////////////////////////////////////

// #include"registration3.cpp"


// // void c_tf()
// // {
// //     std::cout << "tf ID: " << std::this_thread::get_id() << std::endl;
// //     ros::NodeHandle nh("/");
// //     ros::Subscriber _tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, &tfCallback);
// //     ros::spin();
// // }

// // void c_cloud()
// // {
// //     std::cout << "cloud ID: " << std::this_thread::get_id() << std::endl;
// //     ros::NodeHandle nh("/");
// //     ros::Subscriber _cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("rgb_cloud", 1, &cloudCallback);
// //     ros::spin();
// // }

// // void count(registration2& position)
// // {
    
// //     std::cout<<"in count id: "<< std::this_thread::get_id() << std::endl;
// //     //while(ros::ok())
// //     //{
// //     ros::Time tic = ros::Time::now();
// //     position.calculate();
// //     ROS_INFO("total Time: %f", (ros::Time::now() - tic).toSec());
// //     //}
// //     std::cout<<"out count id: "<< std::this_thread::get_id() << std::endl;
// // }

// void count()
// {
//     registration2 position;
//     std::cout<<"in count id: "<< std::this_thread::get_id() << std::endl;
//     while(ros::ok())
//     {
//         ros::Time tic = ros::Time::now();
//         position.calculate();
//         ROS_INFO("total Time: %f", (ros::Time::now() - tic).toSec());
//         //std::cout<<(ros::Time::now() - tic).toSec()<<std::endl;
//     }
//     std::cout<<"out count id: "<< std::this_thread::get_id() << std::endl;
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "registration_node");
//     //std::cout<<"xxx"<<std::endl;
//     // registration2 position;
//     // std::thread tf_(c_tf);
//     // std::thread cloud_(c_cloud);
//     // ros::spin();
    
//     ros::NodeHandle nh("/");
//     ros::Subscriber _tf_sub = nh.subscribe<tf2_msgs::TFMessage>("tf", 1, &tfCallback);
//     ros::Subscriber _cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("rgb_cloud", 1, &cloudCallback);
//     std::thread worker(count);
//     ros::AsyncSpinner spinner(2);
//     ros::Rate sl(30);
//     spinner.start();
//     registration2 position;
//     while(ros::ok())
//     {
//         // ros::Time tic = ros::Time::now();
//         // position.calculate();
//         // ROS_INFO("total Time: %f", (ros::Time::now() - tic).toSec());
//         //std::thread worker(count, std::ref(position));
//         //worker.join();
//         sl.sleep();
//     }
//     std::cout<<"out main id: "<< std::this_thread::get_id() << std::endl;
//     ros::waitForShutdown();
//     worker.join();
//     return 0;
// }


////////////////////////////////////////////////////////////



#include"registration4.cpp"



registration2 position;
void count()
{
    
    std::cout<<"in count id: "<< std::this_thread::get_id() << std::endl;
    while(ros::ok())
    {
        ros::Time tic = ros::Time::now();
        position.calculate();
        ROS_INFO("calculate Time: %f", (ros::Time::now() - tic).toSec());
        //std::cout<<(ros::Time::now() - tic).toSec()<<std::endl;
    }
    std::cout<<"out count id: "<< std::this_thread::get_id() << std::endl;
}

void map()
{
    
    std::cout<<"in count id: "<< std::this_thread::get_id() << std::endl;
    while(ros::ok())
    {
        ros::Time tic = ros::Time::now();
        position.map();
        ROS_INFO("map Time: %f", (ros::Time::now() - tic).toSec());
        //std::cout<<(ros::Time::now() - tic).toSec()<<std::endl;
    }
    std::cout<<"out count id: "<< std::this_thread::get_id() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "registration_node");
    //std::cout<<"xxx"<<std::endl;
    // registration2 position;
    // std::thread tf_(c_tf);
    // std::thread cloud_(c_cloud);
    // ros::spin();
    
    ros::NodeHandle nh("/");
    //ros::Subscriber _tf_sub = nh.subscribe<tf2_msgs::TFMessage>("tf", 1, &tfCallback);
    ros::Subscriber _cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("rgb_cloud", 1, &cloudCallback);
    ros::Subscriber _tf_sub = nh.subscribe("camera/tf", 1, &transformCallback);
    //std::thread worker(count);
    std::thread worker(map);
    //std::thread worker2(map);
    ros::AsyncSpinner spinner(2);
    //ros::Rate sl(30);
    spinner.start();
    registration2 position;
    while(ros::ok())
    {
        // ros::Time tic = ros::Time::now();
        // position.calculate();
        // ROS_INFO("total Time: %f", (ros::Time::now() - tic).toSec());
        //std::thread worker(count, std::ref(position));
        //worker.join();
        ros::Time tic = ros::Time::now();
        position.calculate();
        ROS_INFO("calculate Time: %f", (ros::Time::now() - tic).toSec());
        //sl.sleep();
    }
    std::cout<<"out main id: "<< std::this_thread::get_id() << std::endl;
    ros::waitForShutdown();
    worker.join();
    return 0;
}











































