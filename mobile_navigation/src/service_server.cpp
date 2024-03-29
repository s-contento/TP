// #include "positioning_routine.h"
#include "positioning_routine.cpp"
#include"mobile_navigation/service.h"


using namespace std;


bool service_callback( mobile_navigation::service::Request &req, mobile_navigation::service::Response &res){

    POS_ROUTINE p_r;
    p_r.run();
    std::cout <<"\nafter run\n";
    return true;
}


int main(int argc, char **argv){

    ros::init( argc, argv, "positioning_routine_node");

    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("service", service_callback);

    ROS_INFO( "Ready to receive from client.");

    ros::spin();

    return 0;
}