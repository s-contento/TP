// #include "positioning_routine.h"
#include "positioning_routine.cpp"
#include"ros_service/service.h"


using namespace std;


bool service_callback( ros_service::service::Request &req, ros_service::service::Response &res){

    POS_ROUTINE p_r;
    p_r.run();
    
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