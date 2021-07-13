#include "positioning_routine.h"

using namespace std;


#define LIN_VEL 0.2
#define ANG_VEL 0.5


POS_ROUTINE::POS_ROUTINE() {
    

    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
    position_error = _nh.subscribe("/aruco_marker_publisher/markers", 1, &POS_ROUTINE::vision_cb, this);

    // _service = nh.advertiseService("service", service_callback);
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    current_id = 26;
    aligned_pitch = 0;



    _first_lecture = false; /////
    _in_view = false;

    _finish = false;

    ist = 0;


    // _path_obstacle = false;
    // _human_mode = false;
    // _fv = _rv = 0.0;
    // _record_wp = false;

    // _finish = false;
}


void POS_ROUTINE::vision_cb( aruco_msgs::MarkerArray markers ) {

    if(markers.markers[0].id == current_id){
        relative_pose = markers.markers[0].pose.pose;

        tf::Quaternion q( relative_pose.orientation.x, relative_pose.orientation.y , relative_pose.orientation.z ,  relative_pose.orientation.w );
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        //std::cout << "relative pose " << relative_pose.position.z;

        _first_lecture = true;
        _in_view = true;
    }else{
        _in_view = false;

    }
}


bool POS_ROUTINE::adjusting(){

    bool flag = false;
    geometry_msgs::Twist cmd;

    if (relative_pose.position.z <= 1.5){

        cmd.linear.x = -0.10;

        cmd.angular.z = 0.0;

        _cmd_vel_pub.publish(cmd);
        flag = false;

    }
    else if (relative_pose.position.z >=2){

        cmd.linear.x = 0.10;

        cmd.angular.z = 0.0;

        _cmd_vel_pub.publish(cmd);
        flag = false;

    }else{
        cmd.linear.x = 0.0;

        cmd.angular.z = 0.0;

        _cmd_vel_pub.publish(cmd);
        flag = true;
    }

   
    return flag;
}


//rotate to point to the center of the aruco marker
bool POS_ROUTINE::positioning(){

    bool flag = false;
    geometry_msgs::Twist cmd;

   std::cout << "xX : " << relative_pose.position.x<<endl;
    if(fabs(relative_pose.position.x) >= 0.1){
        cmd.linear.x = 0.0;
        // cmd.linear.z = 0.0;

        cmd.angular.z = -0.10*relative_pose.position.x/fabs(relative_pose.position.x);

        _cmd_vel_pub.publish(cmd);

       // std::cout << "pitch : " << pitch<<endl;
        
        flag = false;


    }else{

        if (relative_pose.position.z <= 1.5){

        cmd.linear.x = -0.10;

        cmd.angular.z = 0.0;

        _cmd_vel_pub.publish(cmd);
        flag = false;

        }
        else if (relative_pose.position.z >=2){

            cmd.linear.x = 0.10;

            cmd.angular.z = 0.0;

            _cmd_vel_pub.publish(cmd);
            flag = false;

        }else{

            aligned_pitch = pitch;
            cmd.linear.x = 0.0;

            cmd.angular.z = 0.0;

            _cmd_vel_pub.publish(cmd);
            flag = true;
        }
        }

    return flag;
}

//For each loop it calculates the desired velocity to move the robot
bool POS_ROUTINE::rotating() {

    bool flag;
    geometry_msgs::Twist cmd;
    float distance = sqrt ( pow( relative_pose.position.z, 2) +  pow(relative_pose.position.x, 2) );

    // float target = 1.571 - fabs(aligned_pitch); 

    if(fabs(aligned_pitch)<= 0.5){
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;

        _cmd_vel_pub.publish(cmd);
        
        flag = true;
        std::cout << "\nangolo piccolo" <<endl;


        _finish = true;
    }else if(fabs(relative_pose.position.x)<=(0.5/distance)){
    
        //float distance = sqrt ( pow( relative_pose.position.z, 2) +  pow(relative_pose.position.x, 2) ); //norm of the motion vector
        if (fabs(relative_pose.position.x)<=(0.4-ist)/distance){
            ist = 0;
            cmd.linear.x = 0.0;
            // cmd.linear.z = 0.0;

            cmd.angular.z = 0.10*pitch/fabs(pitch);

            _cmd_vel_pub.publish(cmd);

            std::cout << "adj x : " << relative_pose.position.x<<endl;
            
            flag = false;

        }
        // else if(fabs(relative_pose.position.x)>=(0.7*relative_pose.position.x/distance)){
        //     ist = 0;
        //     cmd.linear.x = 0.0;
        //     // cmd.linear.z = 0.0;

        //     cmd.angular.z = 0.10*pitch/fabs(pitch);

        //     _cmd_vel_pub.publish(cmd);

        //     std::cout << "adj x : " << relative_pose.position.x<<endl;
            
        //     flag = false;
        // }
        else{

            ist = 0.60;
            if(fabs(relative_pose.position.z)>=1.0){
                cmd.linear.x = 0.10;
                cmd.angular.z = 0.0;

                _cmd_vel_pub.publish(cmd);
                std::cout << "\ngoing\n"<<endl;
                std::cout << "adj z : " << relative_pose.position.z<<endl;
                flag = false;

            }
            else{
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;

                _cmd_vel_pub.publish(cmd);
        
                flag = true;
                ist = 0;

                std::cout << "\nOKOK\n"<<endl;
            }

            
        // std::cout << "\ngo" <<endl;

        }
    }else{
        std::cout << "\nout of fov\n";

        cmd.linear.x = 0.0;
        cmd.angular.z = -0.10*pitch/fabs(pitch);

        _cmd_vel_pub.publish(cmd);
        
        flag = false;

    }
    distance = sqrt ( pow( relative_pose.position.z, 2) +  pow(relative_pose.position.x, 2) );
    return flag;
    
 }

 bool POS_ROUTINE::go() {

     return false;
 }



void POS_ROUTINE::ctrl_loop() {

    std::cout << "\nWaiting first AR lecture!\n";
    while( !_first_lecture ) sleep(1);

    // while( !_path_received ) sleep(1);

    ros::Rate r(100);

    std_msgs::Int32 res;


    geometry_msgs::Twist cmd;

    std::cout << "\nStarting adjusting routine... \n";

    while ( ros::ok() ) {

        // while(!rotating()){
        //     r.sleep();
        // std::cout <<"\nrotating\n";

        // }

        while(!positioning()){
            r.sleep();
            std::cout <<"\nPositioning\n";
        }
        std::cout << "z : " << relative_pose.position.z<<endl;

        while(!adjusting()){
            r.sleep();
            std::cout <<"\nAdjusting\n";

        }

        while (!rotating()){
            r.sleep();
            std::cout <<"\nRotating\n";
        }

        if (_finish == true){

            break;
        }

        // while(!go()){
        //      r.sleep();
        //     std::cout <<"\nGo\n";

        // }

        r.sleep();
    }
    


}

void POS_ROUTINE::run() {

    
    boost::thread ctrl_loop_t( &POS_ROUTINE::ctrl_loop, this );
    ros::spin();
    //---
}



// int main( int argc, char** argv ) {

//     ros::init( argc, argv, "positioning_routine_node");

//     POS_ROUTINE p_r;
//     p_r.run();


//     return 0;
// }