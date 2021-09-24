#include "ros/ros.h"
#include"mobile_navigation/service.h"

//---Input
#include "geometry_msgs/Twist.h" //<-- to control the robot in velocity
#include "geometry_msgs/Pose.h"

#include "nav_msgs/Odometry.h" //<-- to read the current position of the robot
#include "sensor_msgs/LaserScan.h" //<-- to read the 2d Lidar
#include "nav_msgs/Path.h"
#include "std_msgs/Int32.h"

#include "aruco_msgs/MarkerArray.h"
#include "aruco_msgs/Marker.h"


#include "boost/thread.hpp"
#include "tf/tf.h"

using namespace std;

class POS_ROUTINE {

    public:
        POS_ROUTINE();
        void run(); //<-- used to start all the parallel functions of my system
        void ctrl_loop(); //<-- main loop function


        bool rotating();
        bool positioning();
        bool adjusting();
        bool go();

        //histeresys threshold for the field of view on the x
        int ist;

        //callback to retrieve the aruco pose
        void vision_cb( aruco_msgs::MarkerArray markers);

        bool service_callback(mobile_navigation::service::Request &req, mobile_navigation::service::Response &res);

    
    private:

        ros::NodeHandle _nh;
        // geometry_msgs::Point _curr_p;
        // double               _curr_yaw;

        geometry_msgs::Pose relative_pose;      //relative pose turtle->AR
        double roll, pitch, yaw;                 //RPY from quaternion


        //id received from the publisher
        int current_id;

        float aligned_pitch;            //relative pitch when the cart is pointing the AR

        //Publishers and Subscribers
    

        //subscriber that calls the vision_cb for the aruco pose
        ros::Subscriber position_error;

        //publish the desired velocity
        ros::Publisher  _cmd_vel_pub;  

        ros::ServiceServer _service; 

        mobile_navigation::service::Request request;
        mobile_navigation::service::Response response;

        //Service for AR positioning routine
        // ros::ServiceServer _service;

        //Control flags
        bool _first_lecture;
        bool _in_view;

        bool _finish;

};
