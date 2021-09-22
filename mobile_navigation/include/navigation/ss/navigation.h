#include "ros/ros.h"

//---Input
#include "geometry_msgs/Twist.h" //<-- to control the robot in velocity
#include "nav_msgs/Odometry.h" //<-- to read the current position of the robot
#include "sensor_msgs/LaserScan.h" //<-- to read the 2d Lidar
#include "nav_msgs/Path.h"
#include "std_msgs/Int32.h"

#include "boost/thread.hpp"
#include "tf/tf.h"

using namespace std;

class NAVIGATION {

    public:
        NAVIGATION();
        void run(); //<-- used to start all the parallel functions of my system
        void ctrl_loop(); //<-- main loop function

        void odometry_cb( nav_msgs::Odometry odom );
        void fake_path_cb( std_msgs::Int32 fake_value );
        void plan_cb( nav_msgs::Path geom_path);

        // bool service_callback( ros_service::service::Request &req, ros_service::service::Response &res);


        bool navigation( float x, float y , float v_x, float v_y);

    
    private:

        ros::NodeHandle _nh;
        geometry_msgs::Point _curr_p;
        double               _curr_yaw;

        //Publishers and Subscribers
        ros::Subscriber _odom_sub; 
        ros::Subscriber _fake_path_sub; 
        ros::Subscriber _planner_sub;

        ros::Publisher  _cmd_vel_pub;   

        ros::Publisher _test_pub;    
        ros::Publisher _result_pub; 

        //Service for AR positioning routine
        // ros::ServiceServer _service;

        //Control variables
        float b;                        //distance between the contact point of the wheel with the ground and the point B on the sagittal axis

        geometry_msgs::Point _B_p;      //point B along the sagittal axis

        float k[2];




        //Control flags
        bool _path_received;    //wait to receive the path from the planner
        bool _first_odom;       //wait to receive odom data
        bool _path_obstacle;
        
        int _wp_index;

        bool _finish;

        nav_msgs::Path path;
        vector< geometry_msgs::Point > _wp_list;
        vector< geometry_msgs::Vector3 > _wp_vel_list;

};
