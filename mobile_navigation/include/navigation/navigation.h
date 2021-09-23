#include "ros/ros.h"

//---Input
#include "geometry_msgs/Twist.h" //<-- to control the robot in velocity
#include "nav_msgs/Odometry.h" //<-- to read the current position of the robot
#include "geometry_msgs/PoseWithCovarianceStamped.h" //<-- to read the current position of the robot


#include "sensor_msgs/LaserScan.h" //<-- to read the 2d Lidar
#include "nav_msgs/Path.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Pose2D.h>


#include "boost/thread.hpp"
#include "tf/tf.h"

#include <visualization_msgs/Marker.h>


using namespace std;

class NAVIGATION {

    public:
        NAVIGATION();
        void run(); //<-- used to start all the parallel functions of my system
        void ctrl_loop(); //<-- main loop function

        // void odometry_cb( nav_msgs::Odometry odom );
        void odometry_cb( geometry_msgs::PoseWithCovarianceStamped odom );

        void fake_path_cb( std_msgs::Int32 fake_value );
        void plan_cb( nav_msgs::Path geom_path);
        void laser_cb( sensor_msgs::LaserScan geom_path);

        // bool service_callback( ros_service::service::Request &req, ros_service::service::Response &res);


        bool navigation( float x, float y , float v_x, float v_y);

        //saturation on maximum linear and angular velocities
        double lin_sat;
        double ang_sat;

        double scaling_factor;

    
    private:

        visualization_msgs::Marker _target_marker;
        visualization_msgs::Marker _point_b_marker;
        visualization_msgs::Marker _odom_marker;

        ros::NodeHandle _nh;
        geometry_msgs::Point _curr_p;
        double               _curr_yaw;

        nav_msgs::Odometry point_odom;

        //Publishers and Subscribers
        ros::Subscriber _odom_sub; 
        ros::Subscriber _fake_path_sub; 
        ros::Subscriber _planner_sub;
        ros::Subscriber _lidar_sub;

        ros::Publisher  _cmd_vel_pub;   
        ros::Publisher _test_pub;   
        ros::Publisher _point_pose_pub;
        ros::Publisher _result_pub; 
        ros::Publisher errors_pub;
        ros::Publisher desired_traj_pub;

        ros::Publisher _marker_pub;  
        ros::Publisher _point_b_pub;  
        ros::Publisher _odom_marker_pub;  


        //Service for AR positioning routine
        // ros::ServiceServer _service;

        //Control variables
        float b;                        //distance between the contact point of the wheel with the ground and the point B on the sagittal axis

        geometry_msgs::Point _B_p;      //point B along the sagittal axis

        float k[2];

        int _impact_index;
        float _heading_lecture;
        float _right_lecture;
        float _left_lecture;


        float _laser_threshold;



        //Control flags
        bool _path_received;    //wait to receive the path from the planner
        bool _first_odom;       //wait to receive odom data

        bool _path_obstacle_left;
        bool _path_obstacle_right;
        
        int _wp_index;

        bool _finish;

        nav_msgs::Path path;
        vector< geometry_msgs::Point > _wp_list;
        vector< geometry_msgs::Vector3 > _wp_vel_list;

};
