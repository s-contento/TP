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
        void laser_cb( std_msgs::Int32 laser );
        void plan_cb( nav_msgs::Path geom_path);


        bool navigation( float x, float y , float v_x, float v_y);
        void human_input();
        void train_traj();
    private:

        ros::NodeHandle _nh;
        geometry_msgs::Point _curr_p;
        double               _curr_yaw;


        ros::Subscriber _odom_sub; 
        ros::Subscriber _lidar_sub; 
        ros::Subscriber _planner_sub;

        ros::Publisher  _cmd_vel_pub;   

        ros::Publisher _test_pub;     

        //Control variables
        float b;                        //distance between the contact point of the wheel with the ground and the point B on the sagittal axis

        geometry_msgs::Point _B_p;      //point B along the sagittal axis

        float k[2];




        //Control flags
        bool _path_received;
        bool _first_odom;
        bool _path_obstacle;
        bool _human_mode;
        bool _record_wp;
        double _fv;
        double _rv;
        int _wp_index;

        nav_msgs::Path path;
        vector< geometry_msgs::Point > _wp_list;
        vector< geometry_msgs::Vector3 > _wp_vel_list;

};
