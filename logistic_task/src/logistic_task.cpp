#include"ros/ros.h"
#include"std_msgs/Int32.h"
#include"std_msgs/String.h"
#include<iostream>
#include<string>
#include<cstdlib>

#include "boost/thread.hpp"

#include"ros_service/service.h"




//#include"adder/addendi.h"       //custo msgs
//#include"adder/sum.h"

#include "aruco_msgs/MarkerArray.h"
#include "aruco_msgs/Marker.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"



//Do the transformation map->camera
#include <tf/transform_listener.h>

//number of rooms in the scene
#define N_ROOM 4

using namespace std;

//class to subscribe in /numbers and publish in /sum
class ROS_SUB {
    public:
        ROS_SUB();
        void run(); //<-- used to start all the parallel functions of my system
        void task_loop();


        void topic_cb( aruco_msgs::MarkerArray markers );
        void result_cb( std_msgs::Int32 result );

        void clk_p_cb( geometry_msgs::PointStamped clk_p );



        void set_pos( geometry_msgs::Point p, int index);
        void get_pos( geometry_msgs::Point p, int index);

        void take_input();

    private:
        ros::NodeHandle _nh;

        ros::Subscriber _topic_sub;
        ros::Publisher _topic_pub;

        ros::Subscriber _controller_result;
        ros::Subscriber _clk_p;

        ros::Publisher _goal_pub;

        ros::ServiceClient _client;

        //take the position of the AR markers,
        //when position is obtained located = true
        geometry_msgs::Point _rooms[3];
        geometry_msgs::Point _offset[3];

        bool _located[3];

        //TRUE navigate to retrieve position of the rooms
        //FALSE position of the rooms provided
        bool _retrieve_pos;

        bool manual;

        //result from the controller arrived
        bool result_arr;


        //Declare the listener to use c++ tf API
 	    tf::TransformListener listener;
	    //Declare the tranfsorm object to store tf data
        tf::StampedTransform transform;

        //state of the system
        //0 --> reaching the W
        //1 --> searching for the AR Marker
        //2 --> adjusting posture wrt marker routine
        //3 --> reading the Marker
        //4 --> setting the goal and going to R1/R2
        int state;

        //state changed
        bool changed = false;

        //ID of the marker
        std_msgs::Int32 current_Id;


};

ROS_SUB::ROS_SUB() {

    for(int i = 0; i <= N_ROOM; i++){
        _located[i]=false;
    }

    _offset[0].x = 3;
    _offset[0].y = 2;
    _offset[0].z = 0;

    _offset[1].x = -2;
    _offset[1].y = 1.5;
    _offset[1].z = 0;

    _offset[2].x = -2.5;
    _offset[2].y = -0.3;
    _offset[2].z = 0;

    _offset[3].x = 4;
    _offset[3].y = 0;
    _offset[3].z = 0;
///////////////////////////////////////////////////////////add condition to be false
    _retrieve_pos = true;

    result_arr = false;

    state = 10;
    current_Id.data = -1;

    manual = false;

    _client = _nh.serviceClient<ros_service::service>("service");

    _topic_pub = _nh.advertise<std_msgs::Int32>("/recognized_ID",10);
    _goal_pub = _nh.advertise<geometry_msgs::Pose>("/goals",0);

    
    _topic_sub = _nh.subscribe("/aruco_marker_publisher/markers", 1, &ROS_SUB::topic_cb, this);
    _controller_result = _nh.subscribe("/controller_result", 1, &ROS_SUB::result_cb, this);

    _clk_p = _nh.subscribe("/clicked_point", 0, &ROS_SUB::clk_p_cb, this);

    // ros::Rate rate(10);

}

void ROS_SUB::set_pos( geometry_msgs::Point p, int index){

    _rooms[index] = p;

}
void ROS_SUB::get_pos( geometry_msgs::Point p, int index){
    p = _rooms[index];
}


void ROS_SUB::clk_p_cb( geometry_msgs::PointStamped clk_p ){

    std::cout << "clicked";

    geometry_msgs::Pose _w_goal;

    _w_goal.position = clk_p.point;
    
    _goal_pub.publish(_w_goal);

    state = 5;
    changed = true;
}

void ROS_SUB::topic_cb( aruco_msgs::MarkerArray markers){
    if (state == 2){
        if((markers.markers[0].pose.pose.position.z <= 2) && (markers.markers[0].pose.pose.position.x <= 2) && (markers.markers[0].pose.pose.position.y <= 2))
        {
        //if controllo su posa relativa Ar Marker-Camera, in modo da non avere lettura troppo sbagliata

            current_Id.data = markers.markers[0].id;

            ROS_INFO("ID received: [%i]", current_Id.data);
            state = 3;
            changed = true;
        }

    }
    if(state == 4){

        if((markers.markers[0].pose.pose.position.z >= 2) || (markers.markers[0].pose.pose.position.x >= 2) || (markers.markers[0].pose.pose.position.y >= 2))
        {
        //if controllo su posa relativa Ar Marker-Camera, in modo da non avere lettura troppo sbagliata
            ROS_INFO("\n\nMarker Lost!\n\n");
            state = 5;
            changed = true;
        }
    }
    // else if(state == 4){
    //     cout << "\n\n GOAL REACHED! TASK CONCLUDED!\n";
    //     state = 10;
    //     changed = true;
    // }

}

void ROS_SUB::result_cb( std_msgs::Int32 result){

    cout << "\n\n Result arrived : [" << result.data << "] ";
    
    // if(!result_arr){
    if (state == 1){
        if (result.data == 0){

            state = 8;
            changed = true;
            cout << "SUCCEEDED!\n";
        }
        else{
            state = 10;
            changed = true;
            cout << "FAILED!\n";
        }
    }

    else if(state == 4){
        if (result.data != 0){
            state = 10;
            changed = true;
            cout << "\n\n FAILED!\n";

        }
    }

    else if(state == 5){
        if (result.data == 0){
            cout << "\n\n GOAL REACHED! TASK CONCLUDED!\n";

        }else{
            cout << "FAILED!\n";
        }
        state = 10;
        changed = true;
    }
    // }
    // result_arr = true;


}


void ROS_SUB::take_input(){

    int input;
    geometry_msgs::Pose _w_goal;


    std::cout << "\n[0] : Start the logistic task.\n";
    std::cout << "\n[1] : Go to R1.\n";
    std::cout << "\n[2] : Go to R2.\n";
    std::cout << "\n[3] : Go to a desired position clicked on the map.\n";

    cin >> input;

    switch(input){
        case 0:
            state = 0;
            changed = true;
        break;

        case 1:
            if (_nh.hasParam("r1_x"))
            {
                _nh.getParam("r1_x",_w_goal.position.x);
            }
            else{
                _w_goal.position.x = 3.0;
            }
            if (_nh.hasParam("r1_y"))
            {
                _nh.getParam("r1_y",_w_goal.position.y);
            }
            else{
                _w_goal.position.y = 4.0;
            }

            _w_goal.orientation.w = 1.0;
    
            _goal_pub.publish(_w_goal);

            state = 5;
            changed = true;

        break;
        case 2:
            if (_nh.hasParam("r2_x"))
            {
                _nh.getParam("r2_x",_w_goal.position.x);
            }
            else{
                _w_goal.position.x = 3.0;
            }
            if (_nh.hasParam("r2_y"))
            {
                _nh.getParam("r2_y",_w_goal.position.y);
            }
            else{
                _w_goal.position.y = -4.0;
            }

            _w_goal.orientation.w = 1.0;
    
            _goal_pub.publish(_w_goal);

            state = 5;
            changed = true;
        break;

        case 3:
            state = 9;
            changed = true;

        break;
        default:

            std::cout <<"default\n\n";
    }

}



void ROS_SUB::task_loop() {
    geometry_msgs::Pose _w_goal;
    ros_service::service srv;
    //         ros::Rate rate(10);

    // rate.sleep();
    while(ros::ok() && !manual){

        switch(state){

            case 0:
                
                //if(changed){
                std::cout << "\n SENDING GOAL (WH POSITION) TO CONTROLLER.\n\n";
                //}

                if (_nh.hasParam("wh_x"))
                {
                    _nh.getParam("wh_x",_w_goal.position.x);
                }
                else{
                    _w_goal.position.x = 8.0;
                }
                 if (_nh.hasParam("wh_y"))
                {
                    _nh.getParam("wh_y",_w_goal.position.y);
                }
                else{
                    _w_goal.position.y = -4.0;
                }

                _w_goal.orientation.w = 1.0;

                _goal_pub.publish(_w_goal);

                std::cout << "GOAL WH POSITION PUBLISHED";
                state = 1;
                changed = true;

                break;

            case 1:

                if(changed){
                std::cout << "\n\n Waiting for results ...\n";
                
                changed = false;
                }

                break;

            case 2:

                if(changed){
                std::cout << "\n\n Reading ID ...\n";
                changed = false;
                }


                break;


            case 3:

                if(changed){
                std::cout << "\n\n SETTING NEW GOAL...\n";
                changed = false;
                }

                if(current_Id.data == 1){
                    std::cout << "\n\n ROOM 1!\n";
                    state = 4;
                    changed = true;


                    if (_nh.hasParam("r1_x"))
                    {
                        _nh.getParam("r1_x",_w_goal.position.x);
                    }
                    else{
                        _w_goal.position.x = 3.0;
                    }
                    if (_nh.hasParam("r1_y"))
                    {
                        _nh.getParam("r1_y",_w_goal.position.y);
                    }
                    else{
                        _w_goal.position.y = 4.0;
                    }

                    _w_goal.orientation.w = 1.0;

                    _goal_pub.publish(_w_goal);

                }else if(current_Id.data == 2){
                    std::cout << "\n\n ROOM 2!\n";
                    state = 4;
                    changed = true;


                    if (_nh.hasParam("r2_x"))
                    {
                        _nh.getParam("r2_x",_w_goal.position.x);
                    }
                    else{
                        _w_goal.position.x = 3.0;
                    }
                    if (_nh.hasParam("r2_y"))
                    {
                        _nh.getParam("r2_y",_w_goal.position.y);
                    }
                    else{
                        _w_goal.position.y = -4.0;
                    }

                    _w_goal.orientation.w = 1.0;

                    _goal_pub.publish(_w_goal);
                }
                else{
                    cout << "\n\n UNDEFINED ID!";
                    state = 10;
                    changed = true;
                }


                break;

            case 4:

                if(changed){
                std::cout << "\n\n Waiting for AR to leave fov ...\n";
                changed = false;
                }


                break;

            case 5:

                if(changed){
                std::cout << "\n\n Waiting for results ...\n";
                changed = false;
                }


                break;

            // case 8:
            //     if(changed){
            //         std::cout << "\n\n Adjusting routine\n";
            //         changed = false;
            //     }
            //     break;
            case 8:

                std::cout << "\n\n SEARCHING THE MARKER TO READ ...\n";
                

                cout << "\n\nCalling  ... ]\n\r";
                if(_client.call(srv)){
                    cout << "\n\nCalling service ... ]\n\r";
                    cout << "\n\nFrom Client: ["<< srv.request.in << "]\n\r";
                    cout << "\nReceived: " << srv.response.out<<endl;
                }
                else{
                    ROS_ERROR("Failed to call Service");
                }
                cout << "\n\nSERVER RESPONSE ARRIVED!!!\n\r";

                state = 2;
                changed = true;
                


                break;

            case 9:

                if(changed){
                    std::cout << "\n\n Click a point on Rviz to start navigating...\n";
                    changed = false;
                }
            
                break;

            default:
                std::cout << "\nStart\n\n";
            
                manual = true;

        }




        if (manual){
            take_input();

            manual = false;  
            changed = true;      

        }

    }

    
}

void ROS_SUB::run() {

    boost::thread task_loop_t( &ROS_SUB::task_loop, this );
    ros::spin();
    //---
}


int main(int argc, char **argv){
    
    ros::init(argc,argv,"ar_marker_id");
    ROS_SUB rs;

    rs.run();
    return 0;
}