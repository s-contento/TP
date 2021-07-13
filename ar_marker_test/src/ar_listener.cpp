// When an AR Marker is found analize the ID and save the correspondent pose:
//
//                      ID 0 --> pose of W
//                      ID 1 --> pose of R1 ecc..
//
// The position of the room is obtained by adding the relative position between camera-->marker with the TF map-->camera

#include"ros/ros.h"
#include"std_msgs/Int32.h"
#include"std_msgs/String.h"
#include<iostream>
#include<string>
#include<cstdlib>


//#include"adder/addendi.h"       //custo msgs
//#include"adder/sum.h"

#include "aruco_msgs/MarkerArray.h"
#include "aruco_msgs/Marker.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"


//Do the transformation map->camera
#include <tf/transform_listener.h>

//number of rooms in the scene
#define N_ROOM 4

using namespace std;

//class to subscribe in /numbers and publish in /sum
class ROS_SUB {
    public:
        ROS_SUB();

        void topic_cb( aruco_msgs::MarkerArray markers );

        void set_pos( geometry_msgs::Point p, int index);
        void get_pos( geometry_msgs::Point p, int index);

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _topic_sub;
        ros::Publisher _topic_pub;

        //take the position of the AR markers,
        //when position is obtained located = true
        geometry_msgs::Point _rooms[3];
        geometry_msgs::Point _offset[3];

        bool _located[3];

        //TRUE navigate to retrieve position of the rooms
        //FALSE position of the rooms provided
        bool _retrieve_pos;


        //Declare the listener to use c++ tf API
 	    tf::TransformListener listener;
	    //Declare the tranfsorm object to store tf data
        tf::StampedTransform transform;

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

    _topic_pub = _nh.advertise<std_msgs::Int32>("/recognized_ID",10);

    ros::Rate rate(10);
    _topic_sub = _nh.subscribe("/aruco_marker_publisher/markers", 1, &ROS_SUB::topic_cb, this);
}

void ROS_SUB::set_pos( geometry_msgs::Point p, int index){

    _rooms[index] = p;

}
void ROS_SUB::get_pos( geometry_msgs::Point p, int index){
    p = _rooms[index];
}

void ROS_SUB::topic_cb( aruco_msgs::MarkerArray markers){
    std_msgs::Int32 current_Id;
    geometry_msgs::Point map_pos;
    geometry_msgs::PoseStamped p;
    geometry_msgs::PoseStamped trans_p;
    std_msgs::String param_name;

    current_Id.data = markers.markers[0].id;

    _topic_pub.publish(current_Id);

    if(markers.markers[0].pose.pose.position.z <= 2)
    {
//if controllo su posa relativa Ar Marker-Camera, in modo da non avere lettura troppo sbagliata

    if (!_located[current_Id.data]){
        p.header.frame_id = "camera_rgb_optical_frame";
        p.pose.position = markers.markers[0].pose.pose.position;
        p.pose.orientation = markers.markers[0].pose.pose.orientation;
        //tf2::convert(quat_msg , quat_tf);
        //tf::quaternionTFToMsg(markers.markers[0].pose.pose.orientation,p.pose.orientation);

        try {
			//We want the current transform
			ros::Time now = ros::Time::now();

			if( listener.waitForTransform("camera_rgb_optical_frame","/map", now, ros::Duration(1.0)) ) {
				//listener.lookupTransform("/map", "/camera_rgb_frame",  now, transform);
                //listener.lookupTransform("/map", "camera_rgb_optical_frame",  now, transform);
                listener.transformPose("/map", p,trans_p);
				//std::cout << "Translation: " << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl; 
				//std::cout << "Rotation: " << transform.getRotation().w() << " " << transform.getRotation().x() << " " << transform.getRotation().y() << " " << transform.getRotation().z() << std::endl; 

				//std::cout << "RPY: " << roll << ", " << pitch << ", " << yaw << std::endl;
			} else { ROS_WARN("Transform not ready"); }
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
            return;
		}
		ros::Duration(1.0).sleep();	

        map_pos.x = trans_p.pose.position.x + _offset[current_Id.data].x;
        map_pos.y = trans_p.pose.position.y + _offset[current_Id.data].y;
        map_pos.z = trans_p.pose.position.z + _offset[current_Id.data].z;
        //map_pos.x = markers.markers[0].pose.pose.position.x + transform.getOrigin().x();
        //map_pos.y = markers.markers[0].pose.pose.position.y + transform.getOrigin().y();
        //map_pos.z = markers.markers[0].pose.pose.position.z + transform.getOrigin().z();

        set_pos(map_pos,current_Id.data);
        _located[current_Id.data] = true;

        
        //param_name.data = std::to_string(current_Id.data);

        //_nh.setParam("/global_ID", current_Id.data);

        _nh.setParam("/room_" + std::to_string(current_Id.data) + "_x", map_pos.x);
        _nh.setParam("/room_" + std::to_string(current_Id.data) + "_y", map_pos.y);        

        ROS_INFO("ID received: [%i]", current_Id.data);
        ROS_INFO("Pos received: [%f, %f, %f]", p.pose.position.x, p.pose.position.y, p.pose.position.z);
        ROS_INFO("Pos transformed: [%f, %f, %f]", trans_p.pose.position.x, trans_p.pose.position.y, trans_p.pose.position.z);
        ROS_INFO("Pos saved: [%f, %f, %f]", map_pos.x, map_pos.y, map_pos.z);
    }
    }


}

int main(int argc, char **argv){
    
    ros::init(argc,argv,"ar_marker_id");
    ROS_SUB rs;

    ros::spin();
    return 0;
}