#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> 
#include<string>


int main( int argc, char** argv) {

	ros::init(argc, argv, "move_base_client");
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  	ac.waitForServer(); //will wait for infinite time

  	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";

	float_t x;
	float_t y;
    if (nh.getParam("room_2_x", x))
    {
      ROS_INFO("Got param: %s", std::to_string(x));
    }
    else
    {
      ROS_ERROR("Failed to get param 'my_param'");
    }

    if (nh.getParam("room_2_y", y))
    {
      ROS_INFO("Got param: %s", std::to_string(y));
    }
    else
    {
      ROS_ERROR("Failed to get param 'my_param'");
    }



	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;

	goal.target_pose.pose.orientation.w = 3.14;

	ac.sendGoal(goal);
	bool done = false;
	ros::Rate r(10);
	while ( !done ) {
		if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
				  ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED ) {
			done = true;
		}
		r.sleep();
	}
}
