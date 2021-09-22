#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "sensor_msgs/Imu.h"


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include <tf/transform_datatypes.h>
using namespace std;
	
namespace gazebo
{
  class ControllerPlugin : public ModelPlugin
  {

	//---ROS
	private: ros::NodeHandle* _node_handle;
	private: ros::Publisher _w_v_pub;				//info publisher (Not used)
	private: ros::Publisher _odom_pub;

	private: tf::TransformBroadcaster odom_broadcaster;

	        
	private: ros::Subscriber _cmd_vel_sub;
	private: ros::Subscriber _imu_sub;				//Adjust odometry with Imu measurements
	private: ros::Subscriber _vision_sub;			//Add to adjust odometry with respect to AR Marker


	private: sensor_msgs::Imu _imu_read;			//lecture from Imu

	private: std_msgs::Float32MultiArray _w_vel;	//info publisher data

	private: nav_msgs::Odometry _odom;

	//Plugin parameters
	private: std::string odom_frame;
	private: std::string base_frame;
	private: double wheel_separation;
	private: double wheel_diam;
	private: double wheel_torque;
	private: double velocity; 			//test
	private: bool use_imu;

	private: double x = 0.0;
	private: double y = 0.0;
	private: double th = 0.0;

	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;


	//---Gazebo
	private: physics::ModelPtr _model;
	private: physics::JointPtr _front_left_wheel_joint;
	private: physics::JointPtr _front_right_wheel_joint;

	private: physics::LinkPtr _base_link;

 
	private: event::ConnectionPtr updateConnection;

	ros::Time current_time, last_time;
	

	public: void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_) {	

		//set parameters default value before taking them from parameter server
		std::string cmd_topic= "/cmd_vel";
		std::string odom_topic= "/odom";
		std::string imu_topic = "/imu";
		std::string vision_topic = "/vision_odom";

		odom_frame="/odom";
		base_frame="/base_footprint";

		std::string l_wheel_joint="wheel_left_joint";
		std::string r_wheel_joint="wheel_right_joint";

		wheel_separation = 0.287;
		wheel_diam = 0.066;

		wheel_torque = 10;
		velocity = 0.0;
		use_imu = true;

		// Check that the velocity element exists, then read the value
		if (sdf_->HasElement("velocity"))
		velocity = sdf_->Get<double>("velocity");

		if (sdf_->HasElement("startX"))
		x = sdf_->Get<double>("startX");

		if (sdf_->HasElement("startY"))
		y = sdf_->Get<double>("startY");

		if (sdf_->HasElement("startA"))
		th = sdf_->Get<double>("startA");

		if (sdf_->HasElement("useImu"))
		use_imu = sdf_->Get<int>("useImu");

		if (sdf_->HasElement("odometryTopic"))
		odom_topic = sdf_->Get<string>("odometryTopic");

		if (sdf_->HasElement("commandTopic"))
		cmd_topic = sdf_->Get<string>("commandTopic");

		if (sdf_->HasElement("imuTopic"))
		imu_topic = sdf_->Get<string>("imuTopic");

		if (sdf_->HasElement("visionTopic"))
		vision_topic = sdf_->Get<string>("visionTopic");

		if (sdf_->HasElement("odometryFrame"))
		odom_frame = sdf_->Get<string>("odometryFrame");
		if (sdf_->HasElement("robotBaseFrame"))
		base_frame = sdf_->Get<string>("robotBaseFrame");

		if (sdf_->HasElement("leftJoint"))
		l_wheel_joint = sdf_->Get<string>("leftJoint");
		if (sdf_->HasElement("rightJoint"))
		r_wheel_joint = sdf_->Get<string>("rightJoint");

		if (sdf_->HasElement("wheelSeparation"))
		wheel_separation = sdf_->Get<double>("wheelSeparation");
		if (sdf_->HasElement("wheelDiameter"))
		wheel_diam = sdf_->Get<double>("wheelDiameter");
		if (sdf_->HasElement("wheelTorque"))
		wheel_torque = sdf_->Get<double>("wheelTorque");

		current_time = ros::Time::now();
		last_time = ros::Time::now();
	
		_node_handle = new ros::NodeHandle();	
		_model = parent_;

		_front_left_wheel_joint = this->_model->GetJoint(l_wheel_joint);
		_front_right_wheel_joint = this->_model->GetJoint(r_wheel_joint);

		_base_link = this->_model->GetLink(base_frame);

		ignition::math::Pose3d pose = this->_base_link->WorldPose();
 		ignition::math::Vector3<double> position = pose.Pos();

		x = position.X();

		y = position.Y();

		th = pose.Yaw();


		// _front_left_wheel_joint = this->_model->GetJoint("wheel_left_joint");
		// _front_right_wheel_joint = this->_model->GetJoint("wheel_right_joint");
		_w_v_pub = _node_handle->advertise< std_msgs::Float32MultiArray >("/diff_wheels/vel", 1);

		_odom_pub = _node_handle->advertise< nav_msgs::Odometry >(odom_topic, 0);

		this->_cmd_vel_sub = _node_handle->subscribe(cmd_topic, 0, &ControllerPlugin::cmd_cb,this);

		this->_imu_sub = _node_handle->subscribe(imu_topic, 0, &ControllerPlugin::imu_cb,this);

		this->_vision_sub = _node_handle->subscribe(vision_topic, 0, &ControllerPlugin::vision_cb,this);

		// _odom_pub = _node_handle->advertise< nav_msgs::Odometry >("/odom", 0);

		// this->_cmd_vel_sub = _node_handle->subscribe("/cmd_vel", 0, &ControllerPlugin::cmd_cb,this);
		_w_vel.data.resize(8);

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ControllerPlugin::OnUpdate, this));
    
	}

	public: void imu_cb(sensor_msgs::Imu imu_data){
	
		_imu_read = imu_data;

	}

	public: void vision_cb(geometry_msgs::Pose marker_pose){
	
		x = marker_pose.position.x;
		y = marker_pose.position.y;

		tf::Quaternion q(marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		th = yaw;

	}

	public: void cmd_cb(geometry_msgs::Twist cmd){

		// _front_left_wheel_joint->SetParam("fmax", 0, 10000.0);
		// _front_left_wheel_joint->SetParam("vel", 0, (2*cmd.linear.x - 0.15*cmd.angular.z)/(2*0.039));

		// _front_right_wheel_joint->SetParam("fmax", 0, 10000.0);
		// _front_right_wheel_joint->SetParam("vel", 0, (2*cmd.linear.x + 0.15*cmd.angular.z)/(2*0.039));
		_front_left_wheel_joint->SetParam("fmax", 0, wheel_torque);
		_front_left_wheel_joint->SetParam("vel", 0, (2*cmd.linear.x - wheel_separation*cmd.angular.z)/wheel_diam);

		_front_right_wheel_joint->SetParam("fmax", 0, wheel_torque);
		_front_right_wheel_joint->SetParam("vel", 0, (2*cmd.linear.x + wheel_separation*cmd.angular.z)/wheel_diam);


		// _front_right_wheel_joint->SetVelocity(0,0);
		
		//_w_v_pub.publish( _w_vel );
	}
	

    // Called by the world update start event
    public: void OnUpdate()  {
			current_time = ros::Time::now();

			if((current_time - last_time).toSec()>= 0.001){


			
			
			vx = 0.5*wheel_diam*( _front_right_wheel_joint->GetVelocity(0) + _front_left_wheel_joint->GetVelocity(0) ) /2;
			vy = 0; // we have a non-holonomic constraint (for a holonomic robot, this is non-zero)
			vth = 0.5*wheel_diam*( _front_right_wheel_joint->GetVelocity(0) - _front_left_wheel_joint->GetVelocity(0) ) / (wheel_separation);

			//compute odometry in a typical way given the velocities of the robot
			double dt = (current_time - last_time).toSec();
			double delta_x = (vx * cos(th+ ( _front_right_wheel_joint->GetVelocity(0) - _front_left_wheel_joint->GetVelocity(0) ) / (2* wheel_separation)) - vy * sin(th)) * dt; //
			double delta_y = (vx * sin(th+ ( _front_right_wheel_joint->GetVelocity(0) - _front_left_wheel_joint->GetVelocity(0) ) / (2* wheel_separation)) + vy * cos(th)) * dt;
			double delta_th = vth * dt;

			// double delta_x = (vx * cos(th + ((vth*dt)/2)) - vy * sin(th)) * dt; //
			// double delta_y = (vx * sin(th+ ((vth*dt)/2)) + vy * cos(th)) * dt;
			// double delta_th = vth * dt;

			if(vth == 0){	
				x += delta_x;
				y += delta_y;
				th += delta_th;
			}
			else{
				double thk = th;

				th += vth*dt;
				x += vx*(sin(th) - sin(thk))/vth;
				y += -vx*(cos(th) - cos(thk))/vth;
			}
			// x += vx*dt*cos(th + (vth*dt/2));
			// y += vx*dt*sin(th + (vth*dt/2));
			// th += vth*dt;

			

			//since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = odom_frame;
			odom_trans.child_frame_id = base_frame;

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			// odom_trans.transform.rotation = odom_quat;

			if(use_imu){
				odom_trans.transform.rotation.x = _imu_read.orientation.x;
				odom_trans.transform.rotation.y = _imu_read.orientation.y;
				odom_trans.transform.rotation.z = _imu_read.orientation.z;
				odom_trans.transform.rotation.w = _imu_read.orientation.w;

			}else{
				odom_trans.transform.rotation = odom_quat;
			}

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			// odom.header.frame_id = "/odom";
			odom.header.frame_id = odom_frame;

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			// odom.pose.pose.orientation = odom_quat;

			if(use_imu){
				odom.pose.pose.orientation.x = _imu_read.orientation.x;
				odom.pose.pose.orientation.y = _imu_read.orientation.y;
				odom.pose.pose.orientation.z = _imu_read.orientation.z;
				odom.pose.pose.orientation.w = _imu_read.orientation.w;
			}else{
				odom.pose.pose.orientation = odom_quat;
			}
			

			//set the velocity
			// odom.child_frame_id = "/base_link";
			odom.child_frame_id = base_frame;
			odom.twist.twist.linear.x = vx/dt;
			odom.twist.twist.linear.y = vy/dt;
			//odom.twist.twist.angular.z = vth/dt;

			if(use_imu){
				odom.twist.twist.angular.x = _imu_read.angular_velocity.x;
				odom.twist.twist.angular.y =_imu_read.angular_velocity.y;
				odom.twist.twist.angular.z = _imu_read.angular_velocity.z;
			}else{
				odom.twist.twist.angular.z = vth/dt;
			}

			
			//publish the message
			_odom_pub.publish(odom);

			last_time = current_time;


			// tf::Quaternion q(_imu_read.orientation.x, _imu_read.orientation.y, _imu_read.orientation.z, _imu_read.orientation.w);
    
			// tf::Matrix3x3 m(q);
			// double roll, pitch, yaw;
			// m.getRPY(roll, pitch, yaw);

			// tf::Quaternion qq(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    
			// tf::Matrix3x3 mm(qq);
			// double rroll, ppitch, yyaw;
			// mm.getRPY(rroll, ppitch, yyaw);
			

			// _w_vel.data[0] = _imu_read.orientation.x;
			// _w_vel.data[1] = _imu_read.orientation.y;
			// _w_vel.data[2] = _imu_read.orientation.z;
			// _w_vel.data[3] = _imu_read.orientation.w;
			// _w_vel.data[4] = odom.pose.pose.orientation.x;
			// _w_vel.data[5] = odom.pose.pose.orientation.y;
			// _w_vel.data[6] = odom.pose.pose.orientation.z;
			// _w_vel.data[7] = odom.pose.pose.orientation.w;
			// _w_v_pub.publish( _w_vel );
			
			}
		}
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
}



