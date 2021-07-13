#include "navigation.h"

using namespace std;


#define LIN_VEL 0.2
#define ANG_VEL 0.5


NAVIGATION::NAVIGATION() {
    _odom_sub  = _nh.subscribe("/odom", 1, &NAVIGATION::odometry_cb, this);
    _lidar_sub = _nh.subscribe("/poppo_scan", 1, &NAVIGATION::laser_cb, this);
    _planner_sub = _nh.subscribe("/path", 1, &NAVIGATION::plan_cb, this);

    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    _test_pub = _nh.advertise<nav_msgs::Path>("/path", 1);
    _result_pub = _nh.advertise<std_msgs::Int32>("/controller_result", 1);

    // _service = nh.advertiseService("service", service_callback);

    if (_nh.hasParam("k1"))
    {
        _nh.getParam("k1",k[0]);
    }
    else{
        k[0] = 0.2;
    }

    if (_nh.hasParam("k2"))
    {
        _nh.getParam("k2",k[1]);
    }
    else{
        k[1] = 0.2;
    }

    if (_nh.hasParam("b"))
    {
        _nh.getParam("b",b);
    }
    else{
        b = 0.1;
    }

    _path_received = false;
    _first_odom = false;
    _path_obstacle = false;
    _human_mode = false;
    _fv = _rv = 0.0;
    _record_wp = false;

    _finish = false;

    cout << "k1/k2/b : " << k[0] << "/" << k[1] << "/" << b << "\n";
}


void NAVIGATION::odometry_cb( nav_msgs::Odometry odom ) {

    _curr_p.x = odom.pose.pose.position.x;
    _curr_p.y = odom.pose.pose.position.y;
    _curr_p.z = 0.0;

    //translate quaternion to euler angles
    tf::Quaternion q( odom.pose.pose.orientation.x, odom.pose.pose.orientation.y , odom.pose.pose.orientation.z ,  odom.pose.pose.orientation.w );
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _curr_yaw);

    //_curr_yaw = -_curr_yaw;

    _B_p.x = _curr_p.x + b*cos(_curr_yaw);
    _B_p.y = _curr_p.y + b*sin(_curr_yaw);
    _B_p.z = 0.0;


    //cout << "Position: " << _curr_p.x << ", " << _curr_p.y << endl;
    //cout << "Orientation: " << _curr_yaw << endl;
    _first_odom = true;
    
}

void NAVIGATION::laser_cb( std_msgs::Int32 laser ) {

    geometry_msgs::PoseStamped p;

    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;

    path.poses.push_back( p );

    // p.pose.position.x = 2.0;
    // p.pose.position.y = 0.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    p.pose.position.x = 2.0;
    p.pose.position.y = 2.0;
    p.pose.position.z = 0.0;

    path.poses.push_back( p );

    p.pose.position.x = 0.0;
    p.pose.position.y = 2.0;
    p.pose.position.z = 0.0;

    path.poses.push_back( p );
    
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;

    path.poses.push_back( p );

    cout << "\nOKOK\n";

    for (int i = 0; i< path.poses.size();i++){
            cout << "\nPOINT [" << i << "] X:" << path.poses[i].pose.position.x;
    }

    _test_pub.publish(path);
    // int start_ind = int( ((90 - 20) / 180.0*M_PI)  / laser.angle_increment );  //
    // int end_ind =   int( ((90 + 20) / 180.0*M_PI)  / laser.angle_increment );//

    // bool found_obstacle = false;
    // int i = start_ind;
    // while ( !found_obstacle &&  i++ < end_ind  ) {
    //     if(  laser.ranges[i] < 0.8 ) found_obstacle = true;
    // }

    // _path_obstacle = found_obstacle;
    

}


//For each loop it calculates the desired velocity to move the robot
bool NAVIGATION::navigation( float x_r, float y_r, float x_r_d, float y_r_d ) {

    geometry_msgs::Twist cmd;
    

    float u[2];

    u[0] = x_r_d + k[0]*(x_r-_B_p.x);
    u[1] = y_r_d + k[1]*(y_r-_B_p.y);

    cmd.linear.x = cos(_curr_yaw)*u[0] + sin(_curr_yaw)*u[1];
    cmd.angular.z = (-sin(_curr_yaw)*u[0]/b + cos(_curr_yaw)*u[1]/b);
    

    double pos_e = sqrt ( pow( x_r - _B_p.x, 2) +  pow(y_r - _B_p.y, 2) ); //norm of the motion vector

    cout << "\nerror:"<< pos_e << "\n";

    if( pos_e < 0.2 ) {
    
        cout << "\ne < 0.2!\n";
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        _cmd_vel_pub.publish( cmd );

        return true;
    }else{

    _cmd_vel_pub.publish( cmd );
    return false;
    }
    
 }


void NAVIGATION::human_input() {

	string input;

	cout << "Keyboard Input: " << endl;
	cout << "[w]: Forward direction velocity" << endl;
	cout << "[x]: Backward direction velocity" << endl;
	cout << "[a]: Left angular velocity" << endl;
	cout << "[d]: Right angular velocity" << endl;
    cout << "[f]: Terminate human mode" << endl;
    cout << "[r]: Record the waypoint" << endl;
	cout << "[s]: stop the robot!" << endl;

	while (ros::ok() && _human_mode) {

		getline( cin, input);

		if( input == "w" ) 
			_fv = (_fv < 0.0 ) ? 0.0 : LIN_VEL;
		else if( input == "x" ) 
			_fv = (_fv > 0.0 ) ? 0.0 : -LIN_VEL;
		else if( input == "a" ) 
			_rv = (_rv > 0.0 ) ? 0.0 : -ANG_VEL;
		else if( input == "d" )
			_rv = (_rv < 0.0 ) ? 0.0 : ANG_VEL;
		else if( input == "s" ) 
			_fv = _rv = 0.0;

        else if( input == "f") 
            _human_mode = false;

        else if( input == "r" ) 
            _record_wp = true;
	}
}


void NAVIGATION::train_traj () {



    ros::Rate r(10);
    
    //_wp_index ++; //skip the previous waypoint 
    _wp_list.erase( _wp_list.begin() + _wp_index ); //I found an obstacle reachiing this wp

    geometry_msgs::Twist cmd;
    _human_mode = true;

    boost::thread human_input_t( &NAVIGATION::human_input, this);

    while ( _human_mode ) {
        if( _record_wp ) { 
            cout << "Record new waypoint" << endl;
            geometry_msgs::Point p;
            p = _curr_p;

            std::vector< geometry_msgs::Point >::iterator it;
            it = _wp_list.begin() + _wp_index;
            _wp_list.insert( it, p );
            
            _record_wp = false;
        }

        cmd.linear.x = _fv;
        cmd.angular.z = _rv;
        _cmd_vel_pub.publish( cmd );

    }




}

void NAVIGATION::plan_cb( nav_msgs::Path geom_path ){
    cout << "\nRECEIVED\n";
    _finish = false;
    
    _wp_list.clear();
    _wp_vel_list.clear();

    float dx = 0;
    float dy = 0;
    float distance = 0;

    geometry_msgs::Vector3 v;
    std::cout << "\nInitial size of PATH :["<< _wp_list.size()<< "] VEL :["<< _wp_vel_list.size()<<"]\n";

    for(int i = 0; i<geom_path.poses.size();i++){

        _wp_list.push_back( geom_path.poses[i].pose.position);

        // cout << "\nLISTA ["<<i<<"] x : "<< _wp_list[i].x <<" ("<<geom_path.poses[i].pose.position.x << ")\n";
        // cout << "\nLISTA ["<<i<<"] y : "<< _wp_list[i].y <<" ("<<geom_path.poses[i].pose.position.y << ")\n";
    }
    
    for(int i= 0; i< geom_path.poses.size()-1;i++){

        dx = geom_path.poses[i+1].pose.position.x - geom_path.poses[i].pose.position.x;
        dy = geom_path.poses[i+1].pose.position.y - geom_path.poses[i].pose.position.y;
        distance = sqrt ( pow( dx, 2) +  pow(dy, 2) ); //norm of the motion vector

        // std::cout << "\n dx : "<<dx << " dy :" << dy << "distance : " << distance << "\n";

        v.x = 0.1*dx/distance;
        v.y = 0.1*dy/distance;
        // v.x = dx;
        // v.y = dy;
        // cout << "\n v.x : "<<v.x << " v.y :" << v.y << "\n";
        _wp_vel_list.push_back(v);
        // std::cout << "\nVEL ["<<i<< "] x : "<< _wp_vel_list[i].x <<"\n";
        // std::cout << "\nVEL ["<<i<< "] y : "<< _wp_vel_list[i].y <<"\n";
        
    }


    for (int i = 0; i< _wp_vel_list.size(); i++){
        std::cout << "\nVEL ["<<i<< "] x : "<< _wp_vel_list[i].x <<"\n";
        std::cout << "\nVEL ["<<i<< "] y : "<< _wp_vel_list[i].y <<"\n";
    }
    std::cout << "\nsize of PATH :["<< _wp_list.size()<< "] VEL :["<< _wp_vel_list.size()<<"]\n";

    _path_received = true;
}


void NAVIGATION::ctrl_loop() {


    while( !_first_odom ) sleep(1);

    while( !_path_received ) sleep(1);


    //control loop
    //obstacle detection
    //Switching on human control

    ros::Rate r(100);

    std_msgs::Int32 res;

    

    // geometry_msgs::Vector3 v;
    // geometry_msgs::Point p;
    // p.x = 2.0;
    // p.y = 0.0;
    // p.z = 0.0;

    // v.x = 0.1;
    // v.y = 0.0;
    // v.z = 0.0;
    // _wp_list.push_back( p );
    // _wp_vel_list.push_back(v);
    // p.x = 2.0;
    // p.y = 2.0;
    // p.z = 0.0;

    // v.x = 0.0;
    // v.y = 0.1;
    // v.z = 0.0;

    // _wp_list.push_back( p );
    // _wp_vel_list.push_back(v);
    // p.x = 0.0;
    // p.y = 2.0;
    // p.z = 0.0;

    // v.x = -0.1;
    // v.y = 0.0;
    // v.z = 0.0;
    // _wp_list.push_back( p );
    // _wp_vel_list.push_back(v);
    // p.x = 0.0;
    // p.y = 0.0;
    // p.z = 0.0;

    // v.x = 0.0;
    // v.y = -0.1;
    // v.z = 0.0;

    // _wp_list.push_back( p );
    // _wp_vel_list.push_back(v);

    _wp_index = 0;

    geometry_msgs::Twist cmd;
    std::cout << "\nPATH RECEIVED! _wp_index : "<< _wp_index<< "_wp_list.size : "<<_wp_list.size()<<"\n";
    std::cout << "finished : " << _finish << "\n";
    while ( ros::ok() ) {
        std::cout <<"\n ..>.<";

        if((_wp_list.size() < 1) && !_finish){
            std::cout << "\nPATH NOT FOUND\n";

            _finish = true;

            res.data = 1;
            _result_pub.publish (res);
        }

        while( _wp_index < _wp_list.size() && !_finish ) {
            cout << "\nINDEX"<<_wp_index;
            while ( !_path_obstacle && !navigation( _wp_list[_wp_index+1].x,  _wp_list[_wp_index+1].y,_wp_vel_list[_wp_index].x,  _wp_vel_list[_wp_index].y ) ) {
                // cout << "\ncmd : "<<cmd.linear.x;
                r.sleep();  

                // if(_reached){
                //     _reached = false;
                // }
            }

            if(navigation( _wp_list[_wp_list.size()-1].x,  _wp_list[_wp_list.size()-1].y,_wp_vel_list[_wp_list.size()-1].x,  _wp_vel_list[_wp_list.size()-1].y )){
                ROS_WARN("\n\n Goal Reached!\n");
                _finish = true;
                res.data = 0;
                _result_pub.publish (res);
            }

            

            if( _path_obstacle ) {
                ROS_WARN("Found obstacle on the path");
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                _cmd_vel_pub.publish( cmd );

                //Start new function to add waypoint to the list of wps
                train_traj();

            }
            else {
                _wp_index ++;
            }
                

                
        }

        // if(!_reached){
            
        //     _reached = true;

        // }
        std::cout <<"\n end loop";
        _wp_index = 0;
        r.sleep();
    }
    


}

void NAVIGATION::run() {

    
    boost::thread ctrl_loop_t( &NAVIGATION::ctrl_loop, this );
    ros::spin();
    //---
}



int main( int argc, char** argv ) {

    ros::init( argc, argv, "mobile_navigation_node");

    NAVIGATION nav;
    nav.run();


    return 0;
}