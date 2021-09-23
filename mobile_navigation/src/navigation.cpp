#include "navigation/navigation.h"

using namespace std;


#define LIN_VEL 0.1
#define ANG_VEL 0.1

double t;

NAVIGATION::NAVIGATION() {
    // _odom_sub  = _nh.subscribe("/odom", 1, &NAVIGATION::odometry_cb, this); //receive odometry (dead reckoning)
     _odom_sub  = _nh.subscribe("/amcl_pose", 1, &NAVIGATION::odometry_cb, this); //receive odometry (from AMCL)
    _planner_sub = _nh.subscribe("/path", 1, &NAVIGATION::plan_cb, this);   //receive the path from the planner

    _lidar_sub = _nh.subscribe("/scan", 1, &NAVIGATION::laser_cb, this);


    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);      
    _result_pub = _nh.advertise<std_msgs::Int32>("/controller_result", 1);  //provide result of the panning/motion to the task


    //publisher and subscriber to send a path without the planner
    _point_pose_pub = _nh.advertise<nav_msgs::Odometry>("/point_odom", 1);
    _test_pub = _nh.advertise<nav_msgs::Path>("/path", 1);
    _fake_path_sub = _nh.subscribe("/fake_path", 1, &NAVIGATION::fake_path_cb, this);

    //Graph publishers
    errors_pub = _nh.advertise<geometry_msgs::Pose2D>("/errors_pub",1000);
	desired_traj_pub = _nh.advertise<geometry_msgs::Pose2D>("/desired_traj_pub",1000);

    _marker_pub = _nh.advertise<visualization_msgs::Marker>("/target", 1000);
    _point_b_pub = _nh.advertise<visualization_msgs::Marker>("/point_b", 1000);

    _odom_marker_pub = _nh.advertise<visualization_msgs::Marker>("/odom_marker", 1000);
    

    // _service = nh.advertiseService("service", service_callback);

    if (_nh.hasParam("k1"))
    {
        _nh.getParam("k1",k[0]);
    }
    else{
        k[0] = 0.1;
    }

    if (_nh.hasParam("k2"))
    {
        _nh.getParam("k2",k[1]);
    }
    else{
        k[1] = 0.1;
    }

    if (_nh.hasParam("b"))
    {
        _nh.getParam("b",b);
    }
    else{
        b = 0.3;
    }

    if (_nh.hasParam("lin_sat"))
    {
        _nh.getParam("lin_sat",lin_sat);
    }
    else{
        lin_sat = LIN_VEL;
    }

    if (_nh.hasParam("ang_sat"))
    {
        _nh.getParam("ang_sat",ang_sat);
    }
    else{
        ang_sat = ANG_VEL;
    }

    if (_nh.hasParam("scale"))
    {
        _nh.getParam("scale",scaling_factor);
    }
    else{
        scaling_factor = 0.05;
    }

    if (_nh.hasParam("laser_th"))
    {
        _nh.getParam("laser_th",_laser_threshold);
    }
    else{
        _laser_threshold = 0.15;
    }

    _path_received = false;     
    _first_odom = false;        

    _path_obstacle_left = false;     
    _path_obstacle_right = false;

    _finish = false;

    _impact_index = -1;

    //print the used parameters
    cout << "k1/k2/b : " << k[0] << "/" << k[1] << "/" << b << "\n\n";
    cout << "MAX_LIN/MAX_ANG : " << lin_sat << "/" << ang_sat <<"\n\n";
}


void NAVIGATION::laser_cb( sensor_msgs::LaserScan laser ) {

    // _heading_lecture = laser.ranges[(0 / 180.0*M_PI)  / laser.angle_increment];
    // _left_lecture = laser.ranges[(90 / 180.0*M_PI)  / laser.angle_increment];
    // _right_lecture = laser.ranges[(270 / 180.0*M_PI)  / laser.angle_increment];

    // // cout << "h/l/r : " << _heading_lecture << "/" << _left_lecture << "/" << _right_lecture << "\n\n";

    // // int start_ind_h_l = int( ((0) / 180.0*M_PI)  / laser.angle_increment );  //
    // // int end_ind_h_l = int( ((0 + 20) / 180.0*M_PI)  / laser.angle_increment );  //

    // int start_ind_left =   int( ((90 - 45) / 180.0*M_PI)  / laser.angle_increment );//
    // int end_ind_left =   int( ((90 + 45) / 180.0*M_PI)  / laser.angle_increment );//

    // int start_ind_right =   int( ((270 - 45) / 180.0*M_PI)  / laser.angle_increment );//
    // int end_ind_right =   int( ((270 + 45) / 180.0*M_PI)  / laser.angle_increment );//


    // bool found_obstacle_left = false;
    // bool found_obstacle_right = false;
    
    // // int i = start_ind;

    // // while ( !found_obstacle &&  i++ < end_ind  ) {
    // //     if(  laser.ranges[i] < _laser_threshold ){
    // //         found_obstacle = true;
    // //         _impact_index = i;

    // //     } 
    // // }

    // // _path_obstacle = found_obstacle;

    // int i = start_ind_left;

    // while ( !found_obstacle_left &&  i++ < end_ind_left  ) {
    //     if(  laser.ranges[i] < _laser_threshold ){
    //         found_obstacle_left = true;
    //         _impact_index = i;
    //     } 
    // }

    // i = start_ind_right;

    // while ( !found_obstacle_right &&  i++ < end_ind_right  ) {
    //     if(  laser.ranges[i] < _laser_threshold ){
    //         found_obstacle_right = true;
    //         _impact_index = i;
    //     } 
    // }

    // _path_obstacle_left = found_obstacle_left;
    // _path_obstacle_right = found_obstacle_right;
    

}


// void NAVIGATION::odometry_cb( nav_msgs::Odometry odom ) {
void NAVIGATION::odometry_cb(geometry_msgs::PoseWithCovarianceStamped odom ) {


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


    //point_odom = odom;
    point_odom.pose.pose.position.x = _B_p.x;
    point_odom.pose.pose.position.y = _B_p.y;
    point_odom.pose.pose.position.z = _B_p.z;
    
    _point_pose_pub.publish(point_odom);
}

void NAVIGATION::fake_path_cb( std_msgs::Int32 fake_value ) {

    geometry_msgs::PoseStamped p;

    p.pose.position.x = 0.3;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;

    path.poses.push_back( p );

    // p.pose.position.x = 10.0;
    // p.pose.position.y = 0.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    

    p.pose.position.x = 2.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;

    path.poses.push_back( p );

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

    // p.pose.position.x = 3.0;
    // p.pose.position.y = 1.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    // p.pose.position.x = 4.0;
    // p.pose.position.y = 3.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );
    
    // p.pose.position.x = 4.0;
    // p.pose.position.y = 4.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    // p.pose.position.x = 4.0;
    // p.pose.position.y = 5.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    // p.pose.position.x = 3.0;
    // p.pose.position.y = 7.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    // p.pose.position.x = 1.0;
    // p.pose.position.y = 8.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    // p.pose.position.x = 0.0;
    // p.pose.position.y = 8.0;
    // p.pose.position.z = 0.0;

    // path.poses.push_back( p );

    // cout << "\nOKOK\n";

    // for (int i = 0; i< path.poses.size();i++){
    //         cout << "\nPOINT [" << i << "] X:" << path.poses[i].pose.position.x;
    // }

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

    geometry_msgs::Pose2D qd;
    geometry_msgs::Pose2D err;


    geometry_msgs::Point points;
    geometry_msgs::Point b_point;    
    geometry_msgs::Point odom_point;    


    float u[2];
    float sig = 1;

    float prev_x;
    float prev_y;

    float act_x;
    float act_y;

    float v_x;
    float v_y;

    float temp;

    if(_wp_index == 0){
         prev_x = _wp_list[_wp_index].x;
         prev_y = _wp_list[_wp_index].y;
    }
    else{
          prev_x = _wp_list[_wp_index].x;
          prev_y = _wp_list[_wp_index].y;
        
    }

    float norma = (sqrt(pow(x_r - prev_x,2) + pow(y_r - prev_y,2)));


    v_x = scaling_factor*(x_r - prev_x)/norma;
    v_y = scaling_factor*(y_r - prev_y)/norma;
    
    act_x = prev_x + scaling_factor*t*(x_r - prev_x)/norma;
    act_y = prev_y + scaling_factor*t*(y_r - prev_y)/norma;
    

    qd.x = act_x;
    qd.y = act_y;

    qd.theta = atan2(v_x,v_y);

    desired_traj_pub.publish(qd); // publish the desired trajectory coordinates to the plotter node

    // calculate Tracking errors between the robot and desired trajectory coordinates
        //translate quaternion to euler angles
        tf::Quaternion q( point_odom.pose.pose.orientation.x, point_odom.pose.pose.orientation.y , point_odom.pose.pose.orientation.z ,  point_odom.pose.pose.orientation.w );
        double roll, pitch, point_theta;
        tf::Matrix3x3(q).getRPY(roll, pitch, point_theta);

		err.x = (qd.x-point_odom.pose.pose.position.x) * cos(point_theta) + (qd.y- point_odom.pose.pose.position.y) * sin(point_theta);
		err.y = -(qd.x-point_odom.pose.pose.position.x) * sin(point_theta) + (qd.y-point_odom.pose.pose.position.y) * cos(point_theta);
		err.theta = qd.theta - point_theta;
		err.theta = atan2(sin(err.theta),cos(err.theta));// Normalize theta_e between -pi and oi
		errors_pub.publish(err); // publish errors to the plotter node


    points.x = qd.x;
    points.y = qd.y;
    points.z = 0.0;

    _target_marker.points.clear();
    _target_marker.points.push_back(points);
    _target_marker.header.frame_id = "map";
    _target_marker.header.stamp = ros::Time();
    // _target_marker.id = i;
    _target_marker.type = visualization_msgs::Marker::POINTS;
    _target_marker.action = visualization_msgs::Marker::ADD;
    _target_marker.scale.x = 0.1;
    _target_marker.scale.y = 0.1;
    _target_marker.color.a = 1.0;
    _target_marker.color.r = 0.0;
    _target_marker.color.g = 0.0;
    _target_marker.color.b = 1.0;
    _marker_pub.publish(_target_marker);


    b_point.x = _B_p.x;
    b_point.y = _B_p.y;
    b_point.z = 0.0;

    _point_b_marker.points.clear();
    _point_b_marker.points.push_back(b_point);
    _point_b_marker.header.frame_id = "map";
    _point_b_marker.header.stamp = ros::Time();
    // _target_marker.id = i;
    _point_b_marker.type = visualization_msgs::Marker::POINTS;
    _point_b_marker.action = visualization_msgs::Marker::ADD;
    _point_b_marker.scale.x = 0.05;
    _point_b_marker.scale.y = 0.05;
    _point_b_marker.color.a = 1.0;
    _point_b_marker.color.r = 1.0;
    _point_b_marker.color.g = 0.0;
    _point_b_marker.color.b = 0.0;
    _point_b_pub.publish(_point_b_marker);

    odom_point.x = _curr_p.x;
    odom_point.y = _curr_p.y;
    odom_point.z = 0.0;

    _odom_marker.points.clear();
    _odom_marker.points.push_back(odom_point);
    _odom_marker.header.frame_id = "map";
    _odom_marker.header.stamp = ros::Time();
    // _target_marker.id = i;
    _odom_marker.type = visualization_msgs::Marker::POINTS;
    _odom_marker.action = visualization_msgs::Marker::ADD;
    _odom_marker.scale.x = 0.1;
    _odom_marker.scale.y = 0.1;
    _odom_marker.color.a = 1.0;
    _odom_marker.color.r = 1.0;
    _odom_marker.color.g = 1.0;
    _odom_marker.color.b = 1.0;
    _odom_marker_pub.publish(_odom_marker);

    // cout << "\nNORMA [" << norma << "] pX:" << prev_x<< "] pY:" << prev_y;
    // cout << "\nAX [" << act_x << "] AY:" << act_y;
    // cout << "\nX [" << x_r << "] Y:" << y_r;
    // cout << "\nVX [" << v_x << "] VY:" << v_y;
    // cout << "\nt [" << t;

    u[0] = v_x + k[0]*(act_x-_B_p.x);
    u[1] = v_y + k[1]*(act_y-_B_p.y);

    cmd.linear.x = cos(_curr_yaw)*u[0] + sin(_curr_yaw)*u[1];
    cmd.angular.z = (-sin(_curr_yaw)*u[0]/b + cos(_curr_yaw)*u[1]/b);

    if (fabs(cmd.linear.x)> lin_sat){

        if(fabs(cmd.angular.z)> ang_sat){
            sig = fabs(cmd.angular.z)/ang_sat;

            cmd.angular.z = ang_sat*cmd.angular.z/fabs(cmd.angular.z);
            cmd.linear.x = cmd.linear.x/sig;

        }else{
            sig = fabs(cmd.linear.x)/lin_sat;

            cmd.linear.x = lin_sat*cmd.linear.x/fabs(cmd.linear.x);
            cmd.angular.z = cmd.angular.z/sig;
        }

    }
    
    // if (cmd.angular.z >= 2.0){
    //     cout << "\npd:" << "\n";
    //     cmd.angular.z = 2.0;
    // }
    // if (cmd.angular.z <= -2.0){
    //     cout << "\npdpd:" << "\n";
    //     cmd.angular.z = -2.0;
    // }
    // if (cmd.linear.x >= 0.2){
    //     cout << "\nld:" << "\n";
    //     cmd.linear.x = 0.2;
    // }
    // if (cmd.linear.x <= -0.2){
    //     cout << "\nld:" << "\n";
    //     cmd.linear.x = -0.2;
    // }

    double pos_e = sqrt ( pow( x_r - _B_p.x, 2) +  pow(y_r - _B_p.y, 2) ); //norm of the motion vector

    //cout << "\nerror:"<< pos_e << "\n";

    t = t+0.01;


    // if( pos_e < 0.1 ) {
    if( (t >= norma/scaling_factor) || (pos_e < 0.1)) {

        if(_wp_index == _wp_list.size()-2){
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            _cmd_vel_pub.publish( cmd );

        }
        //cout << "\ne < 0.1!\n";
        

        t=0.0;

        return true;
    }else{

    _cmd_vel_pub.publish( cmd );
    return false;
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

    std::cout << "\nsize of PATH :["<< _wp_list.size()<< "] VEL :["<< _wp_vel_list.size()<<"]\n";

    _path_received = true;
}


void NAVIGATION::ctrl_loop() {


    while( !_first_odom ) sleep(1);

    while( !_path_received ) sleep(1);


    ros::Rate r(100);

    std_msgs::Int32 res;

    _wp_index = 0;

    geometry_msgs::Twist cmd;
    std::cout << "\nPATH RECEIVED! _wp_index : "<< _wp_index<< "_wp_list.size : "<<_wp_list.size()<<"\n";
    std::cout << "finished : " << _finish << "\n";

    t = 0.0;

    while ( ros::ok() ) {

        if((_wp_list.size() <= 1) && !_finish){
            std::cout << "\nPATH NOT FOUND\n";

            _finish = true;

            res.data = 1;
            _result_pub.publish (res);
        }

        while( _wp_index < _wp_list.size() && !_finish ) {
            cout << "\nINDEX"<<_wp_index;
            while ( !_path_obstacle_left && !_path_obstacle_right && !navigation( _wp_list[_wp_index+1].x,  _wp_list[_wp_index+1].y,_wp_vel_list[_wp_index].x,  _wp_vel_list[_wp_index].y ) ) {
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

            

            if( _path_obstacle_left || _path_obstacle_right) {
                ROS_WARN("Found obstacle on the path");
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                _cmd_vel_pub.publish( cmd );

                if(_path_obstacle_left){
                    while ((_heading_lecture <= _laser_threshold + 1.0)&& (_left_lecture <= _laser_threshold + 0.5)){

                        cmd.linear.x = 0.0;
                        cmd.angular.z = -0.1;
                        _cmd_vel_pub.publish( cmd );

                    }
                }
                else if(_path_obstacle_right){
                        
                    while ((_heading_lecture <= _laser_threshold + 1.0)&& (_right_lecture <= _laser_threshold + 0.5)){

                        cmd.linear.x = 0.0;
                        cmd.angular.z = 0.1;
                        _cmd_vel_pub.publish( cmd );

                    }

                }

                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                _cmd_vel_pub.publish( cmd );
                ROS_WARN("AVOIDED");

                _finish = true;
                res.data = 2;
                _result_pub.publish (res);
                //Start new function to plan again with updated map

            }
            else {
                _wp_index ++;
            }
                

                
        }

        // if(!_reached){
            
        //     _reached = true;

        // }
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