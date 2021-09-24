#include "rrt/rrt.h"

// define parameters here
#define STEER_LENGTH 0.50
#define TERMINATE_LENGTH 0.70
#define LOOKAHEAD_DISTANCE 0.80
#define MAX_ITERATION 100000
#define INFLATION_RADIUS 5


std::vector<unsigned int> convert_frame(double x_global, double y_global, double x_off = -10.00, double y_off = -10.00) {
    double x_grid = x_global + x_off;
    double y_grid = y_global + y_off;
    unsigned int x_grid_int = (unsigned int)std::round(x_grid / 0.05);
    unsigned int y_grid_int = (unsigned int)std::round(y_grid / 0.05);
    // if (x_grid_int > 100000) {
    //     x_grid_int = 0;
    // }
    // if (y_grid_int > 100000) {
    //     y_grid_int = 0;
    // }
    return {x_grid_int, y_grid_int};
}

// Destructor of the RRT class
RRT::~RRT() {
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh, RRT_type rrt_Type) : nh_(nh), gen((std::random_device())()), rrt_type(rrt_Type) {

    // Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    // nh_.getParam("pose_topic", pose_topic);
    // nh_.getParam("scan_topic", scan_topic);
    
    std::string pose_topic, scan_topic;
    pose_topic = "/odom";
    scan_topic = "/scan";

    // ROS publishers
    // create publishers for the the drive topic, and other topics you might need
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 0);

    drive_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    mapvisual_pub_ = nh_.advertise<visualization_msgs::Marker>("/env_viz", 1000);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("/dynamic_viz", 1000);
    waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("/static_viz", 1000);
    edges_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_lines", 1000);

    obstacles_pub_ = nh_.advertise<visualization_msgs::Marker>("/obstacles", 0);

    // ROS subscribers
    // create subscribers as you need
    goal_sub_ = nh_.subscribe("/goals", 1, &RRT::goal_callback, this);

    map_sub_ = nh_.subscribe("/map", 1, &RRT::map_callback, this);
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::odom_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    // create an occupancy grid
    occupancy_grids_prior = std::vector<std::vector<bool>>(384, std::vector<bool>(384, true));
    
    occupancy_grids = occupancy_grids_prior;

    if (nh_.hasParam("steer_length"))
    {
        nh_.getParam("steer_length",steer_length);
    }
    else{
        steer_length = STEER_LENGTH;
    }

    if (nh_.hasParam("max_iter"))
    {
        nh_.getParam("max_iter",max_iteration);
    }
    else{
        max_iteration = MAX_ITERATION;
    }

    if (nh_.hasParam("inflation_radius"))
    {
        nh_.getParam("inflation_radius",inflation_radius);
    }
    else{
        inflation_radius = INFLATION_RADIUS;
    }

    ROS_INFO("Created new RRT Object.\n");
}

void RRT::scan_callback(const sensor_msgs::LaserScan &scan_msg) {
    // The scan callback, update your occupancy grid here
    // each point scanned results in a square of 0.6m * 0.6m blocked around it
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // reset the occupancy grids
    // occupancy_grids = occupancy_grids_prior;
    // // update occupancy grid
    // double rear_to_lidar = 0.29275;
    // double x_lidar = x_current + rear_to_lidar * std::cos(heading_current);
    // double y_lidar = y_current + rear_to_lidar * std::sin(heading_current);
    // for (unsigned int i = 0; i < scan_msg.ranges.size(); i++) {
    //     if (!std::isinf(scan_msg.ranges[i]) && !std::isnan(scan_msg.ranges[i])) {
    //         double distance = scan_msg.ranges[i];
    //         double local_angle = scan_msg.angle_min + scan_msg.angle_increment * i;
    //         double global_angle = local_angle + heading_current;
    //         double x_obstacle = x_lidar + distance * std::cos(global_angle);
    //         double y_obstacle = y_lidar + distance * std::sin(global_angle);
    //         std::vector<unsigned int> grid_coordinates = convert_frame(x_obstacle, y_obstacle);
    //         for (unsigned int j = std::max((int)grid_coordinates[0] - 6, 0); j <= std::min((int)grid_coordinates[0] + 6, (int)occupancy_grids.size() - 1); j++) {
    //             for (unsigned int k = std::max((int)grid_coordinates[1] - 6, 0); k <= std::min((int)grid_coordinates[1] + 6, (int)occupancy_grids[0].size() - 1); k++) {
    //                 occupancy_grids[j][k] = false;
    //             }
    //         }
    //     }
    // }
}

void RRT::map_callback(const nav_msgs::OccupancyGrid occ_grid) {
    ROS_INFO("\nMap obtained.");

    c_map_flag = true;
    grid = occ_grid;

    float x;
    float y;
    geometry_msgs::Point p;
    geometry_msgs::Point pp;

    obstacle_marker.points.clear();


    //std::vector<float> vec[2];

    for (unsigned int w = 0; w <= occ_grid.info.width; w++) {  
        for (unsigned int h = 0; h < occ_grid.info.height; h++) {
        
            //occupancy_grids[h][w] = occ_grid.data[h*occ_grid.info.width + w];

            if(occ_grid.data[h*occ_grid.info.width + w]>0){ //|| occ_grid.data[h*occ_grid.info.width + w] == -1){
                
                //Obstacles point
                x = w * occ_grid.info.resolution + occ_grid.info.resolution / 2;
                y = h * occ_grid.info.resolution + occ_grid.info.resolution / 2;

                p.x = x-10;
                p.y = y-10;

                obstacles.push_back(p);

                for (int d = 0; d<(2*inflation_radius);d++){

                    for (int h = 0; h< (2*inflation_radius); h++){

                        pp.x = p.x + (h-inflation_radius)*occ_grid.info.resolution + occ_grid.info.resolution / 2;
                        pp.y = p.y + (d-inflation_radius)*occ_grid.info.resolution + occ_grid.info.resolution / 2;
                        obstacles.push_back(pp);

            
                    }

                }

                //std::cout << "x :" <<x-10 ;
                //std::cout << "\n";
                //std::cout << "y :" <<y-10 ;
                //std::cout << "\n";
            }
        }
    }

    

    //Add inflated Obstacles maybe in the grid directly
    // for (int i=0; i< obstacles.size();i++){
        
    // }


    // for (int i = 0;i< obstacles.size(); i++){
    //     obstacle_marker.points.push_back(obstacles[i]);
    // }
    obstacle_marker.points = obstacles;

    std::cout << "\nOBSTACLES size: "<<obstacle_marker.points.size()<<"\n";

    obstacle_marker.header.frame_id = "map";
    obstacle_marker.header.stamp = ros::Time();
    obstacle_marker.type = visualization_msgs::Marker::POINTS;
    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.scale.x = 0.05;
    obstacle_marker.scale.y = 0.05;
    obstacle_marker.color.a = 1.0;
    obstacle_marker.color.r = 1.0;
    obstacle_marker.color.g = 1.0;
    obstacle_marker.color.b = 0.0;

    obstacle_marker.lifetime = ros::Duration(0);

    obstacles_pub_.publish(obstacle_marker);
    //obstacles = vec;
}

void RRT::goal_callback(geometry_msgs::Pose goal) {

    x_goal = goal.position.x;
    y_goal = goal.position.y;

    x_limit_top = 5;
    x_limit_bot = 0;
    y_limit_left = -2;
    y_limit_right = 3;

    bool one=true;

    while(!(c_pos_flag && c_map_flag)){

        if (one){
            std::cout << "\nwaiting for map and initial position.\n";
            one = false;
        }
        
    }
    std::cout << "\n\n\n-------------GOAL RECEIVED---------------\n";

    // tree as std::vector
    std::vector<Node> tree;

    geometry_msgs::PoseStamped p;           //pose is single element of path(.poses[])

    path.poses.clear();

    // the RRT main loop

    // define the starter node
    Node start;
    Node end;

    start.x = x_current;
    start.y = y_current;
    start.is_root = true;
    tree.push_back(start);

    end.x = x_goal;
    end.y = y_goal;
    end.is_root = false;

//First point of the path is the current position
    // p.pose.position.x = start.x;
    // p.pose.position.y = start.y;
    // path.poses.push_back( p );

    // For drawing the sampled points
    marker.points.clear();

    // points to be added for plotting
    geometry_msgs::Point points;

    // vector to store the final path
    std::vector<Node> paths;
    // std::vector<geometry_msgs::Point> geom_path;        //delete
    
    int ita = 0;

    int fin_index = 0;
    // each loop creates a new sample in the space, generate up to MAX_ITERATION samples due to on-board computation constraints
    for (unsigned int i = 0; i < max_iteration; i++) {
        std::vector<double> sampled_point = sample();               // sample the free space
        unsigned int nearest_point = nearest(tree, sampled_point);  // get the tree's nearest point
        Node new_node = steer(tree[nearest_point], sampled_point);  // steer the tree toward the sampled point, get new point
        //new_node.parent = nearest_point;                            // set the parent of the new point
        if (!check_collision(tree[nearest_point], new_node)) {      // collision checking for connecting the new point to the tree

            tree.push_back(new_node);                                       // add the new point to the tree
            tree[nearest_point].adj_list_ind.push_back(tree.size()-1);      //add the new point to the adjacency list of its nearest point

            // visualize the new point in the rviz
            points.x = new_node.x;
            points.y = new_node.y;
            points.z = 0.0;

            marker.points.clear();
            
            marker.points.push_back(points);

            points.x = tree[nearest_point].x;
            points.y = tree[nearest_point].y;
            points.z = 0.0;

            marker.points.push_back(points);

            
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.id = i;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            mapvisual_pub_.publish(marker);

            sleep(0.1);

            if (is_goal(new_node, x_goal, y_goal) ) {  // check if the goal point is reached
                
                // end.x = new_node.x;
                // end.y = new_node.y;
                // end.is_root = false;

                fin_index = tree.size() - 1;
                paths = find_path_A_star(tree, new_node, fin_index);    // return the generated path
                break;
            }
        }
        
        ita = i;
    }


    //paths = find_path_A_star(tree, end, fin_index);

    std::cout << "\n/////////////////////////////////////////////////iteration:" << ita;

    for (int i = 0;i < paths.size(); i++){
        p.pose.position.x = paths[paths.size() - 1 - i].x;
        p.pose.position.y = paths[paths.size() - 1 - i].y;
        path.poses.push_back( p );

    }


    // find the desired way point
    for (unsigned int i = 0; i < paths.size(); i++) {
        if (std::sqrt(std::pow((paths[paths.size() - 1 - i].x - x_current), 2) + std::pow((paths[paths.size() - 1 - i].y - y_current), 2)) >= LOOKAHEAD_DISTANCE) {
            x_target = paths[paths.size() - 1 - i].x;
            y_target = paths[paths.size() - 1 - i].y;
            // ROS_INFO_STREAM("updated");
            break;
        }
    }
    p.pose.position.x = x_goal;
    p.pose.position.y = y_goal;
    path.poses.push_back( p );

    std::cout << "\nsize of PATH :["<< path.poses.size()<< "] TREE [ :"<< tree.size()<<"] PATHS :["<< paths.size()<<"]\n";
    //DEBUG
    for (int i = 0; i< path.poses.size();i++){
        std::cout << "\nGENERATED PATH, POINT ["<<i << "] X :"<< path.poses[i].pose.position.x<<"\n";
        std::cout << "\nGENERATED PATH, POINT ["<<i << "] Y :"<< path.poses[i].pose.position.y<<"\n";
    }

    path_pub_.publish(path);

////////////////////////////////////////////////////////////////////////////////////////////////
// publish the sampled points to be visualized in rviz
    // marker.points.clear();
    // marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time();
    // marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.scale.x = 0.01;
    // marker.scale.y = 0.01;
    // marker.color.a = 1.0;
    // marker.color.r = 0.0;
    // marker.color.g = 0.0;
    // marker.color.b = 1.0;
    // mapvisual_pub_.publish(marker);

    // publish the global goal point to be visualized in rviz
    marker_2.points.clear();
    points.x = x_goal;
    points.y = y_goal;
    marker_2.points.push_back(points);
    marker_2.header.frame_id = "map";
    marker_2.header.stamp = ros::Time();
    marker_2.type = visualization_msgs::Marker::POINTS;
    marker_2.action = visualization_msgs::Marker::ADD;
    marker_2.scale.x = 0.2;
    marker_2.scale.y = 0.2;
    marker_2.color.a = 1.0;  // Don't forget to set the alpha!
    marker_2.color.r = 1.0;
    marker_2.color.g = 0.0;
    marker_2.color.b = 0.0;
    points_pub_.publish(marker_2);

    // publish the target way point to be visualized in rviz
    marker_3.points.clear();
    points.x = x_target;
    points.y = y_target;
    marker_3.points.push_back(points);
    marker_3.header.frame_id = "map";
    marker_3.header.stamp = ros::Time();
    marker_3.type = visualization_msgs::Marker::POINTS;
    marker_3.action = visualization_msgs::Marker::ADD;
    marker_3.scale.x = 0.2;
    marker_3.scale.y = 0.2;
    marker_3.color.a = 1.0;  // Don't forget to set the alpha!
    marker_3.color.r = 0.0;
    marker_3.color.g = 1.0;
    marker_3.color.b = 0.0;
    waypoint_pub_.publish(marker_3);

    // publish the paths to be visualized in rviz
    marker_4.header.frame_id = "map";
    marker_4.header.stamp = ros::Time();
    marker_4.type = visualization_msgs::Marker::LINE_STRIP;
    marker_4.action = visualization_msgs::Marker::ADD;
    marker_4.scale.x = 0.01;
    marker_4.scale.y = 0.1;
    marker_4.color.a = 1.0;  // Don't forget to set the alpha!
    marker_4.color.r = 0.0;
    marker_4.color.g = 1.0;
    marker_4.color.b = 0.0;
    edges_pub_.publish(marker_4);
    // path found as Path message

}

void RRT::odom_callback(const nav_msgs::Odometry &odometry_info) {
    
    if(!c_pos_flag){
        ROS_INFO("\nCurrent pose obtained.\n");
        c_pos_flag = true;
    }
    
    

    x_current = odometry_info.pose.pose.position.x;
    y_current = odometry_info.pose.pose.position.y;

}

std::vector<double> RRT::sample() {
    
    std::vector<double> sampled_point;
    
    x_dist = std::uniform_real_distribution<>(x_limit_bot, x_limit_top);
    y_dist = std::uniform_real_distribution<>(y_limit_right, y_limit_left);
    
    sampled_point.push_back(x_dist(gen));
    sampled_point.push_back(y_dist(gen));

    return sampled_point;
}

unsigned int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
   
    int nearest_node = 0;
    double min_distance = std::pow((tree[0].x - sampled_point[0]), 2) + std::pow((tree[0].y - sampled_point[1]), 2);
    for (unsigned int ite = 1; ite < tree.size(); ite++) {
        if (std::pow((tree[ite].x - sampled_point[0]), 2) + std::pow((tree[ite].y - sampled_point[1]), 2) < min_distance) {
            min_distance = std::pow((tree[ite].x - sampled_point[0]), 2) + std::pow((tree[ite].y - sampled_point[1]), 2);
            nearest_node = ite;
        }
    }    

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {

    Node new_node;
    double act_distance = std::sqrt(std::pow((nearest_node.x - sampled_point[0]), 2) + std::pow((nearest_node.y - sampled_point[1]), 2));
    new_node.x = nearest_node.x + steer_length / act_distance * (sampled_point[0] - nearest_node.x);
    new_node.y = nearest_node.y + steer_length / act_distance * (sampled_point[1] - nearest_node.y);
        
    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    
    unsigned int grid_x;
    unsigned int grid_y;
    std::vector<unsigned int> grid_p;

    bool collision = false;
    for (unsigned int i = 0; i <= 1000; i++) {
        std::vector<float> coordinate;
        coordinate.push_back(nearest_node.x + i * 0.001 * (new_node.x - nearest_node.x));
        coordinate.push_back(nearest_node.y + i * 0.001 * (new_node.y - nearest_node.y));
        
        // if (occupancy_grids[coordinate[0]][coordinate[1]] == false) {
        //     collision = true;
        // }
        
        grid_x = (unsigned int)((coordinate[0] - grid.info.origin.position.x) / grid.info.resolution);
        grid_y = (unsigned int)((coordinate[1] - grid.info.origin.position.y) / grid.info.resolution);

        grid_p = convert_frame(coordinate[0], coordinate[1]);


        if (grid.data[grid_y*grid.info.width + grid_x] >= 1){
            collision = true;
        }

        for (int d = 0; d<(2*inflation_radius);d++){

            for (int h = 0; h< (2*inflation_radius); h++){

                if (grid.data[((grid_y-inflation_radius+d)*grid.info.width) + (grid_x-inflation_radius+h)] >= 1){
                collision = true;

                }

            
            }

        }
        
        // for (unsigned int j=0; j<obstacles.size();j++){

        //     if((std::fabs(coordinate[0]-obstacles[i].x)<0.2) || (std::fabs(coordinate[1]-obstacles[i].y)<0.2)){
        //         collision = true;
        //     }

        //     // if((coordinate[0]==obstacles[j].x) && (coordinate[1]==obstacles[j].y)){
                
        //     // }

        // }

    }

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    
    bool close_enough = false;
    if (std::sqrt(std::pow((latest_added_node.x - goal_x), 2) + std::pow((latest_added_node.y - goal_y), 2)) <= TERMINATE_LENGTH) {
        close_enough = true;
    }

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<Node> found_path;
    geometry_msgs::Point points;
    marker_4.points.clear();
    points.x = x_goal;
    points.y = y_goal;
    points.z = 0.0;
    marker_4.points.push_back(points);
    points.x = latest_added_node.x;
    points.y = latest_added_node.y;
    points.z = 0.0;
    marker_4.points.push_back(points);
    found_path.push_back(latest_added_node);
    Node next_node = tree[latest_added_node.parent];
    while (!next_node.is_root) {
        found_path.push_back(next_node);
        next_node = tree[next_node.parent];
        points.x = next_node.x;
        points.y = next_node.y;
        points.z = 0.0;
        marker_4.points.push_back(points);
    }
    found_path.push_back(tree[0]);
    points.x = tree[0].x;
    points.y = tree[0].y;
    points.z = 0.0;
    marker_4.points.push_back(points);
    return found_path;
}

std::vector<Node> RRT::find_path_A_star(std::vector<Node> &tree, Node &latest_added_node, int fin_index) {
    
    std::vector<Node> found_path;
    std::vector<int> open_list_indices;

    bool found = false;

    int q_s_index = 0;
    int q_g_index = fin_index;//tree.size()-1;
    int q_best_index = 0;

    float h;

    for(int i = 0; i< tree.size();i++){
        tree[i].cost_g = 0;
        tree[i].cost_f = 0;

        tree[i].parent = 0;
    }

    open_list_indices.push_back(q_s_index);
    // found_path.push_back(tree[q_s_index]);

    std::cout << "\n\nA*\n";

    while((open_list_indices.size()>0)&&(!found)){

        //Find and extract Nbest from open
        //if open is ordered for growing costs, q_best is the last element of the vector
        q_best_index = open_list_indices.back();
        open_list_indices.pop_back();

        std::cout << "\n\nq_best_index\n";

        if(q_best_index == q_g_index){
            found = true;
            std::cout << "\n\nFOUND\n";

        }
        else{

            for(int i=0; i< tree[q_best_index].adj_list_ind.size();i++){
                int j = tree[q_best_index].adj_list_ind[i];

                if(tree[j].cost_g == 0){
                    tree[j].cost_g = tree[q_best_index].cost_g + line_cost(tree[q_best_index],tree[j]);

                    tree[j].parent = q_best_index;
                    // found_path.push_back(tree[tree[j]]);        //Add j to T with a pointer to q_best

                    open_list_indices.push_back(j);

                    h = line_cost(tree[q_g_index],tree[j]);

                    tree[j].cost_f = tree[j].cost_g + h;
                }
                else if (tree[q_best_index].cost_g + line_cost(tree[q_best_index],tree[j]) < tree[j].cost_g){

                    tree[j].parent = q_best_index;
                    tree[j].cost_g = tree[q_best_index].cost_g + line_cost(tree[q_best_index],tree[j]);
                
                    bool is_in = false;

                    for (int k = 0; i< open_list_indices.size(); k++){
                        if(open_list_indices[k] == j){
                            is_in = true;
                        }
                    }

                    if(is_in){
                        bool added = false;

                        for(int l = 0; l<open_list_indices.size();l++){

                            if( (tree[open_list_indices[l]].cost_f < tree[j].cost_f) && !added){
                                open_list_indices.insert(open_list_indices.begin()+l,j);
                                added = true;
                                break;
                            }

                        }
                        // std::priority_queue<int> q;
                        // for(int n : open_list_indices)    
                        // q.push(n);
                        // q.push(j);
                        // open_list_indices = queue2vector(q);


                    }
                    else{
                        h = line_cost(tree[q_g_index],tree[j]);

                        tree[j].cost_f = tree[j].cost_g + h;
                    }

                }
            }

        }


    }
    std::cout << "\n\nout of loop\n";

    geometry_msgs::Point points;
    marker_4.points.clear();
    points.x = x_goal;
    points.y = y_goal;
    points.z = 0.0;
    marker_4.points.push_back(points);
    
    points.x = latest_added_node.x;
    points.y = latest_added_node.y;
    points.z = 0.0;
    marker_4.points.push_back(points);
    // found_path.push_back(latest_added_node);
    // Node next_node = tree[latest_added_node.parent];
    found_path.push_back(tree[q_g_index]);
    Node next_node = tree[tree[q_g_index].parent];

    points.x = next_node.x;
    points.y = next_node.y;
    points.z = 0.0;
    marker_4.points.push_back(points);

    //std::cout << "\n\first parent x,y"<< tree[tree[q_g_index].parent].x<< "  " << tree[tree[q_g_index].parent].y<<"\n\n";

    while (!next_node.is_root) {
        found_path.push_back(next_node);
        next_node = tree[next_node.parent];
        points.x = next_node.x;
        points.y = next_node.y;
        points.z = 0.0;
        marker_4.points.push_back(points);
    }
    found_path.push_back(tree[0]);
    points.x = tree[0].x;
    points.y = tree[0].y;
    points.z = 0.0;
    marker_4.points.push_back(points);
    return found_path;
}

double RRT::line_cost(Node &n1, Node &n2) {

    double cost = 0;
    cost = std::sqrt(std::pow((n1.x - n2.x), 2) + std::pow((n1.y - n2.y), 2));
    return cost;
}
