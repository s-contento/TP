#include"ros/ros.h"
#include"ros_service/service.h"
#include<sstream>
#include<iostream>
#include<string>

using namespace std;

int main(int argc, char **argv){
    
    ros::init(argc,argv,"service_client");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::ServiceClient client = nh.serviceClient<ros_service::service>("service");

    cout << "\nInserire indice iniziale \n('0' per uscire): \n";
    int parola = 0;

    while (cin >> parola){
        
        if(parola == 0){           //If type quit exit
            break;
        }

        ros_service::service srv;


        if(client.call(srv)){
            cout << "\n\nCalling service ... ]\n\r";
            
        }
        else{
            ROS_ERROR("Failed to call Service");
            return 1;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    cout << "Good bye\n";
    return 0;
}