#include"ros/ros.h"
#include"mobile_navigation/service.h"
#include<sstream>
#include<iostream>
#include<string>

using namespace std;

int main(int argc, char **argv){
    
    ros::init(argc,argv,"service_client");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::ServiceClient client = nh.serviceClient<mobile_navigation::service>("service");

    cout << "\nInserire indice iniziale \n('0' per uscire): \n";
    int parola = 0;

    while (cin >> parola){
        
        if(parola == 0){           //If type quit exit
            break;
        }

        mobile_navigation::service srv;

        srv.request.in.data = parola;
        if(client.call(srv)){
            cout << "\n\nCalling service ... ]\n\r";
            cout << "\n\nFrom Client: ["<< srv.request.in << "]\n\r";
            cout << "\nReceived: " << srv.response.out<<endl;
        }
        else{
            ROS_ERROR("Failed to call Service");
            return 1;
        }
        cout << "\n\nbobo\n\r";

        ros::spinOnce();

        loop_rate.sleep();
    }

    cout << "Good bye\n";
    return 0;
}