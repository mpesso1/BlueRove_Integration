#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

using namespace std;

void callback(const std_msgs::UInt16& msg) {
    cout << msg.data << endl;
}


int main(int argc, char** argv) {
    ros::init(argc,argv,"tester");
    ros::NodeHandle n;
    ros::Subscriber subscribe = n.subscribe("commands",1000,callback);

    ros::spin();

    return 0;
}