#include "goPath.h"
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <geometry_msgs/Pose.h>
//access token == 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

using namespace std;
using namespace root;

int main(int argc,char** argv) {

    ros::init(argc,argv,"Path_Planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publish publish = n.advertise<geometry_msgs::Pose>("waypoint",1000);


    //Timer time;
    MeanTraj BlueRov(6,20,3); // # of DOF , steps, # of ocv
    BlueRov.set_sensitivity(5.0,2,2); // whole, ability, power
    BlueRov.add_DOF(.1,2,0.,4.,0,true); // acceleration, init velocity, init poition, final position, indx, boolian defining ocv
    BlueRov.add_DOF(.2,2,0.,7.,1,true); 
    BlueRov.add_DOF(.1,2.,0.,12.,2,true);
    BlueRov.add_DOF(.1,2.,0.,4.,3,false);
    BlueRov.add_DOF(.1,2.,0.,4.,4,false);
    BlueRov.add_DOF(.1,2.,0.,5.,5,false);
    std::vector<float> objx;
    objx.push_back(2.59);
    objx.push_back(3.75);
    objx.push_back(2.2);
    objx.push_back(3.0);
    objx.push_back(4.024);
    std::vector<float> objy;
    objy.push_back(2.86);
    objy.push_back(5.46);
    objy.push_back(3.8);
    objy.push_back(4.0);
    objy.push_back(4.710);
    std::vector<float> objz;
    objz.push_back(3.31);
    objz.push_back(8.68);
    objz.push_back(6.5);
    objz.push_back(7.00);
    objz.push_back(6.27);
    BlueRov.optimize(objx,objy,objz); 
    BlueRov.get_something();
    BlueRov.traj(1); // 0-off, 1-on
    BlueRov.surf(0); // 0-none, 1-Kp, 2-Kp_inv, 3-Kv, 4-Kv_in, 5-Ka_inv
    return 0;
}