#include <iostream>
#include <Eigen/Dense>
#include "ros_pid.h"

/*
PID controller with desired DOF
*/

//Contructor
pid::rosPID::rosPID(int dof, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> pg, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ig, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> dg) {
    
    DOF = dof;
    proportional_gain.resize(DOF,1);
    derivative_gain.resize(DOF,1);
    integral_gain.resize(DOF,1);

    if (pg.cols() > 1) {
        proportional_gain = pg.transpose();
    }
    if (ig.cols() > 1) {
        integral_gain = ig.transpose();
    }
    if (dg.cols() > 1) {
        derivative_gain = dg.transpose();
    }
    
    proportional_gain = pg;
    integral_gain = ig;
    derivative_gain = dg;
    controller_output.resize(DOF,1);
    integral_error.resize(DOF,1);
    integral_error = Eigen::Matrix<float,6,1>::Zero();
    area_error= Eigen::Matrix<float,6,1>::Zero();
    old_error = Eigen::Matrix<float,6,1>::Zero();
}

//Destructor
pid::rosPID::~rosPID() {
    std::cout << "PID finished" << std::endl;
}

//PID
void pid::rosPID::run_pid(float dt, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ds, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> cs) {
    if (ds.cols() > 1) {
        ds = ds.transpose();
    }
    if (cs.cols() > 1) {
        cs = cs.transpose();
    }
    area_error = (ds - cs)*(dt);
    integral_error = integral_error + area_error;
    controller_output = dot(proportional_gain,(ds - cs)) + dot(integral_gain,integral_error) + dot(derivative_gain,(old_error-(ds - cs))/dt);
    old_error = ds - cs;
}

//Dot Product
Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> pid::rosPID::dot(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> A, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> B) {
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> C;
    C.resize(DOF,1);
    if (A.size() != B.size()) {
        std::cout << "dot product of different sized matrices" << std::endl;
    }
    else {
        for (int i=0;i<A.size();i++) {
            C(i) = A(i)*B(i); 
        }
        return C;
    }
}























