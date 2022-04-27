/*
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON

PID Implementation File 
*/

#include <iostream>
#include <Eigen/Dense>
#include "ros_pid.h"

/*
Description of file:
    PID controller with desired DOF 

    In order to make use of library you must:

        NOTE: Will need a way of computing the change in time variable dt used within run_pid

        1.) Call the constructor giving:
            - Degrees of Freedom of the System
            - Desired PID gains
        
        2.) Call run_pid giving:
            - a change in time value
            - desired state
            - current state
            - tolerance defining if current state meets desired state
            - boolean defining if the pid storage values i.e. intergra error computation should reset upon reaching desired goal state
*/

//Contructor
pid::rosPID::rosPID(int dof, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> pg, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ig, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> dg) {
    
    DOF = dof;      // Degrees of freedom


    // Gains
    proportional_gain.resize(DOF,1);
    derivative_gain.resize(DOF,1);
    integral_gain.resize(DOF,1);


    // Make sure all gains are in the correct vector orientation
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


    // Output of PID
    controller_output.resize(DOF,1);
    
    // Intergral error storage
    integral_error.resize(DOF,1);
    integral_error = Eigen::Matrix<float,6,1>::Zero();
    area_error= Eigen::Matrix<float,6,1>::Zero();

    // Derivative error storage
    old_error = Eigen::Matrix<float,6,1>::Zero();
}

//Destructor
pid::rosPID::~rosPID() {
    std::cout << "PID finished" << std::endl;
}


// Reset PID integral and derivative storage if the desired pose is reached w/in tolerance... Should only reset values if the robot will not hold this position. i.e if it will hold position do not reset
void pid::rosPID::reset_pid(bool reset) {
    if (reset) {
        integral_error = Eigen::Matrix<float,6,1>::Zero();
        area_error= Eigen::Matrix<float,6,1>::Zero();
        old_error = Eigen::Matrix<float,6,1>::Zero();
    }
}


// PID computation that returns a boolean defining if the desired pose is within tolerance 
bool pid::rosPID::run_pid(float dt, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> ds, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> cs, float tolerance, bool autoReset=false) {

    // Make sure state vectors are in the correct vector orientation
    if (ds.cols() > 1) {
        ds = ds.transpose();
    }
    if (cs.cols() > 1) {
        cs = cs.transpose();
    }
    
    // Integral error computation
    area_error = (ds - cs)*(dt); 
    integral_error = integral_error + area_error;

    // PID id time domain
    controller_output = dot(proportional_gain,(ds - cs)) + dot(integral_gain,integral_error) + dot(derivative_gain,(old_error-(ds - cs))/dt);
    std::cout << "Integral Thrust: \n" << dot(integral_gain,integral_error) << std::endl;

    // derivative error computation
    old_error = ds - cs;

    // Determine if pid has reached its goal within tolerance
    bool output = pid::rosPID::_stateIsWithinTolerance(tolerance, cs, ds);

    // Reset intergral and derivate storage automatically upon desired state being reached
    if (output && autoReset) {
        pid::rosPID::reset_pid(autoReset);
    }

    return output;

}


// Eigen library computation defining is state was reached
bool pid::rosPID::_stateIsWithinTolerance(float tolerance, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> current,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> desired) {
    if (current.isApprox(desired,tolerance)) {
        return true;
    }
    else {
        return false;
    }
}

// dot product
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























