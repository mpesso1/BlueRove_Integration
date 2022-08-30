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


    pz_ph = proportional_gain(2);
}

//Destructor
pid::rosPID::~rosPID() {
    std::cout << "PID finished" << std::endl;
}


// Reset PID integral and derivative storage if the desired pose is reached w/in tolerance... Should only reset values if the robot will not hold this position. i.e if it will hold position do not reset
void pid::rosPID::reset_pid(bool reset, bool keep_z) {
    float inte_error_ph = integral_error(2);
    float area_error_ph = area_error(2);
    if (reset) {
        integral_error = Eigen::Matrix<float,6,1>::Zero();
        area_error= Eigen::Matrix<float,6,1>::Zero();
        old_error = Eigen::Matrix<float,6,1>::Zero();
    }
    if (keep_z) {
        integral_error(2) = inte_error_ph;
        area_error(2) = area_error_ph;
    }

    // std::cout << "HEYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY" << std::endl;
    // std::cout << "integral error" << integral_error << std::endl;

    give_boost_ons = true;
}


void pid::rosPID::give_boost(float ds, float cs, float breakpoint, float PgainPush) {

    if (ds >= breakpoint) {
        if (cs <= breakpoint) {
            proportional_gain(2) = proportional_gain(2) + PgainPush;
        }
    }
    if (cs>=breakpoint) {
        proportional_gain(2) = pz_ph;
    }

}


void pid::rosPID::cut_pzgain(float ds, float cs, float temp_value) {
    if (cs > ds) {
        proportional_gain(2) = temp_value;
        // area_error(2) = 0;
    }
    else {
        proportional_gain(2) = pz_ph;
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


    // cut_pzgain(ds(2), cs(2), -76);


    // give_boost(ds(2), cs(2), .25, 1000);


    // Integral error computation
    area_error = (ds - cs)*(dt); 
    integral_error = integral_error + area_error;

    // PID id time domain
    controller_output = dot(proportional_gain,(ds - cs)) + dot(integral_gain,integral_error) + dot(derivative_gain,(old_error-(ds - cs))/dt);
    //std::cout << "Integral Thrust: \n" << dot(integral_gain,integral_error) << std::endl;



    // std::cout << "intergral error: " << integral_error(2) << std::endl;
    std::cout << "diriv error: " << derivative_gain(5)*(old_error(5)-(ds(5) - cs(5)))/dt << std::endl;

    // derivative error computation
    old_error = ds - cs;

    // Determine if pid has reached its goal within tolerance
    bool output = pid::rosPID::_stateIsWithinTolerance(tolerance, cs, ds);

    // Reset intergral and derivate storage automatically upon desired state being reached
    if (output && autoReset) {
        pid::rosPID::reset_pid(autoReset, true);
    }

    return output;

}


// Eigen library computation defining is state was reached
bool pid::rosPID::_stateIsWithinTolerance(float tolerance, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> current,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> desired) {
    if (abs(desired(0) - current(0)) <= tolerance+.15 && abs(desired(1) - current(1)) <= tolerance+.15 && abs(desired(2) - current(2)) <= tolerance+.2 && abs(desired(5) - current(5)) <= tolerance+.1) {
        // std::cout << "Within tolerence according to pid library\n";
        return true;
    }
    else {
        // std::cout << "X: " << abs(desired(0) - current(0)) <<  std::endl;
        // std::cout << "Y: " << abs(desired(1) - current(1)) <<  std::endl;
        // std::cout << "Z: " << abs(desired(2) - current(2)) << std::endl;
        // std::cout << "THZ: " << abs(desired(5) - current(5)) <<  std::endl;
        return false;
    }
    // if (current.isApprox(desired,tolerance)) {
    //     return true;
    // }
    // else {
    //     return false;
    // }
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























