/*
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON

PID Header File 
*/

#ifndef ROS_PID_H
#define ROS_PID_H
#include <Eigen/Dense>


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

namespace pid {

    class rosPID {
    private:
        // Gain variables
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> proportional_gain;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> integral_gain;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> derivative_gain;

        int DOF; // Degrees of Freedom
          

    public:

        // Intergral Storage
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> area_error;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> integral_error;

        // Derivative Storage
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> old_error;
        // Output of the PID
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> controller_output;

        // Consructor -- set gains and initialize error storage
        rosPID(int,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>);

        // Destructor
        ~rosPID();

        //PID controller
        bool run_pid(float, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>, float, bool);

        // reset pid values
        void reset_pid(bool);

        // check if within tolerence of desired state
        bool _stateIsWithinTolerance(float, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>);

        // dot procuct
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> dot(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>);
    };
}



#endif //ROS_PID_H