#ifndef ROS_PID_H
#define ROS_PID_H
#include <Eigen/Dense>


/*
PID controller with desired DOF.
*/


namespace pid {

    class rosPID {
    private:
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> proportional_gain;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> integral_gain;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> derivative_gain;
        int DOF;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> controller_output;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> area_error;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> integral_error;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> old_error;

    public:
        // Consructor -- set gains
        rosPID(int,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>);

        //Destructor
        ~rosPID();

        //PID controller -- set points and current pose
        void run_pid(float, Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>);

        //Dot procuct between vectors
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> dot(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>,Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>);
    };
}
#endif //ROS_PID_H