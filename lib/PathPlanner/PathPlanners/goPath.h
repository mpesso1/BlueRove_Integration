/*
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
*/

#ifndef GPMP_H
#define GPMP_H
#include <Eigen/Dense>
#include <vector>
#include <iostream>

namespace root {

    class MeanTraj {
        private:

            // -- Prior Variables

            std::vector<float> final_time{}; // final times calculated for each DOF -- indexed based off of the order each DOF is initialized

            Eigen::Matrix<float,1,Eigen::Dynamic> state_time{}; // time at each step taken  //-- specified by max time calculated and the amount of steps specified

            int DsOF{}; // counts the number of DOF that the system pocesses

            float max_time = 0; // largest time calculated from the inputs of each DOF --> Used to set the size of each DOF mean vector and kernal

            Eigen::Matrix<float,4,Eigen::Dynamic> init_parameter{}; // matrix that stores all the initial paramiters that define the prior

            int steps{}; // defines how many steps there are total between initial and final positions -- including the defined starting and final positions
            
            float step{}; // tf / steps --> t0 is always = 0

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> B{}; // Matrix specific to prior algorithm

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Q{}; // Matrix specific to prior algorithm

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Ka_inv{}; //  inverse acceleration kernal

            Eigen:: Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Kv_inv{}; // inverse velocity kernal

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Kv{}; // velocity kernal

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Kp_inv{}; // inverse position kernal

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Kp{}; // position kernal

            float Qc; // sensitivity parameter used for defining the uncertanty in prior trajectory values


            // ----- Optimization Variables


            std::vector<float> sensitivity{}; // stores the sensitiviy of the update rule --> (0) = scales how much entire state is alowed to update, (1) = scales how much the state is allowed to deviate from prior

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> mag_ocv{}; // magnitude of vector responsiple for ocv's.

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> J{};  // Jacobian matrix that is defined by the number of ocv present.  It does not have to be square

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> unit_prime{}; // UNIT vector containing the first derivative of ocv's.

            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> unit_dubprime{}; // vecotor containing the second derivative of ocv's.

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> del_c{}; // obstical gradient cost

            float c; // magnitude of del_c 

            std::vector<int> ocv_idx{}; // idx to which specified DOF will be considered within the the cost function

            int ocv_size{}; // object concerning varibale index vector size

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> final_pos{}; // Matrix containg the final ocv variables after optimization

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> mean_pos{};

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> final_vel{}; // matrix containing the first derivative of the final ocvs after optimization

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> mean_vel{};

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> I{}; // identity matrix thats size gets set by the number of the ocvs

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> IJ{}; // square jacobian that gets used in the optimization portion.  Other J is mearly a place holder.

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> g_pos{}; // position object cost matrix

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> g_vel{}; // velocity object cost matrix

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> obs{}; // vector containing cartesion position of obsticals

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> obs_plot{};

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> check_pos{}; // camparitor to check if optimization reaches criterion



        public:
            enum INPUTS {ACCEL,V0,P0,Pf};

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> thz_init{};

            MeanTraj(int,int,int); 
            // Constructor, allocates space in memory for class

            ~MeanTraj(); 
            // Destructor, removes space in memory that class obtained once code falls out of scope that the class was initialized

            void add_DOF(float,float,float,float,int,bool); 
            // (must clarify the odered number in which this DOF is initialized --> input DOF with desired constant acceleration, 
            // initial velocity, initial position, and final position.  Values get stored in a matrix with all other values. and they are 
            // indexed by the order in which you input the DOF. This is why you must input the oder when initializing function.

            void define_step();
            // calculates the step based off of the max time calculated and the steps desired

            void define_states();
            // fills the state_time vector with each steps time

            void define_B();

            void define_Q();

            void define_Ka_inv();

            void define_kernals();

            void set_sensitivity(float,float,float); // sets the sensitivity of two parameters that vary how large the step is allowed to be within the cost
            // (0) - scales step size ; (1) - scales how far new trajectory can vary from mean, (2) defines the power in the noise of the prior that is created
            // Both of these sensitivity parameters can be found in the update step of GPMP. While the only other sensitivity parameter is Qc, 
            
            void get_something();
            // displays private variables to the terminal

            void traj(int);
            // function used to display to shell perfectly for python code to run custom traj function

            void surf(int);
            // function used to diplay to shell perfectly for python code to run custom surf function

            void update_accel(); 
            // updates the accelerations of each DOF so their elapsed times to reach their goal states are all the same

            void add_ocv(int);
            // keeps track of the index of variabels that we will include in the object cost function.  These variables are the only variables that can affect interaction with objects
            // For example, consider a spherical robot.  We would not be concerned with the oriantation that the robot would be when considering object interation
            // So we would only be concerend with the linear cordiantes of the robot w.r.t to the linear object cordinates.

            void declare_optimization();
            // sets all of the initialized cost variables equal to whatever they are supposed to be
            
            void optimize(std::vector<float>,std::vector<float>,std::vector<float>);
            // calculate cost function with input from sensors defining where opsticals are

            void update_J(int); // the int will define the index of which state we are at
            // since there is no matrix dot product in cpp I will use this function to swap values for the Jacobian

            void update_g(float radius, float scale_error);
            // checks for obsticals at each state and computes obsticle gradient

            void update_optimizaion();
            // function that updates variables used within update rule... as states change

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> trajectory_translational();

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> trajectory_orientation();
    
    };
}
#endif // GPMP_H