#ifndef PATHPLANNER
#define PATHPLANNER

#include <vector>
#include <iostream>
#include <Eigen/Dense>

namespace pp {

using namespace std;

template <const int DOF>
class PathPlanner {
    private:
    protected:

        // Eigen::Matrix<float,60+1, DOF> path_trans;

        
    public:
    
        PathPlanner(){};
        virtual ~PathPlanner(){cout << "Destructor of PathPlanner called\n"; };

        Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic> path_trans;
        Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic> path_angular;

        vector<Eigen::Matrix<float,1,3>> objects;

        Eigen::Matrix<float,1,DOF> init_pose;
        Eigen::Matrix<float,1,DOF> init_vel;
        Eigen::Matrix<float,1,DOF> init_acel;

        Eigen::Matrix<float,1,DOF> goal_pose;
        Eigen::Matrix<float,1,DOF> goal_vel;
        Eigen::Matrix<float,1,DOF> goal_acel;

        virtual void add_object(float x, float y, float z) {objects.push_back(Eigen::Matrix<float,1,3>{x,y,z});}

        virtual void add_init_pose(Eigen::Matrix<float,1,DOF> &pose) {init_pose = pose;}

        virtual void add_init_vel(Eigen::Matrix<float,1,DOF> &vel) {init_vel = vel;}

        virtual void add_init_acel(Eigen::Matrix<float,1,DOF> &acel) {init_acel = acel;}

        virtual void add_goal_pose(Eigen::Matrix<float,1,DOF> &pose) {goal_pose = pose;}

        virtual void add_goal_vel(Eigen::Matrix<float,1,DOF> &vel) {goal_vel = vel;}

        virtual void add_goal_acel(Eigen::Matrix<float,1,DOF> &acel) {goal_acel = acel;}

        virtual float get_pose_linear(int step, int dof) {return this->path_trans(step,dof);}

        virtual float get_pose_angular(int step, int dof) {return this->path_angular(step,dof);}

        virtual void setup(){}

        virtual void compute(){}

        virtual void store_results(){}

        enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
        enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity

};

}

#endif // PATHLPLANNER