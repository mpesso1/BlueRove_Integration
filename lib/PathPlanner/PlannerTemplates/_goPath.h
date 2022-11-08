#ifndef _GOPATH
#define _GOPATH
#include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PlannerTemplates/PathPlanner.h"
#include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PathPlanners/goPath.h"
#include "Eigen/Dense"

using namespace std;

template <const int DOF>
class _goPath : public pp::PathPlanner<DOF> {
    private:

        // CONTROL VARIABLES ---------------
        const float MAX_ACCEL = .1; // maximum acceleration of robot defined
        int STEP = 0; // Index of what step the pid is on. Used to index trajectory. 

        // const int DOF = 6; // Degrees of freedom
        // const int STEPS = 60; // steps within trajectory
        const int OCV = 3; // object concering DOF

        const float WHOLE_SENSITIVITY = 50;
        const float ABILITY_SENSITIVITY = 25;
        const float POWER_SENSITIVITY = 1;


    public:

        root::MeanTraj* BlueRov;
        
        _goPath() {BlueRov = new root::MeanTraj(6,60,3);};
        ~_goPath() override { cout << "Destructor called for _goPath\n"; delete BlueRov;}


        void setup() override {
            BlueRov->set_sensitivity(WHOLE_SENSITIVITY,ABILITY_SENSITIVITY,POWER_SENSITIVITY);
        };

        void compute() override {

            BlueRov->add_DOF(MAX_ACCEL, 0, this->init_pose(this->X), this->goal_pose(this->X), 0, true); 
            BlueRov->add_DOF(MAX_ACCEL, 0, this->init_pose(this->Y), this->goal_pose(this->Y), 1, true); 
            BlueRov->add_DOF(MAX_ACCEL, 0, this->init_pose(this->Z), this->goal_pose(this->Z), 2, true);
            BlueRov->add_DOF(MAX_ACCEL, 0, 0, 0, 3, false);
            BlueRov->add_DOF(MAX_ACCEL, 0, 0, 0, 4, false);
            BlueRov->add_DOF(MAX_ACCEL, 0, 0 /* this->init_pose(this->THZ) */, 0 /*this->goal_pose(this->THZ)*/, 5, false);
//                           in there ->^^
// this->init_vel(this->X)
// this->init_vel(this->Y)
// this->init_vel(this->Z)
// this->init_vel(this->Z_THVEL)


            std::vector<float> obj_x;
            std::vector<float> obj_y;
            std::vector<float> obj_z;
            for (auto itr = this->objects.begin(); itr != this->objects.end(); itr++) {
                obj_x.push_back((*itr)(0));
                obj_y.push_back((*itr)(1));
                obj_z.push_back((*itr)(2));
            }


            if (obj_x.size() == 1) {
                cout << "First oject x: " << obj_x[0] << " objects size: " << obj_x.size() << endl;
                cout << "First oject y: " << obj_y[0] << " objects size: " << obj_y.size() << endl;
                cout << "First oject z: " << obj_z[0] << " objects size: " << obj_z.size() << endl;
            }

            cout << "Goal: " << this->goal_pose << endl;
            cout << "Init: " << this->init_pose << endl;

            this->BlueRov->optimize(obj_x,obj_y,obj_z);

        };

        void store_results() override { 
            this->path_trans.resize(this->path_trans.cols(),3);
            this->path_angular.resize(this->path_angular.cols(),1);
            this->path_trans = this->BlueRov->trajectory_translational();
            this->path_angular = this->BlueRov->trajectory_orientation();

            cout << this->path_trans.transpose() << endl;

            for(auto ii = this->objects.begin(); ii != this->objects.end(); ii++) {
                cout << "object X: " << (*ii)(0) << " object Y: " << (*ii)(1) << " object Z: " << (*ii)(2) << endl;

            }

        }


};

#endif // _GOPATH