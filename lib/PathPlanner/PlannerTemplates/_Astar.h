#ifndef _ASTAR
#define _ASTAR
#include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PlannerTemplates/PathPlanner.h"
#include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PathPlanners/A_star.h"
#include "Eigen/Dense"

using namespace std;

template <const int DOF>
class _Astar : public pp::PathPlanner<DOF> {
    private:
        const float step;
        // std::vector<Eigen::Matrix<float,1,3>> path_buffer;
    protected:
    public:

        Eigen::Matrix<float,1,3> jj;

        star::A_star* BlueRov;

        _Astar(float step) : step (step) {};

        void setup() override {}

        void compute() override {
            BlueRov = new star::A_star(Eigen::Matrix<float,1,3>{this->init_pose(this->X), this->init_pose(this->Y), this->init_pose(this->Z)}, 
                                Eigen::Matrix<float,1,3>{this->goal_pose(this->X), this->goal_pose(this->Y), this->goal_pose(this->Z)}, 
                                this->objects[0].transpose(), step);

            // cout << "computed\n";
        }
        
        void store_results() override {
            std::vector<Eigen::Matrix<float,1,3>> path_buffer = BlueRov->return_path(); 
            this->path_trans.resize(static_cast<int>(path_buffer.size()),3);

            // jj.resize(static_cast<int>(path_buffer.size()),3);



            int idx = 0;
            for (auto i = path_buffer.begin(); i != path_buffer.end(); i++) {
                cout << (*i) << endl;
                this->path_trans.row(idx) = (*i);
                // cout << "Mason\n";
                idx = idx+1;
            }            
            // this->path_trans = path_buffer;

            delete BlueRov;
        }
};


#endif // _ASTAR
