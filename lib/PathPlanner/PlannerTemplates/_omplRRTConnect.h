#ifndef OMPLRRTCONNECT
#define OMPLRRTCONNECT
#include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PlannerTemplates/PathPlanner.h"
#include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PathPlanners/ompl_RRTConnect.h"

template <const int DOF>
class _omplRRTConnect : public pp::PathPlanner<DOF> {
private:
protected:
public:

    _omplRRTConnect() {}

    void setup() override {};

    void compute() override {
        std::vector<Eigen::Matrix<float,1,3>> path_buffer = ompl_RRTConnect::plan();
        this->path_trans.resize(path_buffer.size(),DOF);
        int i = 0;
        for (auto itr = path_buffer.begin(); itr != path_buffer.end(); itr++) {
            this->path_trans.row(i) = (*itr);
            i = i+1;
        }
        // cout << this->path_trans << endl;
    };

    void store_results() override {};

};

#endif // OMPLRRTCONNECT