#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include "Eigen/Dense"
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>
#include "ompl_RRTConnect.h"


namespace og = ompl::geometric;
using namespace std;



ompl_RRTConnect::ValidityChecker::ValidityChecker(const ompl::base::SpaceInformationPtr& si) : ompl::base::StateValidityChecker(si) {};


bool ompl_RRTConnect::ValidityChecker::isValid(const ompl::base::State* state) const{
        return this->clearance(state) > 0.0;
}

double ompl_RRTConnect::ValidityChecker::clearance(const ompl::base::State* state) const {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const auto* state3D =
            state->as<ompl::base::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state3D->values[0];
        double y = state3D->values[1];
        double z = state3D->values[2];

        // Distance formula between two points, offset by the circle's
        // radius
        // 3.33287 -1.66493 -0.838283
        // 4.32952 -4.11436 -2.41773
        double sx = 4.32952;
        double sy = -4.11436;
        double sz = -2.41773;
        double R = 1.5;
        return sqrt((x-sx)*(x-sx) + (y-sy)*(y-sy) + (z-sz)*(z-sz)) - R;
}


std::vector<Eigen::Matrix<float,1,3>> ompl_RRTConnect::plan() {
    std::vector<Eigen::Matrix<float,1,3>> path;
    // construct the state space we are planning in
    // auto space(std::make_shared<ompl::base::SE3StateSpace>());
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

    // set the bounds for the R^3 part of SE(3)
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    si->setup();

    // create a random start state
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 1.36949;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -3.34161;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = -2.45456;
    // 1.36949 ,-3.34161, -2.45456
    // start.random()

    // create a random goal state
    ompl::base::ScopedState<> goal(space);
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 7;
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -4;
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = -2;


    // create a problem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    planner->setRange(.5);

    // perform setup steps for the planner
    planner->setup();

    // std::cout << "THIS IS THE RANGE: \n";



    planner->getRange();

    // std::cout << "MASON RANGE FUNCTIONS WERE CALLED\n";


    // print the settings for this space
    // si->printSettings(std::cout);

    // print the problem settings
    // pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(10);

    if (solved) {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        // ompl::base::PathPtr path = pdef->getSolutionPath().printAsMatrix();
        // std::cout << "Found solution:" << std::endl;

        // // print the path to screen
        // path->print(std::cout);
        ofstream outfile;
        outfile.open("sample.txt");

        pdef->getSolutionPath()->print(outfile);

        ifstream infile;
        infile.open("sample.txt"); // use as a buffer to convert output to usuable form

        string tp;
        

        Eigen::Matrix<float,1,3> cords;

        while(getline(infile,tp)) {
            // cout << tp.find("RealVectorState") << endl;
            string key = "RealVectorState"; // Every vector is printed out in brackets after this word
            if (tp.find(key) != std::string::npos) {  // if find function is unable to find the string it return the largest storable unsigned int by default... that is what the value of npos is
                tp.erase(0, 17); // erase takes in the starting index and the length... other variants exist that use pointers
                // tp.erase(tp.end()-1);
                tp.replace(tp.end()-1,tp.end()," "); // replace takes in the starting index and the length... this is necessary due to our implimentation... we use " " to determine number length

                for (auto i=0; i<3; i++) { // loop over all three numbers using " " to determin number length postion and value... then stor value into cords {vector of position}
                    auto spaceIDX = tp.find(" "); // search for " "
                    if (spaceIDX != std::string::npos) {
                        auto idx = static_cast<int>(spaceIDX);
                        char num1[idx];

                        tp.copy(num1,idx); // store number
                        int num_len = sizeof(num1); // use the size of nuumber to delete from array

                        double x = std::atof(num1); // convert from char[] to double
                        cords(i) = x;

                        tp.erase(0, num_len+1); // erase from string
                    }
                }
                path.push_back(cords);             
            }
        }
        // for (auto i = path.begin(); i != path.end(); i++) {
        //     cout << (*i) << endl;
        // }
    }
    return path;
};






