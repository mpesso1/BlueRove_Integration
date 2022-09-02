#ifndef OMPL_RRTCONNECT
#define OMPL_RRTCONNECT
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

namespace ompl_RRTConnect {

    class ValidityChecker : public ompl::base::StateValidityChecker {
    private:
    protected:
    public:
        ValidityChecker(const ompl::base::SpaceInformationPtr&);

        bool isValid(const ompl::base::State*) const override;

        double clearance(const ompl::base::State*) const override;

    };


    std::vector<Eigen::Matrix<float,1,3>> plan();

}


#endif // OMPL_RRTCONNECT