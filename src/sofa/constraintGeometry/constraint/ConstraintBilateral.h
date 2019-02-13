#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseDirection.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintBBB : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintBBB , BaseConstraint);

    Data<double> d_maxForce;
    Data<collisionAlgorithm::DetectionOutput> d_input;
    core::objectmodel::SingleLink<ConstraintBBB,BaseDirection,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_direction;

    ConstraintBBB()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_input(initData(&d_input, "input", "Link to detection output"))
    , l_direction(initLink("direction", "Link to direction component")){}

    core::behavior::ConstraintResolution* createConstraintResolution1(const InternalConstraint * /*cst*/) const {
        return new BilateralConstraintResolution1(d_maxForce.getValue());
    }

    core::behavior::ConstraintResolution* createConstraintResolution2(const InternalConstraint * /*cst*/) const {
        return new BilateralConstraintResolution2(d_maxForce.getValue());
    }

    core::behavior::ConstraintResolution* createConstraintResolution3(const InternalConstraint * /*cst*/) const {
        return new BilateralConstraintResolution3(d_maxForce.getValue());
    }

    virtual void createConstraints(ConstraintContainer & constraints) {
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            const ConstraintNormal & CN = l_direction->createConstraintNormal(d);

            if (CN.size() == 1) constraints.add(this, d, CN, &ConstraintBBB::createConstraintResolution1);
            else if (CN.size() == 2) constraints.add(this, d, CN, &ConstraintBBB::createConstraintResolution2);
            else if (CN.size() == 3) constraints.add(this, d, CN, &ConstraintBBB::createConstraintResolution3);
        }
    }
};

}

}
