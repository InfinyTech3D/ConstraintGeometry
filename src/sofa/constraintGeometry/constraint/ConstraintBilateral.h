#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintBilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintBilateral , BaseConstraint);

    Data<double> d_maxForce;

    core::objectmodel::SingleLink<ConstraintBilateral,collisionAlgorithm::BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<ConstraintBilateral,collisionAlgorithm::BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
    core::objectmodel::SingleLink<ConstraintBilateral,collisionAlgorithm::BaseAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_algo;

    ConstraintBilateral()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , l_from(initLink("from", "Link to src geometry"))
    , l_dest(initLink("dest", "Link to dst geometry"))
    , l_algo(initLink("algo", "Link to detection output")) {}

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
        if (l_from == NULL) return;
        if (l_dest == NULL) return;
/*
        helper::vector<collisionAlgorithm::PairDetection> detection;
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            const ConstraintNormal & CN = l_direction->createConstraintNormal(d);

            if (CN.size() == 1) constraints.add(this, d, CN, &ConstraintBilateral::createConstraintResolution1);
            else if (CN.size() == 2) constraints.add(this, d, CN, &ConstraintBilateral::createConstraintResolution2);
            else if (CN.size() == 3) constraints.add(this, d, CN, &ConstraintBilateral::createConstraintResolution3);
        }
        */
    }
};

}

}
