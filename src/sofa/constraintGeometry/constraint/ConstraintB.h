#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintB : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintB , BaseConstraint);

    Data<double> d_maxForce;

    ConstraintB()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    core::behavior::ConstraintResolution* createConstraintResolution() const {
        return new BilateralConstraintResolution1(d_maxForce.getValue());
    }

    virtual void createConstraints(ConstraintContainer & constraints) {
//        return InternalConstraint::create(this, d, ConstraintNormal::createFrame(), &ConstraintB::createConstraintResolution);
    }

};

}

}
