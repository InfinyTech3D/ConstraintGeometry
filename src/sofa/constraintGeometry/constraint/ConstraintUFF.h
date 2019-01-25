#pragma once

#include <sofa/constraintGeometry/constraint/ConstraintU.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintUFF : public ConstraintUnilateral {
public:
    SOFA_CLASS(ConstraintUFF , ConstraintUnilateral);

    Data<double> d_maxForce;
    Data<double> d_friction;

    ConstraintUFF()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction")) {}

    virtual InternalConstraint::SPtr createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        if (d_friction.getValue() == 0.0) return InternalConstraint::create(d, ConstraintNormal(getUnilateralNormal(d)));
        else return InternalConstraint::create(d, ConstraintNormal::createFrame(getUnilateralNormal(d)));
    }

    virtual core::behavior::ConstraintResolution* createConstraintResolution() {
        if (d_friction.getValue() == 0.0) return new UnilateralConstraintResolution(d_maxForce.getValue());
        else return new UnilateralFrictionResolution(d_maxForce.getValue(),d_friction.getValue());
    }
};

}

}
