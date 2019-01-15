#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintUFF : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintUFF , BaseConstraint);

    Data<double> d_maxForce;
    Data<double> d_friction;

    ConstraintUFF()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction")) {}

    virtual InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        if (d_friction.getValue() == 0.0) {
            return InternalConstraint(d.first,d.second,
                                      ConstraintNormal::createFromDetection(d),
                                      new UnilateralConstraintResolution(d_maxForce.getValue()));
        } else {
            return InternalConstraint(d.first,d.second,
                                      ConstraintNormal::createFrameFromDetection(d),
                                      new UnilateralFrictionResolution(d_maxForce.getValue(),d_friction.getValue()));
        }
    }
};

}

}
