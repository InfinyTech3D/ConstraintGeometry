#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintBBB : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintBBB , BaseConstraint);

    Data<double> d_maxForce;

    ConstraintBBB()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return InternalConstraint(d.first,
                                  d.second,
                                  ConstraintNormal::createFrameFromDetection(d),
                                  new BilateralConstraintResolution3(d_maxForce.getValue()));
    }
};

}

}
