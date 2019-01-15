#pragma once

#include <sofa/constraintGeometry/resolution/TrajectoryResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintFBB : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintFBB , BaseConstraint);

public:
    Data<double> d_maxForce;

    ConstraintFBB()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
//        return InternalConstraint(d.first,
//                                  d.second,
//                                  ConstraintNormal::createFromDetection(d),
//                                  new BilateralConstraintResolution1(d_maxForce.getValue()));
    }
};

}

}
