#pragma once

#include <sofa/constraintGeometry/resolution/TrajectoryResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintPFF : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintPFF , BaseConstraint);

public:
    Data<double> d_maxForce;
    Data<double> d_angle;
    Data<double> d_friction;

    ConstraintPFF()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_angle(initData(&d_angle, 1.0, "Angle", "Angle needed for penetration [-1..1] "))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction along the normal after the penetration")) {}


    InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
//        return InternalConstraint(d.first,
//                                  d.second,
//                                  ConstraintNormal::createFromDetection(d),
//                                  new BilateralConstraintResolution1(d_maxForce.getValue()));
    }
};

}

}
