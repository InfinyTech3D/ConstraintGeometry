#pragma once

#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintU : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintU , BaseConstraint);

    Data<double> d_maxForce;

    ConstraintU()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    virtual InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return InternalConstraint(d.first,
                                  d.second,
                                  ConstraintNormal::createFromDetection(d),
                                  new UnilateralConstraintResolution(d_maxForce.getValue()));
    }

};

}

}
