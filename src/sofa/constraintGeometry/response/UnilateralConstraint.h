#pragma once

#include <sofa/constraintGeometry/resolution/UnilateralResponse.h>
#include <sofa/constraintGeometry/Constraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class UnilateralConstraintResponse : public BaseResponse {
public:
    SOFA_CLASS(UnilateralConstraintResponse , BaseResponse);

    Data<double> d_maxForce;

    UnilateralConstraintResponse()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    virtual InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput & d) {
        return InternalConstraint(d.getFirstProximity(),
                                  d.getSecondProximity(),
                                  ConstraintNormal::createFromDetection(d),
                                  new UnilateralConstraintResolution(d_maxForce.getValue()));
    }

};

}

}
