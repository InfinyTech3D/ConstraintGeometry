#pragma once

#include <sofa/constraintGeometry/resolution/UnilateralResponse.h>
#include <sofa/constraintGeometry/Constraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class UnilateralConstraintWithFrictionResponse : public BaseResponse {
public:
    SOFA_CLASS(UnilateralConstraintWithFrictionResponse , BaseResponse);

    Data<double> d_maxForce;
    Data<double> d_friction;

    UnilateralConstraintWithFrictionResponse()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction")) {}

    virtual InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput & d) {
        if (d_friction.getValue() == 0.0) {
            return InternalConstraint(d.getFirstProximity(),d.getSecondProximity(),
                                      ConstraintNormal::createFromDetection(d),
                                      new UnilateralConstraintResolution(d_maxForce.getValue()));
        } else {
            return InternalConstraint(d.getFirstProximity(),d.getSecondProximity(),
                                      ConstraintNormal::createFrameFromDetection(d),
                                      new UnilateralFrictionResolution(d_maxForce.getValue(),d_friction.getValue()));
        }
    }
};

}

}
