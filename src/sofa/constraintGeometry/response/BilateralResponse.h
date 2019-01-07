#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResponse.h>
#include <sofa/constraintGeometry/Constraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class BilateralResponse1 : public BaseResponse {
public:
    SOFA_CLASS(BilateralResponse1 , BaseResponse);

    Data<double> d_maxForce;

    BilateralResponse1()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput & d) {
        return InternalConstraint(d.getFirstProximity(),
                                  d.getSecondProximity(),
                                  ConstraintNormal::createFromDetection(d),
                                  new BilateralConstraintResolution1(d_maxForce.getValue()));
    }
};

class BilateralResponse3 : public BaseResponse {
public:
    SOFA_CLASS(BilateralResponse3 , BaseResponse);

    Data<double> d_maxForce;

    BilateralResponse3()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput & d) {
        return InternalConstraint(d.getFirstProximity(),
                                  d.getSecondProximity(),
                                  ConstraintNormal::createFrameFromDetection(d),
                                  new BilateralConstraintResolution3(d_maxForce.getValue()));
    }
};

}

}
