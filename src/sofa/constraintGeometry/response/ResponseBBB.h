#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/Constraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ResponseBBB : public BaseResponse {
public:
    SOFA_CLASS(ResponseBBB , BaseResponse);

    Data<double> d_maxForce;

    ResponseBBB()
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
