#pragma once

#include <sofa/constraintGeometry/resolution/TrajectoryResolution.h>
#include <sofa/constraintGeometry/Constraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ResponseFBB : public BaseResponse {
public:
    SOFA_CLASS(ResponseFBB , BaseResponse);

public:
    Data<double> d_maxForce;

    ResponseFBB()
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
