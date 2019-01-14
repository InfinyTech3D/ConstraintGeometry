#pragma once

#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/constraintGeometry/Constraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ResponseU : public BaseResponse {
public:
    SOFA_CLASS(ResponseU , BaseResponse);

    Data<double> d_maxForce;

    ResponseU()
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
