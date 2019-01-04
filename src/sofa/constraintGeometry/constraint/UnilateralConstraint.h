#pragma once

#include <sofa/constraintGeometry/response/UnilateralResponse.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class UnilateralConstraint : public BaseConstraint {
public:
    SOFA_CLASS(UnilateralConstraint , BaseConstraint);

    Data<double> d_maxForce;

    UnilateralConstraint()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    virtual ConstraintReponse * createResponse(const collisionAlgorithm::DetectionOutput & d) {
        ConstraintNormal cn(d.getNormal());

        return new UnilateralConstraintResolution(cn,
                                                  d.getFirstProximity(),
                                                  d.getSecondProximity(),
                                                  d_maxForce.getValue());
    }

};

}

}
