#pragma once

#include <sofa/constraintGeometry/response/BilateralResponse.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class BilateralResponse1 : public BaseConstraint {
public:
    SOFA_CLASS(BilateralResponse1 , BaseConstraint);

    Data<double> d_maxForce;

    BilateralResponse1()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    ConstraintReponse * createResponse(const collisionAlgorithm::DetectionOutput::SPtr d) {
        ConstraintNormal cn(d->getNormal());
        return new BilateralConstraintResolution<1>(cn,d,d_maxForce.getValue());
    }
};

class BilateralResponse3 : public BaseConstraint {
public:
    SOFA_CLASS(BilateralResponse3 , BaseConstraint);

    Data<double> d_maxForce;

    BilateralResponse3()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    ConstraintReponse * createResponse(const collisionAlgorithm::DetectionOutput::SPtr d) {
        ConstraintNormal cn = ConstraintNormal::createFrame(d->getNormal());
        return new BilateralConstraintResolution<3>(cn,d,d_maxForce.getValue());
    }
};

}

}
