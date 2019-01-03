#pragma once

#include <sofa/constraintGeometry/response/UnilateralResponse.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ContactResponseUFF : public BaseConstraint {
public:
    SOFA_CLASS(ContactResponseUFF , BaseConstraint);

    Data<double> d_maxForce;
    Data<double> d_friction;

    ContactResponseUFF()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "friction", "Friction")) {}

    virtual ConstraintReponse * createResponse(const collisionAlgorithm::DetectionOutput & d) {
        if (d_friction.getValue() == 0.0) {
            ConstraintNormal cn(d.getNormal());

            return new UnilateralConstraintResolution(cn,
                                                      d.getFirstProximity(),
                                                      d.getSecondProximity(),
                                                      d_maxForce.getValue());
        } else {
            ConstraintNormal cn = ConstraintNormal::createFrame(d.getNormal());

            return new UnilateralFrictionResolution(cn,
                                                    d.getFirstProximity(),
                                                    d.getSecondProximity(),
                                                    d_maxForce.getValue(),
                                                    d_friction.getValue());
        }
    }
};

class ContactResponseU : public BaseConstraint {
public:
    SOFA_CLASS(ContactResponseU , BaseConstraint);

    Data<double> d_maxForce;

    ContactResponseU()
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
