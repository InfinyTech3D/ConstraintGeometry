#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/constraintGeometry/BaseDirection.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintUFF : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintUFF , BaseConstraint);

    Data<double> d_maxForce;
    Data<double> d_friction;
    Data<collisionAlgorithm::DetectionOutput> d_input;
    core::objectmodel::SingleLink<ConstraintUFF,BaseDirection,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_direction;

    ConstraintUFF()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction"))
    , d_input(initData(&d_input, "input", "Link to detection output"))
    , l_direction(initLink("direction", "Link to direction component")){}

    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint * /*cst*/) const {
        return new UnilateralConstraintResolution(d_maxForce.getValue());
    }

    core::behavior::ConstraintResolution* createConstraintResolutionWithFriction(const InternalConstraint * /*cst*/) const {
        return new UnilateralFrictionResolution(d_maxForce.getValue(),d_friction.getValue());
    }

    virtual void createConstraints(ConstraintContainer & constraints) {
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            if (d_friction.getValue() == 0.0) {
                const ConstraintNormal & CN = l_direction->createConstraintNormal(1, d);
                constraints.add(this, d, CN, &ConstraintUFF::createConstraintResolution);
            } else {
                const ConstraintNormal & CN = l_direction->createConstraintNormal(3, d);
                constraints.add(this, d, CN, &ConstraintUFF::createConstraintResolutionWithFriction);
            }
        }
    }


};

}

}
