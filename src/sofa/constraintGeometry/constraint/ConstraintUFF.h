#pragma once

#include <sofa/constraintGeometry/constraint/ConstraintU.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintUFF : public ConstraintUnilateral {
public:
    SOFA_CLASS(ConstraintUFF , ConstraintUnilateral);

    Data<double> d_friction;

    ConstraintUFF()
    : d_friction(initData(&d_friction, 0.0, "mu", "Friction")) {}

    core::behavior::ConstraintResolution* createConstraintResolutionWithFriction(const InternalConstraint * /*cst*/) const {
        return new UnilateralFrictionResolution(d_maxForce.getValue(),d_friction.getValue());
    }

    virtual void createConstraints(ConstraintContainer & constraints) {
        collisionAlgorithm::DetectionOutput output;
        l_algo->doDetection(l_from.get(),l_dest.get(), output);

        if (d_friction.getValue() == 0.0) {
            for (unsigned i=0;i<output.size();i++) {
                const collisionAlgorithm::DetectionOutput::PairDetection & d = output[i];
                ConstraintNormal CN = ConstraintNormal(getUnilateralNormal(d));
                constraints.add(this, d, CN, &ConstraintUFF::createConstraintResolution);
            }
        } else {
            for (unsigned i=0;i<output.size();i++) {
                const collisionAlgorithm::DetectionOutput::PairDetection & d = output[i];
                ConstraintNormal CN = ConstraintNormal::createFrame(getUnilateralNormal(d));
                constraints.add(this, d, CN, &ConstraintUFF::createConstraintResolutionWithFriction);
            }
        }
    }


};

}

}
