#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintUFF : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintUFF , BaseConstraint);

    Data<double> d_maxForce;
    Data<double> d_friction;
    Data<collisionAlgorithm::DetectionOutput> d_input;

    ConstraintUFF()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction"))
    , d_input(initData(&d_input, "input", "Link to detection output")) {}

    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint * /*cst*/) const {
        return new UnilateralConstraintResolution(d_maxForce.getValue());
    }

    core::behavior::ConstraintResolution* createConstraintResolutionWithFriction(const InternalConstraint * /*cst*/) const {
        return new UnilateralFrictionResolution(d_maxForce.getValue(),d_friction.getValue());
    }

    defaulttype::Vector3 getUnilateralNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
    //            defaulttype::Vector3 mainDir = d.getFirstProximity()->getPosition() - d.getSecondProximity()->getPosition();

    //////            if (mainDir.norm()>0.00000001) return ConstraintNormal(mainDir);

    //            defaulttype::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
            defaulttype::Vector3 secondDir = d.second->getNormal().normalized();

    //            return ConstraintNormal(mainDir.normalized() + firstDir + secondDir);
    //            return ConstraintNormal(firstDir);
            return secondDir;


    //            defaulttype::Vector3 mainDir = d.getSecondProximity()->getNormal();//pair.first->getPosition() - pair.second->getPosition();
    //            defaulttype::Vector3 secondDir = -d.getFirstProximity()->getNormal();

    //            if (mainDir.norm()<0.01) mainDir = secondDir;
    //            else {
    //                mainDir.normalize();
    //                if (dot(mainDir,secondDir) < 0) mainDir = secondDir;
    //            }

    //            return ConstraintNormal(mainDir);
    }

    virtual void createConstraints(ConstraintContainer & constraints) {
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        if (d_friction.getValue() == 0.0) {
            for (unsigned i=0;i<input.size();i++) {
                const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];
                ConstraintNormal CN = ConstraintNormal(getUnilateralNormal(d));
                constraints.add(this, d, CN, &ConstraintUFF::createConstraintResolution);
            }
        } else {
            for (unsigned i=0;i<input.size();i++) {
                const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];
                ConstraintNormal CN = ConstraintNormal::createFrame(getUnilateralNormal(d));
                constraints.add(this, d, CN, &ConstraintUFF::createConstraintResolutionWithFriction);
            }
        }
    }


};

}

}
