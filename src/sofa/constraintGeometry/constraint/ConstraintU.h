#pragma once

#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintUnilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintUnilateral , BaseConstraint);

    Data<double> d_maxForce;

    ConstraintUnilateral()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

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

    core::behavior::ConstraintResolution* createConstraintResolution() const {
        return new UnilateralConstraintResolution(d_maxForce.getValue());
    }


    virtual InternalConstraint::SPtr createConstraint(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return InternalConstraint::create(this, d, ConstraintNormal(getUnilateralNormal(d)), &ConstraintUnilateral::createConstraintResolution);
    }

};

}

}
