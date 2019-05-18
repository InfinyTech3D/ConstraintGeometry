#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/directions/BindDirection.h>
#include <sofa/constraintGeometry/constraint/BilateralResolution.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintBilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintBilateral , BaseConstraint);

    Data<helper::vector<double> > d_maxForce;
    core::objectmodel::SingleLink<ConstraintBilateral,ConstraintDirection, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_directions;

    ConstraintBilateral()
        : d_maxForce(initData(&d_maxForce, "maxForce", "Max force"))
    , l_directions(initLink("directions", "link to the default direction")) {}

    void init() { // make sure we have a direction
        if (this->l_directions == NULL) l_directions = sofa::core::objectmodel::New<BindDirection>();
        this->addSlave(l_directions.get());
    }

    virtual ConstraintNormal createConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & detection) const override {
        return l_directions->createConstraintsNormal(detection);
    }

    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint & cst) const {
        if (cst.size() == 1) return new BilateralConstraintResolution1(d_maxForce.getValue());
        else if (cst.size() == 2) return new BilateralConstraintResolution2(d_maxForce.getValue());
        else if (cst.size() == 3) return new BilateralConstraintResolution3(d_maxForce.getValue());
        std::cerr << "Error the size of the constraint is not correct in ConstraintBilateral size=" << cst.size() << std::endl;
        return NULL;
    }
};

}

}
