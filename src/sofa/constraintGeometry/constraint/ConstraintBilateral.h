#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/normals/DataConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintBilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintBilateral , BaseConstraint);

    Data<double> d_maxForce;
    Data<collisionAlgorithm::DetectionOutput> d_input;
    Data<ConstraintDirection> d_direction;

    ConstraintBilateral()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_input(initData(&d_input, "input", "Link to detection output"))
    , d_direction(initData(&d_direction, ConstraintDirection(std::bind(&ConstraintBilateral::defaultGetNormals, this, std::placeholders::_1)), "directions", "Link to detection output")){}

    void createConstraints(ConstraintContainer & constraints) {
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            ConstraintNormal CN = d_direction.getValue().getConstraintNormal(d);

            constraints.push_back(d, CN, std::bind(&ConstraintBilateral::createConstraintResolution,this,std::placeholders::_1));
        }
    }

    ConstraintNormal defaultGetNormals(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return ConstraintNormal(
            (d.first->getPosition() - d.second->getPosition())
            .normalized()
        );
    }

    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint * cst) const {
        if (cst->size() == 1) return new BilateralConstraintResolution1(d_maxForce.getValue());
        else if (cst->size() == 2) return new BilateralConstraintResolution2(d_maxForce.getValue());
        else if (cst->size() == 3) return new BilateralConstraintResolution3(d_maxForce.getValue());
        std::cerr << "Error the size of the constraint is not correct in ConstraintBilateral" << std::endl;
        return NULL;
    }
};

}

}
