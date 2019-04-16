#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/normals/ContactNormal.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintBilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintBilateral , BaseConstraint);

    Data<double> d_maxForce;
    Data<collisionAlgorithm::DetectionOutput> d_input;

    Data<std::vector<defaulttype::Vec3> > d_directions ;

//    core::objectmodel::SingleLink<
//        ConstraintBilateral,
//        BaseDirectionGenerator,
//        BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_direction_generator;

    ConstraintBilateral()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_input(initData(&d_input, "input", "Link to detection output"))
    , d_directions(initData(&d_directions, "directions", "static constraint directions for each pair of proximities")) {}

    void init () {

    }

    void createConstraints(ConstraintContainer & constraints) {
        //*
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            defaulttype::Vector3 dir = (d.first->getPosition() - d.second->getPosition()).normalized();

            ConstraintNormal CN (dir);

            if (CN.size() == 1) {
                constraints.push_back(this, d, CN, &ConstraintBilateral::createConstraintResolution1);
            } else if (CN.size() == 2) {
                constraints.push_back(this, d, CN, &ConstraintBilateral::createConstraintResolution2);
            } else if (CN.size() == 3) {
                constraints.push_back(this, d, CN, &ConstraintBilateral::createConstraintResolution3);
            }
        }
        //*/
    }

protected:
    core::behavior::ConstraintResolution* createConstraintResolution1(const InternalConstraint * /*cst*/) const {
        return new BilateralConstraintResolution1(d_maxForce.getValue());
    }

    core::behavior::ConstraintResolution* createConstraintResolution2(const InternalConstraint * /*cst*/) const {
        return new BilateralConstraintResolution2(d_maxForce.getValue());
    }

    core::behavior::ConstraintResolution* createConstraintResolution3(const InternalConstraint * /*cst*/) const {
        return new BilateralConstraintResolution3(d_maxForce.getValue());
    }
};

}

}
