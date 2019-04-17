#pragma once

#include <sofa/constraintGeometry/resolution/BilateralResolution.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/normals/ContactNormal.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <math.h>
#include <memory>
#include <functional>
#include <iostream>
#include <algorithm>

namespace sofa {

namespace constraintGeometry {


class DataConstraintNormal : public helper::vector<defaulttype::Vector3> {
public :
    typedef std::function<ConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d)> GeneratorFunction;
    typedef std::pair<collisionAlgorithm::BaseProximity::SPtr,collisionAlgorithm::BaseProximity::SPtr> PairDetection;

    // std::placeholders::_1 is the first parameter
    DataConstraintNormal(GeneratorFunction fct = std::bind(&defaultGetNormals, std::placeholders::_1)) : m_functor(fct) {}

    ConstraintNormal getConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d) const {
        if (size() == 0) return m_functor(d); // return the functor value
        return ConstraintNormal(*this); //use helper::vector Constructor
    }

private:
    static ConstraintNormal defaultGetNormals(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return ConstraintNormal((d.first->getPosition() - d.second->getPosition()).normalized());
    }

    GeneratorFunction m_functor;
};

class TestConstraintBilateral : public sofa::core::objectmodel::BaseObject {
public:
    Data<DataConstraintNormal> d_direction;

    TestConstraintBilateral()
    : d_direction(initData(&d_direction, DataConstraintNormal(std::bind(advancedGetNormals, std::placeholders::_1)), "directions", "Link to detection output")){}

private:
    //this is a specific function that will replace the default behavior. It can be defined anywhere (even eventually be a lambda function)
    static ConstraintNormal advancedGetNormals(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        std::cout << "Advanced" << std::endl;
        return ConstraintNormal((d.first->getPosition() - d.second->getPosition()).normalized());
    }
};

class ConstraintBilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintBilateral , BaseConstraint);

    Data<double> d_maxForce;
    Data<collisionAlgorithm::DetectionOutput> d_input;
    Data<DataConstraintNormal> d_direction;

    ConstraintBilateral()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_input(initData(&d_input, "input", "Link to detection output"))
    , d_direction(initData(&d_direction, "directions", "Link to detection output")){}

    void createConstraints(ConstraintContainer & constraints) {
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            ConstraintNormal CN = d_direction.getValue().getConstraintNormal(d);

            if (CN.size() == 1) {
                constraints.push_back(this, d, CN, &ConstraintBilateral::createConstraintResolution1);
            } else if (CN.size() == 2) {
                constraints.push_back(this, d, CN, &ConstraintBilateral::createConstraintResolution2);
            } else if (CN.size() == 3) {
                constraints.push_back(this, d, CN, &ConstraintBilateral::createConstraintResolution3);
            }
        }
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
