#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/directions/ContactDirection.h>
#include <sofa/constraintGeometry/constraint/UnilateralResolution.h>

namespace sofa {

namespace constraintGeometry {


/*!
 * \brief The ConstraintUnilateral class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ConstraintUnilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintUnilateral , BaseConstraint);

    Data<double> d_friction;
    core::objectmodel::SingleLink<ConstraintUnilateral,ConstraintDirection, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_directions;

    ConstraintUnilateral()
    : d_friction(initData(&d_friction, 0.0, "mu", "Friction"))
    , l_directions(initLink("directions", "link to the default direction")) {}

    void init() { // make sure we have a direction
        if (this->l_directions == NULL) l_directions = sofa::core::objectmodel::New<ContactDirection>();
        this->addSlave(l_directions.get());
    }

    ConstraintNormal createConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & detection) const override {
        ConstraintNormal CN = l_directions->createConstraintsNormal(detection);

        if (d_friction.getValue() != 0.0) {
            CN.addOrthogonalDirection();
            CN.addOrthogonalDirection();
        }

        return CN;
    }

    /*!
     * \brief createConstraintResolution : factory method for constraint solvers
     * \param cst : InternalConstraint
     * \return (UnilateralConstraint|UnilateralFriction)Resolution => abstract ConstraintResolution ptr
     */
    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint & cst) const {
        if (cst.size() == 1) return new UnilateralConstraintResolution();
        else if (cst.size() == 3) return new UnilateralFrictionResolution(d_friction.getValue());
        std::cerr << "Error the size of the constraint is not correct in ConstraintUnilateral size=" << cst.size() << std::endl;
        return NULL;
    }

};

}

}
