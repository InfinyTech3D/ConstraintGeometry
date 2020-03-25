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
    Data<double> d_maxforce0;
    Data<double> d_maxforce1;
    Data<double> d_maxforce2;
    core::objectmodel::SingleLink<ConstraintUnilateral,ConstraintDirection, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_directions;

    ConstraintUnilateral()
    : d_friction(initData(&d_friction, 0.0, "mu", "Friction"))
    , d_maxforce0(initData(&d_maxforce0, std::numeric_limits<double>::max(), "maxForce0", "Max force applied on the first axis"))
    , d_maxforce1(initData(&d_maxforce1, std::numeric_limits<double>::max(), "maxForce1", "Max force applied on the second axis"))
    , d_maxforce2(initData(&d_maxforce2, std::numeric_limits<double>::max(), "maxForce2", "Max force applied on the third axis"))
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

    ConstraintNormal UpdateConstraintNormalAndViolationWithProximityPosition(unsigned cid, const collisionAlgorithm::PairDetection & detection, defaulttype::Vec3 pf, bool getF, defaulttype::Vec3 pd, bool getD, defaulttype::BaseVector *delta) const override {
        ConstraintNormal CN = l_directions->UpdateConstraintNormalWithProximityPosition(detection, pf, getF, pd, getD);

        if (d_friction.getValue() != 0.0) {
            CN.addOrthogonalDirection();
            CN.addOrthogonalDirection();
        }

        CN.UpdateConstraintViolationWithProximityPosition(cid, detection, pf, getF, pd, getD, delta);

        return CN;
    }

    /*!
     * \brief createConstraintResolution : factory method for constraint solvers
     * \param cst : InternalConstraint
     * \return (UnilateralConstraint|UnilateralFriction)Resolution => abstract ConstraintResolution ptr
     */
    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint & cst) const {
        if (cst.size() == 1) return new UnilateralConstraintResolution();
        else if (cst.size() == 3) return new UnilateralFrictionResolution(d_friction.getValue(),d_maxforce0.getValue(),d_maxforce1.getValue(),d_maxforce2.getValue());
        std::cerr << "Error the size of the constraint is not correct in ConstraintUnilateral size=" << cst.size() << std::endl;
        return NULL;
    }

};

}

}
