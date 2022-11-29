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
class ConstraintUnilateral : public TBaseConstraint<collisionAlgorithm::BaseProximity,collisionAlgorithm::BaseProximity> {
public:
    SOFA_CLASS(ConstraintUnilateral , SOFA_TEMPLATE2(TBaseConstraint,collisionAlgorithm::BaseProximity,collisionAlgorithm::BaseProximity) );

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

    ConstraintNormal createConstraintNormal(const BaseProximity::SPtr & first, const BaseProximity::SPtr & second) const override {
        if (l_directions==NULL) return ConstraintNormal();

        ConstraintNormal CN = l_directions->createConstraintsNormal(first,second);

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
    core::behavior::ConstraintResolution* createConstraintResolution(const BaseInternalConstraint * cst) const override {
        if (cst->size() == 1) return new UnilateralConstraintResolution(d_maxforce0.getValue());
        else if (cst->size() == 3) return new UnilateralFrictionResolution(d_friction.getValue(),d_maxforce0.getValue(),d_maxforce1.getValue(),d_maxforce2.getValue());
        std::cerr << "Error the size of the constraint is not correct in ConstraintUnilateral size=" << cst->size() << std::endl;
        return NULL;
    }

};

}

}
