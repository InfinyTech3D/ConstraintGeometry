#pragma once

#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The SecondDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class SecondDirection : public ConstraintDirection {
public:
    SOFA_CLASS(SecondDirection , ConstraintDirection);

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & d) const override {
        return ConstraintNormal(d.second->getNormal());
    }

    ConstraintNormal UpdateConstraintNormalWithProximityPosition(const collisionAlgorithm::PairDetection & d, type::Vec3 /*pf*/, bool /*getF*/, type::Vec3 /*pd*/, bool /*getD*/) const override {
        return createConstraintsNormal(d);
    }
};

}

}
