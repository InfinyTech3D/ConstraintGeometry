#pragma once

#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The FirstDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class FirstDirection : public ConstraintDirection {
public:
    SOFA_CLASS(FirstDirection , ConstraintDirection);

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & d) const override {
        return ConstraintNormal(-d.first->getNormal());
    }

};

}

}
