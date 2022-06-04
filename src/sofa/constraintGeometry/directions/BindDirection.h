#pragma once

#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The BindDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class BindDirection : public ConstraintDirection {
public:
    SOFA_CLASS(BindDirection , ConstraintDirection);

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const ConstraintProximity::SPtr & first, const ConstraintProximity::SPtr & second) const override {
        return ConstraintNormal(first->getPosition() - second->getPosition());
    }

};

}
