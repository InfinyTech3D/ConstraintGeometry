#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

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
    ConstraintNormal createConstraintsNormal(const ConstraintProximity::SPtr & first, const ConstraintProximity::SPtr & ) const override {
        return ConstraintNormal(-first->getNormal());
    }

};

}
