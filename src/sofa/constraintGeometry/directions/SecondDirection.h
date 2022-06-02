#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

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
    ConstraintNormal createConstraintsNormal(const ConstraintPairsOutput::ConstraintPairs & d) const override {
        return ConstraintNormal(d.second->getNormal());
    }

};

}
