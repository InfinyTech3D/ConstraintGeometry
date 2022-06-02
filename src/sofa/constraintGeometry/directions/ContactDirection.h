#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The ContactDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ContactDirection : public ConstraintDirection {
public:
    SOFA_CLASS(ContactDirection , ConstraintDirection);

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const ConstraintPairsOutput::ConstraintPairs & d) const override {
        type::Vector3 mainDir = (d.first->getPosition() - d.second->getPosition()).normalized();

        type::Vector3 secondDir = d.second->getNormal();

        if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;

        return ConstraintNormal(mainDir);
    }


};

}
