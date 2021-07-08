#pragma once

#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

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
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & d) const override {
        return ConstraintNormal(d.first->getPosition() - d.second->getPosition());
    }

    ConstraintNormal UpdateConstraintNormalWithProximityPosition(const collisionAlgorithm::PairDetection & d, type::Vec3 pf, bool getF, type::Vec3 pd, bool getD) const override {
        type::Vector3 pfrom = d.first->getPosition();
        type::Vector3 pdest = d.second->getPosition();
        if(getF) pfrom = pf;
        if(getD) pdest = pd;
        return ConstraintNormal(pfrom - pdest);

    }

};

}

}
