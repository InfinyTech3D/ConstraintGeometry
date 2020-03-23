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

    ConstraintNormal UpdateConstraintsNormalWithProximityPosition(const collisionAlgorithm::PairDetection & d, defaulttype::Vec3 pf, bool getF, defaulttype::Vec3 pd, bool getD) const override {
        if(getF&&getD){
            return ConstraintNormal(pf - pd);
        }
        else if(getF){
            return ConstraintNormal(pf - d.second->getPosition());
        }
        else if(getD){
            return ConstraintNormal(d.first->getPosition() - pd);
        }
        else return ConstraintNormal(defaulttype::Vector3(0, 0, 0));

    }

};

}

}
