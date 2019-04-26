#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/constraintGeometry/normals/DataConstraintDirection.h>
#include <sofa/constraintGeometry/normals/ContactNormal.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The SecondNormalDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class SecondNormalDirection : public sofa::core::objectmodel::BaseObject{
public:
    SOFA_CLASS(SecondNormalDirection , sofa::core::objectmodel::BaseObject);

    Data<ConstraintDirection> d_direction;

    /*!
     * \brief SecondNormalDirection constructor
     */
    SecondNormalDirection()
        : d_direction(initData(&d_direction, ConstraintDirection(std::bind(&SecondNormalDirection::defaultGetNormals, this, std::placeholders::_1)), "directions", "Link to detection output")){}

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal defaultGetNormals(const collisionAlgorithm::PairDetection & d) {
        return ConstraintNormal (d.second->getNormal());

    }

};

}

}
