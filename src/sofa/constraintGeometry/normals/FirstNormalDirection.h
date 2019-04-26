#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/constraintGeometry/normals/DataConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The FirstNormalDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class FirstNormalDirection : public sofa::core::objectmodel::BaseObject{
public:
    SOFA_CLASS(FirstNormalDirection , sofa::core::objectmodel::BaseObject);

    Data<ConstraintDirection> d_direction;

    /*!
     * \brief FirstNormalDirection constructor
     */
    FirstNormalDirection()
        : d_direction(initData(&d_direction, ConstraintDirection(std::bind(&FirstNormalDirection::defaultGetNormals, this, std::placeholders::_1)), "directions", "Link to detection output")){}

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal defaultGetNormals(const collisionAlgorithm::PairDetection & d) {
        return ConstraintNormal (-d.first->getNormal());

    }

};

}

}
