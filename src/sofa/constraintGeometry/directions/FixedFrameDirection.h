#pragma once

#include <sofa/constraintGeometry/ConstraintDirection.h>
#include <sofa/type/Vec.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The BindDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class FixedFrameDirection : public ConstraintDirection {
public:
    SOFA_CLASS(FixedFrameDirection , ConstraintDirection);

//    Data<std::vector<type::Vec3> > d_frame ;

    FixedFrameDirection ()
//        : d_frame (initData(&d_frame, "frame", "frame vectors"))
    {}

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & /*d*/) const override {
        ConstraintNormal CN ;
        return CN.addOrthogonalDirection()
                 .addOrthogonalDirection()
                 .addOrthogonalDirection();
    }

};

}

}
