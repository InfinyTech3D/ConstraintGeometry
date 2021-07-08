#pragma once

#include <sofa/constraintGeometry/ConstraintDirection.h>
#include <sofa/defaulttype/Vec.h>

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

    ConstraintNormal UpdateConstraintNormalWithProximityPosition(const collisionAlgorithm::PairDetection & d, type::Vec3 /*pf*/, bool /*getF*/, type::Vec3 /*pd*/, bool /*getD*/) const override {
        return createConstraintsNormal(d);
    }

};

}

}
