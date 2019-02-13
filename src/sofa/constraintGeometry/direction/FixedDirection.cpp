#include <sofa/constraintGeometry/BaseDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

class FixedDirection : public BaseDirection {
public:

    SOFA_CLASS(FixedDirection,BaseDirection);

    Data<helper::vector<defaulttype::Vector3> > d_directions;

    FixedDirection()
    : d_directions(initData(&d_directions, "directions", "list of directions")) {}


    ConstraintNormal createConstraintNormal(unsigned /*size*/, const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return ConstraintNormal(d_directions.getValue());
    }

    ConstraintNormal createConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return ConstraintNormal(d_directions.getValue());
    }
};


int FixedDirectionClass = core::RegisterObject("FixedDirection")
.add< FixedDirection >();

}

}

