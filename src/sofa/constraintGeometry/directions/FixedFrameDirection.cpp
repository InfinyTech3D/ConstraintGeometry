#include <sofa/constraintGeometry/directions/FixedFrameDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int FixedFrameDirectionClass = core::RegisterObject("FixedFrameDirection")
.add< FixedFrameDirection >();

}

}

