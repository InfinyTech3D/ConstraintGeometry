#include <sofa/constraintGeometry/directions/FixedFrameDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(FixedFrameDirection);

int FixedFrameDirectionClass = core::RegisterObject("FixedFrameDirection")
.add< FixedFrameDirection >();

}

}

