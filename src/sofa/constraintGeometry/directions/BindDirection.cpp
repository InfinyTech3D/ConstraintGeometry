#include <sofa/constraintGeometry/directions/BindDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int BindDirectionClass = core::RegisterObject("BindDirection")
.add< BindDirection >();

}

}

