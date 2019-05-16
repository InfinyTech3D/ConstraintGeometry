#include <sofa/constraintGeometry/directions/BindDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(BindDirection);

int BindDirectionClass = core::RegisterObject("BindDirection")
.add< BindDirection >();

}

}

