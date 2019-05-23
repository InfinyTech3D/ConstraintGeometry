#include <sofa/constraintGeometry/directions/FirstDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(FirstDirection)

int FirstDirectionClass = core::RegisterObject("FirstDirection")
.add< FirstDirection >();

}

}

