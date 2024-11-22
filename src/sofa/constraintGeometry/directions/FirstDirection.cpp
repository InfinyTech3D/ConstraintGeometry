#include <sofa/constraintGeometry/directions/FirstDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int FirstDirectionClass = core::RegisterObject("FirstDirection")
.add< FirstDirection >();

}

}

