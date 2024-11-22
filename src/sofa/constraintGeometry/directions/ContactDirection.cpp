#include <sofa/constraintGeometry/directions/ContactDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ContactDirectionClass = core::RegisterObject("ContactDirection")
.add< ContactDirection >();

}

}

