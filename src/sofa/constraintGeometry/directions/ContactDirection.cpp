#include <sofa/constraintGeometry/directions/ContactDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(ContactDirection)

int ContactDirectionClass = core::RegisterObject("ContactDirection")
.add< ContactDirection >();

}

}

