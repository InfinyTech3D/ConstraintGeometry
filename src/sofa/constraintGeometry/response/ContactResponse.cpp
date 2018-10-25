#include <sofa/constraintGeometry/response/ContactResponse.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ContactResponseClass = core::RegisterObject("ContactResponse")
.add< ContactResponseU >()
.add< ContactResponseUFF >();

}

}

