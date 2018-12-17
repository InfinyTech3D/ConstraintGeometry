#include <sofa/constraintGeometry/constraint/ContactResponse.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ContactResponseUClass = core::RegisterObject("ContactUResponse")
.add< ContactResponseU >()
.addAlias("ConstraintU");

int ContactResponseClass = core::RegisterObject("ContactUFFResponse")
.add< ContactResponseUFF >()
.addAlias("ConstraintUFF");

}

}

