#include <sofa/constraintGeometry/constraint/Constraintless.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintlessClass = core::RegisterObject("Constraintless")
.add< Constraintless >();

}

}

