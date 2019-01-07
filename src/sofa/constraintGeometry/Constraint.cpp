#include <sofa/constraintGeometry/Constraint.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintClass = core::RegisterObject("Constraint")
.add< Constraint >();

}

}

