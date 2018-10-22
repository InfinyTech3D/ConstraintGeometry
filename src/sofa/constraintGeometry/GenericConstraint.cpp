#include <sofa/constraintGeometry/GenericConstraint.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int GenericConstraintClass = core::RegisterObject("GenericConstraint")
.add<GenericConstraint>();

}

}
