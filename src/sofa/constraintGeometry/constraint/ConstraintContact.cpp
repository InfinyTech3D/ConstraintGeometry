#include <sofa/constraintGeometry/constraint/ConstraintContact.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintContactClass = core::RegisterObject("ConstraintContact")
.add< ConstraintContact >();

}

}

