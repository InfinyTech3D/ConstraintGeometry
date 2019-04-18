#include <sofa/constraintGeometry/constraint/ConstraintContact.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(ConstraintContact);

int ConstraintContactClass = core::RegisterObject("ConstraintContact")
.add< ConstraintContact >();

}

}

