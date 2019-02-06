#include <sofa/constraintGeometry/constraint/ConstraintContact.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintUFFClass = core::RegisterObject("ConstraintUFF")
.add< ConstraintUFF >()
.addAlias("ConstraintContact");

}

}

