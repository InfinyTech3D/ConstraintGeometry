#include <sofa/constraintGeometry/constraint/ConstraintU.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintUClass = core::RegisterObject("ConstraintU")
.add< ConstraintU >()
.addAlias("ConstraintU");

}

}

