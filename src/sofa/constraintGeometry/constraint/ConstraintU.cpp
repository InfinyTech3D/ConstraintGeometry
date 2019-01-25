#include <sofa/constraintGeometry/constraint/ConstraintU.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintUnilateralClass = core::RegisterObject("ConstraintUnilateral")
.add< ConstraintUnilateral >()
.addAlias("ConstraintUnilateralnilateral");

}

}

