#include <sofa/constraintGeometry/constraint/UnilateralConstraint.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int UnilateralConstraintClass = core::RegisterObject("UnilateralConstraint")
.add< UnilateralConstraint >()
.addAlias("ConstraintU");

}

}

