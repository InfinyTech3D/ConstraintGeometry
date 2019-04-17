#include <sofa/constraintGeometry/constraint/ConstraintBilateral.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int TestConstraintBilateralClass = core::RegisterObject("TestConstraintBilateral")
.add< TestConstraintBilateral >();

int ConstraintBilateralClass = core::RegisterObject("ConstraintBilateral")
.add< ConstraintBilateral >();

}

}

