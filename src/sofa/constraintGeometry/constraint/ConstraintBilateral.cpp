#include <sofa/constraintGeometry/constraint/ConstraintBilateral.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintConstraintB3Class = core::RegisterObject("ConstraintBBB")
.add< ConstraintBBB >()
.addAlias("BilateralConstraint3");

}

}

