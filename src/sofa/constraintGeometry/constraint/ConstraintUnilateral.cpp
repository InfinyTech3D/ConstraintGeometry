#include <sofa/constraintGeometry/constraint/ConstraintUnilateral.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(ConstraintUnilateral)

int ConstraintUnilateralClass = core::RegisterObject("ConstraintUnilateral")
.add< ConstraintUnilateral >();

}

}

