#include <sofa/constraintGeometry/constraint/ConstraintBilateral.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(ConstraintBilateral) ;

int ConstraintBilateralClass = core::RegisterObject("ConstraintBilateral")
.add< ConstraintBilateral >();

}

}

