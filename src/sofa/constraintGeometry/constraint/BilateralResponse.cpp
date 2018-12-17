#include <sofa/constraintGeometry/constraint/BilateralResponse.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintResponseB1Class = core::RegisterObject("BilateralResponse1")
.add< BilateralResponse1 >()
.addAlias("ConstraintB")    ;

int ConstraintResponseB3Class = core::RegisterObject("BilateralResponse3")
.add< BilateralResponse3 >()
.addAlias("ConstraintBBB");

}

}

