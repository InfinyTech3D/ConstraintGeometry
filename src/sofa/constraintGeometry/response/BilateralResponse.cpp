#include <sofa/constraintGeometry/response/BilateralResponse.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintResponseB1Class = core::RegisterObject("BilateralResponse1")
.add< BilateralResponse1 >()
.addAlias("ResponseB")    ;

int ConstraintResponseB3Class = core::RegisterObject("BilateralResponse3")
.add< BilateralResponse3 >()
.addAlias("ResponseBBB");

}

}

