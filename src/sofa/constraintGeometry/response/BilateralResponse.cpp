#include <sofa/constraintGeometry/response/BilateralResponse.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintResponseUClass = core::RegisterObject("BilateralResponse")
.add< BilateralResponse<1> >()
.add< BilateralResponse<2> >()
.add< BilateralResponse<3> >();

}

}

