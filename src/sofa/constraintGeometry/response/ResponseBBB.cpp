#include <sofa/constraintGeometry/response/ResponseBBB.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintResponseB3Class = core::RegisterObject("ResponseBBB")
.add< ResponseBBB >()
.addAlias("BilateralResponse3");

}

}

