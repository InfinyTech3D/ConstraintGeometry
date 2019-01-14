#include <sofa/constraintGeometry/response/ResponseB.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ResponseBClass = core::RegisterObject("ResponseB")
.add< ResponseB >()
.addAlias("BilateralResponse")    ;

}

}

