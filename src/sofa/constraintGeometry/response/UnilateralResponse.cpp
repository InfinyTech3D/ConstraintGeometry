#include <sofa/constraintGeometry/response/UnilateralResponse.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int UFFResponseClass = core::RegisterObject("UFFResponse")
.add<UFFResponse>();

}

}

