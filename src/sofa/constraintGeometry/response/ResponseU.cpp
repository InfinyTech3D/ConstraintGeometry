#include <sofa/constraintGeometry/response/ResponseU.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ResponseUClass = core::RegisterObject("ResponseU")
.add< ResponseU >()
.addAlias("ResponseU");

}

}

