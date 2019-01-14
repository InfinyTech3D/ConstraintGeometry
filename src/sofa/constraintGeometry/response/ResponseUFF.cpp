#include <sofa/constraintGeometry/response/ResponseUFF.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ResponseUFFClass = core::RegisterObject("ResponseUFF")
.add< ResponseUFF >()
.addAlias("ResponseUFF");

}

}

