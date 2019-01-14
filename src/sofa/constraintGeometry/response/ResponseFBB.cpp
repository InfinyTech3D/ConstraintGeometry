#include <sofa/constraintGeometry/response/ResponseFBB.h>
#include <sofa/core/ObjectFactory.h>


namespace sofa {

namespace constraintGeometry {

int ResponseFBBClass = core::RegisterObject("ResponseFBB")
.add< ResponseFBB >();

}

}
