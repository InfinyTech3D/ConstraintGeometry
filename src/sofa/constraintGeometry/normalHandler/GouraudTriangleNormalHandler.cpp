#include <sofa/constraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry {

SOFA_DECL_CLASS(GouraudTriangleNormalHandler)

int GouraudTriangleNormalHandlerClass = core::RegisterObject("GouraudTriangleNormalHandler")
.add< GouraudTriangleNormalHandler>();

}
