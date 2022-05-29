#include <sofa/constraintGeometry/normalHandler/GravityPointNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry {

SOFA_DECL_CLASS(GravityPointNormalHandler)

int GravityPointNormalHandlerClass = core::RegisterObject("return the normal between the gravity center of the object and each point")
.add< GravityPointNormalHandler>();

}
