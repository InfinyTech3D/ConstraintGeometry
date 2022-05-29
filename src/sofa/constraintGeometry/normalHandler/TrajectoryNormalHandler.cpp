#include <sofa/constraintGeometry/normalHandler/TrajectoryNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry {

SOFA_DECL_CLASS(TrajectoryNormalHandler)

int TrajectoryNormalHandlerClass = core::RegisterObject("Default implementation of normal computations return normal in the direction of the edge")
.add< TrajectoryNormalHandler<sofa::defaulttype::Vec3dTypes > >();

}
