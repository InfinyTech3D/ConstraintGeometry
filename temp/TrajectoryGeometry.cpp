#include <sofa/collisionAlgorithm/geometry/TrajectoryGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(EdgeNormalHandler)

int TrajectoryGeometryClass = core::RegisterObject("Default implementation of normal computations return normal in the direction of the edge")
.add< TrajectoryGeometry<sofa::defaulttype::Vec3dTypes > >();

}
