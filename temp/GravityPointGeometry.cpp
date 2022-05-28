#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/geometry/GravityPointGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(GravityPointGeometry)

int GravityPointGeometryClass = core::RegisterObject("return the normal between the gravity center of the object and each point")
.add< GravityPointGeometry<sofa::defaulttype::Vec3dTypes> >();

}
