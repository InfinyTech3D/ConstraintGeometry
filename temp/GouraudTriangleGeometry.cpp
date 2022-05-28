#include <sofa/collisionAlgorithm/geometry/GouraudTriangleGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(GouraudTriangleGeometry)

int GouraudTriangleGeometryClass = core::RegisterObject("GouraudTriangleGeometry")
.add< GouraudTriangleGeometry<sofa::defaulttype::Vec3dTypes> >();

}
