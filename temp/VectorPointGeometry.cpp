#include <sofa/collisionAlgorithm/geometry/VectorPointGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(VectorPointGeometry)

int VectorPointGeometryClass = core::RegisterObject("Default implementation of normal computation for point geometry")
.add< VectorPointGeometry<sofa::defaulttype::Vec3dTypes> >();

}
