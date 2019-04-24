#include <sofa/core/ObjectFactory.h>
#include <sofa/constraintGeometry/normals/ProjectiveNormals.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(ProjectiveNormals);

int ProjectiveNormalsClass = core::RegisterObject("ProjectiveNormals")
.add< ProjectiveNormals >();

}

}
