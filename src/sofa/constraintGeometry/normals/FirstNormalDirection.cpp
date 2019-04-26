#include <sofa/constraintGeometry/normals/FirstNormalDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(FirstNormalDirection);

int FirstNormalDirectionClass = core::RegisterObject("FirstNormalDirection")
.add< FirstNormalDirection >();

}

}

