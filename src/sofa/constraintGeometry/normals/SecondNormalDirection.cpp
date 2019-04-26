#include <sofa/constraintGeometry/normals/SecondNormalDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(SecondNormalDirection);

int SecondNormalDirectionClass = core::RegisterObject("SecondNormalDirection")
.add< SecondNormalDirection >();

}

}

