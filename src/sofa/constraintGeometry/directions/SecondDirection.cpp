#include <sofa/constraintGeometry/directions/SecondDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

SOFA_DECL_CLASS(SecondDirection);

int SecondDirectionClass = core::RegisterObject("SecondDirection")
.add< SecondDirection >();

}

}

