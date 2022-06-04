#include <sofa/constraintGeometry/ConstraintGenerator.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry {

SOFA_DECL_CLASS(ConstraintGenerator>)

int ConstraintGeneratorClass = core::RegisterObject("return the normal between the gravity center of the object and each point")
.add< ConstraintGenerator>();

}
