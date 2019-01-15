#include <sofa/constraintGeometry/constraint/ConstraintFBB.h>
#include <sofa/core/ObjectFactory.h>


namespace sofa {

namespace constraintGeometry {

int ConstraintFBBClass = core::RegisterObject("ConstraintFBB")
.add< ConstraintFBB >();

}

}
