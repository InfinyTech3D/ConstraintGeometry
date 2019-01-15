#include <sofa/constraintGeometry/constraint/ConstraintB.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

int ConstraintBClass = core::RegisterObject("ConstraintB")
.add< ConstraintB >()
.addAlias("BilateralConstraint")    ;

}

}

