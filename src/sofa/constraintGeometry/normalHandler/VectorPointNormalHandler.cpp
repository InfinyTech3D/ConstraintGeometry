#include <sofa/constraintGeometry/normalHandler/VectorPointNormalHandler.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>

namespace sofa::constraintGeometry {

SOFA_DECL_CLASS(VectorPointNormalHandler)

int VectorPointNormalHandlerClass = core::RegisterObject("Default implementation of normal computation for point geometry")
.add< VectorPointNormalHandler>();

int vectorPoint_reg = ConstraintProximityOperation::register_func<VectorPointNormalHandler, collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >
(&VectorPointNormalHandler::buildCstProximity<collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >);

}
