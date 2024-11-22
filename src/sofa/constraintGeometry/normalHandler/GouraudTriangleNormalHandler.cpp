#include <sofa/constraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>

namespace sofa::constraintGeometry {

int GouraudTriangleNormalHandlerClass = core::RegisterObject("GouraudTriangleNormalHandler")
.add< GouraudTriangleNormalHandler>();

int gouraud_reg_tri = ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler,collisionAlgorithm::TriangleProximity>
(&GouraudTriangleNormalHandler::buildCstProximity<collisionAlgorithm::TriangleProximity>);

int gouraud_reg_mech = ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler,collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >
(&GouraudTriangleNormalHandler::buildCstProximity<collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>);


}
