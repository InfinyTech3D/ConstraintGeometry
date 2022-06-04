#include <sofa/constraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/constraintGeometry/ConstraintProximityOperation.h>

namespace sofa::constraintGeometry {

SOFA_DECL_CLASS(GouraudTriangleNormalHandler)

int GouraudTriangleNormalHandlerClass = core::RegisterObject("GouraudTriangleNormalHandler")
.add< GouraudTriangleNormalHandler>();



int gouraud_reg_tri = ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler,collisionAlgorithm::TriangleProximity>
(&GouraudTriangleNormalHandler::buildCstProximity<collisionAlgorithm::TriangleProximity>);

int gouraud_reg_mech = ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler,collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >
(&GouraudTriangleNormalHandler::buildCstProximity<collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>);


}
