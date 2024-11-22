#include <sofa/constraintGeometry/normalHandler/PhongTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>

namespace sofa::constraintGeometry {

int PhongTriangleNormalHandlerClass = core::RegisterObject("PhongTriangleNormalHandler")
.add< PhongTriangleNormalHandler >();

int phong_reg_tri = ConstraintProximityOperation::register_func<PhongTriangleNormalHandler,collisionAlgorithm::TriangleProximity>
(&PhongTriangleNormalHandler::buildCstProximity<collisionAlgorithm::TriangleProximity> );


int phong_reg_mech = ConstraintProximityOperation::register_func<PhongTriangleNormalHandler,collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >
(&PhongTriangleNormalHandler::buildCstProximity<collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>> );


}
