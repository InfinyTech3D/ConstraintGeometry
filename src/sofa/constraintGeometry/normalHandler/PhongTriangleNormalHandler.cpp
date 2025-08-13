#include <sofa/constraintGeometry/normalHandler/PhongTriangleNormalHandler.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry
{

void registerPhongTriangleNormalHandler(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("").add<PhongTriangleNormalHandler>());
}

int phong_reg_tri =
    ConstraintProximityOperation::register_func<PhongTriangleNormalHandler,
                                                collisionAlgorithm::TriangleProximity>(
        &PhongTriangleNormalHandler::buildCstProximity<collisionAlgorithm::TriangleProximity>);

int phong_reg_mech = ConstraintProximityOperation::register_func<
    PhongTriangleNormalHandler,
    collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(
    &PhongTriangleNormalHandler::buildCstProximity<
        collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>);

}  // namespace sofa::constraintGeometry
