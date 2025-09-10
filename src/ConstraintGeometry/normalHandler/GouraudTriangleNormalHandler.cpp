#include <ConstraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry
{

void registerGouraudTriangleNormalHandler(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("").add<GouraudTriangleNormalHandler>());
}

int gouraud_reg_tri =
    ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler,
                                                collisionAlgorithm::TriangleProximity>(
        &GouraudTriangleNormalHandler::buildCstProximity<collisionAlgorithm::TriangleProximity>);

int gouraud_reg_mech = ConstraintProximityOperation::register_func<
    GouraudTriangleNormalHandler,
    collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(
    &GouraudTriangleNormalHandler::buildCstProximity<
        collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>);

}  // namespace sofa::constraintGeometry
