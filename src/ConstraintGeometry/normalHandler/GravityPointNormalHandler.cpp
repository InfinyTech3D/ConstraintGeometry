#include <sofa/constraintGeometry/normalHandler/GravityPointNormalHandler.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry
{

void registerGravityPointNormalHandler(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "return the normal between the gravity center of the object and each point")
            .add<GravityPointNormalHandler>());
}

int gravityPoint_reg =
    ConstraintProximityOperation::register_func<GravityPointNormalHandler,
                                                collisionAlgorithm::PointProximity>(
        &GravityPointNormalHandler::buildCstProximity<collisionAlgorithm::PointProximity>);

}  // namespace sofa::constraintGeometry
