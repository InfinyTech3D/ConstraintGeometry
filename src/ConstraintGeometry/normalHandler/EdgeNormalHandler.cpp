#include <sofa/constraintGeometry/normalHandler/EdgeNormalHandler.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry
{

void registerEdgeNormalHandler(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("Default implementation of normal computations return "
                                           "normal in the direction of the edge")
            .add<EdgeNormalHandler>());
}

int edge_reg = ConstraintProximityOperation::register_func<EdgeNormalHandler,
                                                           collisionAlgorithm::EdgeProximity>(
    &EdgeNormalHandler::buildCstProximity<collisionAlgorithm::EdgeProximity>);

}  // namespace sofa::constraintGeometry
