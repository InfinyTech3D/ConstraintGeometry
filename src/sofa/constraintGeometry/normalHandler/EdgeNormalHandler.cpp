#include <sofa/constraintGeometry/normalHandler/EdgeNormalHandler.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/constraintGeometry/operations/ConstraintProximityOperation.h>

namespace sofa::constraintGeometry {

SOFA_DECL_CLASS(EdgeNormalHandler)

int EdgeNormalHandlerClass = core::RegisterObject("Default implementation of normal computations return normal in the direction of the edge")
.add< EdgeNormalHandler>();

int edge_reg = ConstraintProximityOperation::register_func<EdgeNormalHandler,collisionAlgorithm::EdgeProximity>
(&EdgeNormalHandler::buildCstProximity<collisionAlgorithm::EdgeProximity>);

}
