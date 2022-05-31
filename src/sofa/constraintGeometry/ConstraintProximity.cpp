#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>
#include <sofa/constraintGeometry/normalHandler/PhongTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry {

int gouraud_reg = ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler>(&GouraudTriangleNormalHandler::buildConstraintProximity);

int phong_reg = ConstraintProximityOperation::register_func<PhongTriangleNormalHandler>(&PhongTriangleNormalHandler::buildConstraintProximity);

}
