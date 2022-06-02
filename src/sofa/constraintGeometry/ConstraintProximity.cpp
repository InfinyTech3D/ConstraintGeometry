#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/normalHandler/GouraudTriangleNormalHandler.h>
#include <sofa/constraintGeometry/normalHandler/PhongTriangleNormalHandler.h>
#include <sofa/constraintGeometry/normalHandler/EdgeNormalHandler.h>
#include <sofa/constraintGeometry/normalHandler/GravityPointNormalHandler.h>
#include <sofa/constraintGeometry/normalHandler/VectorPointNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::constraintGeometry {

int gouraud_reg = ConstraintProximityOperation::register_func<GouraudTriangleNormalHandler>(&GouraudTriangleNormalHandler::buildConstraintProximity);

int phong_reg = ConstraintProximityOperation::register_func<PhongTriangleNormalHandler>(&PhongTriangleNormalHandler::buildConstraintProximity);

int edge_reg = ConstraintProximityOperation::register_func<EdgeNormalHandler>(&EdgeNormalHandler::buildConstraintProximity);

int gravityPoint_reg = ConstraintProximityOperation::register_func<GravityPointNormalHandler>(&GravityPointNormalHandler::buildConstraintProximity);

int vectorPoint_reg = ConstraintProximityOperation::register_func<VectorPointNormalHandler<sofa::defaulttype::Vec3dTypes>>(&VectorPointNormalHandler<sofa::defaulttype::Vec3dTypes>::buildConstraintProximity);

}
