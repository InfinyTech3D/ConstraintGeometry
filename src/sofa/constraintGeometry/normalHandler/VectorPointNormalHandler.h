#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/MechanicalProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/constraintProximities/VectorPointConstraintProximity.h>

namespace sofa::constraintGeometry {

template<class DataTypes>
class VectorPointNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(SOFA_TEMPLATE(VectorPointNormalHandler,DataTypes), BaseNormalHandler);

    Data<type::vector<type::Vector3>> d_normals;

    VectorPointNormalHandler()
    : d_normals(initData(&d_normals, "normals", "Vector of normals")) {}


    void prepareDetection() override {}

    ConstraintProximity::SPtr buildConstraintProximity(collisionAlgorithm::BaseProximity::SPtr prox) override {
        if (typename collisionAlgorithm::MechanicalProximity<DataTypes>::SPtr vpprox = std::dynamic_pointer_cast<collisionAlgorithm::MechanicalProximity<DataTypes>>(prox)) {
            //TODO : return NEW VECTOR_POINT CSTPROX
            return ConstraintProximity::SPtr(new VectorPointConstraintProximity<DataTypes>(vpprox,d_normals.getValue()));
        }

        return NULL;
    }

    const std::type_info & getTypeInfo() override { return typeid(VectorPointNormalHandler); }

};


}
