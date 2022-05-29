#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/MechanicalProximity.h>

namespace sofa::constraintGeometry {

template<class DataTypes>
class VectorPointNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(SOFA_TEMPLATE(VectorPointNormalHandler,DataTypes), BaseNormalHandler);

    Data<type::vector<type::Vector3>> d_normals;

    VectorPointNormalHandler()
    : d_normals(initData(&d_normals, "normals", "Vector of normals")) {}

    bool getNormal(collisionAlgorithm::BaseProximity::SPtr prox,type::Vector3 & N) override {
        if (typename collisionAlgorithm::MechanicalProximity<DataTypes>::SPtr tprox =
                std::dynamic_pointer_cast<collisionAlgorithm::MechanicalProximity<DataTypes> >(prox)) {

            if (tprox->getPId()>=d_normals.getValue().size()) {
                std::cerr << "Wrong id in VectorPointNormalHandler" << std::endl;
                return false;
            }

            N = d_normals.getValue()[tprox->getPId()];
            return true;
        }

        std::cerr << "Error the proximity is not a MechanicalProximity in VectorPointNormalHandler" << std::endl;
        return false;
    }

    void prepareDetection() override {}

};


}
