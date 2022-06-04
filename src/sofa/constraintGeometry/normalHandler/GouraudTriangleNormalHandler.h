#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/proximity/MechanicalProximity.h>

namespace sofa::constraintGeometry {

class GouraudTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(GouraudTriangleNormalHandler, BaseNormalHandler);

    void prepareDetection() override {}

    const std::type_info & getTypeInfo() override { return typeid(GouraudTriangleNormalHandler); }

    template<class PROXIMITY>
    type::Vector3 getNormal(const typename PROXIMITY::SPtr & prox);

    template<class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(GouraudTriangleNormalHandler * handler, typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(prox,std::bind(&GouraudTriangleNormalHandler::getNormal<PROXIMITY>,handler,std::placeholders::_1));
    }
};

template<>
inline type::Vector3 GouraudTriangleNormalHandler::getNormal<collisionAlgorithm::TriangleProximity>(const collisionAlgorithm::TriangleProximity::SPtr & prox) {
    return prox->element()->getTriangleInfo().N;
}


template<>
inline type::Vector3 GouraudTriangleNormalHandler::getNormal<collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>>(const collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>::SPtr & prox) {
    return type::Vector3(0,1,0);//m_prox->element()->getTriangleInfo().N;
}

}
