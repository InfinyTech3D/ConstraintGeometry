#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::constraintGeometry {

class EdgeNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(EdgeNormalHandler, BaseNormalHandler);

    void prepareDetection() override {}

    template<class PROXIMITY>
    type::Vector3 getNormal(const typename PROXIMITY::SPtr & prox);

    const std::type_info & getTypeInfo() override { return typeid(EdgeNormalHandler); }

    template<class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(EdgeNormalHandler * handler, typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(prox,std::bind(&EdgeNormalHandler::getNormal<PROXIMITY>,handler,std::placeholders::_1));
    }
};

template<>
type::Vector3 EdgeNormalHandler::getNormal<collisionAlgorithm::EdgeProximity>(const collisionAlgorithm::EdgeProximity::SPtr & prox) {
    return (prox->element()->getP1()->getPosition() - prox->element()->getP0()->getPosition()).normalized();
}

}
