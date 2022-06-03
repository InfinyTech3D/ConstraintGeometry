#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/collisionAlgorithm/proximity/MechanicalProximity.h>

namespace sofa::constraintGeometry {

class PhongTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(PhongTriangleNormalHandler, BaseNormalHandler);

    void prepareDetection() override {}

    const std::type_info & getTypeInfo() override { return typeid(PhongTriangleNormalHandler); }

    template<class PROXIMITY>
    type::Vector3 getNormal(const typename PROXIMITY::SPtr & prox);

    template<class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(PhongTriangleNormalHandler * handler, typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(prox,std::bind(&PhongTriangleNormalHandler::getNormal<PROXIMITY>,handler,std::placeholders::_1));
    }
};

template<>
inline type::Vector3 PhongTriangleNormalHandler::getNormal<collisionAlgorithm::TriangleProximity>(const collisionAlgorithm::TriangleProximity::SPtr & prox) {
    auto element = prox->element();

    collisionAlgorithm::PointElement::SPtr p0 = element->pointElements()[0];
    collisionAlgorithm::PointElement::SPtr p1 = element->pointElements()[1];
    collisionAlgorithm::PointElement::SPtr p2 = element->pointElements()[2];

    type::Vector3 N0_point;
    for (auto it = p0->triangleAround().cbegin();it!=p0->triangleAround().cend();it++) {
        N0_point+=(*it)->getTriangleInfo().N;
    }

    type::Vector3 N1_point;
    for (auto it = p1->triangleAround().cbegin();it!=p1->triangleAround().cend();it++) {
        N1_point+=(*it)->getTriangleInfo().N;
    }

    type::Vector3 N2_point;
    for (auto it = p2->triangleAround().cbegin();it!=p2->triangleAround().cend();it++) {
        N2_point+=(*it)->getTriangleInfo().N;
    }

    type::Vector3 N = N0_point.normalized() * prox->f0() +
        N1_point.normalized() * prox->f1() +
        N2_point.normalized() * prox->f2();

    return N;
}



template<>
inline type::Vector3 PhongTriangleNormalHandler::getNormal<collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >(const collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>::SPtr & prox) {
//    auto element = prox->element();

//    collisionAlgorithm::PointElement::SPtr p0 = element->pointElements()[0];
//    collisionAlgorithm::PointElement::SPtr p1 = element->pointElements()[1];
//    collisionAlgorithm::PointElement::SPtr p2 = element->pointElements()[2];

//    type::Vector3 N0_point;
//    for (auto it = p0->triangleAround().cbegin();it!=p0->triangleAround().cend();it++) {
//        N0_point+=(*it)->getTriangleInfo().N;
//    }

//    type::Vector3 N1_point;
//    for (auto it = p1->triangleAround().cbegin();it!=p1->triangleAround().cend();it++) {
//        N1_point+=(*it)->getTriangleInfo().N;
//    }

//    type::Vector3 N2_point;
//    for (auto it = p2->triangleAround().cbegin();it!=p2->triangleAround().cend();it++) {
//        N2_point+=(*it)->getTriangleInfo().N;
//    }

//    type::Vector3 N = N0_point.normalized() * prox->f0() +
//        N1_point.normalized() * prox->f1() +
//        N2_point.normalized() * prox->f2();

//    return N;
    return type::Vector3(0,1,0);//m_prox->element()->getTriangleInfo().N;
}

}
