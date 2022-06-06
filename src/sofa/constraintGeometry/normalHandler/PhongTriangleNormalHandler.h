#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/collisionAlgorithm/proximity/MechanicalProximity.h>

namespace sofa::constraintGeometry {

class PhongTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(PhongTriangleNormalHandler, BaseNormalHandler);

    core::objectmodel::SingleLink<PhongTriangleNormalHandler, BaseGeometry, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    PhongTriangleNormalHandler()
    : l_geometry(initLink("geometry", "link to the second normal handler")) {
        l_geometry.setPath("@.");
    }

    BaseGeometry * getGeometry() override { return l_geometry.get(); }

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

    const collisionAlgorithm::BaseProximity::SPtr & p0 = element->pointElements()[0]->getP0();
    const collisionAlgorithm::BaseProximity::SPtr & p1 = element->pointElements()[1]->getP0();
    const collisionAlgorithm::BaseProximity::SPtr & p2 = element->pointElements()[2]->getP0();

    ConstraintProximityOperation::FUNC operation = ConstraintProximityOperation::get(getTypeInfo(),p0->getTypeInfo());

    type::Vector3 N0_point = operation(this,p0)->getNormal();
    type::Vector3 N1_point = operation(this,p1)->getNormal();
    type::Vector3 N2_point = operation(this,p2)->getNormal();

    type::Vector3 N = N0_point.normalized() * prox->f0() +
                      N1_point.normalized() * prox->f1() +
                      N2_point.normalized() * prox->f2();

    return N;
}



template<>
inline type::Vector3 PhongTriangleNormalHandler::getNormal<collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes> >(const collisionAlgorithm::MechanicalProximity<sofa::defaulttype::Vec3dTypes>::SPtr & prox) {
    const collisionAlgorithm::PointElement::SPtr & element = prox->getGeometry()->pointElements()[prox->getPId()];

    type::Vector3 N0_point;
    for (auto it = element->triangleAround().cbegin();it!=element->triangleAround().cend();it++) {
        N0_point+=(*it)->getTriangleInfo().N;
    }

    return N0_point.normalized();
}

}
