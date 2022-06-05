#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>

namespace sofa::constraintGeometry {

class GravityPointNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(GravityPointNormalHandler, BaseNormalHandler);

    core::objectmodel::SingleLink<GravityPointNormalHandler,collisionAlgorithm::BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    GravityPointNormalHandler()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){
        l_geometry.setPath("@.");
    }

    BaseGeometry * getGeometry() override { return l_geometry.get(); }

    void prepareDetection() override {
        m_gcenter = type::Vector3();

        unsigned nbPoints = 0;
        for (auto it=l_geometry->pointBegin();it!=l_geometry->end();it++) {
            m_gcenter += it->element()->pointElements()[0]->getP0()->getPosition();
            nbPoints++;
        }

        if (nbPoints) m_gcenter*=1.0/nbPoints;
    }

    template<class PROXIMITY>
    type::Vector3 getNormal(const typename PROXIMITY::SPtr & prox);

    const std::type_info & getTypeInfo() override { return typeid(GravityPointNormalHandler); }

    template<class PROXIMITY>
    static inline ConstraintProximity::SPtr buildCstProximity(GravityPointNormalHandler * handler, typename PROXIMITY::SPtr prox) {
        return TConstraintProximity<PROXIMITY>::create(prox,std::bind(&GravityPointNormalHandler::getNormal<PROXIMITY>,handler,std::placeholders::_1));
    }

private :
    type::Vector3 m_gcenter;
};

template<>
inline type::Vector3 GravityPointNormalHandler::getNormal<collisionAlgorithm::PointProximity>(const collisionAlgorithm::PointProximity::SPtr & prox) {
    type::Vector3 N = (prox->getPosition() - m_gcenter).normalized();
    return N;
}

}
