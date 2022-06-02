#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/constraintProximities/GravityPointConstraintProximity.h>

namespace sofa::constraintGeometry {

class GravityPointNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(GravityPointNormalHandler, BaseNormalHandler);

    core::objectmodel::SingleLink<GravityPointNormalHandler,collisionAlgorithm::BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    GravityPointNormalHandler()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}


    void prepareDetection() override {
        m_gcenter = type::Vector3();

        unsigned nbPoints = 0;
        for (auto it=l_geometry->pointBegin();it!=l_geometry->end();it++) {
            m_gcenter += it->element()->pointElements()[0]->getP0()->getPosition();
            nbPoints++;
        }

        if (nbPoints) m_gcenter*=1.0/nbPoints;
    }

    static ConstraintProximity::SPtr buildConstraintProximity(GravityPointConstraintProximity::SPtr prox) {
//        return ConstraintProximity::SPtr(new GravityPointConstraintProximity(gpprox,m_gcenter));
    }

    const std::type_info & getTypeInfo() override { return typeid(GravityPointNormalHandler); }

private :
    type::Vector3 m_gcenter;
};


}
