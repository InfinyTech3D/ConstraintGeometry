#ifndef SOFA_COMPONENT_POINTGEOMETRY_INL
#define SOFA_COMPONENT_POINTGEOMETRY_INL

#include "ConstraintGeometry.inl"
#include "PointGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "ConstraintProximity.h"
#include <GL/gl.h>

namespace sofa {

namespace core {

namespace behavior {

PointGeometry::PointGeometry()
: d_checkConnectivity(initData(&d_checkConnectivity, false, "checkConnectivity", "d_checkConnectivity")) {}

defaulttype::Vector3 PointGeometry::getNormal(const ConstraintProximity & /*pinfo*/) {
    return defaulttype::Vector3();
}

ConstraintProximity PointGeometry::getPointProximity(unsigned eid) {
    ConstraintProximity res(this,eid);
    res.push(m_indices[eid],1.0);
    return res;
}

void PointGeometry::init() {
    if (d_checkConnectivity.getValue()) {
        for (int i=0;i<this->getTopology()->getNbPoints();i++) {
            const core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->getTopology()->getTrianglesAroundVertex(i);
            if (tav.size()) m_indices.push_back(i);
        }
    } else {
        for (int i=0;i<this->getTopology()->getNbPoints();i++) {
            m_indices.push_back(i);
        }
    }
}

double PointGeometry::projectPoint(unsigned eid,const defaulttype::Vector3 & T,ConstraintProximity & pinfo) {
    pinfo = getPointProximity(eid);
    return (pinfo.getPosition() - T).norm();
}

int PointGeometry::getNbElements() {
    return m_indices.size();
}

int PointGeometry::getNbPoints() {
    return m_indices.size();
}

void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glColor3f(0.9,0.46,0);
    for (int i=0;i<m_indices.size();i++) {
        vparams->drawTool()->drawSphere(x[m_indices[i]],0.001);
    }
}

} //controller

} //component

}//Sofa

#endif
