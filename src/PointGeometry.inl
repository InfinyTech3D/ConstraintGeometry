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
    if (d_checkConnectivity.getValue()) {
        ConstraintProximity res(this,eid);
        res.push(m_indices[eid],1.0);
        return res;
    }else {
        ConstraintProximity res(this,eid);
        res.push(eid,1.0);
        return res;
    }
}

void PointGeometry::bwdInit() {
    if (d_checkConnectivity.getValue()) {
        for (int i=0;i<this->getTopology()->getNbPoints();i++) {
            const core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->getTopology()->getTrianglesAroundVertex(i);
            if (tav.size()) m_indices.push_back(i);
        }
    }
}

double PointGeometry::projectPoint(unsigned eid,const defaulttype::Vector3 & T,ConstraintProximity & pinfo) {    
    if (d_checkConnectivity.getValue()) {
        pinfo = getPointProximity(m_indices[eid]);
    }else {
        pinfo = getPointProximity(eid);
    }
    return (pinfo.getPosition() - T).norm();
}

int PointGeometry::getNbElements() {
    if (d_checkConnectivity.getValue()) return m_indices.size();
    else return this->getTopology()->getNbPoints();
}

int PointGeometry::getNbPoints() {

    if (d_checkConnectivity.getValue())  return m_indices.size();
    else return this->getTopology()->getNbPoints();
}

void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    if (d_checkConnectivity.getValue()) {
        for (int i=0;i<m_indices.size();i++) {
            vparams->drawTool()->drawSphere(x[m_indices[i]],0.001);
        }
    }else {
        glColor3f(0.9,0.46,0);
        for (int i=0;i<this->getTopology()->getNbPoints();i++) {
            vparams->drawTool()->drawSphere(x[i],0.001);
        }
    }
}

} //controller

} //component

}//Sofa

#endif
