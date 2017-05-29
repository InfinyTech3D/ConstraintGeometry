#ifndef SOFA_COMPONENT_TRIANGLEGEOMETRY_INL
#define SOFA_COMPONENT_TRIANGLEGEOMETRY_INL

#include "ConstraintProximity.h"
#include "TriangleGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {

ConstraintProximity TriangleGeometry::getTriangleProximity(unsigned eid,double fact_w,double fact_u,double fact_v) {
    ConstraintProximity res(this,eid);
    const sofa::core::topology::BaseMeshTopology::Triangle tri = this->getTopology()->getTriangle(eid);

    res.push(tri[0],fact_w);
    res.push(tri[1],fact_u);
    res.push(tri[2],fact_v);

    return res;
}


void TriangleGeometry::prepareDetection() {
    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    m_triangle_info.resize(this->getTopology()->getNbTriangles());

    for (int t=0;t<this->getTopology()->getNbTriangles();t++) {
        TriangleInfo & tinfo = m_triangle_info[t];

        const sofa::core::topology::BaseMeshTopology::Triangle tri = this->getTopology()->getTriangle(t);

        //Compute Bezier Positions
        defaulttype::Vector3 p0 = x[tri[0]];
        defaulttype::Vector3 p1 = x[tri[1]];
        defaulttype::Vector3 p2 = x[tri[2]];

        tinfo.v0 = p1 - p0;
        tinfo.v1 = p2 - p0;

        tinfo.d00 = dot(tinfo.v0, tinfo.v0);
        tinfo.d01 = dot(tinfo.v0, tinfo.v1);
        tinfo.d11 = dot(tinfo.v1, tinfo.v1);

        tinfo.invDenom = 1.0 / (tinfo.d00 * tinfo.d11 - tinfo.d01 * tinfo.d01);

        tinfo.ax1 = tinfo.v0;
        tinfo.tn = cross(tinfo.v0,tinfo.v1);
        tinfo.ax2 = cross(tinfo.v0,tinfo.tn);

        tinfo.ax1.normalize();
        tinfo.tn.normalize();
        tinfo.ax2.normalize();
    }

    m_pointNormal.resize(x.size());
    for (unsigned p=0;p<x.size();p++) {
        const core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->getTopology()->getTrianglesAroundVertex(p);
        m_pointNormal[p] = defaulttype::Vector3(0,0,0);
        for (unsigned t=0;t<tav.size();t++) {
            m_pointNormal[p] += this->m_triangle_info[tav[t]].tn;
        }
        m_pointNormal[p].normalize();
    }
}

//proj_P must be on the plane

void TriangleGeometry::computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) {
    defaulttype::Vector3 v2 = proj_P - p0;

    double d20 = dot(v2, tinfo.v0);
    double d21 = dot(v2, tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0 - fact_v  - fact_w;
}

int TriangleGeometry::getNbElements() {
    return this->getTopology()->getNbTriangles();
}

//Barycentric coordinates are computed according to
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates

double TriangleGeometry::projectPoint(unsigned tid,const defaulttype::Vector3 & s,ConstraintProximity & pinfo) {
    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    const sofa::core::topology::BaseMeshTopology::Triangle tri = this->getTopology()->getTriangle(tid);

    //Compute Bezier Positions
    defaulttype::Vector3 p0 = x[tri[0]];
    defaulttype::Vector3 p1 = x[tri[1]];
    defaulttype::Vector3 p2 = x[tri[2]];

    defaulttype::Vector3 x1x2 = s - p0;

    const TriangleInfo & tinfo = m_triangle_info[tid];

    //corrdinate on the plane
    double c0 = dot(x1x2,tinfo.ax1);
    double c1 = dot(x1x2,tinfo.ax2);
    defaulttype::Vector3 proj_P = p0 + c0 * tinfo.ax1 + c1 * tinfo.ax2;

    double fact_u;
    double fact_v;
    double fact_w;

    computeBaryCoords(proj_P, tinfo, p0, fact_u,fact_v,fact_w);

    if (fact_u<0) {
        defaulttype::Vector3 v3 = p1 - p2;
        defaulttype::Vector3 v4 = proj_P - p2;
        double alpha = dot (v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 0;
        fact_v = alpha;
        fact_w = 1.0 - alpha;
    } else if (fact_v<0) {
        defaulttype::Vector3 v3 = p0 - p2;
        defaulttype::Vector3 v4 = proj_P - p2;
        double alpha = dot (v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = alpha;
        fact_v = 0;
        fact_w = 1.0 - alpha;
    } else if (fact_w<0) {
        defaulttype::Vector3 v3 = p1 - p0;
        defaulttype::Vector3 v4 = proj_P - p0;
        double alpha = dot (v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 1.0 - alpha;
        fact_v = alpha;
        fact_w = 0;
    }

    pinfo = getTriangleProximity(tid,fact_u,fact_v,fact_w);

    return (pinfo.getPosition() - s).norm();
}


defaulttype::Vector3 TriangleGeometry::getNormal(const ConstraintProximity & pinfo) {
//    if (pinfo.m_fact[0] <= 0 || pinfo.m_fact[1] <= 0 || pinfo.m_fact[2] <= 0 || pinfo.m_fact[0] >= 1 || pinfo.m_fact[1] >= 1 || pinfo.m_fact[2] >= 0) {
//        return m_pointNormal[pinfo.m_pid[0]] * pinfo.m_fact[0] +
//               m_pointNormal[pinfo.m_pid[1]] * pinfo.m_fact[1] +
//               m_pointNormal[pinfo.m_pid[2]] * pinfo.m_fact[2];
//    } else {
      return m_triangle_info[pinfo.getEid()].tn;
//    }
}


void TriangleGeometry::drawTriangle(const core::visual::VisualParams * vparams,const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C) {
    glColor3f(1,0.45,0);helper::gl::glVertexT(A);
    glColor3f(1,0.40,0);helper::gl::glVertexT(B); // A<->B

    if (vparams->displayFlags().getShowWireFrame()) {glColor3f(1,0.40,0);helper::gl::glVertexT(B);} //A<->B
    glColor3f(1,0.48,0);helper::gl::glVertexT(C);

    if (vparams->displayFlags().getShowWireFrame()) {glColor3f(1,0.48,0);helper::gl::glVertexT(C);} // C<->A
    if (vparams->displayFlags().getShowWireFrame()) {glColor3f(1,0.45,0);helper::gl::glVertexT(A);}// C<->A
}


void TriangleGeometry::draw(const core::visual::VisualParams * vparams) {

    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glDisable(GL_LIGHTING);

    if (vparams->displayFlags().getShowWireFrame()) glBegin(GL_LINES);
    else {
        glEnable(GL_CULL_FACE);
        glBegin(GL_TRIANGLES);
    }

    for(int t=0;t<this->getTopology()->getNbTriangles();t++) {
        const sofa::core::topology::BaseMeshTopology::Triangle tri = this->getTopology()->getTriangle(t);

        //Compute Bezier Positions
        defaulttype::Vector3 p0 = x[tri[0]];
        defaulttype::Vector3 p1 = x[tri[1]];
        defaulttype::Vector3 p2 = x[tri[2]];

        drawTriangle(vparams,p0,p1,p2);
    }

    glEnd();
}

} //controller

} //component

}//Sofa

#endif
