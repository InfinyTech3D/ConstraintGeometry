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

TriangleGeometry::TriangleGeometry()
: d_phong(initData(&d_phong, false, "phong", "Phong interpolation of the normal")) {

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

void TriangleGeometry::computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const {
    defaulttype::Vector3 v2 = proj_P - p0;

    double d20 = dot(v2, tinfo.v0);
    double d21 = dot(v2, tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0 - fact_v  - fact_w;
}

int TriangleGeometry::getNbTriangles() const {
    return this->getTopology()->getNbTriangles();
}

int TriangleGeometry::getNbElements() const {
    return getNbTriangles();
}

//Barycentric coordinates are computed according to
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates

ConstraintProximityPtr TriangleGeometry::projectPoint(const defaulttype::Vector3 & s,unsigned eid) const {
    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    const sofa::core::topology::BaseMeshTopology::Triangle tri = this->getTopology()->getTriangle(eid);

    //Compute Bezier Positions
    defaulttype::Vector3 p0 = x[tri[0]];
    defaulttype::Vector3 p1 = x[tri[1]];
    defaulttype::Vector3 p2 = x[tri[2]];

    defaulttype::Vector3 x1x2 = s - p0;

    const TriangleInfo & tinfo = m_triangle_info[eid];

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

    return getTriangleProximity(eid, tri[0],fact_u,tri[1],fact_v,tri[2],fact_w);
}

ConstraintProximityPtr TriangleGeometry::getTriangleProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) const {
    if (d_phong.getValue()) return ConstraintProximityPtr (new TrianglePhongConstraintProximity(this, p1,f1,p2,f2,p3,f3));
    else return ConstraintProximityPtr (new TriangleConstraintProximity(this, eid, p1,f1,p2,f2,p3,f3));
}

void TriangleGeometry::drawTriangle(const core::visual::VisualParams * vparams,const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C) {
    double delta = 0.05;
    glColor4f(d_color.getValue()[0],d_color.getValue()[1]-delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(A);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1]-2*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(B); // A<->B

    if (vparams->displayFlags().getShowWireFrame()) {
        glColor4f(d_color.getValue()[0],d_color.getValue()[1]-2*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(B);
    } //A<->B

    glColor4f(d_color.getValue()[0],d_color.getValue()[1]-0.5*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(C);

    if (vparams->displayFlags().getShowWireFrame()) {
        glColor4f(d_color.getValue()[0],d_color.getValue()[1]-0.5*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(C);
    } // C<->A
    if (vparams->displayFlags().getShowWireFrame()) {
        glColor4f(d_color.getValue()[0],d_color.getValue()[1]-delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(A);
    }// C<->A
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
