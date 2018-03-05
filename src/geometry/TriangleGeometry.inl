#ifndef SOFA_COMPONENT_TRIANGLEGEOMETRY_INL
#define SOFA_COMPONENT_TRIANGLEGEOMETRY_INL

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

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

class TriangleProximity : public BaseGeometry::ConstraintProximity {
public:
    friend class TriangleGeometry;

    TriangleProximity(const TriangleGeometry * geo,unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) {
        m_geo = geo;

        m_eid = eid;

        m_pid[0] = p1;
        m_fact[0] = f1;

        m_pid[1] = p2;
        m_fact[1] = f2;

        m_pid[2] = p3;
        m_fact[2] = f3;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId vid) const {
        return m_geo->getPos(m_pid[0],vid) * m_fact[0] + m_geo->getPos(m_pid[1],vid) * m_fact[1] + m_geo->getPos(m_pid[2],vid) * m_fact[2];
    }

    void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) {
        DataMatrixDeriv & c_d = *cId[m_geo->getMstate()].write();
        MatrixDeriv & c = *c_d.beginEdit();
        MatrixDerivRowIterator c_it1 = c.writeLine(cline);

        c_it1.addCol(m_pid[0],N*m_fact[0]);
        c_it1.addCol(m_pid[1],N*m_fact[1]);
        c_it1.addCol(m_pid[2],N*m_fact[2]);

        c_d.endEdit();
    }

    defaulttype::Vector3 getNormal() const {
        return ((const TriangleGeometry *)m_geo)->m_triangle_info[m_eid].tn;
    }

//    void refineToClosestPoint(const Coord & P) {
//        ((const TriangleGeometry*) m_geo)->projectPoint(P,this);
//    }

protected:
    unsigned m_pid[3];
    double m_fact[3];
    unsigned m_eid;
    const TriangleGeometry * m_geo;
};

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

class TriangleElement : public BaseGeometry::ConstraintElement {
public:

    TriangleElement(const TriangleGeometry * geo,unsigned eid) {
        m_eid = eid;
        m_geo = geo;

        topology::BaseMeshTopology::Triangle edge = geo->getTopology()->getTriangle(eid);
        m_pid[0] = edge[0];
        m_pid[1] = edge[1];
        m_pid[2] = edge[2];
    }

    ConstraintProximityPtr getProximity(double f1,double f2,double f3) {
        return std::make_shared<TriangleProximity>(m_geo,m_eid, m_pid[0],f1,m_pid[1],f2,m_pid[2],f3);
    }

    ConstraintProximityPtr getDefaultProximity() {
        return getProximity(1.0/3.0,1.0/3.0,1.0/3.0);
    }

    //this function returns a vector with all the control points of the element
    ConstraintProximityPtr getConstrolPoint(const int cid) {
        if (cid == 0) return getProximity(1.0,0.0,0.0);
        else if (cid == 1) return getProximity(0.0,1.0,0.0);
        else if (cid == 2) return getProximity(0.0,0.0,1.0);

        return NULL;
    }

    unsigned size() {
        return 3;
    }

//    //this function project the point P on the element and return the corresponding proximity
//    ConstraintProximityPtr project(defaulttype::Vector3 P) {
//        double fact_u,fact_v;

//        defaulttype::Vector3 P1 = m_geo->getPos(m_pid[0]);
//        defaulttype::Vector3 P2 = m_geo->getPos(m_pid[1]);

//        defaulttype::Vector3 v = P2-P1;
//        fact_v = dot (P - P1,v) / dot (v,v);

//        if (fact_v<0.0) fact_v = 0.0;
//        else if (fact_v>1.0) fact_v = 1.0;

//        fact_u = 1.0-fact_v;

//        return getProximity(fact_u,fact_v);
//    }

    //proj_P must be on the plane

    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleGeometry::TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const {
        defaulttype::Vector3 v2 = proj_P - p0;

        double d20 = dot(v2, tinfo.v0);
        double d21 = dot(v2, tinfo.v1);

        fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
        fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
        fact_u = 1.0 - fact_v  - fact_w;
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates

    ConstraintProximityPtr project(defaulttype::Vector3 P) {
        defaulttype::Vector3 P0 = m_geo->getPos(m_pid[0]);
        defaulttype::Vector3 P1 = m_geo->getPos(m_pid[1]);
        defaulttype::Vector3 P2 = m_geo->getPos(m_pid[2]);

        defaulttype::Vector3 x1x2 = P - P0;

        const TriangleGeometry::TriangleInfo & tinfo = m_geo->m_triangle_info[m_eid];

        //corrdinate on the plane
        double c0 = dot(x1x2,tinfo.ax1);
        double c1 = dot(x1x2,tinfo.ax2);
        defaulttype::Vector3 proj_P = P0 + c0 * tinfo.ax1 + c1 * tinfo.ax2;

        double fact_u,fact_v,fact_w;

        computeBaryCoords(proj_P, tinfo, P0, fact_u,fact_v,fact_w);

        if (fact_u<0) {
            defaulttype::Vector3 v3 = P1 - P2;
            defaulttype::Vector3 v4 = proj_P - P2;
            double alpha = dot (v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = 0;
            fact_v = alpha;
            fact_w = 1.0 - alpha;
        } else if (fact_v<0) {
            defaulttype::Vector3 v3 = P0 - P2;
            defaulttype::Vector3 v4 = proj_P - P2;
            double alpha = dot (v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = alpha;
            fact_v = 0;
            fact_w = 1.0 - alpha;
        } else if (fact_w<0) {
            defaulttype::Vector3 v3 = P1 - P0;
            defaulttype::Vector3 v4 = proj_P - P2;
            double alpha = dot (v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = 1.0 - alpha;
            fact_v = alpha;
            fact_w = 0;
        }

        return getProximity(fact_u,fact_v,fact_w);
    }

protected:
    unsigned m_eid;
    unsigned m_pid[3];
    const TriangleGeometry * m_geo;
};

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

TriangleGeometry::TriangleGeometry()
: d_phong(initData(&d_phong, false, "phong", "Phong interpolation of the normal")) {

}

ConstraintElementPtr TriangleGeometry::getElement(unsigned eid) const {
    return std::make_shared<TriangleElement>(this, eid);
}

unsigned TriangleGeometry::getNbElements() const {
    return getNbTriangles();
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

int TriangleGeometry::getNbTriangles() const {
    return this->getTopology()->getNbTriangles();
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
    if (d_color.getValue()[3] == 0.0) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glDisable(GL_LIGHTING);

    if (vparams->displayFlags().getShowWireFrame()) glBegin(GL_LINES);
    else {
//        glEnable(GL_CULL_FACE);
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
