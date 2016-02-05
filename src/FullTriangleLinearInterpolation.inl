#ifndef SOFA_COMPONENT_FULLTRIANGLELINEARINTERPOLATION_INL
#define SOFA_COMPONENT_FULLTRIANGLELINEARINTERPOLATION_INL

#include "FullTriangleLinearInterpolation.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>

namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
FullTriangleLinearInterpolation<DataTypes>::FullTriangleLinearInterpolation()
: TriangleInterpolation<DataTypes>() {
    this->f_listening.setValue(true);
}

template<class DataTypes>
void FullTriangleLinearInterpolation<DataTypes>::prepareDetection() {
    helper::ReadAccessor<Data <VecCoord> > x = *this->m_state->read(core::VecCoordId::position());

    m_triangle_info.resize(this->m_container->getNbTriangles());
    for (int t=0;t<this->m_container->getNbTriangles();t++) {
        TriangleInfo & tinfo = m_triangle_info[t];

        const topology::Triangle tri = this->m_container->getTriangle(t);

        //Compute Bezier Positions
        Vector3 p0 = x[tri[0]];
        Vector3 p1 = x[tri[1]];
        Vector3 p2 = x[tri[2]];

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

        if (this->d_flipNormals.getValue()) tinfo.tn *= -1;
    }

    m_point_normal.resize(x.size());
    for (unsigned p=0;p<x.size();p++) {
        const core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->m_container->getTrianglesAroundVertex(p);

        m_point_normal[p] = Vector3(0,0,0);
        for (unsigned t=0;t<tav.size();t++) {
            m_point_normal[p] += m_triangle_info[tav[t]].tn;
        }

        m_point_normal[p].normalize();
    }
}

template<class DataTypes>
void FullTriangleLinearInterpolation<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event) {
    //At the begining of the time step we check if we crossed the boundary use the last proximity as entring point
    if (dynamic_cast<simulation::AnimateBeginEvent*>(event)) {
        prepareDetection();
    }
}


//proj_P must be on the plane
template<class DataTypes>
void FullTriangleLinearInterpolation<DataTypes>::computeBaryCoords(const Vector3 & proj_P,const TriangleInfo & tinfo, const Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) {
    Vector3 v2 = proj_P - p0;

    double d20 = dot(v2, tinfo.v0);
    double d21 = dot(v2, tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0- fact_v  - fact_w;
}

//Barycentric coordinates are computed according to
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
template<class DataTypes>
void FullTriangleLinearInterpolation<DataTypes>::projectPointOnTriangle(const Vector3 & s,const TriangleInfo & tinfo, const Vector3 & p0,const Vector3 & p1,const Vector3 & p2,double & fact_u,double & fact_v, double & fact_w) {
    Vector3 x1x2 = s - p0;

    //corrdinate on the plane
    double c0 = dot(x1x2,tinfo.ax1);
    double c1 = dot(x1x2,tinfo.ax2);
    Vector3 proj_P = p0 + c0 * tinfo.ax1 + c1 * tinfo.ax2;

    computeBaryCoords(proj_P,tinfo,p0, fact_u,fact_v,fact_w);

    // clip the bary coord if it is out of the plane
    if (fact_u<-0.00001 || fact_u>1.00001 || fact_v<-0.00001 || fact_v>1.00001 || fact_w<0.00001 || fact_w>1.00001) {
        //the point is not inside the triangle we project on the 3 edges and take the closest projection to s

        Vector3 edge2 = p2 - p1;

        double dot0 = dot(proj_P - p0,tinfo.v0) / tinfo.d00;
        double dot1 = dot(proj_P - p0,tinfo.v1) / tinfo.d11;
        double dot2 = dot(proj_P - p1,edge2) / dot(edge2,edge2);

        if (dot0<0.0) dot0 = 0.0;
        else if (dot0>1.0) dot0 = 1.0;

        if (dot1<0.0) dot1 = 0.0;
        else if (dot1>1.0) dot1 = 1.0;

        if (dot2<0.0) dot2 = 0.0;
        else if (dot2>1.0) dot2 = 1.0;

        Vector3 edge_P0 = p0 + tinfo.v0 * dot0;
        Vector3 edge_P1 = p0 + tinfo.v1 * dot1;
        Vector3 edge_P2 = p1 + edge2 * dot2;

        double dist0 = (s-edge_P0).norm();
        double dist1 = (s-edge_P1).norm();
        double dist2 = (s-edge_P2).norm();

        //use the closer point on the edges
        if (dist0<=dist1 && dist0<=dist2) computeBaryCoords(edge_P0,tinfo,p0,fact_u,fact_v,fact_w);
        else if (dist1<=dist0 && dist1<=dist2) computeBaryCoords(edge_P1,tinfo,p0,fact_u,fact_v,fact_w);
        else if (dist2<=dist0 && dist2<=dist1) computeBaryCoords(edge_P2,tinfo,p0,fact_u,fact_v,fact_w);
    }
}

template<class DataTypes>
void FullTriangleLinearInterpolation<DataTypes>::fillProximity(const Coord & P,ConstraintProximity & pinfo) {
    helper::ReadAccessor<Data <VecCoord> > x = *this->m_state->read(core::VecCoordId::position());

    pinfo.pid.resize(3);
    pinfo.fact.resize(3);

    double minDist = 0;
    for(unsigned t=0;t<m_triangle_info.size();t++) {
        const topology::Triangle tri = this->m_container->getTriangle(t);

        //Compute Bezier Positions
        Vector3 p0 = x[tri[0]];
        Vector3 p1 = x[tri[1]];
        Vector3 p2 = x[tri[2]];

        double fact_u;
        double fact_v;
        double fact_w;

        const TriangleInfo & tinfo = m_triangle_info[t];

        projectPointOnTriangle(P,tinfo,p0,p1,p2,fact_u,fact_v,fact_w);

        Vector3 Q = p0 * fact_u + p1 * fact_v + p2 * fact_w;

        //1.0 + 0.1 i.e. 0.1 is to avoid zero
        double dist = (Q-P).norm();

        if ((t==0) || (dist < minDist)) {
            pinfo.pid[0] = tri[0];
            pinfo.pid[1] = tri[1];
            pinfo.pid[2] = tri[2];

            pinfo.fact[0] = fact_u;
            pinfo.fact[1] = fact_v;
            pinfo.fact[2] = fact_w;

            minDist = dist;
        }
    }
}

template<class DataTypes>
void FullTriangleLinearInterpolation<DataTypes>::fillConstraintNormal(const ConstraintProximity & pinfo,ConstraintNormal & ninfo) {
    Vector3 normal = m_point_normal[pinfo.pid[0]] * pinfo.fact[0] + m_point_normal[pinfo.pid[1]] * pinfo.fact[1] + m_point_normal[pinfo.pid[2]] * pinfo.fact[2];

    Vector3 N1 = -normal;
    N1.normalize();

    Vector3 N2 = cross(N1,((fabs(dot(N1,Vector3(1,0,0)))>0.999999) ? Vector3(0,1,0) : Vector3(1,0,0)));
    N2.normalize();

    Vector3 N3 = cross(N1,N2);
    N3.normalize();

    ninfo.normals.clear();
    ninfo.normals.push_back(N1);
    ninfo.normals.push_back(N2);
    ninfo.normals.push_back(N3);
}

template<class DataTypes>
void FullTriangleLinearInterpolation<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    if (!this->d_draw.getValue()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->m_state->read(core::VecCoordId::position());
    for(unsigned t=0;t<m_triangle_info.size();t++) {
        const topology::Triangle tri = this->m_container->getTriangle(t);

        //Compute Bezier Positions
        Vector3 p0 = x[tri[0]];
        Vector3 p1 = x[tri[1]];
        Vector3 p2 = x[tri[2]];

        vparams->drawTool()->drawArrow(p0,p0 + m_point_normal[tri[0]],0.01, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
        vparams->drawTool()->drawArrow(p1,p1 + m_point_normal[tri[1]],0.01, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
        vparams->drawTool()->drawArrow(p2,p2 + m_point_normal[tri[2]],0.01, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
    }
}

} //controller

} //component

}//Sofa

#endif
