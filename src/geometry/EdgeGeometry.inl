#ifndef SOFA_COMPONENT_EDGEGEOMETRY_INL
#define SOFA_COMPONENT_EDGEGEOMETRY_INL

#include "EdgeGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

class EdgeProximity : public BaseGeometry::ConstraintProximity {
public:
    friend class EdgeGeometry;

    EdgeProximity(const EdgeGeometry * geo, unsigned p1, double f1,unsigned p2, double f2) {
        m_geo = geo;

        m_pid[0] = p1;
        m_fact[0] = f1;

        m_pid[1] = p2;
        m_fact[1] = f2;
    }

    void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) {
        DataMatrixDeriv & c_d = *cId[m_geo->getMstate()].write();
        MatrixDeriv & c = *c_d.beginEdit();
        MatrixDerivRowIterator c_it1 = c.writeLine(cline);

        c_it1.addCol(m_pid[0],N*m_fact[0]);
        c_it1.addCol(m_pid[1],N*m_fact[1]);

        c_d.endEdit();
    }

    defaulttype::Vector3 getPosition(core::VecCoordId vid) const {
        return m_geo->getPos(m_pid[0],vid) * m_fact[0] + m_geo->getPos(m_pid[1],vid) * m_fact[1];
    }

    defaulttype::Vector3 getNormal() const {
//        return m_geo->pointNormal(m_pid[0],vid) * m_fact[0] + m_geo->getPos(m_pid[1],vid) * m_fact[1];
//        const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getMstate()->read(core::VecCoordId::position());

//        defaulttype::Vector3 En = x[m_pid[1]] - x[m_pid[0]];
//        defaulttype::Vector3 Z = defaulttype::Vector3(0,0,1);

//        En.normalize();
//        if (dot(En,Z) < 0.000000000001) Z=defaulttype::Vector3(0,1,0);

//        return cross(En,Z);
    }

//    void refineToClosestPoint(const Coord & P) {
//        ((const EdgeGeometry*) m_geo)->projectPoint(P,this);
//    }

    unsigned m_pid[2];
    double m_fact[2];
    const EdgeGeometry * m_geo;
};

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

class EdgeElement : public BaseGeometry::ConstraintElement {
public:

    EdgeElement(const EdgeGeometry * geo,unsigned eid) {
        m_eid = eid;
        m_geo = geo;

        topology::BaseMeshTopology::Edge edge = geo->getTopology()->getEdge(eid);
        m_pid[0] = edge[0];
        m_pid[1] = edge[1];


    }

    ConstraintProximityPtr getProximity(double f1,double f2) {
        return std::make_shared<EdgeProximity>(m_geo,m_pid[0],f1,m_pid[1],f2);
    }

    ConstraintProximityPtr getDefaultProximity() {
        return getProximity(0.5,0.5);
    }

    ConstraintProximityPtr getConstrolPoint(const int cid) {
        if (cid == 0) return getProximity(1.0,0.0);
        else if (cid == 1) return getProximity(0.0,1.0);
        else return NULL;

        return getDefaultProximity();
    }

    unsigned size() {
        return 2;
    }

    //this function project the point P on the element and return the corresponding proximity
    ConstraintProximityPtr project(defaulttype::Vector3 P) {
        double fact_u,fact_v;

        defaulttype::Vector3 P1 = m_geo->getPos(m_pid[0]);
        defaulttype::Vector3 P2 = m_geo->getPos(m_pid[1]);

        defaulttype::Vector3 v = P2-P1;
        fact_v = dot (P - P1,v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        return getProximity(fact_u,fact_v);
    }


protected:
    unsigned m_eid;
    unsigned m_pid[2];
    const EdgeGeometry * m_geo;
};

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

unsigned EdgeGeometry::getNbEdges() const {
    return this->getTopology()->getNbEdges();
}

ConstraintElementPtr EdgeGeometry::getElement(unsigned eid) const {
    return std::make_shared<EdgeElement>(this, eid);
}

unsigned EdgeGeometry::getNbElements() const {
    return getNbEdges();
}

//void EdgeGeometry::projectPoint(const defaulttype::Vector3 & P, EdgeConstraintProximity *pinfo) const {
//    helper::vector<defaulttype::Vector3> pos;
//    for (unsigned i=0;i<pinfo->size();i++) pos.push_back(pinfo->getControlPoint(i));

//    double & fact_u = pinfo->m_fact[0];
//    double & fact_v = pinfo->m_fact[1];

//    Coord v = pos[1] - pos[0];
//    fact_v = dot (P - pos[0],v) / dot (v,v);

//    if (fact_v<0.0) fact_v = 0.0;
//    else if (fact_v>1.0) fact_v = 1.0;

//    fact_u = 1.0-fact_v;
//}

void EdgeGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1],d_color.getValue()[2],d_color.getValue()[3]);
    for(int e=0;e<this->getTopology()->getNbEdges();e++) {
        const sofa::core::topology::BaseMeshTopology::Edge edge= this->getTopology()->getEdge(e);

        //Compute Bezier Positions
        defaulttype::Vector3 p0 = x[edge[0]];
        defaulttype::Vector3 p1 = x[edge[1]];

        helper::gl::glVertexT(p0);
        helper::gl::glVertexT(p1);
    }
    glEnd();
}

} //controller

} //component

}//Sofa

#endif
