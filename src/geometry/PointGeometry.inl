#ifndef SOFA_COMPONENT_POINTGEOMETRY_INL
#define SOFA_COMPONENT_POINTGEOMETRY_INL

#include "PointGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa {

namespace core {

namespace behavior {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

class PointProximity : public BaseGeometry::ConstraintProximity {
public :
    PointProximity(const PointGeometry * geo,unsigned pid) {
        m_geo = geo;
        m_pid = pid;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId vid) const {
        return m_geo->getPos(m_pid,vid);
    }

//    defaulttype::Vector3 getNormal(core::VecCoordId /*vid*/) {
//        return defaulttype::Vector3();
//    }

    void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) {
        DataMatrixDeriv & c_d = *cId[m_geo->getMstate()].write();
        MatrixDeriv & c = *c_d.beginEdit();
        MatrixDerivRowIterator c_it1 = c.writeLine(cline);
        c_it1.addCol(m_pid,N);
        c_d.endEdit();
    }

    const PointGeometry * m_geo;
    unsigned m_pid;
};

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

class PointElement : public BaseGeometry::ConstraintElement {
public:

    PointElement(const PointGeometry * geo,unsigned pid) {
        m_pid = pid;
        m_geo = geo;
    }

    ConstraintProximityPtr getDefaultProximity() {
        return std::make_shared<PointProximity>(m_geo,m_pid);
    }

    ConstraintProximityPtr getConstrolPoint(const int cid) {
        if (cid == 0) return getDefaultProximity();
        return NULL;
    }

    unsigned size() {
        return 1;
    }

    //this function project the point P on the element and return the corresponding proximity
    virtual ConstraintProximityPtr project(defaulttype::Vector3 /*P*/) {
        return getDefaultProximity();
    }


protected:
    unsigned m_pid;
    const PointGeometry * m_geo;
};

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

ConstraintElementPtr PointGeometry::getPointElement(unsigned eid) const {
    return std::make_shared<PointElement>(this,eid);
}

unsigned PointGeometry::getNbPoints() const {
    return getTopology()->getNbPoints();
}

unsigned PointGeometry::getNbElements() const {
    return getNbPoints();
}

ConstraintElementPtr PointGeometry::getElement(unsigned eid) const {
    return getPointElement(eid);
}

void PointGeometry::draw(const core::visual::VisualParams * vparams) {
    if (!vparams->displayFlags().getShowCollisionModels()) return;

    double norm = (this->f_bbox.getValue().maxBBox() - this->f_bbox.getValue().minBBox()).norm();

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    glDisable(GL_LIGHTING);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1],d_color.getValue()[2],d_color.getValue()[3]);
    for (unsigned i=0;i<this->getNbPoints();i++) {
        vparams->drawTool()->drawSphere(x[i],norm*0.01);
    }
}

} //controller

} //component

}//Sofa

#endif
