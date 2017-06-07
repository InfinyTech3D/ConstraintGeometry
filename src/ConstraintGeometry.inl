#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_INL
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_INL

#include "ConstraintGeometry.h"
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

BaseGeometry::BaseGeometry()
: d_color(initData(&d_color, defaulttype::Vec4f(1,0.5,0,1), "color", "Color of the collision model")) {}

BaseGeometry * BaseDecorator::getGeometry() {
    BaseGeometry * geo = NULL;

    this->getContext()->get(geo, core::objectmodel::BaseContext::Local);

    if (geo == NULL) serr << "Error cannot find the geometry" << sendl;

    return geo;
}

void BaseGeometry::init() {
    this->getContext()->get(m_topology);
    this->getContext()->get(m_state);

    if (getTopology() == NULL) serr << "Error cannot find the topology" << sendl;
    if (getMstate() == NULL) serr << "Error cannot find the topology" << sendl;
}

std::unique_ptr<BaseConstraintIterator> BaseGeometry::getIterator(const ConstraintProximity & P) {
    BaseDecorator * decorator;
    this->getContext()->get(decorator);

    if (decorator != NULL) return decorator->getIterator(P);
    std::unique_ptr<BaseConstraintIterator> foo (new DefaultConstraintIterator(this));
    return foo;
}

void BaseGeometry::addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal) {
    DataMatrixDeriv & c_d = *cId[getMstate()].write();

    MatrixDeriv & c = *c_d.beginEdit();

    MatrixDerivRowIterator c_it1 = c.writeLine(cline);

    for (unsigned p=0;p<pinfo.size();p++) {
        if (pinfo.m_fact[p] == 0.0) continue;
        c_it1.addCol(pinfo.m_pid[p], normal * pinfo.m_fact[p]);
    }

    c_d.endEdit();
}

//linear version of getPosition
defaulttype::Vector3 BaseGeometry::getPosition(const ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    defaulttype::Vector3 P;
    for (unsigned i=0;i<pinfo.size();i++) {
        P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
    }
    return P;
}

defaulttype::Vector3 BaseGeometry::getFreePosition(const ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::freePosition());

    defaulttype::Vector3 P;
    for (unsigned i=0;i<pinfo.size();i++) {
        P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
    }
    return P;
}

defaulttype::Vector3 BaseGeometry::getRestPosition(const ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::restPosition());

    defaulttype::Vector3 P;
    for (unsigned i=0;i<pinfo.size();i++) {
        P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
    }
    return P;
}

void BaseGeometry::computeBBox(const core::ExecParams* params, bool /*onlyVisible*/) {
    SReal minBBox[3] = {1e10,1e10,1e10};
    SReal maxBBox[3] = {-1e10,-1e10,-1e10};

    m_g = defaulttype::Vector3(0,0,0);

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    for (unsigned i=0;i<x.size();i++) {
        m_g += x[i];

        for (int c=0; c<3; c++)
        {
            if (x[i][c] > maxBBox[c]) maxBBox[c] = x[i][c];
            if (x[i][c] < minBBox[c]) minBBox[c] = x[i][c];
        }
    }

    m_g *= 1.0/x.size();
    m_norm = (defaulttype::Vector3(minBBox[0],minBBox[1],minBBox[2]) - defaulttype::Vector3(maxBBox[0],maxBBox[1],maxBBox[2])).norm();

    this->f_bbox.setValue(params,sofa::defaulttype::TBoundingBox<SReal>(minBBox,maxBBox));
}

double BaseGeometry::getNorm() {
    return m_norm;
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
