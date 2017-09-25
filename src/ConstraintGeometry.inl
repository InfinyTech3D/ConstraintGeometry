#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_INL
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_INL

#include "ConstraintGeometry.h"
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

BaseGeometry::BaseGeometry()
: d_color(initData(&d_color, defaulttype::Vec4f(1,0.5,0,1), "color", "Color of the collision model")) {}

void BaseGeometry::init() {
    this->getContext()->get(m_topology);
    this->getContext()->get(m_state);

    if (getTopology() == NULL) serr << "Error cannot find the topology" << sendl;
    if (getMstate() == NULL) serr << "Error cannot find the topology" << sendl;
}

BaseConstraintIteratorPtr BaseGeometry::getIterator(const ConstraintProximityPtr & /*E*/) const {
    return std::unique_ptr<BaseConstraintIterator>(new DefaultConstraintIterator(this));
}

void BaseGeometry::addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal) const {
    DataMatrixDeriv & c_d = *cId[getMstate()].write();

    MatrixDeriv & c = *c_d.beginEdit();

    MatrixDerivRowIterator c_it1 = c.writeLine(cline);

    for (unsigned p=0;p<pinfo.m_pid.size();p++) {
        if (pinfo.m_fact[p] == 0.0) continue;
        c_it1.addCol(pinfo.m_pid[p], normal * pinfo.m_fact[p]);
    }

    c_d.endEdit();
}

void BaseGeometry::computeBBox(const core::ExecParams* params, bool /*onlyVisible*/) {
    SReal minBBox[3] = {1e10,1e10,1e10};
    SReal maxBBox[3] = {-1e10,-1e10,-1e10};

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    for (unsigned i=0;i<x.size();i++) {
        for (int c=0; c<3; c++)
        {
            if (x[i][c] > maxBBox[c]) maxBBox[c] = x[i][c];
            if (x[i][c] < minBBox[c]) minBBox[c] = x[i][c];
        }
    }

    this->f_bbox.setValue(params,sofa::defaulttype::TBoundingBox<SReal>(minBBox,maxBBox));
}

void BaseGeometry::prepareDetection() {}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
