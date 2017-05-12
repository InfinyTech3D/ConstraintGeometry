#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H

#include "ConstraintGeometry.h"
#include <math.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace core {

namespace behavior {


class ConstraintProximity {
public :
    ConstraintProximity() {
        m_cg = NULL;
    }

    ConstraintProximity(BaseGeometry * cg,unsigned eid) {
        m_cg = cg;
        m_eid = eid;
    }

    unsigned getEid() const {
        return m_eid;
    }

    unsigned size() const {
        return m_pid.size();
    }

    void push(unsigned id, double f) {
        m_pid.push_back(id);
        m_fact.push_back(f);
    }

    BaseGeometry * getGeometry() {
        return m_cg;
    }

    void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const defaulttype::Vector3 & normal) const {
        if (m_cg == NULL) return;
        m_cg->addConstraint(cId,cline,*this,normal);
    }

    defaulttype::Vector3 getPosition() const {
        if (m_cg == NULL) return defaulttype::Vector3();
        return m_cg->getPosition(*this);
    }

    defaulttype::Vector3 getFreePosition() const {
        if (m_cg == NULL) return defaulttype::Vector3();
        return m_cg->getFreePosition(*this);
    }

    defaulttype::Vector3 getNormal() const {
        if (m_cg == NULL) return defaulttype::Vector3();
        return m_cg->getNormal(*this);
    }

    helper::vector<unsigned> m_pid;
    helper::vector<double> m_fact;
    unsigned m_eid;

protected:
    BaseGeometry * m_cg;

};

class ConstraintNormal {
public :

    void addNormal(defaulttype::Vector3 N1) {
        N1.normalize();
        m_normals.push_back(N1);
    }

    void addConstraint(core::MultiMatrixDerivId cId, unsigned & cline, const ConstraintProximity & pinfo) {
        if (empty()) return;

        for (unsigned i=0;i<m_normals.size();i++) {
            pinfo.addConstraint(cId,cline+i,m_normals[i]);
        }

        cline += m_normals.size();
    }

    void addConstraint(core::MultiMatrixDerivId cId, unsigned & cline, const ConstraintProximity & pinfo1, const ConstraintProximity & pinfo2) {
        if (empty()) return;

        for (unsigned i=0;i<m_normals.size();i++) {
            pinfo1.addConstraint(cId,cline+i,m_normals[i]);
            pinfo2.addConstraint(cId,cline+i,-m_normals[i]);
        }

        cline += m_normals.size();
    }

    void addViolation(defaulttype::BaseVector *v,unsigned & cid, const ConstraintProximity & pinfo1, const ConstraintProximity & pinfo2) {
        if (empty()) return;

        defaulttype::Vector3 PFree = pinfo1.getFreePosition();
        defaulttype::Vector3 QFree = pinfo2.getFreePosition();
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_normals.size();i++) {
            v->set(cid+i,dot(m_normals[i],PQFree));
        }

        cid += m_normals.size();
    }

    void addConstraintResolution(std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset, core::behavior::ConstraintResolution* cr) {
        if (empty()) return;

        resTab[offset] = cr;

        offset+=m_normals.size();
    }


    unsigned size() {
        return m_normals.size();
    }

    bool empty() {
        return m_normals.empty();
    }


    helper::vector<defaulttype::Vector3> m_normals;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
