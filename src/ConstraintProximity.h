#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "ConstraintGeometry.h"

namespace sofa {

namespace core {

namespace behavior {


class ConstraintProximity {
public :
    friend class BaseConstraintGeometry;

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


    void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const defaulttype::Vector3 & normal) const {
        if (m_cg == NULL) return ;
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

    void addFrame(defaulttype::Vector3 N1) {
        N1.normalize();

        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(1,0,0)))>0.99) ? defaulttype::Vector3(0,1,0) : defaulttype::Vector3(1,0,0)));
        N2.normalize();

        defaulttype::Vector3 N3 = cross(N1,N2);
        N3.normalize();

        m_normals.push_back(N1);
        m_normals.push_back(N2);
        m_normals.push_back(N3);
    }

    void addFrame(defaulttype::Vector3 N1,defaulttype::Vector3 N2,defaulttype::Vector3 N3) {
        N1.normalize();
        N2.normalize();
        N3.normalize();

        m_normals.push_back(N1);
        m_normals.push_back(N2);
        m_normals.push_back(N3);
    }

    void addConstraint(core::MultiMatrixDerivId cId, unsigned & cline, const ConstraintProximity & pinfo) {
        for (unsigned i=0;i<m_normals.size();i++) {
            pinfo.addConstraint(cId,cline+i,m_normals[i]);
        }

        cline += m_normals.size();
    }

    void addConstraint(core::MultiMatrixDerivId cId, unsigned & cline, const ConstraintProximity & pinfo1, const ConstraintProximity & pinfo2) {
        for (unsigned i=0;i<m_normals.size();i++) {
            pinfo1.addConstraint(cId,cline+i,m_normals[i]);
            pinfo2.addConstraint(cId,cline+i,-m_normals[i]);
        }

        cline += m_normals.size();
    }

    void addViolation(defaulttype::BaseVector *v,unsigned & cid, defaulttype::Vector3 PQFree) {
        for (unsigned i=0;i<m_normals.size();i++) {
            v->set(cid+i,dot(m_normals[i],PQFree));
        }

        cid += m_normals.size();
    }

    unsigned size() {
        return m_normals.size();
    }

    void clear() {
        return m_normals.clear();
    }

    helper::vector<defaulttype::Vector3> m_normals;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
