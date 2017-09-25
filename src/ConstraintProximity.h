#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H

#include "ConstraintGeometry.h"
#include <math.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace core {

namespace behavior {

class CollisionAlgorithm;
class ConstraintNormal;

class ConstraintProximity {
    friend class BaseGeometry;
    friend class CollisionAlgorithm;
    friend class ConstraintNormal;

public :
    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;

    ConstraintProximity(const BaseGeometry * geo) {
        m_cg = geo;
    }

    virtual void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const defaulttype::Vector3 & normal) const {
        m_cg->addConstraint(cId,cline,*this,normal);
    }

    unsigned size() const {
        return m_pid.size();
    }

    virtual defaulttype::Vector3 getPosition() const {
        return getPosition(*m_cg->getMstate()->read(core::VecCoordId::position()));
    }

    virtual defaulttype::Vector3 getFreePosition() const {
        return getPosition(*m_cg->getMstate()->read(core::VecCoordId::freePosition()));
    }

    virtual defaulttype::Vector3 getRestPosition() const {
        return getPosition(*m_cg->getMstate()->read(core::VecCoordId::restPosition()));
    }

    virtual defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3();
    }

    virtual double distance(const ConstraintProximityPtr & D) const {
        return (D->getPosition() - this->getPosition()).norm();
    }

    bool operator ==(const ConstraintProximity & b) const {
        if (size() != b.size()) return false;

        for (unsigned i=0;i<size();i++) {
            bool find = false;
            unsigned j=0;
            while (j<size() && !find) {
                find = m_pid[i] == b.m_pid[j] && m_fact[i] == b.m_fact[j];
                j++;
            }
            if (!find) return false;
        }

        return true;
    }

    bool operator !=(const ConstraintProximity & b) const {
        return ! (*this == b);
    }

    inline friend std::ostream& operator << ( std::ostream& out, const ConstraintProximity& c )
    {
        out << c.m_cg->getName() << ":" ;
        for (unsigned i=0;i<c.m_fact.size();i++) {
            out << "[" << c.m_pid[i] << "," << c.m_fact[i] << "]";
        }
        return out;
    }

    const BaseGeometry * getGeometry() const {
        return m_cg;
    }

    const helper::vector<int> & getPid() const {
        return m_pid;
    }

    const helper::vector<double> & getFact() const {
        return m_fact;
    }

protected:
    const BaseGeometry * m_cg;
    helper::vector<int> m_pid;
    helper::vector<double> m_fact;

    virtual defaulttype::Vector3 getPosition(const helper::ReadAccessor<Data <VecCoord> >& x) const {
        defaulttype::Vector3 P;
        for (unsigned i=0;i<m_pid.size();i++) {
            P += x[m_pid[i]] * m_fact[i];
        }
        return P;
    }
};

class ConstraintNormal {
public :

    void addNormal(defaulttype::Vector3 N1) {
        N1.normalize();
        m_normals.push_back(N1);
    }

    static ConstraintNormal createConstraint(defaulttype::Vector3 N1) {
        ConstraintNormal res;

        if (N1.norm() == 0) return res;

        N1.normalize();

        res.m_normals.push_back(N1);

        return res;
    }

    static ConstraintNormal createFrameConstraint(defaulttype::Vector3 N1) {
        ConstraintNormal res;

        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);

        N1.normalize();

        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
        N2.normalize();

        defaulttype::Vector3 N3 = cross(N1,N2);
        N3.normalize();

        res.m_normals.push_back(N1);
        res.m_normals.push_back(N2);
        res.m_normals.push_back(N3);

        return res;
    }

    void addConstraint(core::MultiMatrixDerivId cId, unsigned & cline, const ConstraintProximityPtr & pinfo) {
        if (empty()) return;

        for (unsigned i=0;i<m_normals.size();i++) {
            pinfo->addConstraint(cId,cline+i,m_normals[i]);
        }

        cline += m_normals.size();
    }

    void addConstraint(core::MultiMatrixDerivId cId, unsigned & cline, const ConstraintProximityPtr & pinfo1, const ConstraintProximityPtr & pinfo2) {
        if (empty()) return;

        for (unsigned i=0;i<m_normals.size();i++) {
            pinfo1->addConstraint(cId,cline+i,m_normals[i]);
            pinfo2->addConstraint(cId,cline+i,-m_normals[i]);
        }

        cline += m_normals.size();
    }

    void addViolation(defaulttype::BaseVector *v,unsigned & cid, const ConstraintProximityPtr & pinfo1, const ConstraintProximityPtr & pinfo2) {
        if (empty()) return;

        defaulttype::Vector3 PFree = pinfo1->getFreePosition();
        defaulttype::Vector3 QFree = pinfo2->getFreePosition();
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_normals.size();i++) {
            v->set(cid+i,dot(m_normals[i],PQFree));
        }

        cid += m_normals.size();
    }

    void addViolation(defaulttype::BaseVector *v,unsigned & cid, const ConstraintProximity & pinfo1, const defaulttype::Vector3 & QFree) {
        if (empty()) return;

        defaulttype::Vector3 PFree = pinfo1.getFreePosition();
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
