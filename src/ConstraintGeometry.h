#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/collision/Pipeline.h>
#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>

namespace sofa {

namespace core {

namespace behavior {

class ConstraintProximity;

class BaseGeometry : public core::BehaviorModel {
public :
    SOFA_CLASS(BaseGeometry, core::BehaviorModel);

    topology::TopologyContainer * getTopology() {
        return dynamic_cast<topology::TopologyContainer *>(getContext()->getTopology());
    }

    //this function is called during the collision visitor.
    virtual void prepareDetection() {}

    virtual void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal) = 0;

    virtual defaulttype::Vector3 getPosition(const ConstraintProximity & pinfo) = 0;

    virtual defaulttype::Vector3 getFreePosition(const ConstraintProximity & pinfo) = 0;

private :
    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }
};

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

template<class DataTypes>
class GeometryImpl : public BaseGeometry {
public :
    friend class ConstraintProximity;

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(SOFA_TEMPLATE(GeometryImpl,DataTypes) , BaseGeometry);

    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() {
        return dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> * >(getContext()->getMechanicalState());
    }

    void init() {
        if (getTopology() == NULL) serr << "Error cannot find the topology" << sendl;
        if (getMstate() == NULL) serr << "Error cannot find the topology" << sendl;
        prepareDetection();
    }

    void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal) {
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
    virtual defaulttype::Vector3 getPosition(const ConstraintProximity & pinfo) {
        const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

        defaulttype::Vector3 P;
        for (unsigned i=0;i<pinfo.size();i++) {
            P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
        }
        return P;
    }

    virtual defaulttype::Vector3 getFreePosition(const ConstraintProximity & pinfo) {
        const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::freePosition());

        defaulttype::Vector3 P;
        for (unsigned i=0;i<pinfo.size();i++) {
            P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
        }
        return P;
    }
};

template<class DataTypes>
class GeometryDecorator : public core::BehaviorModel {
public:
    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() {
        return dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> * >(getContext()->getMechanicalState());
    }

    topology::TopologyContainer * getTopology() {
        return dynamic_cast<topology::TopologyContainer *>(getContext()->getTopology());
    }

    void init() {
        if (getTopology() == NULL) serr << "Error cannot find the topology" << sendl;
        if (getMstate() == NULL) serr << "Error cannot find the topology" << sendl;
        prepareDetection();
    }

    //this function is called during the collision visitor.
    virtual void prepareDetection() {}

private :
    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }
};

template<class DataTypes>
class PointGeometry : public GeometryImpl<DataTypes> {
public:
    ConstraintProximity getPointProximity(unsigned eid) {
        ConstraintProximity res(this, eid);
        res.push(eid,1.0);
        return res;
    }

    virtual ConstraintProximity findClosestProximity(const defaulttype::Vector3 & P) = 0;

};

template<class DataTypes>
class EdgeGeometry : public PointGeometry<DataTypes> {
public:
    ConstraintProximity getEdgeProximity(unsigned eid,double f1,double f2) {
        sofa::core::topology::Topology::Edge edge = this->getTopology()->getEdge(eid);
        ConstraintProximity res(this, eid);
        res.push(edge[0],f1);
        res.push(edge[1],f2);
        return res;
    }

    virtual ConstraintProximity findClosestProximity(const defaulttype::Vector3 & P) = 0;

    virtual defaulttype::Vector3 getEdgeNormal(unsigned eid) {
        const helper::ReadAccessor<Data <typename DataTypes::VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());
        sofa::core::topology::Topology::Edge edge = this->getTopology()->getEdge(eid);

        defaulttype::Vector3 N1 = x[edge[1]] - x[edge[0]];
        N1.normalize();
        return N1;
    }
};

template<class DataTypes>
class TriangleGeometry : public EdgeGeometry<DataTypes> {
public:
    class TriangleDecorator : public GeometryDecorator<DataTypes> {
    public:
        virtual ConstraintProximity findClosestTriangle(TriangleGeometry<DataTypes> * geo, const defaulttype::Vector3 & P) = 0;
    };

    ConstraintProximity getTriangleProximity(unsigned eid,double f1,double f2,double f3) {
        sofa::core::topology::Topology::Triangle tri = this->getTopology()->getTriangle(eid);

        ConstraintProximity res(this, eid);
        res.push(tri[0],f1);
        res.push(tri[1],f2);
        res.push(tri[2],f3);

        return res;
    }

    ConstraintProximity findClosestProximity(const defaulttype::Vector3 & P) {
        TriangleDecorator * decorator;
        this->getContext()->get(decorator);

        if (decorator == NULL) {
            ConstraintProximity min_pinfo;
            double minDist = 0;

            for(int tri=0;tri<this->getTopology()->getNbTriangles();tri++) {
                ConstraintProximity pinfo = projectPointOnTriangle(tri,P);

                defaulttype::Vector3 Q = pinfo.getPosition();

                double dist = (Q-P).norm();

                if ((tri==0) || (dist < minDist)) {
                    min_pinfo = pinfo;
                    minDist = dist;
                }
            }

            return min_pinfo;
        } else {
            return decorator->findClosestTriangle(this,P);
        }
    }

    virtual defaulttype::Vector3 getSurfaceNormal(const ConstraintProximity & pinfo) = 0;

    virtual defaulttype::Vector3 getTriangleNormal(unsigned eid) {
        const helper::ReadAccessor<Data <typename DataTypes::VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());
        sofa::core::topology::Topology::Triangle tri = this->getTopology()->getTriangle(eid);

        defaulttype::Vector3 N1 = cross(x[tri[1]] - x[tri[0]],x[tri[2]] - x[tri[0]]);
        N1.normalize();

        return N1;
    }

    virtual ConstraintProximity projectPointOnTriangle(unsigned tid, const defaulttype::Vector3 & P) = 0;

};



} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
