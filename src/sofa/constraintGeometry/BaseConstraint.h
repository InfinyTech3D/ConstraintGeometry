#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/objectmodel/DataLink.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>

namespace sofa {

namespace constraintGeometry {

//This class should not be creataed directly
class ConstraintNormal {
public:
    unsigned size() const {
        return m_normals.size();
    }

    ConstraintNormal() {}

    ConstraintNormal(defaulttype::Vector3 n1) {
        m_normals.push_back(n1.normalized());
    }

    ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2) {
        m_normals.push_back(n1.normalized());
        m_normals.push_back(n2.normalized());
    }

    ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2,defaulttype::Vector3 n3) {
        m_normals.push_back(n1.normalized());
        m_normals.push_back(n2.normalized());
        m_normals.push_back(n3.normalized());
    }

    void normalize(unsigned sz) {
        if (m_normals.size() > sz) m_normals.resize(sz);
    }

    static ConstraintNormal createFrame(defaulttype::Vector3 N1 = defaulttype::Vector3()) {
        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
        N1.normalize();
        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
        N2.normalize();
        defaulttype::Vector3 N3 = cross(N1,N2);
        N3.normalize();

        ConstraintNormal cn;
        cn.m_normals.push_back(N1);
        cn.m_normals.push_back(N2);
        cn.m_normals.push_back(N3);
        return cn;
    }

    std::vector<defaulttype::Vector3> m_normals;
};

class InternalConstraint {
public:
    typedef std::shared_ptr<InternalConstraint> SPtr;

    virtual void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int constraintId) = 0;

    virtual void getConstraintViolation(defaulttype::BaseVector *v,unsigned cid) = 0;

    virtual unsigned size() = 0;

    virtual void normalize(unsigned sz) = 0;

    virtual void draw(const core::visual::VisualParams* vparams, double scale, defaulttype::Vector4 c) = 0;

    static InternalConstraint::SPtr createPairConstraint(collisionAlgorithm::PairProximity pproxy, ConstraintNormal cn);
};

class PairConstraint : public InternalConstraint {
public:
    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    PairConstraint(collisionAlgorithm::PairProximity pproxy, ConstraintNormal cn) : m_pproxy(pproxy), m_directions(cn) {}

    virtual void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int constraintId) {
        collisionAlgorithm::ConstraintProximity::SPtr prox1 = m_pproxy.first;

        { // object 1
            DataMatrixDeriv & c1_d = *cId[prox1->getState()].write();
            MatrixDeriv & c1 = *c1_d.beginEdit();

            std::map<unsigned, double> m1 = prox1->getContributions();
            for (unsigned j=0;j<m_directions.size();j++) {
                MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);

                for (auto it=m1.begin();it!=m1.end();it++) {
                    c_it.addCol(it->first, m_directions.m_normals[j] * it->second);
                }
            }

            c1_d.endEdit();
        }

        { // object 2
            collisionAlgorithm::ConstraintProximity::SPtr prox2 = m_pproxy.second;
            DataMatrixDeriv & c2_d = *cId[prox2->getState()].write();
            MatrixDeriv & c2 = *c2_d.beginEdit();

            std::map<unsigned, double> m2 = prox2->getContributions();
            for (unsigned j=0;j<m_directions.size();j++) {
                MatrixDerivRowIterator c_it = c2.writeLine(constraintId+j);

                for (auto it=m2.begin();it!=m2.end();it++) {
                    c_it.addCol(it->first, -m_directions.m_normals[j] * it->second);
                }
            }

            c2_d.endEdit();
        }
    }

    virtual void getConstraintViolation(defaulttype::BaseVector *v,unsigned cid) {
        collisionAlgorithm::ConstraintProximity::SPtr prox1 = m_pproxy.first;
        collisionAlgorithm::ConstraintProximity::SPtr prox2 = m_pproxy.second;

        defaulttype::Vector3 PFree = prox1->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 QFree = prox2->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_directions.m_normals.size();i++) {
            v->set(cid+i,dot(PQFree,m_directions.m_normals[i]));
        }
    }

    virtual unsigned size() {
        return m_directions.size();
    }

    virtual void normalize(unsigned sz) {
        m_directions.normalize(sz);
    }


    virtual void draw(const core::visual::VisualParams* vparams, double scale, defaulttype::Vector4 c) {
        for (unsigned i=0;i<m_directions.size();i++) {
            vparams->drawTool()->drawArrow(m_pproxy.second->getPosition(),
                                           m_pproxy.second->getPosition() + m_directions.m_normals[i] * scale,
                                           scale*0.1,
                                           c);

        }
    }

protected:
    collisionAlgorithm::PairProximity m_pproxy;
    ConstraintNormal m_directions;

};

InternalConstraint::SPtr InternalConstraint::createPairConstraint(collisionAlgorithm::PairProximity pproxy, ConstraintNormal cn) {
    return std::shared_ptr<InternalConstraint>(new PairConstraint(pproxy,cn));
}

class BaseConstraint : public sofa::core::behavior::BaseConstraint {
public:
    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    DataLink<BaseResponse> d_response;
    Data<double> d_drawScale;
    Data<defaulttype::Vector4> d_drawColor;

    BaseConstraint()
    : d_response(initData(&d_response, "response", "Response"))
    , d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale"))
    , d_drawColor(initData(&d_drawColor, defaulttype::Vector4(1,0,0,1), "draw_color", "draw color")){}

    virtual void createConstraints() = 0;

    virtual void getState(std::set<sofa::core::behavior::MechanicalState<DataTypes>* > & list_state) = 0;

    void addConstraint(InternalConstraint::SPtr c) {
        m_constraints.push_back(c);
    }

    void processGeometricalData() {
        m_constraints.clear();
        m_state.clear();

        getState(m_state);
        createConstraints();

        unsigned sz = d_response->size();
        for (unsigned i=0;i<m_constraints.size();i++) m_constraints[i]->normalize(sz);
    }

    void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->buildConstraintMatrix(cId,constraintId);
            constraintId += m_constraints[i]->size();
        }
    }

    void getConstraintViolation(const core::ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned cid) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->getConstraintViolation(v, cid);
            cid += m_constraints[i]->size();
        }
    }

    void getConstraintResolution(const core::ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            resTab[offset] = d_response->getConstraintResolution();
            offset+=m_constraints[i]->size();
        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowInteractionForceFields()) return;

        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->draw(vparams,d_drawScale.getValue(),d_drawColor.getValue());
        }
    }

    void storeLambda(const core::ConstraintParams* cParams, Data<VecDeriv>& result, const Data<MatrixDeriv>& jacobian, const sofa::defaulttype::BaseVector* lambda) {
        auto res = sofa::helper::write(result, cParams);
        const MatrixDeriv& j = jacobian.getValue(cParams);
        j.multTransposeBaseVector(res, lambda ); // lambda is a vector of scalar value so block size is one.
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) {
        if (cParams) {
            for (auto it=m_state.cbegin();it!=m_state.cend();it++) {
                storeLambda(cParams, *res[(*it)].write(), *cParams->readJ((*it)), lambda);
            }
        }
    }

    void updateForceMask() {}

protected:        
    helper::vector<InternalConstraint::SPtr> m_constraints;
    std::set<sofa::core::behavior::MechanicalState<DataTypes> * > m_state;
};

}

}
