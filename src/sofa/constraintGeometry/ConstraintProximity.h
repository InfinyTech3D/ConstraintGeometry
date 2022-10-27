#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <memory>

namespace sofa::constraintGeometry {

class ConstraintProximity : public virtual collisionAlgorithm::BaseBaseProximity {
public:

    typedef std::shared_ptr<ConstraintProximity> SPtr;

    virtual type::Vector3 getNormal() const = 0;

};

template<class PROXIMITY>
class TConstraintProximity : public ConstraintProximity {
public:
    typedef std::function<type::Vector3(const typename PROXIMITY::SPtr & p)> FUNC_GET_NORMAL;

    TConstraintProximity(const typename PROXIMITY::SPtr & p,FUNC_GET_NORMAL n)
    : m_prox(p)
    , m_normalFunc(n) {}

    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_prox->getPosition(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        m_prox->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_prox->storeLambda(cParams,resId,cid_global,cid_local,lambda);
    }

    type::Vector3 getNormal() const override {
        return m_normalFunc(m_prox);
    }

    static ConstraintProximity::SPtr create(const typename PROXIMITY::SPtr & p,FUNC_GET_NORMAL n) {
        return ConstraintProximity::SPtr(new TConstraintProximity(p,n));
    }

protected:
    typename PROXIMITY::SPtr m_prox;
    FUNC_GET_NORMAL m_normalFunc;
};

template<class PROXIMITY>
class BaseProximityNormal : public PROXIMITY, public constraintGeometry::ConstraintProximity {
public:
    typedef std::shared_ptr<BaseProximityNormal> SPtr;

    template<class... ARGS>
    BaseProximityNormal(ARGS... args)
    : PROXIMITY(args...) {}

    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return PROXIMITY::getPosition(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId id, const sofa::type::vector<sofa::type::Vector3> & N, double f, Index i) const override {
        PROXIMITY::buildJacobianConstraint(id,N,f,i);
    }

    void storeLambda(const core::ConstraintParams* cp, core::MultiVecDerivId id, Index i, Index j, const sofa::linearalgebra::BaseVector* l) const override {
        PROXIMITY::storeLambda(cp,id,i,j,l);
    }

    virtual sofa::type::Vector3 getNormal() const = 0;
};



class DefaultConstraintProximity : public ConstraintProximity {
public:

    typedef std::shared_ptr<DefaultConstraintProximity> SPtr;

    DefaultConstraintProximity(const collisionAlgorithm::BaseProximity::SPtr & p)
    : m_prox(p) {}

    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_prox->getPosition(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        m_prox->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_prox->storeLambda(cParams,resId,cid_global,cid_local,lambda);
    }

    type::Vector3 getNormal() const override {
        return type::Vector3();
    }

    static ConstraintProximity::SPtr create(const collisionAlgorithm::BaseProximity::SPtr & p) {
        return ConstraintProximity::SPtr(new DefaultConstraintProximity(p));
    }

protected:
    collisionAlgorithm::BaseProximity::SPtr m_prox;
};



class MultiConstraintProximity : public ConstraintProximity {
public:
    typedef std::shared_ptr<MultiConstraintProximity> SPtr;

    MultiConstraintProximity(const std::vector<ConstraintProximity::SPtr> & p)
    : m_proximities(p) {}

    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        sofa::type::Vector3 P(0,0,0);
        for (unsigned i=0; i<m_proximities.size(); i++) P+=m_proximities[i]->getPosition(v);
        return P*1.0/m_proximities.size();
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        fact *= 1.0/m_proximities.size();
        for (unsigned i=0; i<m_proximities.size(); i++) {
            m_proximities[i]->buildJacobianConstraint(cId,dir,fact,constraintId);
        }
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        for (unsigned i=0; i<m_proximities.size(); i++) {
            m_proximities[i]->storeLambda(cParams,resId,cid_global,cid_local,lambda);
        }
    }

    type::Vector3 getNormal() const override {
        sofa::type::Vector3 N(0,0,0);
        for (unsigned i=0; i<m_proximities.size(); i++) N+=m_proximities[i]->getNormal();
        return N*1.0/m_proximities.size();
    }

    static ConstraintProximity::SPtr create(const std::vector<ConstraintProximity::SPtr> & p) {
        return ConstraintProximity::SPtr(new MultiConstraintProximity(p));
    }

protected:
    std::vector<ConstraintProximity::SPtr> m_proximities;
};

}
