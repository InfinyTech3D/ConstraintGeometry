#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <memory>

namespace sofa::constraintGeometry {

class ConstraintProximity {
public:

    typedef std::shared_ptr<ConstraintProximity> SPtr;

    virtual sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId , const sofa::type::vector<sofa::type::Vector3> & , double , Index ) const = 0;

    virtual void storeLambda(const core::ConstraintParams* , core::MultiVecDerivId , Index , Index , const sofa::linearalgebra::BaseVector* ) const = 0;

    virtual type::Vector3 getNormal() = 0;

};

template<class PROXIMITY>
class TConstraintProximity : public ConstraintProximity {
public:
    typedef std::function<type::Vector3(const typename PROXIMITY::SPtr & p)> FUNC_GET_NORMAL;

    TConstraintProximity(typename PROXIMITY::SPtr & p,FUNC_GET_NORMAL n)
    : m_prox(p)
    , m_normalFunc(n) {}

    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        return m_prox->getPosition(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const {
        m_prox->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const {
        m_prox->storeLambda(cParams,resId,cid_global,cid_local,lambda);
    }

    type::Vector3 getNormal() {
        return m_normalFunc(m_prox);
    }

    static ConstraintProximity::SPtr create(typename PROXIMITY::SPtr & p,FUNC_GET_NORMAL n) {
        return ConstraintProximity::SPtr(new TConstraintProximity(p,n));
    }

protected:
    typename PROXIMITY::SPtr m_prox;
    FUNC_GET_NORMAL m_normalFunc;
};

}
