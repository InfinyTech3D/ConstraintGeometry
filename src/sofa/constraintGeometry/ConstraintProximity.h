#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <memory>

namespace sofa::constraintGeometry {

class ConstraintProximity {
public:

    typedef std::shared_ptr<ConstraintProximity> SPtr;

    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        return getProximity()->getPosition(v);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const {
        getProximity()->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const {
        getProximity()->storeLambda(cParams,resId,cid_global,cid_local,lambda);
    }

    virtual type::Vector3 getNormal() = 0;

    virtual collisionAlgorithm::BaseProximity::SPtr getProximity() const = 0;

};

}
