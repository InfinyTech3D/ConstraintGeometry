#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseForceField.h>

namespace sofa {

namespace constraintGeometry {

template<class DataTypes>
class CollisionForceField :  public sofa::core::behavior::BaseForceField {
public:
    SOFA_CLASS(CollisionForceField, sofa::core::behavior::BaseForceField);

    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef Data<VecDeriv> DataVecDeriv;

    Data<double> d_stiffness;

    CollisionForceField();

    void addForce(const core::MechanicalParams* mparams, core::MultiVecDerivId fId );

//    void addToMatrix(BaseMatrix *M);

    virtual void addDForce(const core::MechanicalParams* mparams, core::MultiVecDerivId dfId ) {}

    virtual SReal getPotentialEnergy( const core::MechanicalParams* mparams) const {}

    virtual void addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix ) {}

    void draw(const core::visual::VisualParams * );

    void updateForceMask() {}

protected:
    core::objectmodel::SingleLink<CollisionForceField,collisionAlgorithm::BaseCollisionAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_collision;
};

}

}
