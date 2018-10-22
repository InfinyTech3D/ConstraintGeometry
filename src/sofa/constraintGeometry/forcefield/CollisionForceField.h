#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/objectmodel/DataLink.h>

namespace sofa {

namespace constraintGeometry {

template<class DataTypes>
class CollisionForceField :  public sofa::core::behavior::BaseForceField {
public:

    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef Data<VecDeriv> DataVecDeriv;

    Data<double> d_stiffness;
    DataLink<collisionAlgorithm::BaseCollisionAlgorithm> d_collision;

    CollisionForceField();

    void addForce(const core::MechanicalParams* mparams, core::MultiVecDerivId fId );

//    void addToMatrix(BaseMatrix *M);

    virtual void addDForce(const core::MechanicalParams* mparams, core::MultiVecDerivId dfId ) {}

    virtual SReal getPotentialEnergy( const core::MechanicalParams* mparams) const {}

    virtual void addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix ) {}

    void draw(const core::visual::VisualParams * );

    void updateForceMask() {}
};

}

}
