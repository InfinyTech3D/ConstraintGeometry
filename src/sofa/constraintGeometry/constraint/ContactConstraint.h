#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/objectmodel/DataLink.h>
#include <sofa/constraintGeometry/BaseConstraint.h>

namespace sofa {

namespace constraintGeometry {

class ContactConstraint : public BaseConstraint {
public:
    SOFA_CLASS(ContactConstraint, BaseConstraint);

    DataLink<collisionAlgorithm::BaseCollisionAlgorithm> d_algo;

    ContactConstraint()
    : d_algo(initData(&d_algo, "algo", "algo")) {}

    void createConstraints() {
        for (unsigned i=0;i<d_algo->getCollisionPairs().size();i++) {
            collisionAlgorithm::PairProximity pair = d_algo->getCollisionPairs()[i];
            ConstraintNormal cn = ConstraintNormal::createFrame(pair.first->getPosition() - pair.second->getPosition());
            addConstraint(InternalConstraint::createPairConstraint(pair,cn));
        }
    }

    void getState(std::set<sofa::core::behavior::MechanicalState<DataTypes>* > & list_state) {
        d_algo->getState(list_state);
    }
};

}

}
