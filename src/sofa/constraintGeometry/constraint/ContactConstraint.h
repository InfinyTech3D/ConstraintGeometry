#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/constraintGeometry/BaseConstraint.h>

namespace sofa {

namespace constraintGeometry {

class ContactConstraint : public BaseConstraint {
public:
    SOFA_CLASS(ContactConstraint, BaseConstraint);

    ContactConstraint()
    : l_algo(initLink("algo", "Link to algo")) {}

    void createConstraints() {
        for (unsigned i=0;i<l_algo->getCollisionPairs().size();i++) {
            collisionAlgorithm::PairProximity pair = l_algo->getCollisionPairs()[i];

            defaulttype::Vector3 mainDir = pair.first->getPosition() - pair.second->getPosition();
            defaulttype::Vector3 secondDir = pair.second->getNormal();

            if (mainDir.norm()<0.01) mainDir = secondDir;
            else {
                mainDir.normalize();
                if (dot(mainDir,secondDir) < 0) mainDir = secondDir;
            }


            ConstraintNormal cn = ConstraintNormal::createFrame(mainDir);

            addConstraint(InternalConstraint::createPairConstraint(pair,cn));
        }
    }

    void getState(std::set<sofa::core::behavior::MechanicalState<DataTypes>* > & list_state) {
        l_algo->getState(list_state);
    }

protected:
    core::objectmodel::SingleLink<ContactConstraint,collisionAlgorithm::BaseCollisionAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_algo;

};

}

}
