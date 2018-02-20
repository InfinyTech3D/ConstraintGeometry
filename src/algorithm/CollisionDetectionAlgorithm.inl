#ifndef SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H
#define SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H

#include "CollisionDetectionAlgorithm.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>

namespace sofa {

namespace core {

namespace behavior {


CollisionDetectionAlgorithm::CollisionDetectionAlgorithm()
{}

PariProximity  CollisionDetectionAlgorithm::getClosestPoint(ConstraintElementPtr efrom,BaseGeometry * dest) {
    double min_dist = std::numeric_limits<double>::max();

    ElementIteratorPtr it = dest->getElementIterator();

    ConstraintProximityPtr pfrom = efrom->getDefaultProximity();
    defaulttype::Vector3 P = pfrom->getPosition();
    PariProximity min_pair;

    do {
        ConstraintProximityPtr pdest = it->getElement()->project(P);
        pfrom = efrom->project(pdest->getPosition());

        //iterate until to find the correct location on pfrom
        for (int itearation = 0;itearation<10 && (P-pfrom->getPosition()).norm()>0.0001;itearation++) {
            P = pfrom->getPosition();
            pdest = it->getElement()->project(P);
            pfrom = efrom->project(pdest->getPosition());
        }

        //compute all the distances with to elements
        double dist = (pfrom->getPosition() - pdest->getPosition()).norm();

        if (dist<min_dist) {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pdest;
        }
    } while (it->next());

    return min_pair;
}

void CollisionDetectionAlgorithm::processAlgorithm() {
    ElementIteratorPtr iter = m_from->getElementIterator();

    do {
        PariProximity pair = getClosestPoint(iter->getElement(),m_dest);

        if (pair.first == NULL) continue;
        if (pair.second == NULL) continue;

        m_pairDetection.push_back(pair);
    } while(iter->next());
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
