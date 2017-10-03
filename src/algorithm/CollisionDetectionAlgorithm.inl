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
: d_from(initData(&d_from, "from", "From geometry"))
, d_dest(initData(&d_dest, "dest", "Dest geometry"))
, d_maxf(initData(&d_maxf, std::numeric_limits<double>::max(), "maxForce", "maxForce"))
{}

void CollisionDetectionAlgorithm::init() {
    this->getContext()->get(m_from,d_from.getValue());
    this->getContext()->get(m_dest,d_dest.getValue());

    if (m_from == NULL) serr << "Error cannot find from topology" << std::endl;
    if (m_dest == NULL) serr << "Error cannot find dest topology" << std::endl;
}

CollisionDetectionAlgorithm::PariProximity CollisionDetectionAlgorithm::getClosestPoint(unsigned i) {
    double min_dist = std::numeric_limits<double>::max();

    PariProximity min_pair;
    min_pair.first = NULL;
    min_pair.second = NULL;

    ConstraintProximityPtr initFrom = m_from->getElementProximity(i);

    ElementIteratorPtr it;
    if (BroadPhase* broad = dynamic_cast<BroadPhase*>(m_dest)) it = broad->getBroadPhaseIterator(initFrom);
    else it = m_dest->getElementIterator();

    do {
        ConstraintProximityPtr pfrom = m_from->getElementProximity(i);
        defaulttype::Vector3 P = pfrom->getPosition();

        ConstraintProximityPtr pdest = it->getElementProximity();
        pdest->refineToClosestPoint(P);

        //iterate until to find the correct location on pfrom
        for (int itearation = 0;itearation<30;itearation++) {
            pfrom->refineToClosestPoint(pdest->getPosition());
            if ((P-pfrom->getPosition()).norm() < 0.0001) break;
            P = pfrom->getPosition();
            pdest->refineToClosestPoint(P);
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

void CollisionDetectionAlgorithm::processAlgorithm(helper::vector<ConstraintNormalPtr> & cn) {
    if (m_from == NULL) return;
    if (m_dest == NULL) return;

    for (int i=0;i<m_from->getNbElements();i++) {
        PariProximity pair = getClosestPoint(i);
        if (pair.first == NULL) continue;
        if (pair.second == NULL) continue;

        helper::fixed_array<defaulttype::Vector3,1> normals;
        defaulttype::Vector3 P = pair.first->getPosition();
        defaulttype::Vector3 Q = pair.second->getPosition();
        defaulttype::Vector3 N = pair.second->getNormal().normalized();


        if (dot(P-Q,N)<=0 || (P-Q).norm()<0.1) {
            normals[0] = N;
        } else {
            normals[0] = P-Q;
        }

        cn.push_back(std::make_shared<FrictionConstraintNormal>(pair,normals));
    }
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
