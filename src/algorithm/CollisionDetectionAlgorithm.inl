#pragma once

#include <algorithm/CollisionDetectionAlgorithm.h>

namespace sofa {

namespace core {

namespace behavior {

CollisionDetectionAlgorithm::CollisionDetectionAlgorithm()
//: d_from(initData(&d_from, "from", "Dest geometry"))
//, d_dest(initData(&d_dest, "dest", "Dest geometry"))
{}

PariProximity  CollisionDetectionAlgorithm::getClosestPoint(ConstraintElementPtr efrom,BaseGeometry * dest) {
    double min_dist = std::numeric_limits<double>::max();

    ConstraintProximityPtr pfrom = efrom->getDefaultProximity();
    defaulttype::Vector3 P = pfrom->getPosition();
    PariProximity min_pair;

    for (unsigned i=0;i<dest->getNbElements();i++) {
        ConstraintElementPtr edest = dest->getElement(i);
        ConstraintProximityPtr pdest = edest->project(P);
        pfrom = efrom->project(pdest->getPosition());

        //iterate until to find the correct location on pfrom
        for (int itearation = 0;itearation<10 && (P-pfrom->getPosition()).norm()>0.0001;itearation++) {
            P = pfrom->getPosition();
            pdest = edest->project(P);
            pfrom = efrom->project(pdest->getPosition());
        }

        //compute all the distances with to elements
        double dist = (pfrom->getPosition() - pdest->getPosition()).norm();

        if (dist<min_dist) {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pdest;
        }
    }

    return min_pair;
}

void CollisionDetectionAlgorithm::processAlgorithm(BaseGeometry * from, BaseGeometry * dest, PariProximityVector & detection) {
    for (unsigned i=0;i<from->getNbElements();i++) {
        PariProximity pair = getClosestPoint(from->getElement(i),dest);

        if (pair.first == NULL) continue;
        if (pair.second == NULL) continue;

        detection.push_back(pair);
    }
}

} // namespace controller

} // namespace component

} // namespace sofa
