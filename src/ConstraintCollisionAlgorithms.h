#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "ConstraintGeometry.h"

namespace sofa {

namespace core {

namespace behavior {

class CollisionDecorator {

   void findClosestPoint(ConstraintProximity & src, ConstraintProximity & dst);

};


class CollisionAlgorithm {

    static void findClosestPoint(ConstraintProximity & from, ConstraintProximity & dst) {
        dst.getCollisionAlgorithm()->findClosestPoint(from);
    }


};

void CollisionAlgorithm::findClosestPoint(ConstraintProximity & src, ConstraintProximity & dst) {




    TriangleDecorator * decorator;
    this->getContext()->get(decorator);

    if (decorator == NULL) {
        ConstraintProximity min_pinfo;
        double minDist = 0;

        for(int tri=0;tri<this->getTopology()->getNbTriangles();tri++) {
            ConstraintProximity pinfo = projectPointOnTriangle(tri,P);

            defaulttype::Vector3 Q = pinfo.getPosition();

            double dist = (Q-P).norm();

            if ((tri==0) || (dist < minDist)) {
                min_pinfo = pinfo;
                minDist = dist;
            }
        }

        return min_pinfo;
    } else {
        return decorator->findClosestTriangle(this,P);
    }
}


} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
