#pragma once

#include <CollisionAlgorithm.h>

namespace sofa {

namespace core {

namespace behavior {

class CollisionDetectionAlgorithm : public CollisionAlgorithm {
public:

    SOFA_CLASS(CollisionDetectionAlgorithm , CollisionAlgorithm );

    CollisionDetectionAlgorithm();

    void processAlgorithm(BaseGeometry * from, BaseGeometry * dest, PariProximityVector & detection);

private:

    PariProximity getClosestPoint(ConstraintElementPtr efrom, BaseGeometry * dest);

};

} // namespace controller

} // namespace component

} // namespace sofa
