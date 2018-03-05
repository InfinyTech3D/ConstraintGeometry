#pragma once

#include <BaseGeometry.h>

namespace sofa {

namespace core {

namespace behavior {

typedef std::pair<ConstraintProximityPtr,ConstraintProximityPtr> PariProximity;
typedef helper::vector<PariProximity > PariProximityVector;

class CollisionAlgorithm : public sofa::core::objectmodel::BaseObject {
public :
    SOFA_CLASS(CollisionAlgorithm, sofa::core::objectmodel::BaseObject);

    virtual void processAlgorithm(BaseGeometry * from, BaseGeometry * dest, PariProximityVector & detection) = 0;

};

} // namespace controller

} // namespace component

} // namespace sofa

