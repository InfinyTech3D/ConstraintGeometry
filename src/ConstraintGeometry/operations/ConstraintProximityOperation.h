#pragma once

#include <CollisionAlgorithm/BaseOperation.h>
#include <ConstraintGeometry/ConstraintProximity.h>

namespace sofa::constraintGeometry {

class BaseNormalHandler;

//Specific operation to find the closest point on a geometry (the code is in the c++ class)
class ConstraintProximityOperation : public collisionAlgorithm::Operations::GenericOperation2<ConstraintProximityOperation,//operation type
                                                                                              ConstraintProximity::SPtr, // default return
                                                                                              BaseNormalHandler *, collisionAlgorithm::BaseProximity::SPtr // parameters
                                                                                             > {
public:

    ConstraintProximity::SPtr defaultFunc(BaseNormalHandler *, collisionAlgorithm::BaseProximity::SPtr ) const override {
        return NULL;
    }

    void notFound(const std::type_info & id1,const std::type_info & id2) const override {
        std::cerr << "ConstraintProximityOperation has no registered function for type " << sofa::helper::NameDecoder::decodeFullName(id1) << "-" << sofa::helper::NameDecoder::decodeFullName(id2) << std::endl;
    }

};

}
