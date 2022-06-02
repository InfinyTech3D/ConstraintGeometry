#pragma once

#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/BaseNormalHandler.h>

namespace sofa::constraintGeometry {

//Specific operation to find the closest point on a geometry (the code is in the c++ class)
class ConstraintProximityOperation : public collisionAlgorithm::Operations::GenericOperation<ConstraintProximityOperation,//operation type
                                                                                             ConstraintProximity::SPtr, // default return
                                                                                             BaseNormalHandler *, collisionAlgorithm::BaseProximity::SPtr // parameters
                                                                                            > {
public:

    ConstraintProximity::SPtr defaultFunc(BaseNormalHandler *, collisionAlgorithm::BaseProximity::SPtr ) const override {
        return NULL;
    }

    void notFound(const std::type_info & id) const override {
        std::cerr << "ConstraintProximityOperation has no registered function for type " << sofa::helper::NameDecoder::decodeFullName(id) << std::endl;
    }

};

}
