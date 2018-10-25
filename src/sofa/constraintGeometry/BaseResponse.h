#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <memory>

namespace sofa {

namespace constraintGeometry {

class ConstraintNormal;

class BaseResponse : public core::objectmodel::BaseObject {
public:
    SOFA_CLASS(BaseResponse, core::objectmodel::BaseObject);

    virtual unsigned size() = 0;

    virtual core::behavior::ConstraintResolution* getConstraintResolution() = 0;

};


template<class RESOLUTION>
class TResponse : public BaseResponse {
public:
    SOFA_CLASS(SOFA_TEMPLATE(TResponse,RESOLUTION) , BaseResponse);

    virtual std::string getTemplateName() const {
        return templateName(this);
    }

    static std::string templateName(const TResponse<RESOLUTION>* = NULL) {
        return RESOLUTION::Name();
    }

    static std::string className(const TResponse<RESOLUTION>* = NULL) {
        return "ConstraintResponse";
    }

//    virtual std::string getClassName() const {
//        return "ConstraintResponse";
//    }

//    virtual unsigned size() {
//        return RESOLUTION::size();
//    }
};

}

}
