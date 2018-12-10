#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <memory>

namespace sofa {

namespace constraintGeometry {

//This class should not be creataed directly
class ConstraintNormal {
public:
    unsigned size() const {
        return m_normals.size();
    }

    ConstraintNormal() {}

    ConstraintNormal(defaulttype::Vector3 n1) {
        m_normals.push_back(n1.normalized());
    }

    ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2) {
        m_normals.push_back(n1.normalized());
        m_normals.push_back(n2.normalized());
    }

    ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2,defaulttype::Vector3 n3) {
        m_normals.push_back(n1.normalized());
        m_normals.push_back(n2.normalized());
        m_normals.push_back(n3.normalized());
    }

    void normalize(unsigned sz) {
        if (m_normals.size() > sz) m_normals.resize(sz);
    }

    static ConstraintNormal createFrame(defaulttype::Vector3 N1 = defaulttype::Vector3()) {
        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
        defaulttype::Vector3 N3 = cross(N1,N2);

        return ConstraintNormal(N1,N2,N3);
    }

    std::vector<defaulttype::Vector3> m_normals;
};

class BaseResponse : public core::objectmodel::BaseObject {
public:
    SOFA_CLASS(BaseResponse, core::objectmodel::BaseObject);

    virtual unsigned size() = 0;

    virtual core::behavior::ConstraintResolution* getConstraintResolution() = 0;

    virtual ConstraintNormal createConstraintNormal(defaulttype::Vector3 mainDir) = 0;

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
