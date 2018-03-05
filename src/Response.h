#pragma once

#include <BaseGeometry.h>
#include <CollisionAlgorithm.h>

namespace sofa {

namespace core {

namespace behavior {

class ConstraintNormal {
public:

    typedef std::shared_ptr<ConstraintNormal> ConstraintNormalPtr;

    ConstraintNormal(PariProximity pp, defaulttype::Vector3 N1) {
        m_pproxy = pp;
        m_normals.push_back(N1);
    }

    ConstraintNormal(PariProximity pp, defaulttype::Vector3 N1, defaulttype::Vector3 N2) {
        m_pproxy = pp;
        m_normals.push_back(N1);
        m_normals.push_back(N2);
    }

    ConstraintNormal(PariProximity pp, defaulttype::Vector3 N1,defaulttype::Vector3 N2,defaulttype::Vector3 N3) {
        m_pproxy = pp;
        m_normals.push_back(N1);
        m_normals.push_back(N2);
        m_normals.push_back(N3);
    }

    static ConstraintNormal createSingle(PariProximity pp, defaulttype::Vector3 N1) {
        if (N1.norm() < 0.0000001) N1 = defaulttype::Vector3(1,0,0);
        N1.normalize();

        return ConstraintNormal(pp,N1);
    }

    static ConstraintNormal createFrame(PariProximity pp, defaulttype::Vector3 N1) {
        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
        N1.normalize();
        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
        N2.normalize();
        defaulttype::Vector3 N3 = cross(N1,N2);
        N3.normalize();

        return ConstraintNormal(pp, N1,N2,N3);
    }

    const helper::vector<defaulttype::Vector3> & normals() const {
        return m_normals;
    }

    unsigned size() const {
        return m_normals.size();
    }

    void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned int constraintId) const {
        for (unsigned n = 0;n<m_normals.size();n++) {
            m_pproxy.first->buildConstraintMatrix(cParams,cId,constraintId+n,m_normals[n]);
            m_pproxy.second->buildConstraintMatrix(cParams,cId,constraintId+n,-m_normals[n]);
        }
    }

private:
    PariProximity m_pproxy;
    helper::vector<defaulttype::Vector3> m_normals;
};


class Response : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(Response, sofa::core::objectmodel::BaseObject);

    virtual void getConstraintNormals(const PariProximityVector & pp, std::vector<ConstraintNormal> & out) = 0;

    virtual void getConstraintViolation(const PariProximity & detection, const ConstraintNormal & normal, defaulttype::BaseVector *v,unsigned cid) = 0;

    virtual core::behavior::ConstraintResolution * getResolution(const ConstraintNormal & normal) = 0;
};


} // namespace controller

} // namespace component

} // namespace sofa

