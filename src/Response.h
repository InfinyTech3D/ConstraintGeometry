#pragma once

#include <constraintGeometry.h>
#include <Collision.h>
#include <memory>

namespace constraintGeometry {

class ConstraintNormal {
public:

    typedef std::shared_ptr<ConstraintNormal> ConstraintNormalPtr;

    ConstraintNormal(collisionAlgorithm::PairProximity pp, Vector3 N1) {
        m_pproxy = pp;
        m_normals.push_back(N1);
    }

    ConstraintNormal(collisionAlgorithm::PairProximity pp, Vector3 N1, Vector3 N2) {
        m_pproxy = pp;
        m_normals.push_back(N1);
        m_normals.push_back(N2);
    }

    ConstraintNormal(collisionAlgorithm::PairProximity pp, Vector3 N1,Vector3 N2,Vector3 N3) {
        m_pproxy = pp;
        m_normals.push_back(N1);
        m_normals.push_back(N2);
        m_normals.push_back(N3);
    }

    static ConstraintNormal createSingle(collisionAlgorithm::PairProximity pp, Vector3 N1) {
        if (N1.norm() < 0.0000001) N1 = Vector3(1,0,0);
        N1.normalize();

        return ConstraintNormal(pp,N1);
    }

    static ConstraintNormal createFrame(collisionAlgorithm::PairProximity pp, Vector3 N1) {
        if (N1.norm() == 0) N1 = Vector3(1,0,0);
        N1.normalize();
        Vector3 N2 = cross(N1,((fabs(dot(N1,Vector3(0,1,0)))>0.99) ? Vector3(0,0,1) : Vector3(0,1,0)));
        N2.normalize();
        Vector3 N3 = cross(N1,N2);
        N3.normalize();

        return ConstraintNormal(pp, N1,N2,N3);
    }

    const std::vector<Vector3> & normals() const {
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
    collisionAlgorithm::PairProximity m_pproxy;
    std::vector<Vector3> m_normals;
};


class Response : public BaseObject {
public:
    virtual void getConstraintNormals(const PariProximityVector & pp, std::vector<ConstraintNormal> & out) = 0;

    virtual void getConstraintViolation(const PariProximity & detection, const ConstraintNormal & normal, defaulttype::BaseVector *v,unsigned cid) = 0;

    virtual core::behavior::ConstraintResolution * getResolution(const ConstraintNormal & normal) = 0;
};


}

