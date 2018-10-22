#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <memory>

namespace sofa {

namespace constraintGeometry {

class ConstraintResponse;

class ConstraintResponse : public core::objectmodel::BaseObject {
public:

    class ConstraintNormal {
        friend class ConstraintResponse;

    public:
        ConstraintNormal(ConstraintResponse * r, collisionAlgorithm::PairProximity pp) : m_response(r), m_pproxy(pp) {}

        unsigned size() const {
            return m_normals.size();
        }

        inline void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int constraintId) const {
            collisionAlgorithm::ConstraintProximity::SPtr p1 = m_pproxy.first;
            collisionAlgorithm::ConstraintProximity::SPtr p2 = m_pproxy.second;

            std::map<unsigned,double> c1 = p1->getContributions();
            std::map<unsigned,double> c2 = p2->getContributions();



//            m_response->buildConstraintMatrix(*this, cId,constraintId);
        }

        inline void getConstraintViolation(defaulttype::BaseVector *v,unsigned cid) const {
//            m_response->getConstraintViolation(*this, v,cid);
        }

        inline core::behavior::ConstraintResolution* getConstraintResolution() const {
//            return m_response->getResolution();
        }

        void draw(const core::visual::VisualParams* /*vparams*/) {}

    protected:
        ConstraintResponse * m_response;
        collisionAlgorithm::PairProximity m_pproxy;
        std::vector<defaulttype::Vector3> m_normals;
    };

    DataLink<collisionAlgorithm::BaseCollisionAlgorithm> d_algo;
    Data<std::string> d_response;

    ConstraintResponse()
    : d_algo(initData(&d_algo, "algo", "Algorithm"))
    , d_response(initData(&d_response, "UFF", "reponse", "Reponse Type ")){}

    virtual void createResponse(helper::vector<ConstraintNormal> & normals) {
        unsigned size = d_response.getValue().length();
        if (size > 3 ) {
            size = 3;
            std::cerr << "Error only 3 first component of the reponse are used" << std::endl;
        }

        for (unsigned i=0;i<d_algo->getCollisionPairs().size();i++) {
            ConstraintNormal cn(this, d_algo->getCollisionPairs()[i]);
            for (unsigned sz = 0;sz<size;sz++) addConstraint(cn);
            normals.push_back(cn);
        }
    }

    virtual void addConstraint(ConstraintNormal & normal) = 0;

    virtual void getConstraintViolation(const defaulttype::Vector3 & N, collisionAlgorithm::ConstraintProximity::SPtr p1, collisionAlgorithm::ConstraintProximity::SPtr p2) const = 0;

    virtual core::behavior::ConstraintResolution * getResolution() = 0;

};




//    ConstraintNormal(collisionAlgorithm::PairProximity pp, Vector3 N1, Vector3 N2) {
//        m_pproxy = pp;
//        m_normals.push_back(N1);
//        m_normals.push_back(N2);
//    }

//    ConstraintNormal(collisionAlgorithm::PairProximity pp, Vector3 N1,Vector3 N2,Vector3 N3) {
//        m_pproxy = pp;
//        m_normals.push_back(N1);
//        m_normals.push_back(N2);
//        m_normals.push_back(N3);
//    }

//    static ConstraintNormal createSingle(collisionAlgorithm::PairProximity pp, Vector3 N1) {
//        if (N1.norm() < 0.0000001) N1 = Vector3(1,0,0);
//        N1.normalize();

//        return ConstraintNormal(pp,N1);
//    }

//    static ConstraintNormal createFrame(collisionAlgorithm::PairProximity pp, Vector3 N1) {
//        if (N1.norm() == 0) N1 = Vector3(1,0,0);
//        N1.normalize();
//        Vector3 N2 = cross(N1,((fabs(dot(N1,Vector3(0,1,0)))>0.99) ? Vector3(0,0,1) : Vector3(0,1,0)));
//        N2.normalize();
//        Vector3 N3 = cross(N1,N2);
//        N3.normalize();

//        return ConstraintNormal(pp, N1,N2,N3);
//    }
}

}

