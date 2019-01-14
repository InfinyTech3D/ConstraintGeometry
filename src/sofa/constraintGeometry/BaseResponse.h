#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <memory>

namespace sofa {

namespace constraintGeometry {

class BaseResponse : public sofa::core::objectmodel::BaseObject {
public:

    SOFA_ABSTRACT_CLASS(BaseResponse, sofa::core::objectmodel::BaseObject);

    class InternalConstraint;

    class ConstraintNormal {
        friend class InternalConstraint;

    public:
        ConstraintNormal(defaulttype::Vector3 n1) {
            m_dirs.push_back(n1.normalized());
        }

        ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2) {
            m_dirs.push_back(n1.normalized());
            m_dirs.push_back(n2.normalized());
        }

        ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2,defaulttype::Vector3 n3) {
            m_dirs.push_back(n1.normalized());
            m_dirs.push_back(n2.normalized());
            m_dirs.push_back(n3.normalized());
        }

        static ConstraintNormal createFrame(defaulttype::Vector3 N1 = defaulttype::Vector3()) {
            if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
            defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
            defaulttype::Vector3 N3 = cross(N1,N2);

            return ConstraintNormal(N1,N2,N3);
        }

        static ConstraintNormal createFromDetection(const collisionAlgorithm::DetectionOutput & d) {
//            defaulttype::Vector3 mainDir = d.getFirstProximity()->getPosition() - d.getSecondProximity()->getPosition();

//////            if (mainDir.norm()>0.00000001) return ConstraintNormal(mainDir);

//            defaulttype::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
            defaulttype::Vector3 secondDir = d.getSecondProximity()->getNormal().normalized();

//            return ConstraintNormal(mainDir.normalized() + firstDir + secondDir);
//            return ConstraintNormal(firstDir);
            return ConstraintNormal(secondDir);


//            defaulttype::Vector3 mainDir = d.getSecondProximity()->getNormal();//pair.first->getPosition() - pair.second->getPosition();
//            defaulttype::Vector3 secondDir = -d.getFirstProximity()->getNormal();

//            if (mainDir.norm()<0.01) mainDir = secondDir;
//            else {
//                mainDir.normalize();
//                if (dot(mainDir,secondDir) < 0) mainDir = secondDir;
//            }

//            return ConstraintNormal(mainDir);
        }

        static ConstraintNormal createFrameFromDetection(const collisionAlgorithm::DetectionOutput & d) {
            return createFrame(ConstraintNormal::createFromDetection(d).m_dirs[0]);
        }

        unsigned size() {
            return m_dirs.size();
        }

    protected:
        helper::vector<defaulttype::Vector3> m_dirs;
    };

    class InternalConstraint {
    public:
        InternalConstraint(collisionAlgorithm::BaseProximity::SPtr p1,collisionAlgorithm::BaseProximity::SPtr p2,ConstraintNormal normals,sofa::core::behavior::ConstraintResolution * resolution)
        : m_p1(p1)
        , m_p2(p2)
        , m_resolution(resolution)
        , m_normals(normals) {
            if (m_normals.size() != m_resolution->getNbLines()) {
                std::cerr << "ERROR you provided a ConstraintResolution and a Constraint normals with different size" << std::endl;

//                m_normals = normals.resize()
            }
        }

        void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int & constraintId) {
            m_p1->buildJacobianConstraint(cId, m_normals.m_dirs,  1.0, constraintId);
            m_p2->buildJacobianConstraint(cId, m_normals.m_dirs, -1.0, constraintId);

            constraintId += m_normals.size();
        }

        void getConstraintViolation(defaulttype::BaseVector *v,unsigned & cid) {
            defaulttype::Vector3 PFree = m_p1->getPosition(core::VecCoordId::freePosition());
            defaulttype::Vector3 QFree = m_p2->getPosition(core::VecCoordId::freePosition());
            defaulttype::Vector3 PQFree = PFree - QFree;

            for (unsigned i=0;i<m_normals.size();i++) {
                v->set(cid+i,dot(PQFree,m_normals.m_dirs[i]));
            }

            cid += m_normals.size();
        }

        void getConstraintResolution(std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned & offset) {
            resTab[offset] = m_resolution;
            offset += m_normals.size();
        }

        void draw(const core::visual::VisualParams* vparams,double scale) {
            if (m_normals.size()>0)
                vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[0] * scale, scale * 0.1, defaulttype::Vector4(1,0,0,1));

            if (m_normals.size()>1)
                vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[1] * scale, scale * 0.1, defaulttype::Vector4(0,1,0,1));

            if (m_normals.size()>2)
                vparams->drawTool()->drawArrow(m_p1->getPosition(), m_p1->getPosition() + m_normals.m_dirs[2] * scale, scale * 0.1, defaulttype::Vector4(0,0,1,1));
        }

        collisionAlgorithm::BaseProximity::SPtr getFirstProximity() const {
            return m_p1;
        }

        collisionAlgorithm::BaseProximity::SPtr getSecondProximity() const {
            return m_p2;
        }

     private:
        collisionAlgorithm::BaseProximity::SPtr m_p1;
        collisionAlgorithm::BaseProximity::SPtr m_p2;
        sofa::core::behavior::ConstraintResolution * m_resolution;
        ConstraintNormal m_normals;
    };

    virtual InternalConstraint createConstraint(const collisionAlgorithm::DetectionOutput & out) = 0;

};

}

}
