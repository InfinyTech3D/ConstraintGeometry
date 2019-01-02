#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/defaulttype/VecTypes.h>
#include <memory>

namespace sofa {

namespace constraintGeometry {

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

    defaulttype::Vector3 operator[](unsigned sz) {
        return m_normals[sz];
    }

    std::vector<defaulttype::Vector3> m_normals;
    collisionAlgorithm::ConstraintProximity::SPtr m_prox;
};


class ConstraintReponse : public sofa::core::behavior::ConstraintResolution {
public:

    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    ConstraintReponse(ConstraintNormal n, const collisionAlgorithm::DetectionOutput::SPtr o)
    : ConstraintResolution(m_normals.size())
    , m_normals(n)
    , m_output(o) {}

    void buildConstraintMatrix(core::MultiMatrixDerivId cId, unsigned int constraintId) {
        for (unsigned i=0;i<m_output->size();i++) {
            collisionAlgorithm::ConstraintProximity::SPtr prox = m_output->getProximity(i);

            DataMatrixDeriv & c1_d = *cId[prox->getState()].write();
            MatrixDeriv & c1 = *c1_d.beginEdit();

            std::map<unsigned, double> m1 = prox->getContributions();
            for (unsigned j=0;j<m_normals.size();j++) {
                MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);

                for (auto it=m1.begin();it!=m1.end();it++) {
                    c_it.addCol(it->first, m_normals[j] * it->second);
                }
            }

            c1_d.endEdit();
        }

//        collisionAlgorithm::ConstraintProximity::SPtr prox1 = m_pproxy.first;
//        { // object 1
//            DataMatrixDeriv & c1_d = *cId[prox1->getState()].write();
//            MatrixDeriv & c1 = *c1_d.beginEdit();

//            std::map<unsigned, double> m1 = prox1->getContributions();
//            for (unsigned j=0;j<m_directions.size();j++) {
//                MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);

//                for (auto it=m1.begin();it!=m1.end();it++) {
//                    c_it.addCol(it->first, m_directions.m_normals[j] * it->second);
//                }
//            }

//            c1_d.endEdit();
//        }

//        { // object 2
//            collisionAlgorithm::ConstraintProximity::SPtr prox2 = m_pproxy.second;
//            DataMatrixDeriv & c2_d = *cId[prox2->getState()].write();
//            MatrixDeriv & c2 = *c2_d.beginEdit();

//            std::map<unsigned, double> m2 = prox2->getContributions();
//            for (unsigned j=0;j<m_directions.size();j++) {
//                MatrixDerivRowIterator c_it = c2.writeLine(constraintId+j);

//                for (auto it=m2.begin();it!=m2.end();it++) {
//                    c_it.addCol(it->first, -m_directions.m_normals[j] * it->second);
//                }
//            }

//            c2_d.endEdit();
//        }
    }

    void getConstraintViolation(defaulttype::BaseVector *v,unsigned cid) {
//        collisionAlgorithm::ConstraintProximity::SPtr prox1 = m_pproxy.first;
//        collisionAlgorithm::ConstraintProximity::SPtr prox2 = m_pproxy.second;

//        defaulttype::Vector3 PFree = prox1->getPosition(core::VecCoordId::freePosition());
//        defaulttype::Vector3 QFree = prox2->getPosition(core::VecCoordId::freePosition());
//        defaulttype::Vector3 PQFree = PFree - QFree;

//        for (unsigned i=0;i<m_directions.m_normals.size();i++) {
//            v->set(cid+i,dot(PQFree,m_directions.m_normals[i]));
//        }
    }

    void draw(const core::visual::VisualParams* vparams, double scale, defaulttype::Vector4 c) {
//        for (unsigned i=0;i<m_directions.size();i++) {
//            vparams->drawTool()->drawArrow(m_pproxy.second->getPosition(),
//                                           m_pproxy.second->getPosition() + m_directions.m_normals[i] * scale,
//                                           scale*0.1,
//                                           c);

//        }
    }

    unsigned size() {
        return m_normals.size();
    }

    ConstraintNormal m_normals;
    collisionAlgorithm::DetectionOutput::SPtr m_output;
};

}

}
