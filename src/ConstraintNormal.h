#ifndef SOFA_COMPONENT_CONSTRAINT_NORMAL_H
#define SOFA_COMPONENT_CONSTRAINT_NORMAL_H

#include "BaseGeometry.h"
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "ConstraintProximity.h"
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {

class ConstraintNormal {
public :

//    ConstraintNormal(ConstraintProximityPtr p1) {
//        pinfo1 = p1;
//        pinfo2 = NULL;
//    }

    ConstraintNormal(ConstraintProximityPtr p1,ConstraintProximityPtr p2) {
        pinfo1 = p1;
        pinfo2 = p2;
    }


    void setNormal(defaulttype::Vector3 N) {
        m_normals.resize(1);
        m_normals[0] = N;
    }

    void setNormal(defaulttype::Vector3 N1,defaulttype::Vector3 N2,defaulttype::Vector3 N3) {
        m_normals[0] = N1;
        m_normals[1] = N2;
        m_normals[2] = N3;

        m_normals[0].normalize();
        m_normals[1].normalize();
        m_normals[2].normalize();
    }

//    void createConstraint(defaulttype::Vector3 N1) {
//        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
//        N1.normalize();
//        m_normals.push_back(N1);
//    }

//    void createFrame(defaulttype::Vector3 N1) {
//        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
//        N1.normalize();
//        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
//        N2.normalize();
//        defaulttype::Vector3 N3 = cross(N1,N2);
//        N3.normalize();

//        m_normals.resize(3);
//        m_normals[0] = N1;
//        m_normals[1] = N2;
//        m_normals[2] = N3;
//    }

    void orthogonalize(unsigned i) {
        if (m_normals.size() != i) {
            std::cout << "TOTO RESIZE in ConstraintNormal" <<  m_normals.size() << " " << i<< std::endl;
            return;
        }

        for (unsigned i=0;i<m_normals.size();i++) m_normals[i].normalize();
    }

    void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned & cline) {
        for (unsigned i=0;i<m_normals.size();i++) {
            pinfo1->buildConstraintMatrix(cParams,cId,cline+i,m_normals[i]);
            pinfo2->buildConstraintMatrix(cParams,cId,cline+i,-m_normals[i]);
        }

        cline += m_normals.size();
    }

    void getConstraintViolation(const ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned & cid) {
        defaulttype::Vector3 PFree = pinfo1->getFreePosition();
        defaulttype::Vector3 QFree = pinfo2->getFreePosition();
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_normals.size();i++) v->set(cid+i,dot(m_normals[i],PQFree));

        cid+=m_normals.size();
    }

    unsigned size() {
        return m_normals.size();
    }

    bool empty() {
        return m_normals.size() == 0;
    }

    ConstraintProximityPtr getPinfo1() {
        return pinfo1;
    }

    ConstraintProximityPtr getPinfo2() {
        return pinfo2;
    }

    const helper::vector<defaulttype::Vector3> & getNormals() {
        return m_normals;
    }

private:
    helper::vector<defaulttype::Vector3> m_normals;
    ConstraintProximityPtr pinfo1;
    ConstraintProximityPtr pinfo2;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
