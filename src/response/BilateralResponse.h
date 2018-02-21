#ifndef SOFA_COMPONENT_CONSTRAINT_UNILATERALRESPONSE_H
#define SOFA_COMPONENT_CONSTRAINT_UNILATERALRESPONSE_H

#include "ConstraintResponse.h"
#include "BaseGeometry.h"
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {

class BilateralConstraintResolution : public core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution(double m = std::numeric_limits<double>::max())
    : m_maxForce(m)
    {}

    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/)
    {
        if (w[line][line] == 0.0) force[line] = 0.0;
        else force[line] -= d[line] / w[line][line];

        if (force[line]*w[line][line]>m_maxForce) force[line] = m_maxForce;
        else if (force[line]*w[line][line]<-m_maxForce) force[line] = -m_maxForce;
    }

    double m_maxForce;
};

class BilateralResponse : public ConstraintResponse {
public:

    BilateralResponse(ConstraintProximityPtr p1,ConstraintProximityPtr p2,defaulttype::Vector3 N)
    : m_pfrom(p1)
    , m_pdest(p2)
    {
        m_normals.push_back(N);
    }

    void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned cline) {
        for (unsigned i=0;i<m_normals.size();i++) {
            m_pfrom->buildConstraintMatrix(cParams,cId,cline+i,m_normals[i]);
            m_pdest->buildConstraintMatrix(cParams,cId,cline+i,-m_normals[i]);
        }
    }

    void getConstraintViolation(const ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned cid) {
        defaulttype::Vector3 PFree = m_pfrom->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 QFree = m_pdest->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 PQFree = PFree - QFree;

        for (unsigned i=0;i<m_normals.size();i++) v->set(cid+i,dot(m_normals[i],PQFree));
    }

    core::behavior::ConstraintResolution * getResolution() {
        return new BilateralConstraintResolution();
    }

    unsigned size() {
        return m_normals.size();
    }

    const helper::vector<defaulttype::Vector3> & getNormals() {
        return m_normals;
    }

//    static helper::fixed_array<defaulttype::Vector3,3> createFrame(defaulttype::Vector3 N1) {
//        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
//        N1.normalize();
//        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
//        N2.normalize();
//        defaulttype::Vector3 N3 = cross(N1,N2);
//        N3.normalize();

//        helper::fixed_array<defaulttype::Vector3,3> normals;
//        normals[0] = N1;
//        normals[1] = N2;
//        normals[2] = N3;

//        return normals;
//    }

protected:
    ConstraintProximityPtr m_pfrom;
    ConstraintProximityPtr m_pdest;
    helper::vector<defaulttype::Vector3> m_normals;

};

//typedef std::shared_ptr<ConstraintNormal> ConstraintNormalPtr;

//class FrictionConstraintNormal : public ConstraintNormal {
//public:
//    FrictionConstraintNormal(std::pair<ConstraintProximityPtr,ConstraintProximityPtr> p,
//                             helper::fixed_array<defaulttype::Vector3,1> N,
//                             double maxF = std::numeric_limits<double>::max())
//    : ConstraintNormal(p) {
//        m_normals.push_back(N[0].normalized());
//        m_maxForce = maxF;
//        m_force = 0.0;
//    }

//    FrictionConstraintNormal(std::pair<ConstraintProximityPtr,ConstraintProximityPtr> p,
//                             helper::fixed_array<defaulttype::Vector3,3> N,
//                             double maxF = std::numeric_limits<double>::max())
//    : ConstraintNormal(p) {
//        m_normals.push_back(N[0].normalized());
//        m_normals.push_back(N[1].normalized());
//        m_normals.push_back(N[2].normalized());
//        m_maxForce = maxF;
//        m_force = 0.0;
//    }

//    core::behavior::ConstraintResolution * getResolution() {
//        if (size() == 1) return new UnilateralConstraintResolution1(m_maxForce,m_force);
////        else if (sz == 3) return new UnilateralConstraintResolution3(m_maxForce);

//        std::cerr << "Error call of FrictionConstraintNormal::getConstraintResolution with unsuported size" << std::endl;
//        return NULL;
//    }


//    double m_maxForce;
//    double m_force;

//};


//class BilateralConstraintNormal : public ConstraintNormal {
//public:
//    BilateralConstraintNormal(std::pair<ConstraintProximityPtr,ConstraintProximityPtr> p,
//                              helper::fixed_array<defaulttype::Vector3,1> N,
//                              double maxF = std::numeric_limits<double>::max())
//    : ConstraintNormal(p) {
//        m_normals.push_back(N[0].normalized());
//        m_maxForce = maxF;
//    }

//    BilateralConstraintNormal(std::pair<ConstraintProximityPtr,ConstraintProximityPtr> p,
//                              helper::fixed_array<defaulttype::Vector3,3> N,
//                              double maxF = std::numeric_limits<double>::max())
//    : ConstraintNormal(p) {
//        m_normals.push_back(N[0].normalized());
//        m_normals.push_back(N[1].normalized());
//        m_normals.push_back(N[2].normalized());
//        m_maxForce = maxF;
//    }

//    core::behavior::ConstraintResolution * getResolution() {
////        if (size() == 1) return new UnilateralConstraintResolution1(m_maxForce);
////        else if (sz == 3) return new UnilateralConstraintResolution3(m_maxForce);

//        std::cerr << "Error call of BilateralConstraint::getConstraintResolution with unsuported size" << std::endl;
//        return NULL;
//    }

//protected:
//    double m_maxForce;
//};
////class ConstraintNormalPair1 : public ConstraintNormalPair {
////public :

////    ConstraintNormalPair1(ConstraintResponse r,std::pair<ConstraintProximityPtr,ConstraintProximityPtr> p,defaulttype::Vector3 N1)
////    : ConstraintNormalPair(r,p) {
////        m_normals.resize(1);
////        m_normals[0] = N1;
////        m_normals[0].normalize();
////    }

////    void setNormal(defaulttype::Vector3 N1,defaulttype::Vector3 N2,defaulttype::Vector3 N3) {
////        m_normals[0] = N1;
////        m_normals[1] = N2;
////        m_normals[2] = N3;

////        m_normals[0].normalize();
////        m_normals[1].normalize();
////        m_normals[2].normalize();
////    }

////    void createConstraint(defaulttype::Vector3 N1) {
////        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
////        N1.normalize();
////        m_normals.push_back(N1);
////    }

////    void createFrame(defaulttype::Vector3 N1) {
////        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
////        N1.normalize();
////        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
////        N2.normalize();
////        defaulttype::Vector3 N3 = cross(N1,N2);
////        N3.normalize();

////        m_normals.resize(3);
////        m_normals[0] = N1;
////        m_normals[1] = N2;
////        m_normals[2] = N3;
////    }

////    void orthogonalize(unsigned i) {
////        if (m_normals.size() != i) {
////            std::cout << "TOTO RESIZE in ConstraintNormal" <<  m_normals.size() << " " << i<< std::endl;
////            return;
////        }

////        for (unsigned i=0;i<m_normals.size();i++) m_normals[i].normalize();
////    }
////};
////class ConstraintNormalFactory {
////public:

////    static ConstraintNormalPtr create(std::pair<ConstraintProximityPtr,ConstraintProximityPtr> p,defaulttype::Vector3 & N,ConstraintResponse r) {
////        return ConstraintNormalPtr(new ConstraintNormalPair1(p,N,r));
////    }

////};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
