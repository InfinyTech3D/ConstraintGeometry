//#pragma once

//#include <Response.h>
//#include "BaseGeometry.h"
//#include <math.h>

//namespace constraintGeometry {

//template<int C>
//class UnilateralConstraintResolution : public ConstraintResolution {
//public:
//    UnilateralConstraintResolution(double m)
//    : m_maxForce(m) {
//        nbLines = C;
//    }

//    virtual void init(int line, double** w, double */*force*/)
//    {
//        sofa::defaulttype::Mat<C,C,double> temp;
//        for (unsigned j=0;j<C;j++) {
//            for (unsigned i=0;i<C;i++) {
//                temp[j][i] = w[line+j][line+i];
//            }
//        }
//        invertMatrix(invW, temp);
//    }

//    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
//    {
//        for(int i=0; i<C; i++) {
//            for(int j=0; j<C; j++)
//                force[line+i] -= d[line+j] * invW[i][j];
//        }


//        for(int i=0; i<C; i++) {
//            if (force[line+i]>m_maxForce) force[line+i] = m_maxForce;
//            else if (force[line+i]<0) force[line+i] = 0.0;
//        }
//    }

//    double m_maxForce;
//    sofa::defaulttype::Mat<C,C,double> invW;
//};

//class UnilateralResponse : public Response {
//public:
//    SOFA_CLASS(UnilateralResponse, Response);

//    Data<double> d_maxForce;

//    UnilateralResponse()
//    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

//    void getConstraintNormals(const PariProximityVector & pp, std::vector<ConstraintNormal> & out) {
//        for (unsigned i=0;i<pp.size();i++) {
//            defaulttype::Vector3 P = pp[i].first->getPosition();
//            defaulttype::Vector3 Q = pp[i].second->getPosition();

//            defaulttype::Vector3 PQ = P-Q;

//            PQ.normalize();
//            defaulttype::Vector3 N = pp[i].second->getNormal();

//            double dotPQ = dot(PQ,N);
//            if (dotPQ<0) PQ *= -1;

//            out.push_back(ConstraintNormal(pp[i], PQ));
//        }
//    }

//    void getConstraintViolation(const collisionAlgorithm::PairProximity & detection, const ConstraintNormal & normal, defaulttype::BaseVector *v,unsigned cid) {
//        Vector3 PFree = detection.first->getPosition(core::VecCoordId::freePosition());
//        Vector3 QFree = detection.second->getPosition(core::VecCoordId::freePosition());
//        Vector3 PQFree = PFree - QFree;

//        for (unsigned i=0;i<normal.normals().size();i++) {
//            v->set(cid+i,dot(normal.normals()[i],PQFree));
//        }
//    }

//    ConstraintResolution * getResolution(const ConstraintNormal & normal) {
//        if (normal.size() == 1) return new UnilateralConstraintResolution<1>(d_maxForce.getValue());
//        else if (normal.size() == 2) return new UnilateralConstraintResolution<2>(d_maxForce.getValue());
//        else if (normal.size() == 3) return new UnilateralConstraintResolution<3>(d_maxForce.getValue());
//        return NULL;
//    }
//};

//}
