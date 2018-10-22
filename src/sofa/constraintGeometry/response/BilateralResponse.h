//#pragma once

//#include <Response.h>
//#include "BaseGeometry.h"
//#include <math.h>

//namespace constraintGeometry {

//template<int C>
//class BilateralConstraintResolution : public ConstraintResolution {
//public:
//    BilateralConstraintResolution(double m)
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
//    }

//    double m_maxForce;
//    sofa::defaulttype::Mat<C,C,double> invW;
//};

//class BilateralResponse : public Response {
//public:

//    Data<double> d_maxForce;

//    BilateralResponse()
//    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

//    void getConstraintNormals(const PariProximityVector & pp, std::vector<ConstraintNormal> & out) {
//        for (unsigned i=0;i<pp.size();i++) {
//            defaulttype::Vector3 P = pp[i].first->getPosition();
//            defaulttype::Vector3 Q = pp[i].second->getPosition();
//            out.push_back(ConstraintNormal::createSingle(pp[i],P-Q));
//        }
//    }

//    void getConstraintViolation(const PariProximity & detection,const ConstraintNormal & normal, defaulttype::BaseVector *v,unsigned cid) {
//        defaulttype::Vector3 PFree = detection.first->getPosition(core::VecCoordId::freePosition());
//        defaulttype::Vector3 QFree = detection.second->getPosition(core::VecCoordId::freePosition());
//        defaulttype::Vector3 PQFree = PFree - QFree;

//        for (unsigned i=0;i<normal.normals().size();i++) {
//            v->set(cid+i,dot(normal.normals()[i],PQFree));
//        }
//    }

//    core::behavior::ConstraintResolution * getResolution(const ConstraintNormal & normal) {
//        if (normal.size() == 1) return new BilateralConstraintResolution<1>(d_maxForce.getValue());
//        else if (normal.size() == 2) return new BilateralConstraintResolution<2>(d_maxForce.getValue());
//        else if (normal.size() == 3) return new BilateralConstraintResolution<3>(d_maxForce.getValue());
//        return NULL;
//    }
//};

//}
