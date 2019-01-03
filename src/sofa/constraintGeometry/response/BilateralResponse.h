#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class BilateralConstraintResolution1 : public ConstraintReponse {
public:
    BilateralConstraintResolution1(ConstraintNormal n, collisionAlgorithm::ConstraintProximity::SPtr p1,collisionAlgorithm::ConstraintProximity::SPtr p2, double m) : ConstraintReponse(n,p1,p2), m_maxForce(m) {}

    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/) {
        force[line] -= d[line] / w[line][line];
    }

    double m_maxForce;
};

class BilateralConstraintResolution3 : public ConstraintReponse {
public:
    BilateralConstraintResolution3(ConstraintNormal n, collisionAlgorithm::ConstraintProximity::SPtr p1,collisionAlgorithm::ConstraintProximity::SPtr p2, double m) : ConstraintReponse(n,p1,p2), m_maxForce(m) {}

    virtual void init(int line, double** w, double */*force*/)
    {
        sofa::defaulttype::Mat<3,3,double> temp;
        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                temp[j][i] = w[line+j][line+i];
            }
        }

        sofa::defaulttype::invertMatrix(invW,temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
    {
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++)
                force[line+i] -= d[line+j] * invW[i][j];
        }
    }

    double m_maxForce;
    sofa::defaulttype::Mat<3,3,double> invW;
};

}

}
