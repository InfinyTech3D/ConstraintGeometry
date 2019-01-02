#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

template<int C>
class BilateralConstraintResolution : public ConstraintReponse {
public:
    BilateralConstraintResolution(ConstraintNormal n, const collisionAlgorithm::DetectionOutput::SPtr o,double m) : ConstraintReponse(n,o), m_maxForce(m) {}

    virtual void init(int line, double** w, double */*force*/)
    {
        sofa::defaulttype::Mat<C,C,double> temp;
        for (unsigned j=0;j<C;j++) {
            for (unsigned i=0;i<C;i++) {
                temp[j][i] = w[line+j][line+i];
            }
        }

        sofa::defaulttype::invertMatrix(invW,temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
    {
        for(int i=0; i<C; i++) {
            for(int j=0; j<C; j++)
                force[line+i] -= d[line+j] * invW[i][j];
        }
    }

    double m_maxForce;
    sofa::defaulttype::Mat<C,C,double> invW;
};

}

}
