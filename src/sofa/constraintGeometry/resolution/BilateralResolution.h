#pragma once

#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/defaulttype/Mat.h>

namespace sofa {

namespace constraintGeometry {

class BilateralConstraintResolution1 : public sofa::core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution1(double m) : sofa::core::behavior::ConstraintResolution(1), m_maxForce(m) {}

    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/) {
        force[line] -= d[line] / w[line][line];
        if (force[line] > m_maxForce) force[line]=m_maxForce;
        if (force[line] < -m_maxForce) force[line]=-m_maxForce;
    }

    double m_maxForce;
};

class BilateralConstraintResolution2 : public sofa::core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution2(double m) : sofa::core::behavior::ConstraintResolution(2), m_maxForce(m) {}

    virtual void init(int line, double** w, double * /*force*/)
    {
        sofa::defaulttype::Mat<2,2,double> temp;
        for (unsigned j=0;j<2;j++) {
            for (unsigned i=0;i<2;i++) {
                temp[j][i] = w[line+j][line+i];
            }
        }

        sofa::defaulttype::invertMatrix(invW,temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
    {
        for(int i=0; i<2; i++) {
            for(int j=0; j<2; j++)
                force[line+i] -= d[line+j] * invW[i][j];
        }
    }

    double m_maxForce;
    sofa::defaulttype::Mat<2,2,double> invW;
};


class BilateralConstraintResolution3 : public sofa::core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution3(double m) : sofa::core::behavior::ConstraintResolution(3), m_maxForce(m) {}

    virtual void init(int line, double** w, double * /*force*/)
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

        for(int i=0; i<3; i++) {
            if (force[line+i] > m_maxForce) force[line+i]=m_maxForce;
            if (force[line+i] < -m_maxForce) force[line+i]=-m_maxForce;
        }
    }

    double m_maxForce;
    sofa::defaulttype::Mat<3,3,double> invW;
};

}

}
