#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class UnilateralConstraintResolution : public ConstraintReponse {
public:
    UnilateralConstraintResolution(ConstraintNormal n, collisionAlgorithm::ConstraintProximity::SPtr p1,collisionAlgorithm::ConstraintProximity::SPtr p2, double m) : ConstraintReponse(n,p1,p2), m_maxForce(m){}

    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/) {
        force[line] -= d[line] / w[line][line];

        if (force[line]>m_maxForce) force[line] = m_maxForce;
        else if (force[line]<0) force[line] = 0.0;
    }

    double m_maxForce;
};

class UnilateralFrictionResolution : public ConstraintReponse {
public:
    UnilateralFrictionResolution(ConstraintNormal n, collisionAlgorithm::ConstraintProximity::SPtr p1,collisionAlgorithm::ConstraintProximity::SPtr p2, double m,double f) : ConstraintReponse(n,p1,p2), m_maxForce(m), m_friction(f) {}

    virtual void init(int line, double** w, double */*force*/) {
        sofa::defaulttype::Mat<3,3,double> temp;
        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                temp[j][i] = w[line+j][line+i];
            }
        }

        invertMatrix(invW, temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/) {
        double bforce[3];
        for(int i=0; i<3; i++) bforce[line+i] = bforce[line+i];

        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++)
                force[line+i] -= d[line+j] * invW[i][j];
        }


        for(int i=0; i<3; i++) {
            if (force[line+i]>m_maxForce) force[line+i] = m_maxForce;
            else if (force[line+i]<0) force[line+i] = 0.0;
        }
    }

    double m_maxForce;
    double m_friction;
    sofa::defaulttype::Mat<3,3,double> invW;
};

}

}
