#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/directions/BindDirection.h>
#include <sofa/core/behavior/ConstraintResolution.h>

namespace sofa {

namespace constraintGeometry {

//class InsertionConstraintResolution1 : public sofa::core::behavior::ConstraintResolution {
//public:
//    InsertionConstraintResolution1(double maxf1 = std::numeric_limits<double>::max(), double comp1 = 0.0) : sofa::core::behavior::ConstraintResolution(1) {
//        m_maxForce = maxf1;
//
//        m_compliance = comp1;
//    }
//
//    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/) {
//        force[line] -= d[line] / (w[line][line] + m_compliance);
//        if (force[line] > m_maxForce) force[line]=m_maxForce;
//        if (force[line] < -m_maxForce) force[line]=-m_maxForce;
//    }
//
//    double m_compliance;
//    double m_maxForce;
//};
//
//class InsertionConstraintResolution2 : public sofa::core::behavior::ConstraintResolution {
//public:
//    InsertionConstraintResolution2(double maxf1 = std::numeric_limits<double>::max(),double maxf2 = std::numeric_limits<double>::max(),
//                                   double comp1 = 0.0,double comp2 = 0.0) : sofa::core::behavior::ConstraintResolution(2) {
//        m_maxForce[0] = maxf1;
//        m_maxForce[1] = maxf2;
//
//        m_compliance[0] = comp1;
//        m_compliance[1] = comp2;
//    }
//
//    virtual void init(int line, double** w, double * /*force*/)
//    {
//        sofa::type::Mat<2,2,double> temp;
//        for (unsigned j=0;j<2;j++) {
//            for (unsigned i=0;i<2;i++) {
//                temp[j][i] = w[line+j][line+i];
//
//                if (i == j) temp[j][i]+=m_compliance[i];
//            }
//        }
//
//        SOFA_UNUSED(sofa::type::invertMatrix(invW,temp));
//    }
//
//    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
//    {
//        for(int i=0; i<2; i++) {
//            for(int j=0; j<2; j++)
//                force[line+i] -= d[line+j] * invW[i][j];
//        }
//
//        for(int i=0; i<2; i++) {
//            if (force[line+i] > m_maxForce[i]) force[line+i]=m_maxForce[i];
//            else if (force[line+i] < -m_maxForce[i]) force[line+i]=-m_maxForce[i];
//        }
//    }
//
//    double m_maxForce[2];
//    double m_compliance[2];
//    sofa::type::Mat<2,2,double> invW;
//};


class InsertionConstraintResolution : public sofa::core::behavior::ConstraintResolution {
public:
    InsertionConstraintResolution(SReal frictionLimit, double maxf1 = std::numeric_limits<double>::max(),double maxf2 = std::numeric_limits<double>::max(),double maxf3 = std::numeric_limits<double>::max(),
                                   double comp1 = 0.0,double comp2 = 0.0,double comp3 = 0.0) : sofa::core::behavior::ConstraintResolution(3) {
        m_maxForce[0] = maxf1;
        m_maxForce[1] = maxf2;
        m_maxForce[2] = maxf3;

        m_compliance[0] = comp1;
        m_compliance[1] = comp2;
        m_compliance[2] = comp3;

        m_frictionLimit = frictionLimit;
    }

    virtual void init(int line, double** w, double * /*force*/)
    {
        sofa::type::Mat<3,3,double> temp;
        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                temp[j][i] = w[line+j][line+i];

                if (i == j) temp[j][i]+=m_compliance[i];
            }
        }

        SOFA_UNUSED(sofa::type::invertMatrix(invW,temp));
    }

    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/)
    {
        //we solve the first two bilateral constraints
        double inv;
        inv=1.0/(w[line][line]);
        double corr0 = -inv * d[line+0];
        force[line+0] += corr0;

        d[line+1] += w[line+1][line+0] * corr0;
        inv=1.0/(w[line+1][line+1]);
        double corr1= -inv * d[line+1];
        force[line+1] += corr1;

        for(int i=0; i<2; i++) {
            if (force[line+i] > m_maxForce[i]) force[line+i]=m_maxForce[i];
            else if (force[line+i] < -m_maxForce[i]) force[line+i]=-m_maxForce[i];
        }

        // then solve for the frictional constraint
        d[line+2] += w[line+2][line+0] * corr0 + w[line+2][line+1] * corr1;
        inv=1.0/(w[line+2][line+2]);
        double corr2= -inv * d[line+2];
        force[line+2] += corr2;

        SReal slip_force = m_frictionLimit;
        if (force[line+2] >  slip_force) force[line+2] = slip_force;
        else if (force[line+2] < -slip_force) force[line+2] = -slip_force;

        if (force[line+2] > m_maxForce[2]) force[line+2]=m_maxForce[2];
        else if (force[line+2] < -m_maxForce[2]) force[line+2]=-m_maxForce[2];
    }

    double m_maxForce[3];
    double m_compliance[3];
    sofa::type::Mat<3,3,double> invW;
    SReal m_frictionLimit;
};

}

}
